#include <Wire.h>
#include <math.h>

// ============================================================================
// ROBOCON BAC NINH 2026 - KE 1 - FINAL SEQUENCE
// Arduino Mega 2560 + 4 mecanum + MPU6500 + 2 stepper + Raspberry Pi 5
//
// PHAN CUNG CO DINH:
//   FL: IN1=22 IN2=24 PWM=2
//   FR: IN1=26 IN2=28 PWM=4
//   RL: IN1=30 IN2=32 PWM=6
//   RR: IN1=34 IN2=36 PWM=8
//
//   MPU6500: SDA=20, SCL=21, addr auto 0x68/0x69
//
//   STEP1 LEFT : STEP=38 DIR=40 EN=42
//   STEP2 RIGHT: STEP=44 DIR=46 EN=48
//
//   USB Serial 115200
//
// CAMERA:
//   /dev/video2 = LINE
//   /dev/video0 = AI
//   Chi mot camera active tai mot thoi diem.
//
// KICH BAN KE 1:
//   1) LINE_ON
//   2) [TIEN] 3 giay, line giu thang; LOST_LINE tam thoi -> MPU giu yaw
//   3) DUNG + CAM_OFF
//   4) AI2_ON: Pi quet cap hang va GUI live AIM2:<xerr>,<boxHeight>
//   5) Hai stepper nang so bo DONG THOI
//   6) [TIEN CHAM] tiep can bang camera AI:
//        - xerr can tam mecanum
//        - boxHeight lam proxy khoang cach
//        - khi boxHeight >= AI_NEAR_BOX_HEIGHT va tam on dinh -> dung
//      Neu AI mat qua lau -> fallback [TIEN] 3 giay bang MPU.
//   7) Hai stepper NHICH LEN de nhac pallet khoi ke
//   8) CAM_OFF
//   9) [LUI CHEO PHAI] 10 giay + MPU giu yaw
//  10) [XOAY PHAI] 90 do bang MPU
//  11) PHAN LOAI: LEFT truoc, RIGHT sau. KHONG invent route.
//
// PROTOCOL GOC GIU NGUYEN:
// Mega -> Pi: LINE_ON / AI_ON / AI2_ON / CAM_OFF
// Pi -> Mega:
//   PI_READY
//   XLINE:<px> / LOST_LINE
//   TARGET:<id> / AI_DONE
//   LEFT:<id> / RIGHT:<id> / AI2_DONE
//
// BO SUNG DE CAN TAM + "DO KHOANG CACH" BANG KICH THUOC BOX:
//   AIM:<xerr>,<boxHeight>
//   AIM2:<xerr>,<boxHeight>
//
// AI MAPPING TUYET DOI KHONG DOI:
//   1 = yt   = SAMSUNG
//   2 = chip = FOXCONN
//   3 = al   = AMKOR
//   4 = qr   = HANA MICRON VINA
// ============================================================================


// ============================================================================
// MOTOR
// ============================================================================

#define FL_IN1 22
#define FL_IN2 24
#define FL_PWM 2

#define FR_IN1 26
#define FR_IN2 28
#define FR_PWM 4

#define RL_IN1 30
#define RL_IN2 32
#define RL_PWM 6

#define RR_IN1 34
#define RR_IN2 36
#define RR_PWM 8

int PWM_FL = 100;
int PWM_FR = 100;
int PWM_RL = 100;
int PWM_RR = 100;

// Base movement PWM.
int PWM_TIEN_LINE = 100;
int PWM_TIEN_AI   = 75;
int PWM_CHEO      = 75;
int PWM_XOAY_FAST = 70;
int PWM_XOAY_SLOW = 45;


// ============================================================================
// MPU6500
// ============================================================================

uint8_t MPU_ADDR = 0;
bool mpuOK = false;

const float GYRO_SCALE = 65.5f;  // +/-500 dps
float gyroZBias = 0.0f;
float gzDps = 0.0f;
float yawDeg = 0.0f;
unsigned long lastImuUs = 0;

float KP = 10.0f;
float KD = 0.35f;

int MIN_CORR = 18;
int MAX_CORR = 35;

float YAW_DEADZONE = 0.30f;
float GZ_DEADZONE = 0.8f;


// ============================================================================
// STEPPER
// ============================================================================

#define STEP1_STEP 38
#define STEP1_DIR  40
#define STEP1_EN   42

#define STEP2_STEP 44
#define STEP2_DIR  46
#define STEP2_EN   48

const uint8_t STEPPER_EN_ON  = LOW;
const uint8_t STEPPER_EN_OFF = HIGH;

bool STEP1_DIR_UP = HIGH;
bool STEP2_DIR_UP = HIGH;

unsigned int STEPPER_PULSE_US = 800;

// CHI LA GIA TRI TEST - PHAI TUNE CO KHI.
long PRE_PICK_UP_STEPS = 1400;
long PICK_NUDGE_STEPS  = 300;


// ============================================================================
// LINE
// ============================================================================

int errorLine = 0;
bool coLine = false;
unsigned long lastLineMs = 0;

// Tune line: deadband nho hon + sua ngang manh hon.
int LINE_DEAD = 10;
int LINE_MAX_X_CORR = 38;

// Neu test thay error duong ma robot lai di TRAI, doi thanh -1.
int LINE_X_SIGN = +1;

const unsigned long LINE_TIMEOUT_MS = 250;
const unsigned long START_LINE_RUN_MS = 3000;


// ============================================================================
// AI LIVE APPROACH
// ============================================================================

int aiXError = 0;
int aiBoxHeight = 0;
bool aiAimFresh = false;
unsigned long lastAiAimMs = 0;

// AIM cu qua lau thi dung som, tranh chay theo du lieu stale.
const unsigned long AI_AIM_TIMEOUT_MS = 220;

// Can tam theo pixel.
int AI_CENTER_DEAD_PX = 12;
int AI_MAX_X_CORR = 50;

// Hieu chinh lech co khi/camera:
// - AI_X_SIGN = +1: target o ben PHAI => robot strafe PHAI.
// - Neu test thuc te nguoc chieu, doi AI_X_SIGN = -1.
// - AI_CENTER_OFFSET_PX: tam mong muon trong anh.
//   0 = dung tam anh; +/- de bu camera/khung nang lech tam.
int AI_X_SIGN = +1;
int AI_CENTER_OFFSET_PX = 0;

// "Khoang cach" duoc suy ra tu boxHeight.
// Box cang lon => kien cang gan.
// PHAI tune tren camera that.
int AI_NEAR_BOX_HEIGHT = 180;

// Can dung on dinh mot chut moi nhich stepper.
const unsigned long AI_NEAR_STABLE_MS = 250;

const unsigned long AI_APPROACH_TIMEOUT_MS = 6500;
const unsigned long AI_FALLBACK_FORWARD_MS = 3000;


// ============================================================================
// AI IDs
// ============================================================================

uint8_t leftId = 0;
uint8_t rightId = 0;

bool gotLeft = false;
bool gotRight = false;
bool ai2Done = false;

unsigned long ai2StartMs = 0;
const unsigned long AI2_SCAN_TIMEOUT_MS = 9000;


// ============================================================================
// KICH BAN
// ============================================================================

const unsigned long LUI_CHEO_PHAI_MS = 10000;

const float TURN_RIGHT_TARGET_DEG = 90.0f;
const unsigned long TURN_TIMEOUT_MS = 5000;


// ============================================================================
// SERIAL
// ============================================================================

char rxBuf[72];
uint8_t rxLen = 0;

bool piReady = false;
bool emergencyStop = false;


// ============================================================================
// STATE MACHINE
// ============================================================================

enum State
{
  WAIT_PI,

  K1_LINE_START,
  K1_LINE_RUN,

  K1_AI2_START,
  K1_AI2_SCAN_AND_PRELIFT,

  K1_AI_APPROACH,

  K1_PICK_NUDGE,

  K1_RETREAT_DIAG_RIGHT,

  K1_TURN_RIGHT_90,

  K1_CLASSIFY_READY,

  K1_DONE,
  ERROR_STATE
};

State state = WAIT_PI;

unsigned long stateStartMs = 0;
bool lineRunStarted = false;
bool preLiftDone = false;


// ============================================================================
// FORWARD DECLARATIONS
// ============================================================================

void docSerial();
void stopMotor();


// ============================================================================
// MOTOR LOW LEVEL
// ============================================================================

void setMotor(int in1, int in2, int pwmPin, int value)
{
  value = constrain(value, -255, 255);

  if (value > 0)
  {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    analogWrite(pwmPin, value);
  }
  else if (value < 0)
  {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    analogWrite(pwmPin, -value);
  }
  else
  {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    analogWrite(pwmPin, 0);
  }
}


void setMecanumRaw(int fl, int fr, int rl, int rr)
{
  setMotor(FL_IN1, FL_IN2, FL_PWM, fl);
  setMotor(FR_IN1, FR_IN2, FR_PWM, fr);
  setMotor(RL_IN1, RL_IN2, RL_PWM, rl);
  setMotor(RR_IN1, RR_IN2, RR_PWM, rr);
}


void stopMotor()
{
  setMecanumRaw(0, 0, 0, 0);
}


// ============================================================================
// MECANUM MIXER
//
// vy > 0  = TIEN
// vy < 0  = LUI
//
// vx > 0  = PHAI
// vx < 0  = TRAI
//
// rot > 0 = XOAY PHAI
// rot < 0 = XOAY TRAI
//
// Strafe RIGHT:
// FL +, FR -, RL -, RR +
// ============================================================================

void driveMecanum(int vy, int vx, int rot)
{
  long fl = (long)vy + vx + rot;
  long fr = (long)vy - vx - rot;
  long rl = (long)vy - vx + rot;
  long rr = (long)vy + vx - rot;

  long a = labs(fl);
  long b = labs(fr);
  long c = labs(rl);
  long d = labs(rr);

  long maxAbs = max(max(a, b), max(c, d));

  if (maxAbs > 255)
  {
    fl = fl * 255L / maxAbs;
    fr = fr * 255L / maxAbs;
    rl = rl * 255L / maxAbs;
    rr = rr * 255L / maxAbs;
  }

  setMecanumRaw(
    (int)fl,
    (int)fr,
    (int)rl,
    (int)rr
  );
}


// ============================================================================
// MPU6500
// ============================================================================

bool i2cPing(uint8_t addr)
{
  Wire.beginTransmission(addr);
  return Wire.endTransmission() == 0;
}


void mpuWrite(uint8_t reg, uint8_t value)
{
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}


int16_t mpuRead16(uint8_t reg)
{
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);

  Wire.requestFrom(MPU_ADDR, (uint8_t)2);

  if (Wire.available() < 2)
    return 0;

  return (
    ((int16_t)Wire.read() << 8)
    | Wire.read()
  );
}


bool initMPU6500()
{
  Wire.begin();

  if (i2cPing(0x68))
    MPU_ADDR = 0x68;
  else if (i2cPing(0x69))
    MPU_ADDR = 0x69;
  else
    return false;

  mpuWrite(0x6B, 0x00);  // wake
  delay(50);

  mpuWrite(0x1A, 0x03);  // DLPF
  mpuWrite(0x1B, 0x08);  // +/-500 dps
  mpuWrite(0x19, 0x04);

  lastImuUs = micros();

  return true;
}


void calibrateGyroZ(unsigned int samples = 1200)
{
  if (!mpuOK)
    return;

  stopMotor();
  delay(300);

  long sum = 0;

  for (unsigned int i = 0; i < samples; i++)
  {
    sum += mpuRead16(0x47);
    delay(2);
  }

  gyroZBias = (float)sum / samples;

  yawDeg = 0.0f;
  gzDps = 0.0f;
  lastImuUs = micros();
}


void resetRelativeYaw()
{
  yawDeg = 0.0f;
  gzDps = 0.0f;
  lastImuUs = micros();
}


void updateMPU()
{
  if (!mpuOK)
    return;

  unsigned long nowUs = micros();

  float dt =
    (nowUs - lastImuUs)
    / 1000000.0f;

  lastImuUs = nowUs;

  int16_t rawGz = mpuRead16(0x47);

  gzDps =
    (rawGz - gyroZBias)
    / GYRO_SCALE;

  if (fabs(gzDps) < GZ_DEADZONE)
    gzDps = 0.0f;

  // Quy uoc da test:
  // quay phai => GZ/yaw duong.
  yawDeg += gzDps * dt;
}


int yawCorrection()
{
  if (!mpuOK)
    return 0;

  updateMPU();

  float error = -yawDeg;

  if (fabs(error) <= YAW_DEADZONE)
    return 0;

  float out =
    KP * error
    - KD * gzDps;

  out = constrain(
    out,
    -(float)MAX_CORR,
    (float)MAX_CORR
  );

  int corr = (int)out;

  if (
    corr != 0
    &&
    abs(corr) < MIN_CORR
  )
  {
    corr =
      (corr > 0)
      ? MIN_CORR
      : -MIN_CORR;
  }

  return corr;
}


// ============================================================================
// LINE CONTROL
// ============================================================================

bool lineFresh()
{
  return (
    coLine
    &&
    millis() - lastLineMs <= LINE_TIMEOUT_MS
  );
}


void forwardLineOrYaw()
{
  int vxCorr = 0;

  if (lineFresh())
  {
    if (abs(errorLine) > LINE_DEAD)
    {
      vxCorr =
        constrain(
          LINE_X_SIGN * (errorLine / 4),
          -LINE_MAX_X_CORR,
          LINE_MAX_X_CORR
        );
    }
  }

  int rotCorr = yawCorrection();

  // ===== TIEN =====
  driveMecanum(
    PWM_TIEN_LINE,
    vxCorr,
    rotCorr
  );
}


// ============================================================================
// AI APPROACH CONTROL
// ============================================================================

bool aiAimIsFresh()
{
  return (
    aiAimFresh
    &&
    millis() - lastAiAimMs <= AI_AIM_TIMEOUT_MS
  );
}


int aiCorrectedXError()
{
  return AI_X_SIGN * (aiXError - AI_CENTER_OFFSET_PX);
}


void forwardByAI()
{
  const int err = aiCorrectedXError();
  const int absErr = abs(err);

  int vxCorr = 0;

  if (absErr > AI_CENTER_DEAD_PX)
  {
    // Sua ngang manh hon ban cu (/2 thay vi /4).
    vxCorr =
      constrain(
        err / 2,
        -AI_MAX_X_CORR,
        AI_MAX_X_CORR
      );
  }

  // QUAN TRONG:
  // Neu chua can tam thi khong duoc vua tien nhanh vua sua ngang,
  // de tranh leo vao mep trai/phai cua kien.
  int forwardPwm = PWM_TIEN_AI;

  if (absErr > 70)
    forwardPwm = 0;     // can ngang tai cho
  else if (absErr > 40)
    forwardPwm = 28;    // bo cham
  else if (absErr > 22)
    forwardPwm = 48;    // tien cham
  else
    forwardPwm = PWM_TIEN_AI;

  // MPU chi giu huong, khong cho rot tranh chap qua manh voi can ngang AI.
  int rotCorr =
    constrain(
      yawCorrection(),
      -20,
      20
    );

  driveMecanum(
    forwardPwm,
    vxCorr,
    rotCorr
  );
}


// ============================================================================
// [LUI CHEO PHAI] + MPU
//
// vy < 0 = LUI
// vx > 0 = PHAI
// ============================================================================

void reverseDiagonalRightYaw()
{
  int rotCorr = yawCorrection();

  // ===== LUI CHEO PHAI =====
  driveMecanum(
    -PWM_CHEO,
    +PWM_CHEO,
    rotCorr
  );
}


// ============================================================================
// TURN RIGHT 90
// ============================================================================

bool turnRight90()
{
  if (!mpuOK)
    return false;

  resetRelativeYaw();

  unsigned long t0 = millis();

  while (
    yawDeg < TURN_RIGHT_TARGET_DEG
    &&
    millis() - t0 < TURN_TIMEOUT_MS
  )
  {
    updateMPU();

    float remaining =
      TURN_RIGHT_TARGET_DEG - yawDeg;

    int pwm =
      (remaining > 20.0f)
      ? PWM_XOAY_FAST
      : PWM_XOAY_SLOW;

    // ===== XOAY PHAI =====
    driveMecanum(
      0,
      0,
      +pwm
    );

    docSerial();

    if (emergencyStop)
      break;

    delay(2);
  }

  stopMotor();

  bool ok =
    !emergencyStop
    &&
    fabs(
      yawDeg - TURN_RIGHT_TARGET_DEG
    ) <= 6.0f;

  resetRelativeYaw();

  return ok;
}


// ============================================================================
// STEPPER
// ============================================================================

void enableStepper1(bool enable)
{
  digitalWrite(
    STEP1_EN,
    enable
      ? STEPPER_EN_ON
      : STEPPER_EN_OFF
  );
}


void enableStepper2(bool enable)
{
  digitalWrite(
    STEP2_EN,
    enable
      ? STEPPER_EN_ON
      : STEPPER_EN_OFF
  );
}


void moveBothSteppersUp(
  long steps1,
  long steps2
)
{
  steps1 = max(0L, steps1);
  steps2 = max(0L, steps2);

  digitalWrite(
    STEP1_DIR,
    STEP1_DIR_UP
  );

  digitalWrite(
    STEP2_DIR,
    STEP2_DIR_UP
  );

  enableStepper1(steps1 > 0);
  enableStepper2(steps2 > 0);

  long total =
    max(steps1, steps2);

  for (long i = 0; i < total; i++)
  {
    bool p1 = i < steps1;
    bool p2 = i < steps2;

    if (p1) digitalWrite(STEP1_STEP, HIGH);
    if (p2) digitalWrite(STEP2_STEP, HIGH);

    delayMicroseconds(
      STEPPER_PULSE_US
    );

    if (p1) digitalWrite(STEP1_STEP, LOW);
    if (p2) digitalWrite(STEP2_STEP, LOW);

    delayMicroseconds(
      STEPPER_PULSE_US
    );

    docSerial();

    if (emergencyStop)
      break;
  }

  enableStepper1(false);
  enableStepper2(false);
}


// ============================================================================
// SERIAL PARSER
// ============================================================================

void processMessage(const char *msg)
{
  if (strcmp(msg, "PI_READY") == 0)
  {
    piReady = true;
    return;
  }

  if (
    strncmp(
      msg,
      "XLINE:",
      6
    ) == 0
  )
  {
    errorLine =
      atoi(msg + 6);

    coLine = true;
    lastLineMs = millis();

    return;
  }

  if (strcmp(msg, "LOST_LINE") == 0)
  {
    coLine = false;
    return;
  }

  // AIM2:<xerr>,<boxHeight>
  if (
    strncmp(
      msg,
      "AIM2:",
      5
    ) == 0
  )
  {
    const char *comma =
      strchr(
        msg + 5,
        ','
      );

    if (comma != NULL)
    {
      aiXError =
        atoi(msg + 5);

      aiBoxHeight =
        atoi(comma + 1);

      aiAimFresh = true;
      lastAiAimMs = millis();
    }

    return;
  }

  // Single AI optional.
  if (
    strncmp(
      msg,
      "AIM:",
      4
    ) == 0
  )
  {
    const char *comma =
      strchr(
        msg + 4,
        ','
      );

    if (comma != NULL)
    {
      aiXError =
        atoi(msg + 4);

      aiBoxHeight =
        atoi(comma + 1);

      aiAimFresh = true;
      lastAiAimMs = millis();
    }

    return;
  }

  if (
    strncmp(
      msg,
      "LEFT:",
      5
    ) == 0
  )
  {
    int id =
      atoi(msg + 5);

    if (id >= 1 && id <= 4)
    {
      leftId = id;
      gotLeft = true;
    }

    return;
  }

  if (
    strncmp(
      msg,
      "RIGHT:",
      6
    ) == 0
  )
  {
    int id =
      atoi(msg + 6);

    if (id >= 1 && id <= 4)
    {
      rightId = id;
      gotRight = true;
    }

    return;
  }

  if (strcmp(msg, "AI2_DONE") == 0)
  {
    ai2Done = true;
    return;
  }

  if (strcmp(msg, "PI_STOP") == 0)
  {
    emergencyStop = true;
    stopMotor();
    return;
  }

  // ===== DEBUG =====

  if (strcmp(msg, "S") == 0)
  {
    emergencyStop = true;
    stopMotor();
    return;
  }

  if (strcmp(msg, "C") == 0)
  {
    calibrateGyroZ();
    return;
  }

  if (strcmp(msg, "Z") == 0)
  {
    resetRelativeYaw();
    return;
  }
}


void docSerial()
{
  while (Serial.available())
  {
    char c =
      Serial.read();

    if (c == '\r')
      continue;

    if (c == '\n')
    {
      rxBuf[rxLen] = '\0';

      if (rxLen > 0)
        processMessage(rxBuf);

      rxLen = 0;
    }
    else
    {
      if (
        rxLen
        <
        sizeof(rxBuf) - 1
      )
      {
        rxBuf[rxLen++] = c;
      }
      else
      {
        rxLen = 0;
      }
    }
  }
}


// ============================================================================
// FALLBACK [TIEN 3 GIAY]
// ============================================================================

bool fallbackForward3s()
{
  resetRelativeYaw();

  unsigned long t0 =
    millis();

  while (
    millis() - t0
    <
    AI_FALLBACK_FORWARD_MS
  )
  {
    docSerial();

    if (emergencyStop)
    {
      stopMotor();
      return false;
    }

    int rotCorr =
      yawCorrection();

    // ===== TIEN 3 GIAY =====
    driveMecanum(
      PWM_TIEN_AI,
      0,
      rotCorr
    );

    delay(2);
  }

  stopMotor();

  return true;
}


// ============================================================================
// [LUI CHEO PHAI 10 GIAY]
// ============================================================================

bool retreatDiagonalRight()
{
  resetRelativeYaw();

  unsigned long t0 =
    millis();

  while (
    millis() - t0
    <
    LUI_CHEO_PHAI_MS
  )
  {
    docSerial();

    if (emergencyStop)
    {
      stopMotor();
      return false;
    }

    // ===== LUI CHEO PHAI =====
    reverseDiagonalRightYaw();

    delay(2);
  }

  stopMotor();

  return true;
}


// ============================================================================
// PHAN LOAI - KHONG INVENT ROUTE
// ============================================================================

void classificationReady()
{
  stopMotor();

  Serial.print(
    F("DBG:LEFT_ID=")
  );
  Serial.println(leftId);

  Serial.print(
    F("DBG:RIGHT_ID=")
  );
  Serial.println(rightId);

  if (leftId == rightId)
  {
    Serial.println(
      F("DBG:SAME FACTORY -> GIAO LEFT, HA LEFT, GIU NGUYEN, HA RIGHT")
    );
  }
  else
  {
    Serial.println(
      F("DBG:DIFFERENT FACTORY -> GIAO LEFT TRUOC, SAU DO RIGHT")
    );
  }

  // ======================================================
  // NOI MODULE TEACH & REPLAY CUA BAN VAO DAY.
  //
  // Khong co route nao duoc tu tao trong file nay.
  //
  // Vi du logic:
  //
  // replayRoute(leftId);
  // haStepperTrai();
  //
  // if (leftId == rightId)
  // {
  //     haStepperPhai();
  // }
  // else
  // {
  //     // dung route DA TEACH de toi factory right
  //     ...
  //     haStepperPhai();
  // }
  //
  // Sau do route DA TEACH quay lai dung ke.
  // ======================================================
}


// ============================================================================
// SETUP
// ============================================================================

void setup()
{
  Serial.begin(115200);

  pinMode(FL_IN1, OUTPUT);
  pinMode(FL_IN2, OUTPUT);
  pinMode(FL_PWM, OUTPUT);

  pinMode(FR_IN1, OUTPUT);
  pinMode(FR_IN2, OUTPUT);
  pinMode(FR_PWM, OUTPUT);

  pinMode(RL_IN1, OUTPUT);
  pinMode(RL_IN2, OUTPUT);
  pinMode(RL_PWM, OUTPUT);

  pinMode(RR_IN1, OUTPUT);
  pinMode(RR_IN2, OUTPUT);
  pinMode(RR_PWM, OUTPUT);

  pinMode(STEP1_STEP, OUTPUT);
  pinMode(STEP1_DIR, OUTPUT);
  pinMode(STEP1_EN, OUTPUT);

  pinMode(STEP2_STEP, OUTPUT);
  pinMode(STEP2_DIR, OUTPUT);
  pinMode(STEP2_EN, OUTPUT);

  enableStepper1(false);
  enableStepper2(false);

  stopMotor();

  mpuOK =
    initMPU6500();

  if (mpuOK)
  {
    Serial.print(
      F("DBG:MPU6500 OK addr=0x")
    );
    Serial.println(
      MPU_ADDR,
      HEX
    );

    calibrateGyroZ();
  }
  else
  {
    Serial.println(
      F("DBG:MPU6500 NOT FOUND")
    );
  }

  state = WAIT_PI;
}


// ============================================================================
// LOOP
// ============================================================================

void loop()
{
  docSerial();

  if (emergencyStop)
  {
    stopMotor();
    state = ERROR_STATE;
  }

  switch (state)
  {
    // ======================================================================
    // WAIT PI
    // ======================================================================
    case WAIT_PI:
    {
      stopMotor();

      if (!piReady)
        break;

      Serial.println(
        F("DBG:PI READY -> START KE 1")
      );

      state =
        K1_LINE_START;

      break;
    }


    // ======================================================================
    // ===== LINE_ON + TIEN 3 GIAY =====
    // ======================================================================
    case K1_LINE_START:
    {
      stopMotor();

      coLine = false;
      lineRunStarted = false;

      resetRelativeYaw();

      Serial.println(
        F("LINE_ON")
      );

      state =
        K1_LINE_RUN;

      break;
    }


    case K1_LINE_RUN:
    {
      // Cho duoc line frame dau tien.
      if (!lineRunStarted)
      {
        if (!lineFresh())
        {
          stopMotor();
          break;
        }

        stateStartMs =
          millis();

        lineRunStarted =
          true;

        Serial.println(
          F("DBG:[TIEN] 3S THEO LINE")
        );
      }

      // ===== TIEN =====
      forwardLineOrYaw();

      if (
        millis() - stateStartMs
        >= START_LINE_RUN_MS
      )
      {
        stopMotor();

        Serial.println(
          F("CAM_OFF")
        );

        state =
          K1_AI2_START;
      }

      break;
    }


    // ======================================================================
    // ===== AI2_ON / CAMERA AI HIEN LEN / QUET CAP HANG =====
    // ======================================================================
    case K1_AI2_START:
    {
      stopMotor();

      leftId = 0;
      rightId = 0;

      gotLeft = false;
      gotRight = false;
      ai2Done = false;

      aiAimFresh = false;
      aiXError = 0;
      aiBoxHeight = 0;

      ai2StartMs =
        millis();

      preLiftDone =
        false;

      Serial.println(
        F("DBG:[AI2] QUET HANG")
      );

      Serial.println(
        F("AI2_ON")
      );

      state =
        K1_AI2_SCAN_AND_PRELIFT;

      break;
    }


    // ======================================================================
    // ===== QUET HANG + 2 STEPPER NANG SO BO =====
    //
    // Stepper pulse loop van doc Serial, nen Pi van gui data AI.
    // ======================================================================
    case K1_AI2_SCAN_AND_PRELIFT:
    {
      if (!preLiftDone)
      {
        Serial.println(
          F("DBG:[STEPPER] 2 CON NANG SO BO")
        );

        moveBothSteppersUp(
          PRE_PICK_UP_STEPS,
          PRE_PICK_UP_STEPS
        );

        preLiftDone =
          true;
      }

      if (
        gotLeft
        &&
        gotRight
        &&
        ai2Done
      )
      {
        Serial.print(
          F("DBG:LEFT=")
        );
        Serial.println(leftId);

        Serial.print(
          F("DBG:RIGHT=")
        );
        Serial.println(rightId);

        resetRelativeYaw();

        state =
          K1_AI_APPROACH;

        break;
      }

      if (
        millis() - ai2StartMs
        >
        AI2_SCAN_TIMEOUT_MS
      )
      {
        Serial.println(
          F("DBG:AI2 SCAN TIMEOUT")
        );

        state =
          ERROR_STATE;
      }

      break;
    }


    // ======================================================================
    // ===== TIEN DUNG CAMERA AI DE CAN TAM + UOC LUONG KHOANG CACH =====
    //
    // Pi GUI:
    //   AIM2:<xerr>,<boxHeight>
    //
    // - xerr dung de sua ngang.
    // - boxHeight dung nhu proxy khoang cach.
    //
    // Dieu kien dung:
    //   boxHeight >= AI_NEAR_BOX_HEIGHT
    //   va abs(xerr) <= AI_CENTER_DEAD_PX
    //   on dinh >= AI_NEAR_STABLE_MS
    // ======================================================================
    case K1_AI_APPROACH:
    {
      unsigned long approachStart =
        millis();

      unsigned long nearStart = 0;

      bool success = false;
      bool aiWasSeen = false;

      Serial.println(
        F("DBG:[TIEN AI] CAN TAM + APPROACH")
      );

      while (
        millis() - approachStart
        <
        AI_APPROACH_TIMEOUT_MS
      )
      {
        docSerial();

        if (emergencyStop)
        {
          stopMotor();
          state = ERROR_STATE;
          break;
        }

        if (aiAimIsFresh())
        {
          aiWasSeen = true;

          // ===== TIEN CHAM + AI CAN TAM =====
          forwardByAI();

          bool centered =
            abs(aiCorrectedXError())
            <=
            AI_CENTER_DEAD_PX;

          bool nearEnough =
            aiBoxHeight
            >=
            AI_NEAR_BOX_HEIGHT;

          if (
            centered
            &&
            nearEnough
          )
          {
            if (nearStart == 0)
              nearStart = millis();

            if (
              millis() - nearStart
              >=
              AI_NEAR_STABLE_MS
            )
            {
              success = true;
              break;
            }
          }
          else
          {
            nearStart = 0;
          }
        }
        else
        {
          // Khong co AIM2 moi -> dung, khong lao vao ke.
          stopMotor();
        }

        delay(2);
      }

      stopMotor();

      if (state == ERROR_STATE)
        break;

      if (!success)
      {
        // KHONG tien mu them 3 giay.
        // Mat AIM / khong dat dieu kien gan + can tam => DUNG AN TOAN.
        Serial.println(
          F("DBG:AI APPROACH FAIL -> STOP, KHONG FALLBACK TIEN MU")
        );

        stopMotor();
        state = ERROR_STATE;
        break;
      }

      state =
        K1_PICK_NUDGE;

      break;
    }


    // ======================================================================
    // ===== STEPPER NHICH LEN DE NHAC HANG =====
    // ======================================================================
    case K1_PICK_NUDGE:
    {
      stopMotor();

      Serial.println(
        F("DBG:[STEPPER] NHICH LEN LAY HANG")
      );

      moveBothSteppersUp(
        PICK_NUDGE_STEPS,
        PICK_NUDGE_STEPS
      );

      delay(180);

      Serial.println(
        F("CAM_OFF")
      );

      state =
        K1_RETREAT_DIAG_RIGHT;

      break;
    }


    // ======================================================================
    // ===== LUI CHEO PHAI 10 GIAY =====
    // ======================================================================
    case K1_RETREAT_DIAG_RIGHT:
    {
      Serial.println(
        F("DBG:[LUI CHEO PHAI] 10S")
      );

      if (!retreatDiagonalRight())
      {
        state =
          ERROR_STATE;

        break;
      }

      state =
        K1_TURN_RIGHT_90;

      break;
    }


    // ======================================================================
    // ===== XOAY PHAI 90 DO =====
    // ======================================================================
    case K1_TURN_RIGHT_90:
    {
      Serial.println(
        F("DBG:[XOAY PHAI] 90 DEG")
      );

      if (!turnRight90())
      {
        Serial.println(
          F("DBG:TURN 90 FAIL")
        );

        state =
          ERROR_STATE;

        break;
      }

      state =
        K1_CLASSIFY_READY;

      break;
    }


    // ======================================================================
    // ===== PHAN LOAI HANG HOA =====
    // ======================================================================
    case K1_CLASSIFY_READY:
    {
      Serial.println(
        F("DBG:[PHAN LOAI]")
      );

      classificationReady();

      state =
        K1_DONE;

      break;
    }


    case K1_DONE:
    {
      stopMotor();
      break;
    }


    case ERROR_STATE:
    {
      stopMotor();

      Serial.println(
        F("CAM_OFF")
      );

      delay(250);
      break;
    }
  }
}