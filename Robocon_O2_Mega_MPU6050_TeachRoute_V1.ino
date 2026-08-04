/*
  ============================================================
  ROBOCON BAC NINH O2 - MEGA 2560 + MECANUM + MPU6050
  V1: GIU THANG MPU + TEACH ROUTE + EEPROM + FACTORY ROUTES
  Khong encoder.

  PHAN CUNG:
  - Mega 2560
  - MPU6050: SDA=20, SCL=21
  - Raspberry Pi/AI (neu dung): Serial1 TX1=18, RX1=19
  - Motor pins giu theo code cu cua ban.

  QUY UOC YAW:
  - Quay PHAI: yaw tang (+)
  - Quay TRAI: yaw giam (-)
  - Neu test thuc te nguoc dau, doi YAW_SIGN tu -1.0f thanh +1.0f.

  LENH SERIAL USB:
  HELP
  YAW
  ZERO
  CALIB
  FWD:2000
  BACK:1000
  LEFT:1000
  RIGHT:1000
  TURN:90
  TURN:-90

  TEACH:<1..16>
  f / b / l / r / q / e
  s
  SHOW
  SAVE
  RUN:<1..16>
  LIST
  STOP

  ROUTE GOI Y:
   1 START -> W1
   2 W1 -> HUB
   3 HUB -> SAMSUNG
   4 SAMSUNG -> HUB
   5 HUB -> HANA
   6 HANA -> HUB
   7 HUB -> AMKOR
   8 AMKOR -> HUB
   9 HUB -> FOXCONN
  10 FOXCONN -> HUB
  11 HUB -> W2
  12 W2 -> HUB
  13 HUB -> W3
  14 W3 -> HUB
  15,16 du phong

  LENH GIAO HANG:
  TARGET:1 = chay HUB -> SAMSUNG
  TARGET:2 = chay HUB -> FOXCONN
  TARGET:3 = chay HUB -> AMKOR
  TARGET:4 = chay HUB -> HANA

  RETURN:1..4 = nha may -> HUB

  LUU Y:
  - Route luu GOC TUONG DOI so voi huong robot luc bat dau TEACH.
  - Khi RUN, robot dung huong hien tai lam moc 0 cua route.
  - Cach nay phu hop chia map thanh cac SEGMENT.
  ============================================================
*/

#include <Wire.h>
#include <EEPROM.h>
#include <math.h>

// ============================================================
// MOTOR PINS - GIU THEO CODE CU
// ============================================================

#define FL_IN1 22
#define FL_IN2 24
#define FL_EN   2

#define FR_IN1 26
#define FR_IN2 28
#define FR_EN   4

#define RL_IN1 30
#define RL_IN2 32
#define RL_EN   6

#define RR_IN1 34
#define RR_IN2 36
#define RR_EN   8

// ============================================================
// MOTOR CALIBRATION
//
// Dang giu ti le tu code cu:
// FL=255, FR=110, RL=110, RR=80.
//
// Neu sau nay robot da can co khi / motor tot hon,
// dua 4 gain gan 1.0 va tune lai.
// ============================================================

float MOTOR_GAIN_FL = 1.00f;
float MOTOR_GAIN_FR = 110.0f / 255.0f;
float MOTOR_GAIN_RL = 110.0f / 255.0f;
float MOTOR_GAIN_RR =  80.0f / 255.0f;

// Toc do mac dinh
const int SPEED_FORWARD = 180;
const int SPEED_BACK    = 165;
const int SPEED_STRAFE  = 165;
const int SPEED_MANUAL_TURN = 80;

// ============================================================
// MPU6050
// ============================================================

#define MPU_ADDR 0x68

// +-250 deg/s
const float GYRO_SCALE = 131.0f;

// MPU thuong: quay phai = gyro Z am.
// Ta muon quay phai = yaw duong => -1.
// Neu test bi nguoc, doi thanh +1.
float YAW_SIGN = -1.0f;

float gyroZBias = 0.0f;
float gyroZDps  = 0.0f;
float yawDeg    = 0.0f;

uint32_t lastMpuUs = 0;

// PID/PD giu huong
float KP_YAW = 2.0f;
float KD_YAW = 0.30f;

const int MAX_YAW_CORRECTION = 45;

// ============================================================
// ROUTE EEPROM
// ============================================================

const uint8_t MAX_ROUTES = 16;
const uint8_t MAX_STEPS  = 24;
const uint8_t EEPROM_VERSION = 2;

const uint8_t MAGIC1 = 0x52; // R
const uint8_t MAGIC2 = 0x32; // 2

typedef struct __attribute__((packed))
{
  char action;              // f,b,l,r,q,e
  uint32_t durationMs;      // translation: thoi gian
  int16_t yawRel10;         // yaw tuong doi x10
  uint8_t speed;            // pwm
} LearnedStep;

const int HEADER_SIZE = 4; // magic1,magic2,version,count
const int SLOT_SIZE = HEADER_SIZE + MAX_STEPS * sizeof(LearnedStep);

LearnedStep routeRam[MAX_STEPS];
uint8_t routeStepCount = 0;

bool teachMode = false;
bool moving = false;

char currentAction = 0;
uint8_t currentTeachRoute = 0;

unsigned long actionStartMs = 0;
float actionStartYaw = 0.0f;
float teachBaseYaw = 0.0f;

// ============================================================
// SERIAL COMMAND BUFFER
// ============================================================

char cmdBuf[64];
uint8_t cmdPos = 0;

char piBuf[64];
uint8_t piPos = 0;

// ============================================================
// BASIC HELPERS
// ============================================================

float wrap180(float a)
{
  while (a > 180.0f)  a -= 360.0f;
  while (a < -180.0f) a += 360.0f;
  return a;
}

float angleError(float target, float current)
{
  return wrap180(target - current);
}

int limitPWM(float value)
{
  if (value > 255.0f) value = 255.0f;
  if (value < -255.0f) value = -255.0f;
  return (int)value;
}

// ============================================================
// MOTOR CORE
// ============================================================

void motor(int in1, int in2, int en, int pwm)
{
  pwm = constrain(pwm, -255, 255);

  if (pwm > 0)
  {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    analogWrite(en, pwm);
  }
  else if (pwm < 0)
  {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    analogWrite(en, -pwm);
  }
  else
  {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    analogWrite(en, 0);
  }
}

void set4(int fl, int fr, int rl, int rr)
{
  motor(FL_IN1, FL_IN2, FL_EN, fl);
  motor(FR_IN1, FR_IN2, FR_EN, fr);
  motor(RL_IN1, RL_IN2, RL_EN, rl);
  motor(RR_IN1, RR_IN2, RR_EN, rr);
}

void dung()
{
  set4(0, 0, 0, 0);
}

// vx: ngang (+ phai)
// vy: doc   (+ tien)
// wz: quay  (+ phai)
void mecanum(float vx, float vy, float wz)
{
  float fl = vy + vx + wz;
  float fr = vy - vx - wz;
  float rl = vy - vx + wz;
  float rr = vy + vx - wz;

  float m = fabs(fl);
  if (fabs(fr) > m) m = fabs(fr);
  if (fabs(rl) > m) m = fabs(rl);
  if (fabs(rr) > m) m = fabs(rr);

  if (m > 255.0f)
  {
    float k = 255.0f / m;
    fl *= k;
    fr *= k;
    rl *= k;
    rr *= k;
  }

  fl *= MOTOR_GAIN_FL;
  fr *= MOTOR_GAIN_FR;
  rl *= MOTOR_GAIN_RL;
  rr *= MOTOR_GAIN_RR;

  set4(
    limitPWM(fl),
    limitPWM(fr),
    limitPWM(rl),
    limitPWM(rr)
  );
}

// ============================================================
// MPU6050 LOW LEVEL
// ============================================================

void mpuWrite(uint8_t reg, uint8_t value)
{
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}

int16_t mpuReadGyroZRaw()
{
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x47); // GYRO_ZOUT_H
  Wire.endTransmission(false);

  Wire.requestFrom(MPU_ADDR, (uint8_t)2, (uint8_t)true);

  if (Wire.available() < 2)
    return 0;

  int16_t value = ((int16_t)Wire.read() << 8) | Wire.read();
  return value;
}

void initMPU6050()
{
  Wire.begin();
  Wire.setClock(400000);

  // Wake up, PLL X gyro
  mpuWrite(0x6B, 0x01);

  // DLPF = 3 ~ giam rung motor tot
  mpuWrite(0x1A, 0x03);

  // 200Hz: 1000/(1+4)
  mpuWrite(0x19, 0x04);

  // gyro +-250 deg/s
  mpuWrite(0x1B, 0x00);

  delay(200);
}

void updateMPU()
{
  uint32_t now = micros();

  if (lastMpuUs == 0)
  {
    lastMpuUs = now;
    return;
  }

  float dt = (now - lastMpuUs) * 0.000001f;
  lastMpuUs = now;

  // Bo qua chu ky qua dai de tranh nhay yaw
  if (dt <= 0.0f || dt > 0.05f)
    return;

  int16_t raw = mpuReadGyroZRaw();

  gyroZDps =
    YAW_SIGN *
    (((float)raw - gyroZBias) / GYRO_SCALE);

  // deadband nho
  if (fabs(gyroZDps) < 0.12f)
    gyroZDps = 0.0f;

  yawDeg += gyroZDps * dt;
  yawDeg = wrap180(yawDeg);
}

void waitWithMPU(unsigned long ms)
{
  unsigned long t0 = millis();

  while (millis() - t0 < ms)
  {
    updateMPU();
    readSerialCommands();
  }
}

void calibrateGyroZ()
{
  dung();

  Serial.println(F("MPU CALIBRATE - GIU ROBOT DUNG YEN"));
  delay(500);

  const int samples = 1000;
  long sum = 0;

  for (int i = 0; i < samples; i++)
  {
    sum += mpuReadGyroZRaw();
    delay(3);
  }

  gyroZBias = (float)sum / samples;

  yawDeg = 0.0f;
  gyroZDps = 0.0f;
  lastMpuUs = micros();

  Serial.print(F("BIAS Z = "));
  Serial.println(gyroZBias, 4);
  Serial.println(F("MPU READY"));
}

// ============================================================
// GIU HUONG
// ============================================================

int getYawCorrection(float targetYaw)
{
  float err = angleError(targetYaw, yawDeg);

  float output =
    KP_YAW * err -
    KD_YAW * gyroZDps;

  output = constrain(
    output,
    (float)-MAX_YAW_CORRECTION,
    (float) MAX_YAW_CORRECTION
  );

  return (int)output;
}

void driveHoldYaw(char action, float targetYaw, int speed)
{
  updateMPU();

  int c = getYawCorrection(targetYaw);

  switch (action)
  {
    case 'f':
      mecanum(0, speed, c);
      break;

    case 'b':
      mecanum(0, -speed, c);
      break;

    case 'l':
      mecanum(-speed, 0, c);
      break;

    case 'r':
      mecanum(speed, 0, c);
      break;

    default:
      dung();
      break;
  }
}

// ============================================================
// QUAY THEO MPU
// ============================================================

bool turnToYaw(float targetYaw)
{
  targetYaw = wrap180(targetYaw);

  unsigned long timeoutStart = millis();
  unsigned long stableStart = 0;

  while (millis() - timeoutStart < 4000)
  {
    updateMPU();

    float err = angleError(targetYaw, yawDeg);
    float ae = fabs(err);

    // Da vao goc + gan dung yen
    if (ae < 1.3f && fabs(gyroZDps) < 5.0f)
    {
      if (stableStart == 0)
        stableStart = millis();

      if (millis() - stableStart >= 100)
      {
        dung();
        waitWithMPU(80);
        return true;
      }
    }
    else
    {
      stableStart = 0;
    }

    int pwm;

    if      (ae > 50.0f) pwm = 120;
    else if (ae > 20.0f) pwm = 90;
    else if (ae > 8.0f)  pwm = 65;
    else if (ae > 3.0f)  pwm = 50;
    else                  pwm = 42;

    if (err > 0.0f)
      mecanum(0, 0, pwm);   // phai
    else
      mecanum(0, 0, -pwm);  // trai

    readSerialCommands();
  }

  dung();

  Serial.println(F("WARN: TURN TIMEOUT"));
  return false;
}

// ============================================================
// TIMED MOVE NHUNG MPU GIU THANG
// ============================================================

void runMoveTimed(char action,
                  uint32_t durationMs,
                  float targetYaw,
                  int speed)
{
  unsigned long t0 = millis();

  while (millis() - t0 < durationMs)
  {
    driveHoldYaw(action, targetYaw, speed);
    readSerialCommands();
  }

  dung();
  waitWithMPU(80);
}

// ============================================================
// ROUTE EEPROM
// ============================================================

bool routeHopLe(int route)
{
  return route >= 1 && route <= MAX_ROUTES;
}

int slotBase(uint8_t route)
{
  return (route - 1) * SLOT_SIZE;
}

float relativeYaw(float absoluteYaw, float baseYaw)
{
  return wrap180(absoluteYaw - baseYaw);
}

bool saveCurrentRoute()
{
  if (!teachMode || !routeHopLe(currentTeachRoute))
  {
    Serial.println(F("ERR: CHUA TEACH ROUTE"));
    return false;
  }

  if (moving)
  {
    Serial.println(F("ERR: BAM s TRUOC"));
    return false;
  }

  if (routeStepCount == 0)
  {
    Serial.println(F("ERR: ROUTE RONG"));
    return false;
  }

  int addr = slotBase(currentTeachRoute);

  if (addr + SLOT_SIZE > EEPROM.length())
  {
    Serial.println(F("ERR: EEPROM FULL"));
    return false;
  }

  EEPROM.update(addr++, MAGIC1);
  EEPROM.update(addr++, MAGIC2);
  EEPROM.update(addr++, EEPROM_VERSION);
  EEPROM.update(addr++, routeStepCount);

  for (uint8_t i = 0; i < routeStepCount; i++)
  {
    EEPROM.put(addr, routeRam[i]);
    addr += sizeof(LearnedStep);
  }

  Serial.print(F("SAVE OK ROUTE "));
  Serial.print(currentTeachRoute);
  Serial.print(F(" / STEPS="));
  Serial.println(routeStepCount);

  teachMode = false;
  return true;
}

bool loadRoute(uint8_t route)
{
  if (!routeHopLe(route))
    return false;

  int addr = slotBase(route);

  uint8_t m1 = EEPROM.read(addr++);
  uint8_t m2 = EEPROM.read(addr++);
  uint8_t ver = EEPROM.read(addr++);
  uint8_t count = EEPROM.read(addr++);

  if (m1 != MAGIC1 ||
      m2 != MAGIC2 ||
      ver != EEPROM_VERSION ||
      count == 0 ||
      count > MAX_STEPS)
  {
    return false;
  }

  routeStepCount = count;

  for (uint8_t i = 0; i < routeStepCount; i++)
  {
    EEPROM.get(addr, routeRam[i]);
    addr += sizeof(LearnedStep);
  }

  return true;
}

// ============================================================
// TEACH
// ============================================================

void startTeach(uint8_t route)
{
  if (!routeHopLe(route))
  {
    Serial.println(F("ERR: ROUTE 1..16"));
    return;
  }

  dung();
  waitWithMPU(100);

  teachMode = true;
  moving = false;

  currentTeachRoute = route;
  routeStepCount = 0;

  // Route dung huong hien tai lam moc 0.
  teachBaseYaw = yawDeg;

  Serial.println();
  Serial.println(F("=============================="));
  Serial.print(F("TEACH ROUTE "));
  Serial.println(route);
  Serial.print(F("BASE YAW = "));
  Serial.println(teachBaseYaw, 2);
  Serial.println(F("f/b/l/r = chay + MPU giu thang"));
  Serial.println(F("q/e = quay trai/phai"));
  Serial.println(F("s = dung + ghi step"));
  Serial.println(F("SHOW = xem route"));
  Serial.println(F("SAVE = luu EEPROM"));
  Serial.println(F("=============================="));
}

int defaultSpeedForAction(char a)
{
  switch (a)
  {
    case 'f': return SPEED_FORWARD;
    case 'b': return SPEED_BACK;
    case 'l':
    case 'r': return SPEED_STRAFE;
    case 'q':
    case 'e': return SPEED_MANUAL_TURN;
  }

  return 0;
}

void startAction(char action)
{
  if (!teachMode)
  {
    Serial.println(F("ERR: GUI TEACH:<route> TRUOC"));
    return;
  }

  if (moving)
  {
    Serial.println(F("ERR: BAM s TRUOC"));
    return;
  }

  updateMPU();

  currentAction = action;
  actionStartMs = millis();
  actionStartYaw = yawDeg;

  moving = true;

  Serial.print(F("MOVE "));
  Serial.print(action);
  Serial.print(F(" START_YAW="));
  Serial.println(actionStartYaw, 2);
}

void updateTeachMovement()
{
  if (!teachMode || !moving)
    return;

  switch (currentAction)
  {
    case 'f':
    case 'b':
    case 'l':
    case 'r':
      driveHoldYaw(
        currentAction,
        actionStartYaw,
        defaultSpeedForAction(currentAction)
      );
      break;

    case 'q':
      mecanum(0, 0, -SPEED_MANUAL_TURN);
      break;

    case 'e':
      mecanum(0, 0, SPEED_MANUAL_TURN);
      break;
  }
}

void stopAndRecord()
{
  if (!moving)
  {
    dung();
    Serial.println(F("STOP"));
    return;
  }

  uint32_t duration = millis() - actionStartMs;

  dung();

  // Lay goc SAU khi robot het quan tinh.
  waitWithMPU(120);

  moving = false;

  if (routeStepCount >= MAX_STEPS)
  {
    Serial.println(F("ERR: ROUTE FULL"));
    currentAction = 0;
    return;
  }

  LearnedStep &st = routeRam[routeStepCount];

  st.action = currentAction;
  st.durationMs = duration;
  st.speed = (uint8_t)defaultSpeedForAction(currentAction);

  float saveYaw;

  if (currentAction == 'q' || currentAction == 'e')
  {
    // Quay: luu GOC KET THUC
    saveYaw = yawDeg;
  }
  else
  {
    // Tien/lui/ngang: luu GOC CAN GIU
    saveYaw = actionStartYaw;
  }

  float rel = relativeYaw(saveYaw, teachBaseYaw);
  st.yawRel10 = (int16_t)round(rel * 10.0f);

  routeStepCount++;

  Serial.print(F("REC "));
  Serial.print(routeStepCount);
  Serial.print(F(": "));
  Serial.print(st.action);

  if (st.action == 'q' || st.action == 'e')
  {
    Serial.print(F(" TARGET_REL="));
    Serial.print(st.yawRel10 / 10.0f);
    Serial.println(F(" deg"));
  }
  else
  {
    Serial.print(F(" TIME="));
    Serial.print(st.durationMs);
    Serial.print(F("ms REL_YAW="));
    Serial.print(st.yawRel10 / 10.0f);
    Serial.println(F(" deg"));
  }

  currentAction = 0;
}

void showRoute()
{
  Serial.println(F("========= ROUTE RAM ========="));

  Serial.print(F("ROUTE="));
  Serial.println(currentTeachRoute);

  for (uint8_t i = 0; i < routeStepCount; i++)
  {
    LearnedStep &st = routeRam[i];

    Serial.print(i + 1);
    Serial.print(F(". "));
    Serial.print(st.action);

    Serial.print(F("  t="));
    Serial.print(st.durationMs);

    Serial.print(F("  yawRel="));
    Serial.print(st.yawRel10 / 10.0f);

    Serial.print(F("  pwm="));
    Serial.println(st.speed);
  }

  Serial.println(F("============================="));
}

// ============================================================
// RUN ROUTE
// ============================================================

bool runRoute(uint8_t route)
{
  dung();

  teachMode = false;
  moving = false;

  if (!loadRoute(route))
  {
    Serial.print(F("ERR: ROUTE "));
    Serial.print(route);
    Serial.println(F(" CHUA SAVE"));
    return false;
  }

  waitWithMPU(100);

  // Moc 0 moi cho segment nay
  float runBaseYaw = yawDeg;

  Serial.println();
  Serial.println(F("=============================="));
  Serial.print(F("RUN ROUTE "));
  Serial.println(route);
  Serial.print(F("RUN BASE YAW="));
  Serial.println(runBaseYaw, 2);
  Serial.print(F("STEPS="));
  Serial.println(routeStepCount);
  Serial.println(F("=============================="));

  for (uint8_t i = 0; i < routeStepCount; i++)
  {
    LearnedStep &st = routeRam[i];

    float targetYaw =
      wrap180(runBaseYaw + st.yawRel10 / 10.0f);

    Serial.print(F("STEP "));
    Serial.print(i + 1);
    Serial.print(F("/"));
    Serial.print(routeStepCount);
    Serial.print(F(" "));
    Serial.print(st.action);
    Serial.print(F(" TARGET_YAW="));
    Serial.println(targetYaw, 2);

    if (st.action == 'f' ||
        st.action == 'b' ||
        st.action == 'l' ||
        st.action == 'r')
    {
      runMoveTimed(
        st.action,
        st.durationMs,
        targetYaw,
        st.speed
      );
    }
    else if (st.action == 'q' ||
             st.action == 'e')
    {
      if (!turnToYaw(targetYaw))
      {
        Serial.println(F("RUN ABORT: TURN FAIL"));
        dung();
        return false;
      }
    }
  }

  dung();

  Serial.print(F("ROUTE DONE: "));
  Serial.println(route);

  return true;
}

void listRoutes()
{
  Serial.println(F("====== EEPROM ROUTES ======"));

  for (uint8_t route = 1; route <= MAX_ROUTES; route++)
  {
    int addr = slotBase(route);

    uint8_t m1 = EEPROM.read(addr);
    uint8_t m2 = EEPROM.read(addr + 1);
    uint8_t ver = EEPROM.read(addr + 2);
    uint8_t count = EEPROM.read(addr + 3);

    Serial.print(route);
    Serial.print(F(": "));

    if (m1 == MAGIC1 &&
        m2 == MAGIC2 &&
        ver == EEPROM_VERSION &&
        count > 0 &&
        count <= MAX_STEPS)
    {
      Serial.print(F("SAVED "));
      Serial.print(count);
      Serial.println(F(" steps"));
    }
    else
    {
      Serial.println(F("EMPTY"));
    }
  }

  Serial.println(F("==========================="));
}

// ============================================================
// FACTORY ROUTE MAPPING
// ============================================================

uint8_t routeHubToFactory(uint8_t cargoId)
{
  switch (cargoId)
  {
    case 1: return 3; // Samsung
    case 2: return 9; // Foxconn
    case 3: return 7; // Amkor
    case 4: return 5; // Hana
  }

  return 0;
}

uint8_t routeFactoryToHub(uint8_t cargoId)
{
  switch (cargoId)
  {
    case 1: return 4;  // Samsung -> HUB
    case 2: return 10; // Foxconn -> HUB
    case 3: return 8;  // Amkor -> HUB
    case 4: return 6;  // Hana -> HUB
  }

  return 0;
}

const char* cargoName(uint8_t id)
{
  switch (id)
  {
    case 1: return "SAMSUNG";
    case 2: return "FOXCONN";
    case 3: return "AMKOR";
    case 4: return "HANA";
  }

  return "UNKNOWN";
}

void goFactory(uint8_t cargoId)
{
  uint8_t route = routeHubToFactory(cargoId);

  if (route == 0)
  {
    Serial.println(F("ERR: CARGO ID 1..4"));
    return;
  }

  Serial.print(F("CARGO "));
  Serial.print(cargoId);
  Serial.print(F(" -> "));
  Serial.println(cargoName(cargoId));

  if (runRoute(route))
  {
    Serial.print(F("AT_FACTORY:"));
    Serial.println(cargoId);

    Serial1.print(F("AT_FACTORY:"));
    Serial1.println(cargoId);
  }
}

void returnFromFactory(uint8_t cargoId)
{
  uint8_t route = routeFactoryToHub(cargoId);

  if (route == 0)
  {
    Serial.println(F("ERR: CARGO ID 1..4"));
    return;
  }

  if (runRoute(route))
  {
    Serial.print(F("AT_HUB FROM "));
    Serial.println(cargoName(cargoId));

    Serial1.print(F("AT_HUB:"));
    Serial1.println(cargoId);
  }
}

// ============================================================
// TEST COMMANDS
// ============================================================

void testTimed(char action, unsigned long ms)
{
  dung();
  waitWithMPU(100);

  float target = yawDeg;

  Serial.print(F("TEST "));
  Serial.print(action);
  Serial.print(F(" "));
  Serial.print(ms);
  Serial.print(F("ms TARGET_YAW="));
  Serial.println(target, 2);

  runMoveTimed(
    action,
    ms,
    target,
    defaultSpeedForAction(action)
  );
}

// ============================================================
// COMMAND PARSER
// ============================================================

void printHelp()
{
  Serial.println();
  Serial.println(F("========== LENH =========="));
  Serial.println(F("YAW             : xem yaw"));
  Serial.println(F("ZERO            : dat yaw hien tai = 0"));
  Serial.println(F("CALIB           : calibrate gyro, robot phai dung yen"));
  Serial.println();
  Serial.println(F("FWD:2000        : tien 2000ms, MPU giu thang"));
  Serial.println(F("BACK:1000       : lui"));
  Serial.println(F("LEFT:1000       : ngang trai"));
  Serial.println(F("RIGHT:1000      : ngang phai"));
  Serial.println(F("TURN:90         : quay phai toi +90 deg"));
  Serial.println(F("TURN:-90        : quay trai toi -90 deg"));
  Serial.println();
  Serial.println(F("TEACH:1..16"));
  Serial.println(F("  f/b/l/r       : chay giu huong"));
  Serial.println(F("  q/e           : quay trai/phai"));
  Serial.println(F("  s             : dung + ghi step"));
  Serial.println(F("  SHOW          : xem route RAM"));
  Serial.println(F("  SAVE          : save EEPROM"));
  Serial.println();
  Serial.println(F("RUN:1..16       : replay route"));
  Serial.println(F("LIST            : xem route da save"));
  Serial.println();
  Serial.println(F("TARGET:1        : HUB -> Samsung"));
  Serial.println(F("TARGET:2        : HUB -> Foxconn"));
  Serial.println(F("TARGET:3        : HUB -> Amkor"));
  Serial.println(F("TARGET:4        : HUB -> Hana"));
  Serial.println(F("RETURN:1..4     : factory -> HUB"));
  Serial.println();
  Serial.println(F("STOP"));
  Serial.println(F("=========================="));
}

void processCommand(const char *cmd)
{
  if (strlen(cmd) == 1)
  {
    char c = cmd[0];

    if (c == 'f' ||
        c == 'b' ||
        c == 'l' ||
        c == 'r' ||
        c == 'q' ||
        c == 'e')
    {
      startAction(c);
      return;
    }

    if (c == 's')
    {
      stopAndRecord();
      return;
    }
  }

  if (strcmp(cmd, "HELP") == 0)
  {
    printHelp();
    return;
  }

  if (strcmp(cmd, "YAW") == 0)
  {
    updateMPU();

    Serial.print(F("YAW="));
    Serial.print(yawDeg, 2);
    Serial.print(F("  GZ="));
    Serial.println(gyroZDps, 2);
    return;
  }

  if (strcmp(cmd, "ZERO") == 0)
  {
    updateMPU();
    yawDeg = 0.0f;
    lastMpuUs = micros();

    Serial.println(F("YAW ZERO OK"));
    return;
  }

  if (strcmp(cmd, "CALIB") == 0)
  {
    calibrateGyroZ();
    return;
  }

  if (strcmp(cmd, "SHOW") == 0)
  {
    showRoute();
    return;
  }

  if (strcmp(cmd, "SAVE") == 0)
  {
    saveCurrentRoute();
    return;
  }

  if (strcmp(cmd, "LIST") == 0)
  {
    listRoutes();
    return;
  }

  if (strcmp(cmd, "STOP") == 0)
  {
    dung();
    moving = false;
    Serial.println(F("STOP"));
    return;
  }

  if (strncmp(cmd, "TEACH:", 6) == 0)
  {
    int route = atoi(cmd + 6);
    startTeach((uint8_t)route);
    return;
  }

  if (strncmp(cmd, "RUN:", 4) == 0)
  {
    int route = atoi(cmd + 4);

    if (!routeHopLe(route))
    {
      Serial.println(F("ERR: ROUTE 1..16"));
      return;
    }

    runRoute((uint8_t)route);
    return;
  }

  if (strncmp(cmd, "FWD:", 4) == 0)
  {
    unsigned long ms = strtoul(cmd + 4, NULL, 10);
    testTimed('f', ms);
    return;
  }

  if (strncmp(cmd, "BACK:", 5) == 0)
  {
    unsigned long ms = strtoul(cmd + 5, NULL, 10);
    testTimed('b', ms);
    return;
  }

  if (strncmp(cmd, "LEFT:", 5) == 0)
  {
    unsigned long ms = strtoul(cmd + 5, NULL, 10);
    testTimed('l', ms);
    return;
  }

  if (strncmp(cmd, "RIGHT:", 6) == 0)
  {
    unsigned long ms = strtoul(cmd + 6, NULL, 10);
    testTimed('r', ms);
    return;
  }

  if (strncmp(cmd, "TURN:", 5) == 0)
  {
    float target = atof(cmd + 5);
    turnToYaw(target);
    return;
  }

  if (strncmp(cmd, "TARGET:", 7) == 0)
  {
    int id = atoi(cmd + 7);
    goFactory((uint8_t)id);
    return;
  }

  if (strncmp(cmd, "RETURN:", 7) == 0)
  {
    int id = atoi(cmd + 7);
    returnFromFactory((uint8_t)id);
    return;
  }

  Serial.print(F("UNKNOWN: "));
  Serial.println(cmd);
}

// ============================================================
// SERIAL READ
// ============================================================

void readSerialCommands()
{
  while (Serial.available())
  {
    char c = Serial.read();

    if (c == '\r')
      continue;

    if (c == '\n')
    {
      cmdBuf[cmdPos] = '\0';

      if (cmdPos > 0)
        processCommand(cmdBuf);

      cmdPos = 0;
    }
    else if (cmdPos < sizeof(cmdBuf) - 1)
    {
      cmdBuf[cmdPos++] = c;
    }
    else
    {
      cmdPos = 0;
    }
  }
}

// Pi co the gui:
// TARGET:1
// TARGET:2
// TARGET:3
// TARGET:4
//
// Hoac RETURN:1..4
void readPiCommands()
{
  while (Serial1.available())
  {
    char c = Serial1.read();

    if (c == '\r')
      continue;

    if (c == '\n')
    {
      piBuf[piPos] = '\0';

      if (piPos > 0)
      {
        // Echo de debug
        Serial.print(F("PI> "));
        Serial.println(piBuf);

        processCommand(piBuf);
      }

      piPos = 0;
    }
    else if (piPos < sizeof(piBuf) - 1)
    {
      piBuf[piPos++] = c;
    }
    else
    {
      piPos = 0;
    }
  }
}

// ============================================================
// SETUP / LOOP
// ============================================================

void setup()
{
  Serial.begin(115200);
  Serial1.begin(115200);

  pinMode(FL_IN1, OUTPUT);
  pinMode(FL_IN2, OUTPUT);
  pinMode(FL_EN, OUTPUT);

  pinMode(FR_IN1, OUTPUT);
  pinMode(FR_IN2, OUTPUT);
  pinMode(FR_EN, OUTPUT);

  pinMode(RL_IN1, OUTPUT);
  pinMode(RL_IN2, OUTPUT);
  pinMode(RL_EN, OUTPUT);

  pinMode(RR_IN1, OUTPUT);
  pinMode(RR_IN2, OUTPUT);
  pinMode(RR_EN, OUTPUT);

  dung();

  initMPU6050();
  calibrateGyroZ();

  Serial.println();
  Serial.println(F("ROBOCON O2 V1 READY"));
  Serial.print(F("EEPROM SLOT SIZE="));
  Serial.println(SLOT_SIZE);
  Serial.print(F("TOTAL USED="));
  Serial.println(SLOT_SIZE * MAX_ROUTES);
  Serial.println(F("GUI HELP"));
}

void loop()
{
  updateMPU();

  updateTeachMovement();

  readSerialCommands();
  readPiCommands();
}
