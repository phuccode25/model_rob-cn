// ============================================================
// ROBOCON UNO - CAMERA DUOI DO LINE + CAMERA TREN AI
// Chay cung ren.py tren Raspberry Pi
// ============================================================

#define FL_IN1 A0
#define FL_IN2 A1
#define FL_EN  3

#define RL_IN1 A2
#define RL_IN2 A3
#define RL_EN  5

#define FR_IN1 2
#define FR_IN2 4
#define FR_EN  6

#define RR_IN1 7
#define RR_IN2 8
#define RR_EN  9

// PWM co ban - can tiep tuc tune tren robot
int BASE_FL = 200;
int BASE_FR = 70;
int BASE_RL = 70;
int BASE_RR = 60;

// LINE
int errorLine = 0;
bool coLine = false;
unsigned long lastLine = 0;

const int DEAD_LINE = 20;
const int PIXEL_PER_PWM_LINE = 5;
const int MAX_CORR_LINE = 30;
const unsigned long LINE_TIMEOUT = 400;

// YOLO
int errorHang = 0;
bool coHang = false;
bool hangReady = false;
char loaiHang[12] = "";
unsigned long lastHang = 0;

bool piReady = false;
bool piStop = false;

char rxBuf[64];
uint8_t rxLen = 0;

void motor1(int in1, int in2, int en, int pwm)
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
  motor1(FL_IN1, FL_IN2, FL_EN, fl);
  motor1(FR_IN1, FR_IN2, FR_EN, fr);
  motor1(RL_IN1, RL_IN2, RL_EN, rl);
  motor1(RR_IN1, RR_IN2, RR_EN, rr);
}

void dungMotor()
{
  set4(0, 0, 0, 0);
}

void xuLyLenhPi(const char *s)
{
  if (strcmp(s, "PI_READY") == 0)
  {
    piReady = true;
    return;
  }

  if (strncmp(s, "XLINE:", 6) == 0)
  {
    errorLine = atoi(s + 6);
    coLine = true;
    lastLine = millis();
    return;
  }

  if (strcmp(s, "LOST_LINE") == 0)
  {
    coLine = false;
    return;
  }

  if (strncmp(s, "XHANG:", 6) == 0)
  {
    errorHang = atoi(s + 6);
    coHang = true;
    lastHang = millis();
    return;
  }

  if (strcmp(s, "LOST_HANG") == 0)
  {
    coHang = false;
    hangReady = false;
    return;
  }

  if (strncmp(s, "AI_CLASS:", 9) == 0)
  {
    strncpy(loaiHang, s + 9, sizeof(loaiHang) - 1);
    loaiHang[sizeof(loaiHang) - 1] = '\0';
    return;
  }

  if (strcmp(s, "HANG_READY") == 0)
  {
    hangReady = true;
    return;
  }

  if (strcmp(s, "PI_STOP") == 0)
  {
    piStop = true;
    dungMotor();
    return;
  }
}

void docPi()
{
  while (Serial.available())
  {
    char c = Serial.read();

    if (c == '\n')
    {
      rxBuf[rxLen] = '\0';

      if (rxLen > 0)
        xuLyLenhPi(rxBuf);

      rxLen = 0;
    }
    else if (c != '\r')
    {
      if (rxLen < sizeof(rxBuf) - 1)
        rxBuf[rxLen++] = c;
      else
        rxLen = 0;
    }
  }
}

void batLine()
{
  coLine = false;
  Serial.println("LINE_ON");
  Serial.flush();
}

void batYOLO()
{
  coHang = false;
  hangReady = false;
  loaiHang[0] = '\0';

  Serial.println("YOLO_ON");
  Serial.flush();
}

void tatCamera()
{
  Serial.println("CAM_OFF");
  Serial.flush();
}

void bamLine(unsigned long thoiGian)
{
  batLine();

  unsigned long startWait = millis();

  while (!coLine && millis() - startWait < 2500)
  {
    docPi();
    dungMotor();
  }

  if (!coLine)
  {
    Serial.println("DBG:NO_LINE");
    tatCamera();
    return;
  }

  Serial.println("DBG:LINE_FOUND");

  unsigned long start = millis();

  while (millis() - start < thoiGian)
  {
    docPi();

    if (piStop)
    {
      dungMotor();
      return;
    }

    unsigned long now = millis();

    bool lineOK =
      coLine &&
      (now - lastLine <= LINE_TIMEOUT);

    if (!lineOK)
    {
      dungMotor();
      continue;
    }

    int corr = 0;

    if (abs(errorLine) > DEAD_LINE)
    {
      corr = errorLine / PIXEL_PER_PWM_LINE;
      corr = constrain(corr, -MAX_CORR_LINE, MAX_CORR_LINE);
    }

    int fl = constrain(BASE_FL + corr, 0, 255);
    int fr = constrain(BASE_FR - corr, 0, 255);
    int rl = constrain(BASE_RL - corr, 0, 255);
    int rr = constrain(BASE_RR + corr, 0, 255);

    set4(fl, fr, rl, rr);
  }

  dungMotor();
  tatCamera();

  Serial.println("DBG:LINE_DONE");
}

bool nhanDienHang(unsigned long timeout)
{
  dungMotor();
  batYOLO();

  unsigned long start = millis();

  while (millis() - start < timeout)
  {
    docPi();

    if (piStop)
    {
      dungMotor();
      return false;
    }

    if (loaiHang[0] != '\0' && coHang)
    {
      Serial.print("RESULT_CLASS:");
      Serial.println(loaiHang);

      tatCamera();
      return true;
    }
  }

  Serial.println("RESULT_CLASS:none");
  tatCamera();
  return false;
}

void setup()
{
  Serial.begin(115200);

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

  dungMotor();

  while (!piReady)
  {
    docPi();
    dungMotor();

    Serial.println("HELLO_UNO");
    Serial.flush();

    delay(300);
  }

  Serial.println("DBG:PI_READY_OK");

  // CAMERA DUOI: do line + dieu khien robot 3 giay
  bamLine(3000);

  delay(1000);

  // CAMERA TREN: AI nhan dien kien trong 5 giay
  nhanDienHang(5000);

  dungMotor();
}

void loop()
{
  docPi();
  dungMotor();
}
