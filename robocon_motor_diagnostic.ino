// ============================================================
// ROBOCON UNO - MOTOR DIAGNOSTIC
// KHONG CAN RASPBERRY PI, KHONG CAMERA, KHONG YOLO
//
// MUC DICH:
//   Test tung motor va cac vector Mecanum.
//   Sau khi upload, robot tu chay 1 lan.
//
// CAN KE ROBOT LEN DE BANH KHONG CHAM DAT KHI TEST TUNG MOTOR.
// ============================================================

// FL - truoc trai
#define FL_IN1 A0
#define FL_IN2 A1
#define FL_EN  3

// RL - sau trai
#define RL_IN1 A2
#define RL_IN2 A3
#define RL_EN  5

// FR - truoc phai
#define FR_IN1 2
#define FR_IN2 4
#define FR_EN  6

// RR - sau phai
#define RR_IN1 7
#define RR_IN2 8
#define RR_EN  9

// PWM theo robot cua ban
int TIEN_FL = 200;
int TIEN_FR = 70;
int TIEN_RL = 70;
int TIEN_RR = 60;

int PHAI_FL = 255;
int PHAI_FR = 150;
int PHAI_RL = 110;
int PHAI_RR = 120;

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

void dung()
{
  motor1(FL_IN1, FL_IN2, FL_EN, 0);
  motor1(FR_IN1, FR_IN2, FR_EN, 0);
  motor1(RL_IN1, RL_IN2, RL_EN, 0);
  motor1(RR_IN1, RR_IN2, RR_EN, 0);
}

void all(int fl, int fr, int rl, int rr)
{
  motor1(FL_IN1, FL_IN2, FL_EN, fl);
  motor1(FR_IN1, FR_IN2, FR_EN, fr);
  motor1(RL_IN1, RL_IN2, RL_EN, rl);
  motor1(RR_IN1, RR_IN2, RR_EN, rr);
}

void testMotor(const char* name, int in1, int in2, int en, int pwm)
{
  Serial.print("TEST ");
  Serial.println(name);

  motor1(in1, in2, en, pwm);
  delay(2000);

  motor1(in1, in2, en, 0);
  delay(1000);
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

  dung();
  delay(2000);

  Serial.println("=== MOTOR TEST START ===");

  // 1. Test tung motor
  testMotor("FL", FL_IN1, FL_IN2, FL_EN, 180);
  testMotor("FR", FR_IN1, FR_IN2, FR_EN, 180);
  testMotor("RL", RL_IN1, RL_IN2, RL_EN, 180);
  testMotor("RR", RR_IN1, RR_IN2, RR_EN, 180);

  // 2. Test 4 motor tien
  Serial.println("TEST ALL FORWARD");
  all(TIEN_FL, TIEN_FR, TIEN_RL, TIEN_RR);
  delay(3000);
  dung();
  delay(1500);

  // 3. Test 4 motor lui
  Serial.println("TEST ALL BACKWARD");
  all(-TIEN_FL, -TIEN_FR, -TIEN_RL, -TIEN_RR);
  delay(3000);
  dung();
  delay(1500);

  // 4. Test ngang phai
  Serial.println("TEST STRAFE RIGHT");
  all(PHAI_FL, -PHAI_FR, -PHAI_RL, PHAI_RR);
  delay(3000);
  dung();
  delay(1500);

  // 5. Test ngang trai
  Serial.println("TEST STRAFE LEFT");
  all(-PHAI_FL, PHAI_FR, PHAI_RL, -PHAI_RR);
  delay(3000);
  dung();

  Serial.println("=== MOTOR TEST DONE ===");
}

void loop()
{
  dung();
}
