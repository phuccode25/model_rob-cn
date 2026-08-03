// ============================================================
// ROBOCON UNO SIMPLE V2 - REN.PY + ROUTE OUTPUT
// Compatible with ren.py on Raspberry Pi 5
//
// MUC TIEU BAN NAY:
//   1) Cho Raspberry Pi gui PI_READY
//   2) Do LINE bang camera duoi trong 3 giay
//   3) Dung 1 giay
//   4) Bat YOLO camera tren, nhan class
//   5) Tien 2 giay + YOLO can tam trai/phai
//   6) Tat camera
//   7) Ve moc M0
//   8) Chay map mu den output dung voi class
//
// MAP:
//   al   -> Samsung
//   qr   -> Foxconn
//   yt   -> Amkor
//   chip -> Hana Micron
//
// UNO -> PI:
//   HELLO_UNO
//   LINE_ON
//   YOLO_ON
//   TARGET,<class>,all
//   CAM_OFF
//
// PI -> UNO:
//   PI_READY
//   XLINE:<pixel>
//   LOST_LINE
//   XHANG:<pixel>
//   LOST_HANG
//   AI_CLASS:al|chip|qr|yt
//   HANG_READY
//   PI_STOP
// ============================================================


// ============================================================
// 1. CHAN MOTOR
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


// ============================================================
// 2. PWM DA CAN CHINH CUA ROBOT
// ============================================================

// Tien
int TIEN_FL = 200;
int TIEN_FR = 70;
int TIEN_RL = 70;
int TIEN_RR = 60;

// Lui
int LUI_FL = 200;
int LUI_FR = 70;
int LUI_RL = 70;
int LUI_RR = 60;

// Ngang phai
int PHAI_FL = 255;
int PHAI_FR = 150;
int PHAI_RL = 110;
int PHAI_RR = 120;

// Ngang trai
int TRAI_FL = 255;
int TRAI_FR = 150;
int TRAI_RL = 110;
int TRAI_RR = 120;

// Quay tai cho
int QUAY_FL = 120;
int QUAY_FR = 120;
int QUAY_RL = 120;
int QUAY_RR = 120;

// Toc do khi do line
int LINE_FL = 200;
int LINE_FR = 70;
int LINE_RL = 70;
int LINE_RR = 60;

// Toc do khi tien + YOLO can hang
int HANG_FL = 200;
int HANG_FR = 70;
int HANG_RL = 70;
int HANG_RR = 60;


// ============================================================
// 2B. MAP MU TU KHU KIEN -> OUTPUT
//
// Quy uoc M0:
//   - Robot da ve FACTORY LANE.
//   - Robot cung hang voi FOXCONN.
//   - Mui robot huong vao cac nha may.
//
// Mapping:
//   al   -> Samsung
//   qr   -> Foxconn
//   yt   -> Amkor
//   chip -> Hana Micron
//
// Vi tri doc lane lay tu map cu, CAN TUNE TREN SAN THAT.
// ============================================================

long POS_FOXCONN_MM = 0;
long POS_AMKOR_MM   = 334;
long POS_HANA_MM    = 1000;
long POS_SAMSUNG_MM = 1334;

// Tu factory lane tien vao tam output.
long VAO_OUTPUT_MM = 375;

// Toc do do THUC TE cua robot.
// Day la gia tri khoi dau tu ban map cu; can do lai.
float TIEN_MM_S  = 500.0;
float NGANG_MM_S = 380.0;

// ------------------------------------------------------------
// TU KHU KIEN VE M0
//
// Ta CHUA co so do thoi gian/khoang cach thuc te cua doan nay,
// nen tach tung tham so de test tung buoc.
//
// 0 = bo qua buoc do.
// ------------------------------------------------------------

// Lui ra khoi vi tri kien.
long VE_M0_LUI_MM = 0;

// Quay 180 do bang thoi gian.
// Phai test thuc te de dien vao.
unsigned long VE_M0_QUAY_180_MS = 0;

// Tien tu khu kien den factory lane.
long VE_M0_TIEN_DEN_LANE_MM = 0;

// Dich doc factory lane ve hang Foxconn = M0.
// > 0: ngang trai
// < 0: ngang phai
long VE_M0_DICH_DOC_MM = 0;

// Khoa map de test tung tang.
// 0 = chi nhan dien, khong ve M0
// 1 = lui
// 2 = lui + quay 180
// 3 = lui + quay + tien den factory lane
// 4 = full ve M0
uint8_t M0_TEST_STAGE = 0;

// CHI DOI true sau khi M0_TEST_STAGE=4 da chay dung lap lai.
bool CHO_PHEP_CHAY_OUTPUT = false;

// Vi tri hien tai doc factory lane.
// Sau khi ve M0 thanh cong se dat = 0.
long lanePosMm = 0;


// ============================================================
// 3. THAM SO SUA LINE / YOLO
// ============================================================

const int DEAD_LINE = 25;
const int PIXEL_MOI_PWM_LINE = 5;
const int MAX_SUA_LINE = 25;
const unsigned long TIMEOUT_LINE = 500;

const int DEAD_HANG = 25;
const int PIXEL_MOI_PWM_HANG = 5;
const int MAX_SUA_HANG = 25;
const unsigned long TIMEOUT_HANG = 700;


// ============================================================
// 4. DU LIEU NHAN TU PI
// ============================================================

bool piReady = false;
bool piStop = false;

bool coLine = false;
int errorLine = 0;
unsigned long lanNhanLine = 0;

bool coHang = false;
bool hangReady = false;
int errorHang = 0;
unsigned long lanNhanHang = 0;

char loaiHang[12] = "";
char rxBuffer[64];
uint8_t rxLength = 0;


// ============================================================
// 5. MOTOR CO BAN
// ============================================================

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


void dungMotor()
{
  motor1(FL_IN1, FL_IN2, FL_EN, 0);
  motor1(FR_IN1, FR_IN2, FR_EN, 0);
  motor1(RL_IN1, RL_IN2, RL_EN, 0);
  motor1(RR_IN1, RR_IN2, RR_EN, 0);
}


void dat4Motor(int FL, int FR, int RL, int RR)
{
  motor1(FL_IN1, FL_IN2, FL_EN, FL);
  motor1(FR_IN1, FR_IN2, FR_EN, FR);
  motor1(RL_IN1, RL_IN2, RL_EN, RL);
  motor1(RR_IN1, RR_IN2, RR_EN, RR);
}


// ============================================================
// 6. XU LY SERIAL TU PI
// ============================================================

void xuLyLenhPi(const char *lenh)
{
  if (strcmp(lenh, "PI_READY") == 0)
  {
    piReady = true;
    return;
  }

  if (strncmp(lenh, "XLINE:", 6) == 0)
  {
    errorLine = atoi(lenh + 6);
    coLine = true;
    lanNhanLine = millis();
    return;
  }

  if (strcmp(lenh, "LOST_LINE") == 0)
  {
    coLine = false;
    return;
  }

  if (strncmp(lenh, "XHANG:", 6) == 0)
  {
    errorHang = atoi(lenh + 6);
    coHang = true;
    lanNhanHang = millis();
    return;
  }

  if (strcmp(lenh, "LOST_HANG") == 0)
  {
    coHang = false;
    hangReady = false;
    return;
  }

  if (strncmp(lenh, "AI_CLASS:", 9) == 0)
  {
    strncpy(loaiHang, lenh + 9, sizeof(loaiHang) - 1);
    loaiHang[sizeof(loaiHang) - 1] = '\0';
    return;
  }

  if (strcmp(lenh, "HANG_READY") == 0)
  {
    hangReady = true;
    return;
  }

  if (strcmp(lenh, "PI_STOP") == 0)
  {
    piStop = true;
    dungMotor();
    return;
  }
}


void docPi()
{
  while (Serial.available() > 0)
  {
    char c = Serial.read();

    if (c == '\n')
    {
      rxBuffer[rxLength] = '\0';

      if (rxLength > 0)
      {
        xuLyLenhPi(rxBuffer);
      }

      rxLength = 0;
    }
    else if (c != '\r')
    {
      if (rxLength < sizeof(rxBuffer) - 1)
      {
        rxBuffer[rxLength++] = c;
      }
      else
      {
        rxLength = 0;
      }
    }
  }
}


// ============================================================
// 7. CHO NHUNG VAN DOC SERIAL
// ============================================================

void choVaDocPi(unsigned long ms)
{
  unsigned long start = millis();

  while (millis() - start < ms)
  {
    docPi();

    if (piStop)
    {
      dungMotor();
      return;
    }

    delay(1);
  }
}


// ============================================================
// 8. LENH DI CHUYEN CO BAN
// DE SAU NAY CODE MAP MU
// ============================================================

void tien(unsigned long ms)
{
  if (piStop) return;

  dat4Motor(TIEN_FL, TIEN_FR, TIEN_RL, TIEN_RR);
  choVaDocPi(ms);
  dungMotor();
}


void lui(unsigned long ms)
{
  if (piStop) return;

  dat4Motor(-LUI_FL, -LUI_FR, -LUI_RL, -LUI_RR);
  choVaDocPi(ms);
  dungMotor();
}


void ngangPhai(unsigned long ms)
{
  if (piStop) return;

  dat4Motor(
    PHAI_FL,
    -PHAI_FR,
    -PHAI_RL,
    PHAI_RR
  );

  choVaDocPi(ms);
  dungMotor();
}


void ngangTrai(unsigned long ms)
{
  if (piStop) return;

  dat4Motor(
    -TRAI_FL,
    TRAI_FR,
    TRAI_RL,
    -TRAI_RR
  );

  choVaDocPi(ms);
  dungMotor();
}


void quayPhai(unsigned long ms)
{
  if (piStop) return;

  dat4Motor(
    QUAY_FL,
    -QUAY_FR,
    QUAY_RL,
    -QUAY_RR
  );

  choVaDocPi(ms);
  dungMotor();
}


void quayTrai(unsigned long ms)
{
  if (piStop) return;

  dat4Motor(
    -QUAY_FL,
    QUAY_FR,
    -QUAY_RL,
    QUAY_RR
  );

  choVaDocPi(ms);
  dungMotor();
}


void dung(unsigned long ms)
{
  dungMotor();
  choVaDocPi(ms);
}



// ============================================================
// 8B. OPEN-LOOP THEO mm CHO MAP MU
// ============================================================

unsigned long mmToMs(long mm, float mm_s)
{
  if (mm <= 0 || mm_s <= 0.0f)
    return 0;

  return (unsigned long)((mm * 1000.0f) / mm_s);
}


void tienMm(long mm)
{
  if (mm <= 0) return;

  dat4Motor(TIEN_FL, TIEN_FR, TIEN_RL, TIEN_RR);
  choVaDocPi(mmToMs(mm, TIEN_MM_S));
  dungMotor();
  choVaDocPi(100);
}


void luiMm(long mm)
{
  if (mm <= 0) return;

  dat4Motor(-LUI_FL, -LUI_FR, -LUI_RL, -LUI_RR);
  choVaDocPi(mmToMs(mm, TIEN_MM_S));
  dungMotor();
  choVaDocPi(100);
}


void ngangPhaiMm(long mm)
{
  if (mm <= 0) return;

  dat4Motor(
    PHAI_FL,
    -PHAI_FR,
    -PHAI_RL,
    PHAI_RR
  );

  choVaDocPi(mmToMs(mm, NGANG_MM_S));
  dungMotor();
  choVaDocPi(100);
}


void ngangTraiMm(long mm)
{
  if (mm <= 0) return;

  dat4Motor(
    -TRAI_FL,
    TRAI_FR,
    TRAI_RL,
    -TRAI_RR
  );

  choVaDocPi(mmToMs(mm, NGANG_MM_S));
  dungMotor();
  choVaDocPi(100);
}


// ============================================================
// 8C. VE MOC M0
//
// Lam theo tung stage de khong cho robot tu chay ca map khi
// chua test xong tung doan.
// ============================================================

bool veMocM0()
{
  Serial.print("DBG:M0_STAGE=");
  Serial.println(M0_TEST_STAGE);

  if (M0_TEST_STAGE == 0)
  {
    dungMotor();
    return false;
  }

  // Stage 1: lui khoi khu kien.
  if (VE_M0_LUI_MM > 0)
  {
    Serial.println("DBG:M0_LUI");
    luiMm(VE_M0_LUI_MM);
  }

  if (M0_TEST_STAGE == 1)
    return false;

  // Stage 2: quay 180 do.
  if (VE_M0_QUAY_180_MS > 0)
  {
    Serial.println("DBG:M0_QUAY_180");
    quayPhai(VE_M0_QUAY_180_MS);
    dung(150);
  }

  if (M0_TEST_STAGE == 2)
    return false;

  // Stage 3: tien den factory lane.
  if (VE_M0_TIEN_DEN_LANE_MM > 0)
  {
    Serial.println("DBG:M0_TIEN_DEN_LANE");
    tienMm(VE_M0_TIEN_DEN_LANE_MM);
  }

  if (M0_TEST_STAGE == 3)
    return false;

  // Stage 4: dich doc lane ve Foxconn = M0.
  if (VE_M0_DICH_DOC_MM > 0)
  {
    Serial.println("DBG:M0_DICH_TRAI");
    ngangTraiMm(VE_M0_DICH_DOC_MM);
  }
  else if (VE_M0_DICH_DOC_MM < 0)
  {
    Serial.println("DBG:M0_DICH_PHAI");
    ngangPhaiMm(-VE_M0_DICH_DOC_MM);
  }

  lanePosMm = 0;

  Serial.println("DBG:M0_OK");
  return true;
}


// ============================================================
// 8D. OUTPUT / NHA MAY
// ============================================================

long viTriOutputTheoClass(const char *hang)
{
  if (strcmp(hang, "qr") == 0)
    return POS_FOXCONN_MM;

  if (strcmp(hang, "yt") == 0)
    return POS_AMKOR_MM;

  if (strcmp(hang, "chip") == 0)
    return POS_HANA_MM;

  if (strcmp(hang, "al") == 0)
    return POS_SAMSUNG_MM;

  return -1;
}


const char *tenOutputTheoClass(const char *hang)
{
  if (strcmp(hang, "qr") == 0)   return "FOXCONN";
  if (strcmp(hang, "yt") == 0)   return "AMKOR";
  if (strcmp(hang, "chip") == 0) return "HANA_MICRON";
  if (strcmp(hang, "al") == 0)   return "SAMSUNG";

  return "UNKNOWN";
}


void diDocLaneDen(long targetMm)
{
  long delta = targetMm - lanePosMm;

  Serial.print("DBG:LANE_FROM=");
  Serial.print(lanePosMm);
  Serial.print(" TO=");
  Serial.println(targetMm);

  if (delta > 0)
  {
    // Theo quy uoc map cu: vi tri tang => ngang trai.
    ngangTraiMm(delta);
  }
  else if (delta < 0)
  {
    ngangPhaiMm(-delta);
  }

  lanePosMm = targetMm;
}


void thaKien()
{
  // TODO: ghep servo/fork/lift that vao day.
  dungMotor();

  Serial.println("DBG:THA_KIEN");

  // Tam dung 500 ms de danh dau diem tha.
  choVaDocPi(500);
}


bool giaoKienTheoClass(const char *hang)
{
  long target = viTriOutputTheoClass(hang);

  if (target < 0)
  {
    Serial.println("DBG:OUTPUT_UNKNOWN");
    dungMotor();
    return false;
  }

  Serial.print("DBG:OUTPUT=");
  Serial.println(tenOutputTheoClass(hang));

  // 1. Dich doc factory lane den dung hang nha may.
  diDocLaneDen(target);

  // 2. Tien vao output.
  Serial.println("DBG:VAO_OUTPUT");
  tienMm(VAO_OUTPUT_MM);

  // 3. Tha kien.
  thaKien();

  // 4. Lui ra lai factory lane.
  Serial.println("DBG:RA_OUTPUT");
  luiMm(VAO_OUTPUT_MM);

  dungMotor();

  Serial.println("DBG:DELIVERY_DONE");
  return true;
}


// ============================================================
// 9. LENH CAMERA GUI SANG ren.py
// ============================================================

void batLine()
{
  coLine = false;
  errorLine = 0;
  lanNhanLine = 0;

  Serial.println("LINE_ON");
  Serial.flush();
}


void batYOLO()
{
  coHang = false;
  hangReady = false;
  errorHang = 0;
  lanNhanHang = 0;
  loaiHang[0] = '\0';

  Serial.println("YOLO_ON");
  Serial.flush();
}


void khoaLoaiHang(const char *target)
{
  Serial.print("TARGET,");
  Serial.print(target);
  Serial.println(",all");
  Serial.flush();
}


void tatCamera()
{
  Serial.println("CAM_OFF");
  Serial.flush();
}


// ============================================================
// 10. DI THEO LINE
// ============================================================

bool tienTheoLine(unsigned long thoiGianChay)
{
  if (piStop) return false;

  batLine();

  Serial.println("DBG:WAIT_LINE");

  // Cho camera duoi tim line lan dau.
  unsigned long waitStart = millis();

  while (!coLine && millis() - waitStart < 2500)
  {
    docPi();
    dungMotor();

    if (piStop)
      return false;

    delay(1);
  }

  if (!coLine)
  {
    Serial.println("DBG:NO_LINE");
    dungMotor();
    tatCamera();
    return false;
  }

  Serial.println("DBG:LINE_FOUND");

  unsigned long daChay = 0;
  unsigned long moc = millis();
  unsigned long watchdog = millis();

  while (daChay < thoiGianChay)
  {
    docPi();

    if (piStop)
    {
      dungMotor();
      return false;
    }

    unsigned long now = millis();
    unsigned long dt = now - moc;
    moc = now;

    bool lineHopLe =
      coLine &&
      lanNhanLine > 0 &&
      (now - lanNhanLine <= TIMEOUT_LINE);

    if (!lineHopLe)
    {
      dungMotor();

      if (now - watchdog > thoiGianChay + 5000)
      {
        Serial.println("DBG:LINE_TIMEOUT");
        tatCamera();
        return false;
      }

      delay(1);
      continue;
    }

    int sua = 0;

    if (abs(errorLine) > DEAD_LINE)
    {
      sua = errorLine / max(1, PIXEL_MOI_PWM_LINE);
      sua = constrain(sua, -MAX_SUA_LINE, MAX_SUA_LINE);
    }

    // Tien + vector dich ngang cua mecanum.
    int FL = constrain(LINE_FL + sua, 0, 255);
    int FR = constrain(LINE_FR - sua, 0, 255);
    int RL = constrain(LINE_RL - sua, 0, 255);
    int RR = constrain(LINE_RR + sua, 0, 255);

    dat4Motor(FL, FR, RL, RR);

    daChay += dt;
    delay(1);
  }

  dungMotor();
  tatCamera();

  Serial.println("DBG:LINE_DONE");

  return true;
}


// ============================================================
// 11. QUET YOLO VA LAY CLASS
// ============================================================

bool quetHang(unsigned long timeout)
{
  if (piStop) return false;

  dungMotor();
  batYOLO();

  Serial.println("DBG:WAIT_YOLO");

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
      Serial.print("DBG:CLASS=");
      Serial.println(loaiHang);

      // Khoa ren.py vao dung class vua nhan.
      khoaLoaiHang(loaiHang);

      return true;
    }

    delay(1);
  }

  Serial.println("DBG:NO_HANG");
  return false;
}


// ============================================================
// 12. TIEN + YOLO CAN TAM
// ============================================================

bool tienCanTamHang(unsigned long thoiGianChay)
{
  if (piStop) return false;

  unsigned long daChay = 0;
  unsigned long moc = millis();
  unsigned long watchdog = millis();

  while (daChay < thoiGianChay)
  {
    docPi();

    if (piStop)
    {
      dungMotor();
      return false;
    }

    unsigned long now = millis();
    unsigned long dt = now - moc;
    moc = now;

    bool hangHopLe =
      coHang &&
      lanNhanHang > 0 &&
      (now - lanNhanHang <= TIMEOUT_HANG);

    if (!hangHopLe)
    {
      dungMotor();

      if (now - watchdog > thoiGianChay + 5000)
      {
        Serial.println("DBG:HANG_TIMEOUT");
        return false;
      }

      delay(1);
      continue;
    }

    int sua = 0;

    if (abs(errorHang) > DEAD_HANG)
    {
      sua = errorHang / max(1, PIXEL_MOI_PWM_HANG);
      sua = constrain(sua, -MAX_SUA_HANG, MAX_SUA_HANG);
    }

    int FL = constrain(HANG_FL + sua, 0, 255);
    int FR = constrain(HANG_FR - sua, 0, 255);
    int RL = constrain(HANG_RL - sua, 0, 255);
    int RR = constrain(HANG_RR + sua, 0, 255);

    dat4Motor(FL, FR, RL, RR);

    daChay += dt;
    delay(1);
  }

  dungMotor();

  Serial.println("DBG:HANG_MOVE_DONE");

  return true;
}


// ============================================================
// 13. CHUOI TEST HIEN TAI
// SAU KHI TEST ON, TA SE THAY PHAN NAY BANG MAP THUC TE.
// ============================================================

void chayThuHeThong()
{
  Serial.println("DBG:ROUTE_START");

  // ----------------------------------------------------------
  // 1. DI TOI KHU KIEN BANG CAMERA LINE
  // ----------------------------------------------------------
  bool lineOK = tienTheoLine(3000);

  dung(1000);

  Serial.print("DBG:LINE_RESULT=");
  Serial.println(lineOK ? "OK" : "FAIL");

  // ----------------------------------------------------------
  // 2. YOLO NHAN DIEN + CAN TAM KIEN
  // ----------------------------------------------------------
  bool hangOK = quetHang(4000);

  if (!hangOK)
  {
    Serial.println("RESULT_CLASS:none");
    dungMotor();
    tatCamera();
    Serial.println("DBG:ROUTE_DONE");
    return;
  }

  // Tien toi kien va can trai/phai theo XHANG.
  tienCanTamHang(2000);

  Serial.print("RESULT_CLASS:");
  Serial.println(loaiHang);

  // Tu day khong can camera nua.
  dungMotor();
  tatCamera();
  dung(300);

  // ----------------------------------------------------------
  // 3. VE MOC M0
  // ----------------------------------------------------------
  bool m0OK = veMocM0();

  if (!m0OK)
  {
    Serial.println("DBG:STOP_AFTER_M0_TEST");
    dungMotor();
    Serial.println("DBG:ROUTE_DONE");
    return;
  }

  // ----------------------------------------------------------
  // 4. CHAY DEN OUTPUT THEO CLASS
  // ----------------------------------------------------------
  if (!CHO_PHEP_CHAY_OUTPUT)
  {
    Serial.println("DBG:OUTPUT_ROUTE_LOCKED");
    dungMotor();
    Serial.println("DBG:ROUTE_DONE");
    return;
  }

  giaoKienTheoClass(loaiHang);

  dungMotor();
  Serial.println("DBG:ROUTE_DONE");
}


// ============================================================
// 14. SETUP
// ============================================================

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

  delay(300);

  // Quan trong:
  // Pi mo /dev/ttyUSB0 co the lam UNO reset.
  // Vi vay UNO cu gui HELLO va cho PI_READY.
  unsigned long lastHello = 0;

  while (!piReady)
  {
    docPi();
    dungMotor();

    if (millis() - lastHello >= 500)
    {
      Serial.println("HELLO_UNO");
      Serial.flush();
      lastHello = millis();
    }

    delay(2);
  }

  Serial.println("DBG:PI_READY_OK");
  Serial.flush();

  delay(200);

  chayThuHeThong();

  dungMotor();
}


// ============================================================
// 15. LOOP
// ============================================================

void loop()
{
  // Lo trinh chi chay 1 lan sau moi lan reset.
  docPi();
  dungMotor();

  delay(2);
}
