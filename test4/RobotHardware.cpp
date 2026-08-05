#include "RobotConfig.h"

#if defined(__AVR__)
#include <avr/interrupt.h>
#endif

// ============================================================================
// ROBOT HARDWARE - BẢN REFACTOR GIỮ NGUYÊN API CŨ
//
// Mục tiêu:
//   1. Giữ nguyên tên và chữ ký các hàm để file .ino cũ vẫn gọi được.
//   2. Không tách class/module ở giai đoạn này để dễ copy, test và sửa lỗi.
//   3. Sensor chỉ analogRead đúng một lần trong mỗi lần gọi rd().
//   4. Calibration chỉ cập nhật Min/Max trong đúng 5 giây rồi khóa lại.
//   5. Encoder được đọc/reset trong vùng atomic.
//   6. Tất cả vòng chờ dài đều có timeout chống treo robot.
//   7. Bộ điều khiển bánh xe dùng Feed-forward + PI + Anti-windup.
// ============================================================================


// ============================================================================
// 1. PHẦN CỨNG VÀ BIẾN TOÀN CỤC BẮT BUỘC GIỮ LẠI
// ============================================================================

AF_DCMotor M1(1);  // LF
AF_DCMotor M2(2);  // LR
AF_DCMotor M3(3);  // RR
AF_DCMotor M4(4);  // RF

volatile long count_FL = 0;
volatile long count_FR = 0;
volatile long count_RL = 0;
volatile long count_RR = 0;

bool m1 = false;
bool m2 = false;
bool m3 = false;
bool m4 = false;
bool m5 = false;
bool m6 = false;
bool m7 = false;
bool m8 = false;

float filteredSensor[8] = {0.0f};
float lineError = 0.0f;
TrangThaiLine trangThaiLineHienTai = LINE_LOST;

int base_speed = 120;


// ============================================================================
// 2. CẤU HÌNH NỘI BỘ
//
// Sau khi bản này chạy ổn định, các hằng số này mới chuyển sang RobotConfig.h.
// Giữ tại đây trước để bạn chỉ cần thay một file.
// ============================================================================

namespace {

constexpr uint8_t SENSOR_COUNT = 8;

// Thu tu vat ly tu trai sang phai cua dan sensor:
// m1=A7, m2=A8, m3=A9, m4=A10, m5=A11, m6=A12, m7=A13, m8=A14.
// Hai mat chinh giua cua xe la m4 va m5, tuong ung A10 va A11.
const uint8_t SENSOR_PIN[SENSOR_COUNT] = {
    A7, A8, A9, A10, A11, A12, A13, A14
};

const int16_t LINE_WEIGHT[SENSOR_COUNT] = {
    -3500, -2500, -1500, -500,
      500,  1500,  2500, 3500
};

// --------------------------- Sensor -----------------------------------------

constexpr uint16_t SENSOR_SCALE = 1000;
constexpr uint16_t SENSOR_CALIBRATION_MS = 5000;
constexpr uint16_t SENSOR_MIN_VALID_RANGE = 80;

constexpr uint16_t SENSOR_NOISE_FLOOR = 250;
constexpr uint16_t SENSOR_LINE_THRESHOLD = 460;
constexpr uint16_t SENSOR_NORMAL_MIN = 390;
constexpr uint16_t SENSOR_ALL_BLACK_THRESHOLD = 500;
constexpr uint16_t SENSOR_ALL_BLACK_AVERAGE = 560;
constexpr uint8_t SENSOR_ALL_BLACK_COUNT = 6;
constexpr uint16_t SENSOR_SUM_MIN = 100;

// EMA:
// filtered += alpha * (new - filtered)
constexpr float SENSOR_FILTER_ALPHA = 0.50f;

// --------------------------- Encoder/Motor ----------------------------------

constexpr float ENCODER_PPR = 495.0f;
constexpr float WHEEL_DIAMETER_MM = 60.0f;

// 30 ms cho do phan giai RPM tot hon khi chay cham.
// Voi 495 xung/vong: 1 xung / 30 ms xap xi 4.04 RPM.
constexpr uint16_t MOTOR_SAMPLE_TIME_MS = 30;
constexpr float MOTOR_ACCEL_RPM_PER_SECOND = 3000.0f;
constexpr float MOTOR_DECEL_RPM_PER_SECOND = 4000.0f;
constexpr float RPM_FILTER_ALPHA = 0.35f;

// Feed-forward + PI.
// Đây là giá trị khởi đầu an toàn, vẫn cần tuning theo pin, motor và tải thật.
constexpr float MOTOR_FF = 2.20f;
constexpr float MOTOR_KP = 0.70f;
constexpr float MOTOR_KI = 0.45f;
constexpr float MOTOR_INTEGRAL_LIMIT = 160.0f;

constexpr int16_t MOTOR_PWM_LIMIT = 255;
constexpr int16_t MOTOR_MIN_EFFECTIVE_PWM = 28;
constexpr float MOTOR_STOP_RPM = 0.5f;

// --------------------------- Line PID ---------------------------------------

constexpr float LINE_ERROR_MAX = 3500.0f;
constexpr float LINE_KP = 60.0f;
constexpr float LINE_KD = 2.5f;
constexpr float LINE_D_FILTER_ALPHA = 0.40f;
constexpr float LINE_DEAD_ZONE = 0.06f;
constexpr float LINE_MIN_RPM = 25.0f;
constexpr float LINE_MAX_CORRECTION = 120.0f;

constexpr uint16_t LINE_RECOVERY_TIME_MS = 350;
constexpr float LINE_RECOVERY_RPM = 35.0f;

// --------------------------- Motion Safety ----------------------------------

constexpr uint32_t MOTION_MIN_TIMEOUT_MS = 1500UL;
constexpr uint32_t MOTION_MAX_TIMEOUT_MS = 20000UL;
constexpr uint32_t MOTION_STALL_TIMEOUT_MS = 700UL;

constexpr uint32_t FIND_INTERSECTION_TIMEOUT_MS = 15000UL;
constexpr uint32_t FIND_CORNER_TIMEOUT_MS = 15000UL;
constexpr uint32_t FIND_LINE_TIMEOUT_MS = 5000UL;

constexpr uint16_t CORNER_IGNORE_TIME_MS = 250;
constexpr uint8_t SENSOR_CONFIRM_COUNT = 3;

// Chay ngang: giam toc truoc khi cham vach va dung em sau khi xac nhan.
constexpr uint16_t STRAFE_SLOW_SENSOR_THRESHOLD = 300;
constexpr float STRAFE_SLOW_RPM = 18.0f;
constexpr uint16_t STRAFE_MIN_SLOW_TIME_MS = 120;
constexpr uint16_t STRAFE_DECEL_TIME_MS = 180;


// ============================================================================
// 3. BIẾN NỘI BỘ
// ============================================================================

uint16_t sensorMin[SENSOR_COUNT] = {
    1023, 1023, 1023, 1023,
    1023, 1023, 1023, 1023
};

uint16_t sensorMax[SENSOR_COUNT] = {0};
uint16_t rawSensor[SENSOR_COUNT] = {0};

bool sensorFilterReady = false;
bool calibrationActive = false;
bool calibrationValid = false;

long lastEncoderRF = 0;
long lastEncoderRR = 0;
long lastEncoderLF = 0;
long lastEncoderLR = 0;

float rpmRF = 0.0f;
float rpmRR = 0.0f;
float rpmLF = 0.0f;
float rpmLR = 0.0f;

float commandRF = 0.0f;
float commandRR = 0.0f;
float commandLF = 0.0f;
float commandLR = 0.0f;

float targetRF = 0.0f;
float targetRR = 0.0f;
float targetLF = 0.0f;
float targetLR = 0.0f;

float integralRF = 0.0f;
float integralRR = 0.0f;
float integralLF = 0.0f;
float integralLR = 0.0f;

uint32_t lastRPMTimeMs = 0;

float linePreviousNormalizedError = 0.0f;
float filteredDerivative = 0.0f;
uint32_t lineLastControlTimeUs = 0;

int8_t lastKnownLineSide = 1;
bool dang_cho_phep_cuu_ho = true;


// ============================================================================
// 4. HÀM TIỆN ÍCH NỘI BỘ
// ============================================================================

inline float clampFloat(float value, float minimum, float maximum) {
    if (value < minimum) return minimum;
    if (value > maximum) return maximum;
    return value;
}

inline int16_t clampPWM(int32_t value) {
    if (value > MOTOR_PWM_LIMIT) return MOTOR_PWM_LIMIT;
    if (value < -MOTOR_PWM_LIMIT) return -MOTOR_PWM_LIMIT;
    return static_cast<int16_t>(value);
}

inline int8_t normalizeDirection(int direction) {
    if (direction > 0) return 1;
    if (direction < 0) return -1;
    return 0;
}

inline bool sameSign(float a, float b) {
    return (a >= 0.0f && b >= 0.0f) ||
           (a <= 0.0f && b <= 0.0f);
}

inline float approachValue(float current, float desired, float maxDelta) {
    const float delta = desired - current;

    if (delta > maxDelta) return current + maxDelta;
    if (delta < -maxDelta) return current - maxDelta;
    return desired;
}

float slewTarget(float current, float desired, float dtSeconds) {
    const bool increasingMagnitude =
        sameSign(current, desired) &&
        (fabs(desired) > fabs(current));

    const float rate = increasingMagnitude
        ? MOTOR_ACCEL_RPM_PER_SECOND
        : MOTOR_DECEL_RPM_PER_SECOND;

    return approachValue(current, desired, rate * dtSeconds);
}

void resetMotorControllerState() {
    integralRF = 0.0f;
    integralRR = 0.0f;
    integralLF = 0.0f;
    integralLR = 0.0f;
}

void writeMotor(AF_DCMotor& motor, int16_t pwm) {
    pwm = clampPWM(pwm);

    const uint8_t magnitude =
        static_cast<uint8_t>(pwm >= 0 ? pwm : -pwm);

    motor.setSpeed(magnitude);

    if (pwm > 0) {
        motor.run(FORWARD);
    } else if (pwm < 0) {
        motor.run(BACKWARD);
    } else {
        motor.run(RELEASE);
    }
}

int16_t calculateWheelPWM(
    float targetRPM,
    float measuredRPM,
    float dtSeconds,
    float& integral
) {
    if (fabs(targetRPM) < MOTOR_STOP_RPM) {
        integral = 0.0f;
        return 0;
    }

    const float error = targetRPM - measuredRPM;

    integral += error * dtSeconds;
    integral = clampFloat(
        integral,
        -MOTOR_INTEGRAL_LIMIT,
        MOTOR_INTEGRAL_LIMIT
    );

    float output =
        (MOTOR_FF * targetRPM) +
        (MOTOR_KP * error) +
        (MOTOR_KI * integral);

    output = clampFloat(
        output,
        -static_cast<float>(MOTOR_PWM_LIMIT),
        static_cast<float>(MOTOR_PWM_LIMIT)
    );

    // Bù vùng chết phần cứng.
    if (output > 0.0f && output < MOTOR_MIN_EFFECTIVE_PWM) {
        output = MOTOR_MIN_EFFECTIVE_PWM;
    } else if (output < 0.0f && output > -MOTOR_MIN_EFFECTIVE_PWM) {
        output = -MOTOR_MIN_EFFECTIVE_PWM;
    }

    return static_cast<int16_t>(output);
}

// Encoder ben trai va ben phai co the dem trai dau nhau do lap doi xung.
// PID toc do chi can biet do lon toc do; huong quay da do lenh motor quyet dinh.
// Ham nay dua toc do do duoc ve cung dau voi target RPM.
float measuredRPMInCommandDirection(
    float measuredRPM,
    float targetRPM
) {
    if (targetRPM > MOTOR_STOP_RPM) {
        return fabs(measuredRPM);
    }

    if (targetRPM < -MOTOR_STOP_RPM) {
        return -fabs(measuredRPM);
    }

    return 0.0f;
}

void atomicReadEncoder(
    long& rf,
    long& rr,
    long& lf,
    long& lr
) {
#if defined(__AVR__)
    const uint8_t oldSREG = SREG;
    cli();

    rf = count_FR;
    rr = count_RR;
    lf = count_FL;
    lr = count_RL;

    SREG = oldSREG;
#else
    noInterrupts();

    rf = count_FR;
    rr = count_RR;
    lf = count_FL;
    lr = count_RL;

    interrupts();
#endif
}

void atomicResetEncoder() {
#if defined(__AVR__)
    const uint8_t oldSREG = SREG;
    cli();

    count_FL = 0;
    count_FR = 0;
    count_RL = 0;
    count_RR = 0;

    SREG = oldSREG;
#else
    noInterrupts();

    count_FL = 0;
    count_FR = 0;
    count_RL = 0;
    count_RR = 0;

    interrupts();
#endif
}

void readSensorFrame(bool updateCalibration) {
    for (uint8_t i = 0; i < SENSOR_COUNT; ++i) {
        const uint16_t raw =
            static_cast<uint16_t>(analogRead(SENSOR_PIN[i]));

        rawSensor[i] = raw;

        if (updateCalibration) {
            if (raw < sensorMin[i]) sensorMin[i] = raw;
            if (raw > sensorMax[i]) sensorMax[i] = raw;
        }

        const uint16_t range = sensorMax[i] - sensorMin[i];

        int32_t normalized;

        if (range >= SENSOR_MIN_VALID_RANGE) {
            normalized =
                (static_cast<int32_t>(raw) - sensorMin[i]) *
                SENSOR_SCALE /
                range;

            normalized = 1000L - normalized;
        } else {
            // Fallback trước khi calibration hợp lệ.
            // Giữ đúng cực tính của thuật toán cũ: raw thấp -> giá trị line cao.
            normalized =
                (1023L - static_cast<int32_t>(raw)) *
                SENSOR_SCALE /
                1023L;
        }

        if (normalized < 0) normalized = 0;
        if (normalized > SENSOR_SCALE) normalized = SENSOR_SCALE;

        const float value = static_cast<float>(normalized);

        if (!sensorFilterReady) {
            filteredSensor[i] = value;
        } else {
            filteredSensor[i] +=
                SENSOR_FILTER_ALPHA *
                (value - filteredSensor[i]);
        }
    }

    sensorFilterReady = true;
}

uint32_t calculateMotionTimeoutMs(long distanceMM, float rpm) {
    if (distanceMM <= 0) {
        return MOTION_MIN_TIMEOUT_MS;
    }

    const float safeRPM = fabs(rpm) < 10.0f ? 10.0f : fabs(rpm);
    const float wheelCircumference = PI * WHEEL_DIAMETER_MM;

    // Thời gian lý thuyết x3 + 800ms dự phòng.
    float expectedMs =
        (static_cast<float>(distanceMM) * 60000.0f) /
        (safeRPM * wheelCircumference);

    uint32_t timeoutMs =
        static_cast<uint32_t>(expectedMs * 3.0f + 800.0f);

    if (timeoutMs < MOTION_MIN_TIMEOUT_MS) {
        timeoutMs = MOTION_MIN_TIMEOUT_MS;
    }

    if (timeoutMs > MOTION_MAX_TIMEOUT_MS) {
        timeoutMs = MOTION_MAX_TIMEOUT_MS;
    }

    return timeoutMs;
}

long averageTravelTicks(
    long nowRF,
    long nowRR,
    long nowLF,
    long nowLR,
    long startRF,
    long startRR,
    long startLF,
    long startLR
) {
    return (
        labs(nowRF - startRF) +
        labs(nowRR - startRR) +
        labs(nowLF - startLF) +
        labs(nowLR - startLR)
    ) / 4L;
}

void stopAfterMotion() {
    dung_xe_ngay();
}

bool runDistanceMotion(
    float rpm,
    long distanceMM,
    int8_t directionRF,
    int8_t directionRR,
    int8_t directionLF,
    int8_t directionLR
) {
    if (distanceMM <= 0 || fabs(rpm) < 0.5f) {
        stopAfterMotion();
        return true;
    }

    const long targetTicks = doi_mm_ra_xung(distanceMM);

    long startRF;
    long startRR;
    long startLF;
    long startLR;

    atomicReadEncoder(startRF, startRR, startLF, startLR);

    const float speed = fabs(rpm);

    setTargetRPM(
        speed * directionRF,
        speed * directionRR,
        speed * directionLF,
        speed * directionLR
    );

    const uint32_t startedMs = millis();
    const uint32_t timeoutMs =
        calculateMotionTimeoutMs(distanceMM, speed);

    uint32_t lastProgressMs = startedMs;
    long lastProgressTicks = 0;

    while (true) {
        cap_nhat_pid_dong_co();

        long nowRF;
        long nowRR;
        long nowLF;
        long nowLR;

        atomicReadEncoder(nowRF, nowRR, nowLF, nowLR);

        const long travelled = averageTravelTicks(
            nowRF, nowRR, nowLF, nowLR,
            startRF, startRR, startLF, startLR
        );

        if (travelled >= targetTicks) {
            stopAfterMotion();
            return true;
        }

        const uint32_t nowMs = millis();

        if (travelled > lastProgressTicks + 1L) {
            lastProgressTicks = travelled;
            lastProgressMs = nowMs;
        }

        if ((nowMs - startedMs >= timeoutMs) ||
            (nowMs - lastProgressMs >= MOTION_STALL_TIMEOUT_MS)) {
            stopAfterMotion();
            return false;
        }
    }
}

void applyLineControlFromCurrentFrame(
    float baseRPM,
    bool allowRecovery
) {
    if (trangThaiLineHienTai == LINE_NORMAL) {
        if (lineError > 50.0f) {
            lastKnownLineSide = 1;
        } else if (lineError < -50.0f) {
            lastKnownLineSide = -1;
        }
    }

    if (trangThaiLineHienTai == LINE_LOST) {
        if (!allowRecovery) {
            dung_xe_ngay();
            return;
        }

        dung_xe_ngay();

        const uint32_t startedMs = millis();
        bool foundLine = false;

        while (millis() - startedMs < LINE_RECOVERY_TIME_MS) {
            if (lastKnownLineSide > 0) {
                quay_phai_truc_tiep(LINE_RECOVERY_RPM);
            } else {
                quay_trai_truc_tiep(LINE_RECOVERY_RPM);
            }

            cap_nhat_pid_dong_co();

            rd();
            tinh_sai_so_line(lineError);

            if (trangThaiLineHienTai == LINE_NORMAL) {
                foundLine = true;
                break;
            }
        }

        if (foundLine) {
            can_chinh_dau_xe();
            dat_lai_bo_bam_line();
        } else {
            dung_xe_ngay();
        }

        return;
    }

    const uint32_t nowUs = micros();

    float dtSeconds = 0.020f;

    if (lineLastControlTimeUs != 0) {
        dtSeconds =
            static_cast<float>(nowUs - lineLastControlTimeUs) /
            1000000.0f;
    }

    lineLastControlTimeUs = nowUs;
    dtSeconds = clampFloat(dtSeconds, 0.005f, 0.050f);

    float normalizedError =
        clampFloat(
            lineError / LINE_ERROR_MAX,
            -1.0f,
            1.0f
        );

    const float absError = fabs(normalizedError);

    if (absError < LINE_DEAD_ZONE) {
        normalizedError = 0.0f;
    }

    const float rawDerivative =
        (normalizedError - linePreviousNormalizedError) /
        dtSeconds;

    filteredDerivative +=
        LINE_D_FILTER_ALPHA *
        (rawDerivative - filteredDerivative);

    linePreviousNormalizedError = normalizedError;

    float adjustedBaseRPM = fabs(baseRPM);

    if (absError > 0.30f) {
        adjustedBaseRPM *=
            (1.0f - absError * absError);
    }

    adjustedBaseRPM =
        clampFloat(
            adjustedBaseRPM,
            20.0f,
            fabs(baseRPM)
        );

    const float correction =
        clampFloat(
            LINE_KP * normalizedError +
            LINE_KD * filteredDerivative,
            -LINE_MAX_CORRECTION,
            LINE_MAX_CORRECTION
        );

    float rightRPM = adjustedBaseRPM - correction;
    float leftRPM = adjustedBaseRPM + correction;

    if (rightRPM > 0.0f && rightRPM < LINE_MIN_RPM) {
        rightRPM = LINE_MIN_RPM;
    } else if (rightRPM < 0.0f && rightRPM > -LINE_MIN_RPM) {
        rightRPM = -LINE_MIN_RPM;
    }

    if (leftRPM > 0.0f && leftRPM < LINE_MIN_RPM) {
        leftRPM = LINE_MIN_RPM;
    } else if (leftRPM < 0.0f && leftRPM > -LINE_MIN_RPM) {
        leftRPM = -LINE_MIN_RPM;
    }

    setTargetRPM(
        rightRPM,
        rightRPM,
        leftRPM,
        leftRPM
    );
}

}  // namespace


// ============================================================================
// 5. SENSOR
// ============================================================================

void rd() {
    // Chỉ đọc ADC một lần cho mỗi sensor.
    // Min/Max chỉ được cập nhật khi calibrationActive == true.
    readSensorFrame(calibrationActive);
}

TrangThaiLine tinh_sai_so_line(float& error) {
    int32_t weightedSum = 0;
    int32_t sensorSum = 0;
    int32_t rawSum = 0;

    uint16_t maxSensor = 0;
    uint8_t countAbove500 = 0;
    uint8_t blackMask = 0;

    for (uint8_t i = 0; i < SENSOR_COUNT; ++i) {
        int16_t value =
            static_cast<int16_t>(filteredSensor[i] + 0.5f);

        if (value < 0) value = 0;
        if (value > SENSOR_SCALE) value = SENSOR_SCALE;

        rawSum += value;

        if (value > maxSensor) {
            maxSensor = static_cast<uint16_t>(value);
        }

        if (value >= SENSOR_ALL_BLACK_THRESHOLD) {
            ++countAbove500;
        }

        if (value >= SENSOR_LINE_THRESHOLD) {
            blackMask |= static_cast<uint8_t>(1U << i);
        }

        int16_t processed = value - SENSOR_NOISE_FLOOR;

        if (processed < 0) {
            processed = 0;
        }

        sensorSum += processed;
        weightedSum +=
            static_cast<int32_t>(processed) *
            LINE_WEIGHT[i];
    }

    m1 = (blackMask & (1U << 0)) != 0;
    m2 = (blackMask & (1U << 1)) != 0;
    m3 = (blackMask & (1U << 2)) != 0;
    m4 = (blackMask & (1U << 3)) != 0;
    m5 = (blackMask & (1U << 4)) != 0;
    m6 = (blackMask & (1U << 5)) != 0;
    m7 = (blackMask & (1U << 6)) != 0;
    m8 = (blackMask & (1U << 7)) != 0;

    const bool allBlack =
        countAbove500 >= SENSOR_ALL_BLACK_COUNT &&
        rawSum >=
            static_cast<int32_t>(
                SENSOR_ALL_BLACK_AVERAGE *
                SENSOR_COUNT
            );

    if (allBlack) {
        trangThaiLineHienTai = LINE_ALL_BLACK;
        error = 0.0f;
    } else if (
        maxSensor < SENSOR_NORMAL_MIN ||
        sensorSum < SENSOR_SUM_MIN
    ) {
        trangThaiLineHienTai = LINE_LOST;

        // Giữ error cuối cùng để recovery còn biết hướng line đã mất.
    } else {
        trangThaiLineHienTai = LINE_NORMAL;

        error =
            static_cast<float>(weightedSum) /
            static_cast<float>(sensorSum);
    }

    return trangThaiLineHienTai;
}

bool phat_hien_mau_goc_phai_90() {
    const bool extremeRight =
        filteredSensor[6] >= 450.0f &&
        filteredSensor[7] >= 450.0f;

    const bool centerStillOnLine =
        filteredSensor[3] >= 400.0f ||
        filteredSensor[4] >= 400.0f;

    const bool leftSideClear =
        filteredSensor[0] < 450.0f &&
        filteredSensor[1] < 450.0f;

    return
        trangThaiLineHienTai == LINE_NORMAL &&
        extremeRight &&
        centerStillOnLine &&
        leftSideClear;
}

bool phat_hien_mau_goc_trai_90() {
    const bool extremeLeft =
        filteredSensor[0] >= 450.0f &&
        filteredSensor[1] >= 450.0f;

    const bool centerStillOnLine =
        filteredSensor[3] >= 400.0f ||
        filteredSensor[4] >= 400.0f;

    const bool rightSideClear =
        filteredSensor[6] < 450.0f &&
        filteredSensor[7] < 450.0f;

    return
        trangThaiLineHienTai == LINE_NORMAL &&
        extremeLeft &&
        centerStillOnLine &&
        rightSideClear;
}


// ============================================================================
// 6. MOTOR CƠ SỞ
// ============================================================================

void motorRF(int pwm) {
    writeMotor(M4, static_cast<int16_t>(pwm));
}

void motorRR(int pwm) {
    writeMotor(M3, static_cast<int16_t>(pwm));
}

void motorLF(int pwm) {
    writeMotor(M1, static_cast<int16_t>(pwm));
}

void motorLR(int pwm) {
    writeMotor(M2, static_cast<int16_t>(pwm));
}

void dung_tat_ca_dong_co() {
    M1.run(RELEASE);
    M2.run(RELEASE);
    M3.run(RELEASE);
    M4.run(RELEASE);
}

void phanh_cung_dong_co() {
    M1.run(BRAKE);
    M2.run(BRAKE);
    M3.run(BRAKE);
    M4.run(BRAKE);
}

void dung_xe_ngay() {
    commandRF = 0.0f;
    commandRR = 0.0f;
    commandLF = 0.0f;
    commandLR = 0.0f;

    targetRF = 0.0f;
    targetRR = 0.0f;
    targetLF = 0.0f;
    targetLR = 0.0f;

    resetMotorControllerState();

    // Không gọi cap_nhat_pid_dong_co() trong lúc BRAKE,
    // tránh bộ điều khiển ghi RELEASE đè lên BRAKE.
    phanh_cung_dong_co();

    const uint32_t startedMs = millis();

    while (millis() - startedMs < 60UL) {
        // Thời gian phanh ngắn, hữu hạn.
    }

    dung_tat_ca_dong_co();
}

void dung() {
    dung_xe_ngay();
}

void setTargetRPM(float rf, float rr, float lf, float lr) {
    commandRF = rf;
    commandRR = rr;
    commandLF = lf;
    commandLR = lr;
}

void tien_truc_tiep(float rpm) {
    const float speed = fabs(rpm);

    setTargetRPM(speed, speed, speed, speed);
}

void lui_truc_tiep(float rpm) {
    const float speed = fabs(rpm);

    setTargetRPM(-speed, -speed, -speed, -speed);
}

void quay_trai_truc_tiep(float rpm) {
    const float speed = fabs(rpm);

    setTargetRPM(
         speed,
         speed,
        -speed,
        -speed
    );
}

void quay_phai_truc_tiep(float rpm) {
    const float speed = fabs(rpm);

    setTargetRPM(
        -speed,
        -speed,
         speed,
         speed
    );
}

void tien(int speed) {
    tien_truc_tiep(static_cast<float>(speed));
}

void lui(int speed) {
    lui_truc_tiep(static_cast<float>(speed));
}

void trai(int speed) {
    quay_trai_truc_tiep(static_cast<float>(speed));
}

void phai(int speed) {
    quay_phai_truc_tiep(static_cast<float>(speed));
}


// ============================================================================
// 7. ENCODER VÀ PID BÁNH XE
// ============================================================================

void reset_encoder() {
    atomicResetEncoder();

    lastEncoderRF = 0;
    lastEncoderRR = 0;
    lastEncoderLF = 0;
    lastEncoderLR = 0;

    rpmRF = 0.0f;
    rpmRR = 0.0f;
    rpmLF = 0.0f;
    rpmLR = 0.0f;

    lastRPMTimeMs = millis();

    resetMotorControllerState();
}

void doc_tat_ca_encoder(
    long& rf,
    long& rr,
    long& lf,
    long& lr
) {
    atomicReadEncoder(rf, rr, lf, lr);
}

long doi_mm_ra_xung(long khoangCachMM) {
    if (khoangCachMM <= 0) {
        return 0;
    }

    const float pulses =
        static_cast<float>(khoangCachMM) *
        ENCODER_PPR /
        (PI * WHEEL_DIAMETER_MM);

    return static_cast<long>(pulses + 0.5f);
}

void cap_nhat_pid_dong_co() {
    const uint32_t nowMs = millis();

    if (lastRPMTimeMs == 0) {
        lastRPMTimeMs = nowMs;

        atomicReadEncoder(
            lastEncoderRF,
            lastEncoderRR,
            lastEncoderLF,
            lastEncoderLR
        );

        return;
    }

    const uint32_t elapsedMs = nowMs - lastRPMTimeMs;

    if (elapsedMs < MOTOR_SAMPLE_TIME_MS) {
        return;
    }

    lastRPMTimeMs = nowMs;

    const float dtSeconds =
        static_cast<float>(elapsedMs) / 1000.0f;

    long nowRF;
    long nowRR;
    long nowLF;
    long nowLR;

    atomicReadEncoder(nowRF, nowRR, nowLF, nowLR);

    const float rpmScale =
        60000.0f /
        (ENCODER_PPR * static_cast<float>(elapsedMs));

    const float instantRF =
        static_cast<float>(nowRF - lastEncoderRF) *
        rpmScale;

    const float instantRR =
        static_cast<float>(nowRR - lastEncoderRR) *
        rpmScale;

    const float instantLF =
        static_cast<float>(nowLF - lastEncoderLF) *
        rpmScale;

    const float instantLR =
        static_cast<float>(nowLR - lastEncoderLR) *
        rpmScale;

    rpmRF += RPM_FILTER_ALPHA * (instantRF - rpmRF);
    rpmRR += RPM_FILTER_ALPHA * (instantRR - rpmRR);
    rpmLF += RPM_FILTER_ALPHA * (instantLF - rpmLF);
    rpmLR += RPM_FILTER_ALPHA * (instantLR - rpmLR);

    lastEncoderRF = nowRF;
    lastEncoderRR = nowRR;
    lastEncoderLF = nowLF;
    lastEncoderLR = nowLR;

    targetRF = slewTarget(targetRF, commandRF, dtSeconds);
    targetRR = slewTarget(targetRR, commandRR, dtSeconds);
    targetLF = slewTarget(targetLF, commandLF, dtSeconds);
    targetLR = slewTarget(targetLR, commandLR, dtSeconds);

    const int16_t pwmRF = calculateWheelPWM(
        targetRF,
        measuredRPMInCommandDirection(rpmRF, targetRF),
        dtSeconds,
        integralRF
    );

    const int16_t pwmRR = calculateWheelPWM(
        targetRR,
        measuredRPMInCommandDirection(rpmRR, targetRR),
        dtSeconds,
        integralRR
    );

    const int16_t pwmLF = calculateWheelPWM(
        targetLF,
        measuredRPMInCommandDirection(rpmLF, targetLF),
        dtSeconds,
        integralLF
    );

    const int16_t pwmLR = calculateWheelPWM(
        targetLR,
        measuredRPMInCommandDirection(rpmLR, targetLR),
        dtSeconds,
        integralLR
    );

    motorRF(pwmRF);
    motorRR(pwmRR);
    motorLF(pwmLF);
    motorLR(pwmLR);
}


// ============================================================================
// 8. CALIBRATION VÀ LINE PID
// ============================================================================

void cali_5_giay() {
    dung_xe_ngay();

    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH);

    for (uint8_t i = 0; i < SENSOR_COUNT; ++i) {
        sensorMin[i] = 1023;
        sensorMax[i] = 0;
        filteredSensor[i] = 0.0f;
    }

    sensorFilterReady = false;
    calibrationValid = false;
    calibrationActive = true;

    const uint32_t startedMs = millis();

    while (millis() - startedMs < SENSOR_CALIBRATION_MS) {
        // rd() đọc đúng 8 lần analogRead và tự cập nhật Min/Max
        // vì calibrationActive đang bằng true.
        rd();

        // Giữ motor controller sống nhưng command đang bằng 0.
        cap_nhat_pid_dong_co();
    }

    calibrationActive = false;
    calibrationValid = true;

    for (uint8_t i = 0; i < SENSOR_COUNT; ++i) {
        if (sensorMax[i] - sensorMin[i] <
            SENSOR_MIN_VALID_RANGE) {
            calibrationValid = false;

            // Fallback an toàn để tránh chia cho range quá nhỏ.
            sensorMin[i] = 0;
            sensorMax[i] = 1023;
        }
    }

    digitalWrite(LED_BUILTIN, LOW);

    sensorFilterReady = false;
    dat_lai_bo_bam_line();
}

void dat_lai_bo_bam_line() {
    rd();
    tinh_sai_so_line(lineError);

    lineLastControlTimeUs = 0;
    filteredDerivative = 0.0f;

    linePreviousNormalizedError =
        clampFloat(
            lineError / LINE_ERROR_MAX,
            -1.0f,
            1.0f
        );
}

void bam_line_pid(float tocDoCoSo) {
    rd();
    tinh_sai_so_line(lineError);

    applyLineControlFromCurrentFrame(
        tocDoCoSo,
        dang_cho_phep_cuu_ho
    );
}

void can_chinh_dau_xe() {
    constexpr uint32_t ALIGN_TIMEOUT_MS = 1200UL;
    constexpr float ALIGN_TOLERANCE = 20.0f;
    constexpr float ALIGN_KP = 0.015f;
    constexpr float ALIGN_MIN_RPM = 25.0f;
    constexpr float ALIGN_MAX_RPM = 35.0f;
    constexpr uint8_t ALIGN_CONFIRM_COUNT = 3;

    dung_xe_ngay();
    delay_pid(80);

    uint8_t centeredCount = 0;
    const uint32_t startedMs = millis();

    while (millis() - startedMs < ALIGN_TIMEOUT_MS) {
        rd();
        tinh_sai_so_line(lineError);

        if (trangThaiLineHienTai == LINE_LOST) {
            break;
        }

        if (fabs(lineError) <= ALIGN_TOLERANCE) {
            ++centeredCount;

            if (centeredCount >= ALIGN_CONFIRM_COUNT) {
                break;
            }
        } else {
            centeredCount = 0;
        }

        float turnRPM =
            fabs(lineError) * ALIGN_KP;

        turnRPM =
            clampFloat(
                turnRPM,
                ALIGN_MIN_RPM,
                ALIGN_MAX_RPM
            );

        if (lineError > 0.0f) {
            quay_phai_truc_tiep(turnRPM);
        } else {
            quay_trai_truc_tiep(turnRPM);
        }

        cap_nhat_pid_dong_co();
    }

    dung_xe_ngay();
    delay_pid(50);
}


// ============================================================================
// 9. DELAY CÓ NUÔI MOTOR CONTROL
// ============================================================================

void delay_pid(unsigned long ms) {
    const uint32_t startedMs = millis();

    while (millis() - startedMs < ms) {
        cap_nhat_pid_dong_co();
    }
}


// ============================================================================
// 10. MOTION THEO ENCODER
// ============================================================================

void tien_khoang_cach(float toc_do, long mm) {
    runDistanceMotion(
        toc_do,
        mm,
        1, 1, 1, 1
    );
}

void lui_khoang_cach(float toc_do, long mm) {
    runDistanceMotion(
        toc_do,
        mm,
        -1, -1, -1, -1
    );
}

void bam_line_khoang_cach(float toc_do, long mm) {
    if (mm <= 0) {
        dung_xe_ngay();
        return;
    }

    const long targetTicks = doi_mm_ra_xung(mm);

    long startRF;
    long startRR;
    long startLF;
    long startLR;

    atomicReadEncoder(startRF, startRR, startLF, startLR);
    dat_lai_bo_bam_line();

    const uint32_t startedMs = millis();
    const uint32_t timeoutMs =
        calculateMotionTimeoutMs(mm, toc_do);

    uint32_t lastProgressMs = startedMs;
    long lastProgressTicks = 0;

    while (true) {
        cap_nhat_pid_dong_co();

        rd();
        tinh_sai_so_line(lineError);

        applyLineControlFromCurrentFrame(
            toc_do,
            dang_cho_phep_cuu_ho
        );

        long nowRF;
        long nowRR;
        long nowLF;
        long nowLR;

        atomicReadEncoder(nowRF, nowRR, nowLF, nowLR);

        const long travelled = averageTravelTicks(
            nowRF, nowRR, nowLF, nowLR,
            startRF, startRR, startLF, startLR
        );

        if (travelled >= targetTicks) {
            dung_xe_ngay();
            return;
        }

        const uint32_t nowMs = millis();

        if (travelled > lastProgressTicks + 1L) {
            lastProgressTicks = travelled;
            lastProgressMs = nowMs;
        }

        if ((nowMs - startedMs >= timeoutMs) ||
            (nowMs - lastProgressMs >= MOTION_STALL_TIMEOUT_MS)) {
            dung_xe_ngay();
            return;
        }
    }
}

void di_chuyen_vong(
    float so_vong,
    int target_speed,
    int dir_FL,
    int dir_FR,
    int dir_RL,
    int dir_RR
) {
    if (so_vong <= 0.0f || target_speed == 0) {
        dung_xe_ngay();
        return;
    }

    const long targetTicks =
        static_cast<long>(
            so_vong * ENCODER_PPR + 0.5f
        );

    long startRF;
    long startRR;
    long startLF;
    long startLR;

    atomicReadEncoder(startRF, startRR, startLF, startLR);

    const float speed = fabs(static_cast<float>(target_speed));

    setTargetRPM(
        speed * normalizeDirection(dir_FR),
        speed * normalizeDirection(dir_RR),
        speed * normalizeDirection(dir_FL),
        speed * normalizeDirection(dir_RL)
    );

    const uint32_t startedMs = millis();
    const uint32_t timeoutMs =
        clampFloat(
            (so_vong * 60000.0f / speed) * 3.0f + 1000.0f,
            static_cast<float>(MOTION_MIN_TIMEOUT_MS),
            static_cast<float>(MOTION_MAX_TIMEOUT_MS)
        );

    uint32_t lastProgressMs = startedMs;
    long lastProgressTicks = 0;

    while (true) {
        cap_nhat_pid_dong_co();

        long nowRF;
        long nowRR;
        long nowLF;
        long nowLR;

        atomicReadEncoder(nowRF, nowRR, nowLF, nowLR);

        const long travelled = averageTravelTicks(
            nowRF, nowRR, nowLF, nowLR,
            startRF, startRR, startLF, startLR
        );

        if (travelled >= targetTicks) {
            dung_xe_ngay();
            return;
        }

        const uint32_t nowMs = millis();

        if (travelled > lastProgressTicks + 1L) {
            lastProgressTicks = travelled;
            lastProgressMs = nowMs;
        }

        if ((nowMs - startedMs >=
             static_cast<uint32_t>(timeoutMs)) ||
            (nowMs - lastProgressMs >=
             MOTION_STALL_TIMEOUT_MS)) {
            dung_xe_ngay();
            return;
        }
    }
}

void toivong(float so_vong, int speed) {
    di_chuyen_vong(
        so_vong,
        speed,
        1, 1, 1, 1
    );
}

void chay_ngang_encoder(int target_speed, int huong) {
    chayngang(
        huong,
        static_cast<float>(target_speed)
    );
}

void luithang_encoder(int target_speed) {
    lui_truc_tiep(
        static_cast<float>(target_speed)
    );
}


// ============================================================================
// 11. BÁM LINE ĐẾN SỰ KIỆN
// ============================================================================

void bam_line_den_nga_tu(float toc_do) {
    dat_lai_bo_bam_line();

    const uint32_t startedMs = millis();

    while (millis() - startedMs <
           FIND_INTERSECTION_TIMEOUT_MS) {
        cap_nhat_pid_dong_co();

        rd();
        tinh_sai_so_line(lineError);

        if (trangThaiLineHienTai == LINE_ALL_BLACK) {
            dung_xe_ngay();
            return;
        }

        applyLineControlFromCurrentFrame(
            toc_do,
            dang_cho_phep_cuu_ho
        );
    }

    dung_xe_ngay();
}

void bam_line_den_goc_phai(float toc_do) {
    dat_lai_bo_bam_line();

    const uint32_t startedMs = millis();
    uint8_t confirmCount = 0;

    while (millis() - startedMs <
           FIND_CORNER_TIMEOUT_MS) {
        cap_nhat_pid_dong_co();

        rd();
        tinh_sai_so_line(lineError);

        if (millis() - startedMs >
            CORNER_IGNORE_TIME_MS) {
            if (phat_hien_mau_goc_phai_90()) {
                ++confirmCount;

                if (confirmCount >=
                    SENSOR_CONFIRM_COUNT) {
                    dung_xe_ngay();
                    return;
                }
            } else {
                confirmCount = 0;
            }
        }

        applyLineControlFromCurrentFrame(
            toc_do,
            dang_cho_phep_cuu_ho
        );
    }

    dung_xe_ngay();
}

void bam_line_den_goc_trai(float toc_do) {
    dat_lai_bo_bam_line();

    const uint32_t startedMs = millis();
    uint8_t confirmCount = 0;

    while (millis() - startedMs <
           FIND_CORNER_TIMEOUT_MS) {
        cap_nhat_pid_dong_co();

        rd();
        tinh_sai_so_line(lineError);

        if (millis() - startedMs >
            CORNER_IGNORE_TIME_MS) {
            if (phat_hien_mau_goc_trai_90()) {
                ++confirmCount;

                if (confirmCount >=
                    SENSOR_CONFIRM_COUNT) {
                    dung_xe_ngay();
                    return;
                }
            } else {
                confirmCount = 0;
            }
        }

        applyLineControlFromCurrentFrame(
            toc_do,
            dang_cho_phep_cuu_ho
        );
    }

    dung_xe_ngay();
}


// ============================================================================
// 12. XOAY
// ============================================================================

void xoay_tai_cho_pid(int huong) {
    const int8_t direction =
        huong == 1 ? 1 : -1;

    // ------------------------------------------------------------------------
    // GIAI ĐOẠN 1: Thoát vạch hiện tại.
    // ------------------------------------------------------------------------

    const uint32_t escapeStartedMs = millis();

    while (millis() - escapeStartedMs < 1500UL) {
        rd();

        if (direction > 0) {
            quay_phai_truc_tiep(60.0f);

            if (filteredSensor[6] < 250.0f &&
                filteredSensor[7] < 250.0f) {
                break;
            }
        } else {
            quay_trai_truc_tiep(60.0f);

            if (filteredSensor[0] < 250.0f &&
                filteredSensor[1] < 250.0f) {
                break;
            }
        }

        cap_nhat_pid_dong_co();
    }

    // ------------------------------------------------------------------------
    // GIAI ĐOẠN 2: Quét cạnh line mới.
    // Trình tự: trắng -> đen -> trắng + mắt giữa thấy line.
    // ------------------------------------------------------------------------

    uint8_t scanStage = 0;
    uint8_t stableCount = 0;
    bool previousOuterBlack = false;

    const uint32_t scanStartedMs = millis();

    while (scanStage < 3 &&
           millis() - scanStartedMs < 3500UL) {
        rd();

        bool outerBlack;

        if (direction > 0) {
            quay_phai_truc_tiep(55.0f);

            outerBlack =
                filteredSensor[6] >= 450.0f ||
                filteredSensor[7] >= 450.0f;
        } else {
            quay_trai_truc_tiep(55.0f);

            outerBlack =
                filteredSensor[0] >= 450.0f ||
                filteredSensor[1] >= 450.0f;
        }

        const bool centerBlack =
            filteredSensor[3] >= 400.0f ||
            filteredSensor[4] >= 400.0f;

        cap_nhat_pid_dong_co();

        if (outerBlack == previousOuterBlack) {
            if (stableCount < 255) {
                ++stableCount;
            }
        } else {
            previousOuterBlack = outerBlack;
            stableCount = 1;
        }

        if (stableCount < SENSOR_CONFIRM_COUNT) {
            continue;
        }

        if (scanStage == 0 && !outerBlack) {
            scanStage = 1;
            stableCount = 0;
        } else if (scanStage == 1 && outerBlack) {
            scanStage = 2;
            stableCount = 0;
        } else if (
            scanStage == 2 &&
            !outerBlack &&
            centerBlack
        ) {
            scanStage = 3;
        }
    }

    // ------------------------------------------------------------------------
    // GIAI ĐOẠN 3: PID hút tâm line.
    // ------------------------------------------------------------------------

    constexpr float TURN_ERROR_ALPHA = 0.35f;
    constexpr float TURN_DEAD_ZONE = 8.0f;
    constexpr float TURN_MIN_RPM = 45.0f;
    constexpr float TURN_MAX_RPM = 65.0f;
    constexpr uint8_t TURN_CENTER_CONFIRM = 3;

    rd();
    tinh_sai_so_line(lineError);

    float previousError =
        clampFloat(
            lineError / LINE_ERROR_MAX,
            -1.0f,
            1.0f
        );

    float filteredError = previousError;
    float turnDerivative = 0.0f;

    uint8_t centeredCount = 0;
    uint32_t lastTimeUs = micros();
    const uint32_t pidStartedMs = millis();

    while (millis() - pidStartedMs < 1200UL) {
        rd();
        tinh_sai_so_line(lineError);

        if (trangThaiLineHienTai == LINE_LOST) {
            if (direction > 0) {
                quay_phai_truc_tiep(TURN_MIN_RPM);
            } else {
                quay_trai_truc_tiep(TURN_MIN_RPM);
            }

            cap_nhat_pid_dong_co();
            continue;
        }

        int32_t signalStrength = 0;

        for (uint8_t i = 0; i < SENSOR_COUNT; ++i) {
            int16_t value =
                static_cast<int16_t>(
                    filteredSensor[i] + 0.5f
                ) -
                SENSOR_NOISE_FLOOR;

            if (value > 0) {
                signalStrength += value;
            }
        }

        if (fabs(lineError) <= 35.0f &&
            signalStrength >= 60) {
            ++centeredCount;

            if (centeredCount >=
                TURN_CENTER_CONFIRM) {
                break;
            }
        } else {
            centeredCount = 0;
        }

        const uint32_t nowUs = micros();

        float dtSeconds =
            static_cast<float>(nowUs - lastTimeUs) /
            1000000.0f;

        lastTimeUs = nowUs;
        dtSeconds =
            clampFloat(dtSeconds, 0.005f, 0.050f);

        const float normalizedError =
            clampFloat(
                lineError / LINE_ERROR_MAX,
                -1.0f,
                1.0f
            );

        filteredError +=
            TURN_ERROR_ALPHA *
            (normalizedError - filteredError);

        const float rawDerivative =
            (filteredError - previousError) /
            dtSeconds;

        turnDerivative +=
            0.40f *
            (rawDerivative - turnDerivative);

        previousError = filteredError;

        float correction =
            80.0f * filteredError +
            2.5f * turnDerivative;

        if (fabs(correction) < TURN_DEAD_ZONE) {
            correction = 0.0f;
        } else if (
            correction > 0.0f &&
            correction < TURN_MIN_RPM
        ) {
            correction = TURN_MIN_RPM;
        } else if (
            correction < 0.0f &&
            correction > -TURN_MIN_RPM
        ) {
            correction = -TURN_MIN_RPM;
        }

        correction =
            clampFloat(
                correction,
                -TURN_MAX_RPM,
                TURN_MAX_RPM
            );

        setTargetRPM(
            -correction,
            -correction,
             correction,
             correction
        );

        cap_nhat_pid_dong_co();
    }

    dung_xe_ngay();
    delay_pid(50);
}

void xoay_thong_minh_pid(int huong) {
    tien_khoang_cach(40.0f, 110);
    delay_pid(150);
    xoay_tai_cho_pid(huong);
}

void xoay_thong_minh(int huong) {
    xoay_thong_minh_pid(huong);
}

void xoay(int huong, int speed_nhanh, int speed_cham) {
    const int8_t direction =
        huong == 1 ? 1 : -1;

    const float fastRPM =
        fabs(static_cast<float>(speed_nhanh));

    const float slowRPM =
        fabs(static_cast<float>(speed_cham));

    const uint32_t escapeStartedMs = millis();

    while (millis() - escapeStartedMs < 1200UL) {
        rd();

        if (direction > 0) {
            quay_phai_truc_tiep(fastRPM);

            if (filteredSensor[6] < 250.0f &&
                filteredSensor[7] < 250.0f) {
                break;
            }
        } else {
            quay_trai_truc_tiep(fastRPM);

            if (filteredSensor[0] < 250.0f &&
                filteredSensor[1] < 250.0f) {
                break;
            }
        }

        cap_nhat_pid_dong_co();
    }

    uint8_t confirmCount = 0;
    const uint32_t findStartedMs = millis();

    while (millis() - findStartedMs <
           FIND_LINE_TIMEOUT_MS) {
        rd();

        const bool centerOnLine =
            filteredSensor[3] >= 400.0f ||
            filteredSensor[4] >= 400.0f;

        if (centerOnLine) {
            ++confirmCount;

            if (confirmCount >=
                SENSOR_CONFIRM_COUNT) {
                break;
            }
        } else {
            confirmCount = 0;
        }

        if (direction > 0) {
            quay_phai_truc_tiep(slowRPM);
        } else {
            quay_trai_truc_tiep(slowRPM);
        }

        cap_nhat_pid_dong_co();
    }

    dung_xe_ngay();
}


// ============================================================================
// 13. CHẠY NGANG MECANUM
// ============================================================================

void chayngang(int huong, float rpm) {
    const float speed = fabs(rpm);

    if (huong < 0) {
        // Trượt trái:
        // RF tiến, RR lùi, LF lùi, LR tiến.
        setTargetRPM(
             speed,
            -speed,
            -speed,
             speed
        );
    } else if (huong > 0) {
        // Trượt phải:
        // RF lùi, RR tiến, LF tiến, LR lùi.
        setTargetRPM(
            -speed,
             speed,
             speed,
            -speed
        );
    } else {
        setTargetRPM(0.0f, 0.0f, 0.0f, 0.0f);
    }
}

// Chi dung sau khi xe da ha ve STRAFE_SLOW_RPM.
// Toc do giam deu ve 0, sau do moi phanh giu trong luc xe gan nhu da dung.
static void dung_chay_ngang_tu_tu(
    int huong,
    float tocDoBatDau
) {
    const float startRPM = fabs(tocDoBatDau);
    const uint32_t startedMs = millis();

    while (millis() - startedMs < STRAFE_DECEL_TIME_MS) {
        const uint32_t elapsedMs = millis() - startedMs;

        const float remainingRatio =
            1.0f -
            static_cast<float>(elapsedMs) /
            static_cast<float>(STRAFE_DECEL_TIME_MS);

        chayngang(huong, startRPM * remainingRatio);
        cap_nhat_pid_dong_co();
    }

    setTargetRPM(0.0f, 0.0f, 0.0f, 0.0f);

    // Cho target RPM trong bo dieu khien ve gan 0 roi moi phanh giu.
    delay_pid(60);
    dung_xe_ngay();
}

void truot_ngang_phai_tim_vach(float toc_do) {
    const float fastRPM = fabs(toc_do);
    const float slowRPM =
        fastRPM < STRAFE_SLOW_RPM
            ? fastRPM
            : STRAFE_SLOW_RPM;

    const uint32_t blindStartedMs = millis();

    while (millis() - blindStartedMs < 1000UL) {
        chayngang(1, fastRPM);
        cap_nhat_pid_dong_co();

        // Giữ filter sensor nóng nhưng không kiểm tra line ở đoạn mù.
        rd();
    }

    uint8_t confirmCount = 0;
    bool slowingDown = false;
    uint32_t slowStartedMs = 0;
    const uint32_t findStartedMs = millis();

    while (millis() - findStartedMs <
           FIND_LINE_TIMEOUT_MS) {
        const float currentRPM =
            slowingDown ? slowRPM : fastRPM;

        chayngang(1, currentRPM);
        cap_nhat_pid_dong_co();

        rd();

        if (!slowingDown &&
            filteredSensor[3] >=
                STRAFE_SLOW_SENSOR_THRESHOLD) {
            slowingDown = true;
            slowStartedMs = millis();
        }

        if (filteredSensor[3] >=
                SENSOR_LINE_THRESHOLD &&
            slowingDown &&
            millis() - slowStartedMs >=
                STRAFE_MIN_SLOW_TIME_MS) {
            ++confirmCount;

            if (confirmCount >=
                SENSOR_CONFIRM_COUNT) {
                dung_chay_ngang_tu_tu(1, slowRPM);
                return;
            }
        } else {
            confirmCount = 0;
        }
    }

    dung_xe_ngay();
}

void truot_ngang_trai_tim_vach(float toc_do) {
    const float fastRPM = fabs(toc_do);
    const float slowRPM =
        fastRPM < STRAFE_SLOW_RPM
            ? fastRPM
            : STRAFE_SLOW_RPM;

    const uint32_t blindStartedMs = millis();

    while (millis() - blindStartedMs < 700UL) {
        chayngang(-1, fastRPM);
        cap_nhat_pid_dong_co();
        rd();
    }

    uint8_t confirmCount = 0;
    bool slowingDown = false;
    uint32_t slowStartedMs = 0;
    const uint32_t findStartedMs = millis();

    while (millis() - findStartedMs <
           FIND_LINE_TIMEOUT_MS) {
        const float currentRPM =
            slowingDown ? slowRPM : fastRPM;

        chayngang(-1, currentRPM);
        cap_nhat_pid_dong_co();

        rd();

        if (!slowingDown &&
            filteredSensor[6] >=
                STRAFE_SLOW_SENSOR_THRESHOLD) {
            slowingDown = true;
            slowStartedMs = millis();
        }

        if (filteredSensor[6] >=
                SENSOR_LINE_THRESHOLD &&
            slowingDown &&
            millis() - slowStartedMs >=
                STRAFE_MIN_SLOW_TIME_MS) {
            ++confirmCount;

            if (confirmCount >=
                SENSOR_CONFIRM_COUNT) {
                dung_chay_ngang_tu_tu(-1, slowRPM);
                return;
            }
        } else {
            confirmCount = 0;
        }
    }

    dung_xe_ngay();
}


// ============================================================================
// 14. BÁM LINE ĐẾN KHI MẤT VẠCH
// ============================================================================

void bam_line_den_khi_mat_vach(float toc_do) {
    dat_lai_bo_bam_line();

    dang_cho_phep_cuu_ho = false;

    const uint32_t startedMs = millis();

    while (millis() - startedMs <
           FIND_INTERSECTION_TIMEOUT_MS) {
        cap_nhat_pid_dong_co();

        rd();
        tinh_sai_so_line(lineError);

        if (trangThaiLineHienTai == LINE_LOST) {
            dung_xe_ngay();
            dang_cho_phep_cuu_ho = true;
            return;
        }

        if (trangThaiLineHienTai == LINE_ALL_BLACK ||
            m7 ||
            m8) {
            tien_truc_tiep(toc_do);
        } else {
            applyLineControlFromCurrentFrame(
                toc_do,
                false
            );
        }
    }

    dung_xe_ngay();
    dang_cho_phep_cuu_ho = true;
}
