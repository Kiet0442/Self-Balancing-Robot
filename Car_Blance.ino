/*
  SELF BALANCE ROBOT - ESP32 + A4988 + MPU6050
  Mode: 1/4 STEP (MS1=LOW, MS2=HIGH, MS3=LOW)
  800 xung/vòng
*/

#include <Wire.h>
#include <MPU6050_tockn.h>

// ===================== CHÂN GPIO ESP32 =====================
#define DIR_L    12
#define STEP_L   14
#define DIR_R    27
#define STEP_R   26
#define ENABLE   13
#define MS1      25
#define MS2      33
#define MS3      32

// ===================== THAM SỐ PID =====================
float Kp = 35.0;
float Ki = 0.0;
float Kd = 5.0;
float I_limit = 30.0;

float Offset = 4.0;   // Đo AngleY khi thẳng đứng rồi đặt = -AngleY_đo_được
float Vgo_L  = 0.0;
float Vgo_R  = 0.0;

// ===================== GIỚI HẠN TỐC ĐỘ MOTOR =====================
//
// Timer ngắt mỗi 20us, Count_BOT = số nhịp trong 1 chu kỳ STEP
//
// Chế độ 1/4 step: 800 xung/vòng
//
// SPEED_MIN = 40  -> chu kỳ STEP = 40 x 20us = 800us
//                    f_step = 1250 Hz
//                    RPM = 1250 / 800 * 60 = 93.75 vòng/phút  ← NHANH NHẤT
//
// SPEED_MAX = 1600 -> chu kỳ STEP = 1600 x 20us = 32000us
//                    f_step = 31.25 Hz
//                    RPM = 31.25 / 800 * 60 = 2.34 vòng/phút  ← CHẬM NHẤT
//
// So sánh với 1/16 step cũ:
//   SPEED_MIN: 10  -> 1500 RPM  (1/16)   vs   40  -> 93.75 RPM (1/4) ← êm hơn, momen tốt hơn
//   SPEED_MAX: 400 -> 37.5 RPM  (1/16)   vs   1600 -> 2.34 RPM (1/4)
//
// Gợi ý thực tế cho robot cân bằng:
//   SPEED_MIN = 40  (nhanh nhất ~94 RPM)
//   SPEED_MAX = 400 (chậm nhất ~9.4 RPM) <- chỉ dùng vùng thấp để phản ứng mượt

#define SPEED_MIN   30    // ← TỐC ĐỘ CAO NHẤT  (~94 RPM ở 1/4 step)
#define SPEED_MAX   120  // ← TỐC ĐỘ THẤP NHẤT (~9.4 RPM ở 1/4 step)

// ===================== BIẾN PID =====================
float inputL, inputR;
float I_L = 0, I_R = 0;
float input_lastL = 0, input_lastR = 0;
float OutputL, OutputR;
float MotorL, MotorR;

// ===================== BIẾN ĐIỀU KHIỂN MOTOR =====================
hw_timer_t   *timer   = NULL;
portMUX_TYPE  timerMux = portMUX_INITIALIZER_UNLOCKED;

volatile int8_t  Dir_M_L = 0,       Dir_M_R = 0;
volatile int16_t Count_timer_L = 0, Count_timer_R = 0;
volatile int16_t Count_TOP_L = 0,   Count_BOT_L = 0;
volatile int16_t Count_TOP_R = 0,   Count_BOT_R = 0;
volatile int32_t Step_L = 0,        Step_R = 0;

// ===================== MPU6050 =====================
MPU6050 mpu6050(Wire);

// ===================== ISR TIMER (20us) =====================
void IRAM_ATTR onTimer() {
  portENTER_CRITICAL_ISR(&timerMux);

  // Motor L
  if (Dir_M_L != 0) {
    Count_timer_L++;
    digitalWrite(STEP_L, (Count_timer_L <= Count_TOP_L) ? HIGH : LOW);
    if (Count_timer_L > Count_BOT_L) {
      Count_timer_L = 0;
      (Dir_M_L > 0) ? Step_L++ : Step_L--;
    }
  } else {
    digitalWrite(STEP_L, LOW);
  }

  // Motor R
  if (Dir_M_R != 0) {
    Count_timer_R++;
    digitalWrite(STEP_R, (Count_timer_R <= Count_TOP_R) ? HIGH : LOW);
    if (Count_timer_R > Count_BOT_R) {
      Count_timer_R = 0;
      (Dir_M_R > 0) ? Step_R++ : Step_R--;
    }
  } else {
    digitalWrite(STEP_R, LOW);
  }

  portEXIT_CRITICAL_ISR(&timerMux);
}

// ===================== ĐẶT TỐC ĐỘ MOTOR =====================
void Speed_L(int16_t x) {
  portENTER_CRITICAL(&timerMux);
  if      (x > 0) { Dir_M_L =  1; digitalWrite(DIR_L, HIGH); }
  else if (x < 0) { Dir_M_L = -1; digitalWrite(DIR_L, LOW);  }
  else             { Dir_M_L =  0; }
  Count_BOT_L = abs(x);
  Count_TOP_L = Count_BOT_L / 2;
  portEXIT_CRITICAL(&timerMux);
}

void Speed_R(int16_t x) {
  portENTER_CRITICAL(&timerMux);
  if      (x > 0) { Dir_M_R =  1; digitalWrite(DIR_R, HIGH); }
  else if (x < 0) { Dir_M_R = -1; digitalWrite(DIR_R, LOW);  }
  else             { Dir_M_R =  0; }
  Count_BOT_R = abs(x);
  Count_TOP_R = Count_BOT_R / 2;
  portEXIT_CRITICAL(&timerMux);
}

// ===================== KHỞI TẠO PIN =====================
void pin_INI() {
  pinMode(ENABLE, OUTPUT);
  pinMode(STEP_L, OUTPUT);
  pinMode(DIR_L,  OUTPUT);
  pinMode(STEP_R, OUTPUT);
  pinMode(DIR_R,  OUTPUT);
  pinMode(MS1,    OUTPUT);
  pinMode(MS2,    OUTPUT);
  pinMode(MS3,    OUTPUT);

  digitalWrite(ENABLE, LOW);   // LOW = enable A4988

  // 1/4 step: MS1=LOW, MS2=HIGH, MS3=LOW
  digitalWrite(MS1, LOW);
  digitalWrite(MS2, HIGH);
  digitalWrite(MS3, LOW);
}

// ===================== KHỞI TẠO TIMER =====================
void timer_INI() {
  timer = timerBegin(0, 80, true);
  timerAttachInterrupt(timer, &onTimer, true);
  timerAlarmWrite(timer, 20, true);   // 20us
  timerAlarmEnable(timer);
}

// ===================== OUTPUT PID -> COUNT_BOT =====================
// |output| trong [8, 400] map -> Count_BOT trong [SPEED_MAX, SPEED_MIN]
// output lớn = robot nghiêng nhiều = cần quay nhanh = Count_BOT nhỏ
int16_t outputToCount(float output) {
  int16_t count = (int16_t)map((long)abs(output), 8, 400, SPEED_MAX, SPEED_MIN);
  return constrain(count, SPEED_MIN, SPEED_MAX);
}

// ===================== SETUP =====================
void setup() {
  Serial.begin(115200);
  pin_INI();

  Wire.begin(21, 22);   // SDA=21, SCL=22
  mpu6050.begin();

  Serial.println("Calibrating MPU6050...");
  mpu6050.calcGyroOffsets(true);
  Serial.println("Done! System Ready. Mode: 1/4 STEP");

  timer_INI();
  delay(500);
}

// ===================== LOOP =====================
void loop() {
  mpu6050.update();
  float AngleY = mpu6050.getAngleY();

  // ---------- PID Motor L ----------
  inputL  = AngleY + Offset - Vgo_L;
  I_L    += inputL;
  I_L     = constrain(I_L, -I_limit, I_limit);
  OutputL = Kp * inputL + Ki * I_L + Kd * (inputL - input_lastL);
  input_lastL = inputL;

  if (OutputL > -8 && OutputL < 8) { OutputL = 0; I_L = 0; }  // vùng chết + reset I
  OutputL = constrain(OutputL, -600, 600);

  // ---------- PID Motor R ----------
  inputR  = AngleY + Offset - Vgo_R;
  I_R    += inputR;
  I_R     = constrain(I_R, -I_limit, I_limit);
  OutputR = Kp * inputR + Ki * I_R + Kd * (inputR - input_lastR);
  input_lastR = inputR;

  if (OutputR > -8 && OutputR < 8) { OutputR = 0; I_R = 0; }
  OutputR = constrain(OutputR, -600, 600);

  // ---------- Output -> tốc độ motor ----------
  MotorL = (OutputL == 0) ? 0 : (OutputL > 0 ?  outputToCount(OutputL)
                                               : -outputToCount(OutputL));
  MotorR = (OutputR == 0) ? 0 : (OutputR > 0 ?  outputToCount(OutputR)
                                               : -outputToCount(OutputR));

  Speed_L(MotorL);
  Speed_R(MotorR);

  // ---------- Debug ----------
  Serial.print(millis());
  Serial.print(" -> Angle: ");  Serial.print(AngleY, 2);
  Serial.print(" | OutL: ");    Serial.print(OutputL, 2);
  Serial.print(" | OutR: ");    Serial.print(OutputR, 2);
  Serial.print(" | MotorL: ");  Serial.print(MotorL);
  Serial.print(" | MotorR: ");  Serial.println(MotorR);
}