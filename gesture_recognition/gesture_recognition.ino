/*
 * HandFlow Gesture Control
 * STM32F401 Nucleo-64 + X-Nucleo-53L0A1
 *
 * Библиотеки (Library Manager):
 *   - "X_NUCLEO_53L0A1" от STMicroelectronics
 *     (включва stmpe1600_class, vl53l0x_x_nucleo_53l0a1_class,
 *      tof_gestures, tof_gestures_DIRSWIPE_1)
 *
 * Физическо наредяне: [LEFT] [CENTER] [RIGHT]
 *
 * Жестове:
 *   SWIPE_RIGHT    ←→   ляво → дясно
 *   SWIPE_LEFT     →←   дясно → ляво
 *   SWIPE_DIAG_UR  ↗    долу-ляво → горе-дясно
 *   SWIPE_DIAG_DR  ↘    горе-ляво → долу-дясно
 *   SWIPE_DIAG_UL  ↖    долу-дясно → горе-ляво
 *   SWIPE_DIAG_DL  ↙    горе-дясно → долу-ляво
 *
 * Диагонал: определя се от разликата в разстоянието между
 * LEFT и RIGHT в момента на начало на жеста.
 * Малко разстояние = ръката е ниско/близо до сензора.
 * Голямо разстояние = ръката е нагоре/далеч.
 */

#include <Arduino.h>
#include <Wire.h>
#include <vl53l0x_x_nucleo_53l0a1_class.h>
#include <stmpe1600_class.h>
#include <tof_gestures.h>
#include <tof_gestures_DIRSWIPE_1.h>

#define SerialPort Serial

// ── STMPE1600 GPIO expander-и ────────────────────────────────────
// Адресите са × 2 защото ST библиотеката очаква 8-bit формат
// U19 @ 0x42 → GPIO_15 = XSHUT CENTER
// U21 @ 0x43 → GPIO_14 = XSHUT LEFT
//            → GPIO_15 = XSHUT RIGHT
STMPE1600DigiOut xshutdown_center(&Wire, GPIO_15, (0x42 * 2));
STMPE1600DigiOut xshutdown_left  (&Wire, GPIO_14, (0x43 * 2));
STMPE1600DigiOut xshutdown_right (&Wire, GPIO_15, (0x43 * 2));

// ── VL53L0X сензори ──────────────────────────────────────────────
VL53L0X_X_NUCLEO_53L0A1 sensor_center(&Wire, &xshutdown_center);
VL53L0X_X_NUCLEO_53L0A1 sensor_left  (&Wire, &xshutdown_left);
VL53L0X_X_NUCLEO_53L0A1 sensor_right (&Wire, &xshutdown_right);

// ── ST gesture структура ─────────────────────────────────────────
Gesture_DIRSWIPE_1_Data_t gestureDirSwipeData;

// ── Разстояния ───────────────────────────────────────────────────
uint32_t distance_left, distance_right;

// ── Диагонален snapshot ──────────────────────────────────────────
// Записваме L/R разстоянията при ПЪРВОТО засичане на ръката,
// за да определим наклона на жеста след като ST lib го потвърди.
#define DIAG_THRESHOLD_MM  20   // мин. разлика за диагонал
#define HAND_PRESENT_MM   400   // под това = ръка пред сензора

uint32_t snapL = 0, snapR = 0;
bool     snapshotTaken = false;

// ════════════════════════════════════════════════════════════════
// Single-shot setup — идентично с официалния ST пример
// ════════════════════════════════════════════════════════════════
void setupSingleShot() {
  uint8_t VhvSettings, PhaseCal;
  uint32_t refSpadCount;
  uint8_t isApertureSpads;

  for (VL53L0X_X_NUCLEO_53L0A1 *s : {&sensor_left, &sensor_right, &sensor_center}) {
    s->StaticInit();
    s->PerformRefCalibration(&VhvSettings, &PhaseCal);
    s->PerformRefSpadManagement(&refSpadCount, &isApertureSpads);
    s->SetDeviceMode(VL53L0X_DEVICEMODE_SINGLE_RANGING);
    s->SetMeasurementTimingBudgetMicroSeconds(20000);
  }
}

// ════════════════════════════════════════════════════════════════
// Диагонална класификация върху вече разпознат swipe от ST lib
// ════════════════════════════════════════════════════════════════
/*
 *   L→R: snapL < snapR → ↗ UR  |  snapL > snapR → ↘ DR
 *   R→L: snapR < snapL → ↖ UL  |  snapR > snapL → ↙ DL
 */
void printGesture(int gesture_code) {
  bool diag = snapshotTaken
           && (snapL < 1000) && (snapR < 1000)
           && ((uint32_t)abs((int32_t)snapL - (int32_t)snapR) > DIAG_THRESHOLD_MM);

  SerialPort.print(">>> ");
  if (gesture_code == GESTURES_SWIPE_LEFT_RIGHT) {
    if (!diag)           SerialPort.println("SWIPE_RIGHT    ←→");
    else if (snapL < snapR) SerialPort.println("SWIPE_DIAG_UR  ↗  долу-ляво → горе-дясно");
    else                 SerialPort.println("SWIPE_DIAG_DR  ↘  горе-ляво → долу-дясно");
  } else {
    if (!diag)           SerialPort.println("SWIPE_LEFT     →←");
    else if (snapR < snapL) SerialPort.println("SWIPE_DIAG_UL  ↖  долу-дясно → горе-ляво");
    else                 SerialPort.println("SWIPE_DIAG_DL  ↙  горе-дясно → долу-ляво");
  }

  snapshotTaken = false;
}

// ════════════════════════════════════════════════════════════════
void setup() {
  SerialPort.begin(115200);
  delay(500);
  SerialPort.println("\n=== HandFlow Gesture Control ===");

  Wire.begin();
  Wire.setClock(400000);

  // Изгаси всички сензори
  sensor_center.begin(); sensor_center.VL53L0X_Off();
  sensor_left.begin();   sensor_left.VL53L0X_Off();
  sensor_right.begin();  sensor_right.VL53L0X_Off();

  // Remap адреси — идентично с официалния ST пример
  if (sensor_left.InitSensor(0x12))
    SerialPort.println("ERR: LEFT");
  else
    SerialPort.println("LEFT   OK @ 0x12");

  if (sensor_right.InitSensor(0x14))
    SerialPort.println("ERR: RIGHT");
  else
    SerialPort.println("RIGHT  OK @ 0x14");

  if (sensor_center.InitSensor(0x16))
    SerialPort.println("ERR: CENTER");
  else
    SerialPort.println("CENTER OK @ 0x16");

  // ST gesture lib: maxDist=400mm, minDist=0, window=1000ms
  tof_gestures_initDIRSWIPE_1(400, 0, 1000, &gestureDirSwipeData);

  setupSingleShot();
  SerialPort.println("\nГотов. Очаква жест...\n");
}

// ════════════════════════════════════════════════════════════════
void loop() {
  // ── Стартирай LEFT и RIGHT едновременно ─────────────────────
  sensor_left.StartMeasurement();
  sensor_right.StartMeasurement();

  int left_done = 0, right_done = 0;
  uint8_t ready = 0;
  VL53L0X_RangingMeasurementData_t data;

  // ── Чакай и двата паралелно — идентично с ST примера ────────
  do {
    if (!left_done) {
      sensor_left.GetMeasurementDataReady(&ready);
      if (ready) {
        sensor_left.ClearInterruptMask(0);
        sensor_left.GetRangingMeasurementData(&data);
        distance_left  = (data.RangeStatus == 0) ? data.RangeMilliMeter : 1200;
        left_done = 1;
      }
    }
    if (!right_done) {
      ready = 0;
      sensor_right.GetMeasurementDataReady(&ready);
      if (ready) {
        sensor_right.ClearInterruptMask(0);
        sensor_right.GetRangingMeasurementData(&data);
        distance_right = (data.RangeStatus == 0) ? data.RangeMilliMeter : 1200;
        right_done = 1;
      }
    }
  } while (!left_done || !right_done);

  // ── Snapshot при начало на присъствие на ръка ────────────────
  bool handPresent = (distance_left < HAND_PRESENT_MM)
                  || (distance_right < HAND_PRESENT_MM);
  if (handPresent && !snapshotTaken) {
    snapL = distance_left;
    snapR = distance_right;
    snapshotTaken = true;
  }
  if (!handPresent) snapshotTaken = false;

  // ── ST gesture библиотека ────────────────────────────────────
  int gesture_code = tof_gestures_detectDIRSWIPE_1(
      distance_left, distance_right, &gestureDirSwipeData);

  if (gesture_code == GESTURES_SWIPE_LEFT_RIGHT ||
      gesture_code == GESTURES_SWIPE_RIGHT_LEFT) {
    printGesture(gesture_code);
  }
}