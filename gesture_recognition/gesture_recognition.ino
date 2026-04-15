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
 *   VERTICAL_UP    ↑    ръката се вдига (разстоянието расте)
 *   VERTICAL_DOWN  ↓    ръката се снишава (разстоянието намалява)
 *
 * Диагонал: определя се от минималното разстояние на LEFT и RIGHT
 * по време на целия swipe (не само при начало).
 * По-малко разстояние = ръката е ниско/близо до сензора.
 * По-голямо разстояние = ръката е нагоре/далеч.
 *
 * Вертикал: засича се когато swipe-ът изтече, но ръката е все
 * още над сензорите. Следи се най-близкото разстояние от всичките
 * три сензора, тъй като ръката не може да покрие всичките три
 * едновременно (физически разстояния между тях).
 */

#include <Arduino.h>
#include <Wire.h>
#include <vl53l0x_x_nucleo_53l0a1_class.h>
#include <stmpe1600_class.h>
#include <tof_gestures.h>
#include <tof_gestures_DIRSWIPE_1.h>

#include "secrets.h"  // WIFI_SSID, WIFI_PASS, TV_IP, TV_PSK — not committed

// ── ESP01 WiFi ───────────────────────────────────────────────────
#define ESP_EN    PA8

HardwareSerial Serial1(PA10, PA9); // RX=PA10, TX=PA9

void sendAT(const char* cmd, unsigned long timeout = 3000);
static bool readUntil(char prompt, unsigned long timeout);
void httpPost(const char* host, const char* path, const char* psk, const char* body);
void tvPower(bool on);
void tvMute(bool mute);
void tvVolume(int delta);
void wakeTV();

// ── STMPE1600 GPIO expander-и ────────────────────────────────────
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
uint32_t distance_left = 1200, distance_center = 1200, distance_right = 1200;

// ── Параметри ────────────────────────────────────────────────────
#define HAND_PRESENT_MM      400   // под това = ръка пред сензора
#define DIAG_THRESHOLD_MM     40   // мин. разлика minL/minR за диагонал
#define SWIPE_TIMEOUT_MS    1200   // изчакване за ST lib swipe (> 1000ms прозорец)
#define VERTICAL_TIMEOUT_MS 2000   // макс. престой в VERTICAL_CHECK
#define VERTICAL_THRESHOLD_MM 30   // мин. промяна на разстоянието за вертикал
#define COOLDOWN_TIMEOUT_MS  500   // макс. време в COOLDOWN преди принудителен IDLE
#define HAND_DETECT_FRAMES     3   // последователни кадри за потвърждение на ръка
#define SENSOR_READ_TIMEOUT_MS 100 // макс. изчакване за четене на всичките сензори
#define DISTANCE_NONE_MM      1200 // стойност при липса/невалидно четене

// ── State machine ─────────────────────────────────────────────────
enum State { IDLE, SWIPE_DETECT, VERTICAL_CHECK, COOLDOWN };
State state = IDLE;

// ── Move ─────────────────────────────────────────────────
enum Move {
  MOVE_NONE,
  SWIPE_RIGHT,
  SWIPE_LEFT,
  SWIPE_DIAG_UR,
  SWIPE_DIAG_DR,
  SWIPE_DIAG_UL,
  SWIPE_DIAG_DL,
  VERTICAL_UP,
  VERTICAL_DOWN
};
Move currentMove = MOVE_NONE;

// ── Action ─────────────────────────────────────────────────
enum Action {ACTION_NONE, ACTION_TV, ACTION_LIGHT};
Action current_action = ACTION_NONE;

// ── Swipe tracking ────────────────────────────────────────────────
uint32_t minL, minR;          // мин. разстояние от всяка страна по време на swipe
unsigned long swipeStartTime;

// ── Vertical tracking ─────────────────────────────────────────────
uint32_t vertStartDist;       // разстояние при влизане в VERTICAL_CHECK
uint32_t vertLastDist;        // последното валидно "closest" разстояние
unsigned long vertStartTime;
unsigned long cooldownStartTime;
uint8_t handDetectCount = 0;      // брой последователни кадри с ръка (debounce)

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
// Четене на всичките три сензора паралелно
// ════════════════════════════════════════════════════════════════
void readAllSensors() {
  struct Entry {
    VL53L0X_X_NUCLEO_53L0A1 *sensor;
    uint32_t *distance;
    bool done;
  };
  Entry entries[3] = {
    {&sensor_left,   &distance_left,   false},
    {&sensor_right,  &distance_right,  false},
    {&sensor_center, &distance_center, false},
  };

  for (auto &e : entries) e.sensor->StartMeasurement();

  VL53L0X_RangingMeasurementData_t data;
  unsigned long start = millis();
  int remaining = 3;

  while (remaining > 0) {
    // Прекъсни ако някой сензор е заклещен — така loop() не увисва завинаги
    if (millis() - start > SENSOR_READ_TIMEOUT_MS) {
      for (auto &e : entries) {
        if (!e.done) *e.distance = DISTANCE_NONE_MM;
      }
      return;
    }
    for (auto &e : entries) {
      if (e.done) continue;
      uint8_t ready = 0;
      e.sensor->GetMeasurementDataReady(&ready);
      if (ready) {
        e.sensor->ClearInterruptMask(0);
        e.sensor->GetRangingMeasurementData(&data);
        *e.distance = (data.RangeStatus == 0) ? data.RangeMilliMeter : DISTANCE_NONE_MM;
        e.done = true;
        remaining--;
      }
    }
  }
}

// ════════════════════════════════════════════════════════════════
// Проверка дали ръката е видима от поне един сензор
// ════════════════════════════════════════════════════════════════
bool handPresent() {
  // CENTER сензорът вижда платката и не може да се ползва за детекция
  return (distance_left  < HAND_PRESENT_MM)
      || (distance_right < HAND_PRESENT_MM);
}

// ════════════════════════════════════════════════════════════════
// Най-близкото разстояние от сензорите, които виждат ръката
// ════════════════════════════════════════════════════════════════
uint32_t closestVisible() {
  // CENTER сензорът вижда платката — ползваме само LEFT и RIGHT
  uint32_t best = 1200;
  if (distance_left  < HAND_PRESENT_MM) best = min(best, distance_left);
  if (distance_right < HAND_PRESENT_MM) best = min(best, distance_right);
  return best;
}

// ════════════════════════════════════════════════════════════════
// Класификация и принтиране на swipe жест
// ════════════════════════════════════════════════════════════════
/*
 * Диагонална логика (от минималните разстояния по цялото движение):
 *   L→R: minL < minR → ↗ UR  |  minL > minR → ↘ DR
 *   R→L: minR < minL → ↖ UL  |  minR > minL → ↙ DL
 */
void classifySwipe(int gesture_code) {
  bool diag = (minL < HAND_PRESENT_MM)
           && (minR < HAND_PRESENT_MM)
           && ((uint32_t)abs((int32_t)minL - (int32_t)minR) > DIAG_THRESHOLD_MM);

  Serial.print(">>> ");
  if (gesture_code == GESTURES_SWIPE_LEFT_RIGHT) {
    if (!diag)            { currentMove = SWIPE_RIGHT;    Serial.println("SWIPE_RIGHT    ←→"); }
    else if (minL < minR) { currentMove = SWIPE_DIAG_UR;  Serial.println("SWIPE_DIAG_UR  ↗  долу-ляво → горе-дясно"); }
    else                  { currentMove = SWIPE_DIAG_DR;  Serial.println("SWIPE_DIAG_DR  ↘  горе-ляво → долу-дясно"); }
  } else {
    if (!diag)            { currentMove = SWIPE_LEFT;     Serial.println("SWIPE_LEFT     →←"); }
    else if (minR < minL) { currentMove = SWIPE_DIAG_UL;  Serial.println("SWIPE_DIAG_UL  ↖  долу-дясно → горе-ляво"); }
    else                  { currentMove = SWIPE_DIAG_DL;  Serial.println("SWIPE_DIAG_DL  ↙  горе-дясно → долу-ляво"); }
  }
}

// ════════════════════════════════════════════════════════════════
void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n=== HandFlow Gesture Control ===");

  Wire.begin();
  Wire.setClock(400000);

  // Изгаси всички сензори
  sensor_center.begin(); sensor_center.VL53L0X_Off();
  sensor_left.begin();   sensor_left.VL53L0X_Off();
  sensor_right.begin();  sensor_right.VL53L0X_Off();

  // Remap адреси
  if (sensor_left.InitSensor(0x12))
    Serial.println("ERR: LEFT");
  else
    Serial.println("LEFT   OK @ 0x12");

  if (sensor_right.InitSensor(0x14))
    Serial.println("ERR: RIGHT");
  else
    Serial.println("RIGHT  OK @ 0x14");

  if (sensor_center.InitSensor(0x16))
    Serial.println("ERR: CENTER");
  else
    Serial.println("CENTER OK @ 0x16");

  // ST gesture lib: maxDist=400mm, minDist=0, window=1000ms
  tof_gestures_initDIRSWIPE_1(400, 0, 1000, &gestureDirSwipeData);

  setupSingleShot();

  // Изчисти остатъчни прекъсвания от калибрацията
  sensor_left.ClearInterruptMask(0);
  sensor_right.ClearInterruptMask(0);
  sensor_center.ClearInterruptMask(0);

  // Едно "загряващо" четене за изчистване на евентуални стари данни
  readAllSensors();

  // ── ESP01 WiFi init ─────────────────────────────────────────────
  pinMode(ESP_EN, OUTPUT);
  digitalWrite(ESP_EN, HIGH);
  Serial1.begin(115200);
  delay(1000);
  sendAT("AT");
  sendAT("AT+CWMODE=1");
  sendAT("AT+CWJAP=\"" WIFI_SSID "\",\"" WIFI_PASS "\"", 10000);
  Serial.println("WiFi ready.");
  Serial.println("Go!");

  Serial.println("\nReady.");
}

// ════════════════════════════════════════════════════════════════
void loop() {
  readAllSensors();

  // Винаги подаваме на ST lib (нужда от непрекъснати данни)
  int gesture_code = tof_gestures_detectDIRSWIPE_1(
      distance_left, distance_right, &gestureDirSwipeData);

  switch (state) {

    // ── IDLE ──────────────────────────────────────────────────────
    case IDLE:
      if (handPresent()) {
        handDetectCount++;
        Serial.print("DETECT L="); Serial.print(distance_left);
        Serial.print(" C="); Serial.print(distance_center);
        Serial.print(" R="); Serial.print(distance_right);
        Serial.print(" #"); Serial.println(handDetectCount);
        if (handDetectCount >= HAND_DETECT_FRAMES) {
          state = SWIPE_DETECT;
          swipeStartTime = millis();
          minL = distance_left;
          minR = distance_right;
          handDetectCount = 0;
        }
      } else {
        handDetectCount = 0;
      }
      break;

    // ── SWIPE_DETECT ──────────────────────────────────────────────
    case SWIPE_DETECT:
      // Обновявай минималните разстояния докато ръката е видима
      if (distance_left  < HAND_PRESENT_MM) minL = min(minL, distance_left);
      if (distance_right < HAND_PRESENT_MM) minR = min(minR, distance_right);

      // ST lib засечe swipe?
      if (gesture_code == GESTURES_SWIPE_LEFT_RIGHT ||
          gesture_code == GESTURES_SWIPE_RIGHT_LEFT) {
        classifySwipe(gesture_code);
        state = COOLDOWN;
        cooldownStartTime = millis();
        break;
      }

      // Изтекъл ли е прозорецът?
      if (millis() - swipeStartTime > SWIPE_TIMEOUT_MS) {
        if (handPresent()) {
          // Ръката е все още тук → проверяваме за вертикал
          state = VERTICAL_CHECK;
          vertStartTime = millis();
          vertStartDist = closestVisible();
          vertLastDist  = vertStartDist;
        } else {
          // Ръката е напуснала без swipe → нищо
          state = IDLE;
        }
      }
      break;

    // ── VERTICAL_CHECK ────────────────────────────────────────────
    case VERTICAL_CHECK: {
      if (handPresent()) {
        vertLastDist = closestVisible();
      }

      bool timedOut = (millis() - vertStartTime > VERTICAL_TIMEOUT_MS);
      bool handGone = !handPresent();

      if (handGone || timedOut) {
        int32_t delta = (int32_t)vertLastDist - (int32_t)vertStartDist;
        if (delta > (int32_t)VERTICAL_THRESHOLD_MM) {
          currentMove = VERTICAL_UP;
          Serial.println(">>> VERTICAL_UP    ↑  ръката се вдигна");
        } else if (delta < -(int32_t)VERTICAL_THRESHOLD_MM) {
          currentMove = VERTICAL_DOWN;
          Serial.println(">>> VERTICAL_DOWN  ↓  ръката се сниши");
        }
        // Ако delta е в рамките на прага — без жест (само задържане)
        state = COOLDOWN;
        cooldownStartTime = millis();
      }
      break;
    }

    // ── COOLDOWN ──────────────────────────────────────────────────
    case COOLDOWN:
      // Изчакай докато ръката напусне ИЛИ изтече таймаутът
      if (!handPresent() || millis() - cooldownStartTime > COOLDOWN_TIMEOUT_MS) {
        switch (currentMove) {
          case SWIPE_RIGHT:
            switch (current_action) {
              case ACTION_NONE: current_action = ACTION_TV;   Serial.println("Action: TV"); break;
              case ACTION_TV:   tvPower(true);                break;
              case ACTION_LIGHT:  break;
            }
            break;
          case SWIPE_LEFT:
            switch (current_action) {
              case ACTION_NONE:   break;
              case ACTION_TV:   tvPower(false);               break;
              case ACTION_LIGHT:  break;
            }
            break;
          case SWIPE_DIAG_UR:
            switch (current_action) {
              case ACTION_NONE:   break;
              case ACTION_TV:   tvMute(true);                 break;
              case ACTION_LIGHT:  break;
            }
            break;
          case SWIPE_DIAG_DR:
            switch (current_action) {
              case ACTION_NONE:   break;
              case ACTION_TV:   tvMute(false);                break;
              case ACTION_LIGHT:  break;
            }
            break;
          case SWIPE_DIAG_UL:
            switch (current_action) {
              case ACTION_NONE:   break;
              case ACTION_TV:     break;
              case ACTION_LIGHT:  break;
            }
            break;
          case SWIPE_DIAG_DL:
            current_action = ACTION_NONE;
            Serial.println("Action: NONE");
            break;
          case VERTICAL_UP:
            switch (current_action) {
              case ACTION_NONE:   break;
              case ACTION_TV:   tvVolume(+5);                 break;
              case ACTION_LIGHT:  break;
            }
            break;
          case VERTICAL_DOWN:
            switch (current_action) {
              case ACTION_NONE:   break;
              case ACTION_TV:   tvVolume(-5);                 break;
              case ACTION_LIGHT:  break;
            }
            break;
          case MOVE_NONE:
            break;
        }
        currentMove = MOVE_NONE;
        // Нулирай ST lib — данните от движението при вдигане на ръката се изхвърлят
        tof_gestures_initDIRSWIPE_1(400, 0, 1000, &gestureDirSwipeData);
        state = IDLE;
        Serial.println("Ready.");
      }
      break;
  }
}

// ════════════════════════════════════════════════════════════════
// ESP01 WiFi helpers
// ════════════════════════════════════════════════════════════════
void tvPower(bool on) {
  Serial.println(on ? "TV ON" : "TV OFF");
  if (on) {
    wakeTV();
  } else {
    const char* body = "{\"method\":\"setPowerStatus\",\"params\":[{\"status\":false}],\"id\":55,\"version\":\"1.0\"}";
    httpPost(TV_IP, "/sony/system", TV_PSK, body);
  }
}

void tvMute(bool mute) {
  Serial.println(mute ? "TV MUTE" : "TV UNMUTE");
  char body[128];
  snprintf(body, sizeof(body),
    "{\"method\":\"setAudioMute\",\"params\":[{\"status\":%s}],\"id\":601,\"version\":\"1.0\"}",
    mute ? "true" : "false");
  httpPost(TV_IP, "/sony/audio", TV_PSK, body);
}

void wakeTV() {
  uint8_t mac[6] = { TV_MAC_BYTES };
  uint8_t packet[102];
  for (int i = 0; i < 6; i++) packet[i] = 0xFF;
  for (int i = 0; i < 16; i++)
    for (int j = 0; j < 6; j++)
      packet[6 + i*6 + j] = mac[j];

  sendAT("AT+CIPSTART=\"UDP\",\"" WOL_BROADCAST "\",9", 3000);

  Serial1.println("AT+CIPSEND=102");
  if (!readUntil('>', 3000)) {
    sendAT("AT+CIPCLOSE", 1000);
    return;
  }

  Serial1.write(packet, 102);
  readUntil(0, 3000);

  sendAT("AT+CIPCLOSE", 2000);
}

void httpPost(const char* host, const char* path, const char* psk, const char* body) {
  char req[512];
  int reqLen = snprintf(req, sizeof(req),
    "POST %s HTTP/1.1\r\n"
    "Host: %s\r\n"
    "Content-Type: application/json\r\n"
    "X-Auth-PSK: %s\r\n"
    "Content-Length: %u\r\n"
    "Connection: close\r\n\r\n"
    "%s",
    path, host, psk, (unsigned)strlen(body), body);
  if (reqLen <= 0 || reqLen >= (int)sizeof(req)) {
    Serial.println("httpPost: request too large");
    return;
  }

  char cmd[64];
  snprintf(cmd, sizeof(cmd), "AT+CIPSTART=\"TCP\",\"%s\",80", host);
  sendAT(cmd, 5000);

  snprintf(cmd, sizeof(cmd), "AT+CIPSEND=%d", reqLen);
  Serial1.println(cmd);

  // Чакай '>' преди изпращане; ако ESP върне ERROR или не отговори — излез
  if (!readUntil('>', 3000)) {
    sendAT("AT+CIPCLOSE", 1000);
    return;
  }

  Serial1.write((const uint8_t*)req, reqLen);
  readUntil(0, 5000);

  sendAT("AT+CIPCLOSE", 2000);
}

void tvVolume(int delta) {
  Serial.print("Volume "); Serial.println(delta > 0 ? "+5" : "-5");
  char body[100];
  snprintf(body, sizeof(body),
    "{\"method\":\"setAudioVolume\",\"params\":[{\"volume\":\"%+d\"}],\"id\":601,\"version\":\"1.0\"}",
    delta);
  httpPost(TV_IP, "/sony/audio", TV_PSK, body);
}

// Чете от Serial1 докато: (1) види "OK\r\n" или "ERROR\r\n" и излезе рано,
// (2) види очаквания prompt символ (ако е != 0), или (3) изтече timeout.
// Връща true ако срещне OK или prompt; false при ERROR/timeout.
static bool readUntil(char prompt, unsigned long timeout) {
  unsigned long t = millis();
  // Малък rolling буфер, за да разпознаем "OK\r\n" / "ERROR\r\n"
  char tail[8] = {0};
  uint8_t tlen = 0;
  while (millis() - t < timeout) {
    while (Serial1.available()) {
      char c = Serial1.read();
      Serial.write(c);
      if (prompt && c == prompt) return true;
      if (tlen == sizeof(tail)) {
        memmove(tail, tail + 1, sizeof(tail) - 1);
        tlen--;
      }
      tail[tlen++] = c;
      if (tlen >= 4 && !memcmp(tail + tlen - 4, "OK\r\n", 4))    return true;
      if (tlen >= 7 && !memcmp(tail + tlen - 7, "ERROR\r\n", 7)) return false;
    }
  }
  return false;
}

void sendAT(const char* cmd, unsigned long timeout) {
  Serial1.println(cmd);
  readUntil(0, timeout);
}
