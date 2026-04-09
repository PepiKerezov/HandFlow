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

#define SerialPort Serial

// ── ESP01 WiFi ───────────────────────────────────────────────────
#define ESP_EN    PA8
#define WIFI_SSID "Babanin1"
#define WIFI_PASS "babaanin03"
#define TV_IP     "192.168.100.106"
#define TV_PSK    "1234"

HardwareSerial Serial1(PA10, PA9); // RX=PA10, TX=PA9

void sendAT(const char* cmd, unsigned long timeout = 3000);
void httpPost(const char* host, const char* path, const char* psk, const char* body);
String httpPostRead(const char* host, const char* path, const char* psk, const char* body); // used if needed
void sendIRCC(const char* code);
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
// Помощна функция: чете разстоянието на даден сензор (single-shot)
// ════════════════════════════════════════════════════════════════
uint32_t readSensor(VL53L0X_X_NUCLEO_53L0A1 &sensor) {
  sensor.StartMeasurement();
  uint8_t ready = 0;
  VL53L0X_RangingMeasurementData_t data;
  do {
    sensor.GetMeasurementDataReady(&ready);
  } while (!ready);
  sensor.ClearInterruptMask(0);
  sensor.GetRangingMeasurementData(&data);
  return (data.RangeStatus == 0) ? data.RangeMilliMeter : 1200;
}

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
  sensor_left.StartMeasurement();
  sensor_right.StartMeasurement();
  sensor_center.StartMeasurement();

  int left_done = 0, right_done = 0, center_done = 0;
  uint8_t ready = 0;
  VL53L0X_RangingMeasurementData_t data;

  do {
    if (!left_done) {
      ready = 0;
      sensor_left.GetMeasurementDataReady(&ready);
      if (ready) {
        sensor_left.ClearInterruptMask(0);
        sensor_left.GetRangingMeasurementData(&data);
        distance_left = (data.RangeStatus == 0) ? data.RangeMilliMeter : 1200;
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
    if (!center_done) {
      ready = 0;
      sensor_center.GetMeasurementDataReady(&ready);
      if (ready) {
        sensor_center.ClearInterruptMask(0);
        sensor_center.GetRangingMeasurementData(&data);
        distance_center = (data.RangeStatus == 0) ? data.RangeMilliMeter : 1200;
        center_done = 1;
      }
    }
  } while (!left_done || !right_done || !center_done);
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
void printSwipeGesture(int gesture_code) {
  bool diag = (minL < HAND_PRESENT_MM)
           && (minR < HAND_PRESENT_MM)
           && ((uint32_t)abs((int32_t)minL - (int32_t)minR) > DIAG_THRESHOLD_MM);

  SerialPort.print(">>> ");
  if (gesture_code == GESTURES_SWIPE_LEFT_RIGHT) {
    if (!diag)            { currentMove = SWIPE_RIGHT;    SerialPort.println("SWIPE_RIGHT    ←→"); }
    else if (minL < minR) { currentMove = SWIPE_DIAG_UR;  SerialPort.println("SWIPE_DIAG_UR  ↗  долу-ляво → горе-дясно"); }
    else                  { currentMove = SWIPE_DIAG_DR;  SerialPort.println("SWIPE_DIAG_DR  ↘  горе-ляво → долу-дясно"); }
  } else {
    if (!diag)            { currentMove = SWIPE_LEFT;     SerialPort.println("SWIPE_LEFT     →←"); }
    else if (minR < minL) { currentMove = SWIPE_DIAG_UL;  SerialPort.println("SWIPE_DIAG_UL  ↖  долу-дясно → горе-ляво"); }
    else                  { currentMove = SWIPE_DIAG_DL;  SerialPort.println("SWIPE_DIAG_DL  ↙  горе-дясно → долу-ляво"); }
  }
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

  // Remap адреси
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
  SerialPort.println("WiFi ready.");
  SerialPort.println("Go!");

  SerialPort.println("\nReady.");
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
        SerialPort.print("DETECT L="); SerialPort.print(distance_left);
        SerialPort.print(" C="); SerialPort.print(distance_center);
        SerialPort.print(" R="); SerialPort.print(distance_right);
        SerialPort.print(" #"); SerialPort.println(handDetectCount);
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
        printSwipeGesture(gesture_code);
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
          SerialPort.println(">>> VERTICAL_UP    ↑  ръката се вдигна");
        } else if (delta < -(int32_t)VERTICAL_THRESHOLD_MM) {
          currentMove = VERTICAL_DOWN;
          SerialPort.println(">>> VERTICAL_DOWN  ↓  ръката се сниши");
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
              case ACTION_NONE: current_action = ACTION_TV;   SerialPort.println("Action: TV"); break;
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
            SerialPort.println("Action: NONE");
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
        SerialPort.println("Ready.");
      }
      break;
  }
}

// ════════════════════════════════════════════════════════════════
// ESP01 WiFi helpers
// ════════════════════════════════════════════════════════════════
void tvPower(bool on) {
  SerialPort.println(on ? "TV ON" : "TV OFF");
  if (on) {
    wakeTV();
  } else {
    const char* body = "{\"method\":\"setPowerStatus\",\"params\":[{\"status\":false}],\"id\":55,\"version\":\"1.0\"}";
    httpPost(TV_IP, "/sony/system", TV_PSK, body);
  }
}

void tvMute(bool mute) {
  SerialPort.println(mute ? "TV MUTE" : "TV UNMUTE");
  const char* body = mute
    ? "{\"method\":\"setAudioMute\",\"params\":[{\"status\":true}],\"id\":601,\"version\":\"1.0\"}"
    : "{\"method\":\"setAudioMute\",\"params\":[{\"status\":false}],\"id\":601,\"version\":\"1.0\"}";
  httpPost(TV_IP, "/sony/audio", TV_PSK, body);
}

void wakeTV() {
  uint8_t mac[6] = {0x3A, 0xFC, 0x08, 0x9C, 0xFB, 0x90};
  uint8_t packet[102];
  for (int i = 0; i < 6; i++) packet[i] = 0xFF;
  for (int i = 0; i < 16; i++)
    for (int j = 0; j < 6; j++)
      packet[6 + i*6 + j] = mac[j];

  sendAT("AT+CIPSTART=\"UDP\",\"192.168.100.255\",9", 3000);

  Serial1.println("AT+CIPSEND=102");
  unsigned long t = millis();
  while (millis() - t < 3000) {
    if (Serial1.available() && Serial1.read() == '>') break;
  }

  Serial1.write(packet, 102);

  t = millis();
  while (millis() - t < 3000) {
    while (Serial1.available()) SerialPort.write(Serial1.read());
  }

  sendAT("AT+CIPCLOSE", 2000);
}

void httpPost(const char* host, const char* path, const char* psk, const char* body) {
  String req = "";
  req += "POST "; req += path; req += " HTTP/1.1\r\n";
  req += "Host: ";            req += host;         req += "\r\n";
  req += "Content-Type: application/json\r\n";
  req += "X-Auth-PSK: ";      req += psk;          req += "\r\n";
  req += "Content-Length: ";  req += strlen(body); req += "\r\n";
  req += "Connection: close\r\n\r\n";
  req += body;

  String cipStart = "AT+CIPSTART=\"TCP\",\"";
  cipStart += host; cipStart += "\",80";
  sendAT(cipStart.c_str(), 5000);

  String cipSend = "AT+CIPSEND="; cipSend += req.length();
  Serial1.println(cipSend);

  // Чакай '>' преди изпращане
  unsigned long t = millis();
  while (millis() - t < 3000) {
    if (Serial1.available() && Serial1.read() == '>') break;
  }

  Serial1.print(req);

  t = millis();
  while (millis() - t < 5000) {
    while (Serial1.available()) SerialPort.write(Serial1.read());
  }

  sendAT("AT+CIPCLOSE", 2000);
}

void sendIRCC(const char* code) {
  String body = "<?xml version=\"1.0\"?>"
    "<s:Envelope xmlns:s=\"http://schemas.xmlsoap.org/soap/envelope/\""
    " s:encodingStyle=\"http://schemas.xmlsoap.org/soap/encoding/\">"
    "<s:Body><u:X_SendIRCC xmlns:u=\"urn:schemas-sony-com:service:IRCC:1\">"
    "<IRCCCode>";
  body += code;
  body += "</IRCCCode></u:X_SendIRCC></s:Body></s:Envelope>";

  String req = "POST /upnp/control/IRCC HTTP/1.1\r\n";
  req += "Host: "; req += TV_IP; req += ":52323\r\n";
  req += "Content-Type: text/xml; charset=UTF-8\r\n";
  req += "X-Auth-PSK: "; req += TV_PSK; req += "\r\n";
  req += "SOAPACTION: \"urn:schemas-sony-com:service:IRCC:1#X_SendIRCC\"\r\n";
  req += "Content-Length: "; req += body.length(); req += "\r\n";
  req += "Connection: close\r\n\r\n";
  req += body;

  String cipStart = "AT+CIPSTART=\"TCP\",\""; cipStart += TV_IP; cipStart += "\",52323";
  sendAT(cipStart.c_str(), 2000);

  String cipSend = "AT+CIPSEND="; cipSend += req.length();
  Serial1.println(cipSend);
  unsigned long t = millis();
  while (millis() - t < 1000) {
    if (Serial1.available() && Serial1.read() == '>') break;
  }
  Serial1.print(req);

  t = millis();
  while (millis() - t < 2000) {
    while (Serial1.available()) SerialPort.write(Serial1.read());
  }
  sendAT("AT+CIPCLOSE", 500);
}

void tvVolume(int delta) {
  SerialPort.print("Volume "); SerialPort.println(delta > 0 ? "+5" : "-5");
  char body[100];
  snprintf(body, sizeof(body),
    "{\"method\":\"setAudioVolume\",\"params\":[{\"volume\":\"%+d\"}],\"id\":601,\"version\":\"1.0\"}",
    delta);
  httpPost(TV_IP, "/sony/audio", TV_PSK, body);
}

String httpPostRead(const char* host, const char* path, const char* psk, const char* body) {
  String req = "";
  req += "POST "; req += path; req += " HTTP/1.1\r\n";
  req += "Host: ";            req += host;         req += "\r\n";
  req += "Content-Type: application/json\r\n";
  req += "X-Auth-PSK: ";      req += psk;          req += "\r\n";
  req += "Content-Length: ";  req += strlen(body); req += "\r\n";
  req += "Connection: close\r\n\r\n";
  req += body;

  String cipStart = "AT+CIPSTART=\"TCP\",\"";
  cipStart += host; cipStart += "\",80";
  sendAT(cipStart.c_str(), 5000);

  String cipSend = "AT+CIPSEND="; cipSend += req.length();
  Serial1.println(cipSend);

  unsigned long t = millis();
  while (millis() - t < 3000) {
    if (Serial1.available() && Serial1.read() == '>') break;
  }

  Serial1.print(req);

  String response = "";
  t = millis();
  while (millis() - t < 5000) {
    while (Serial1.available()) {
      char c = Serial1.read();
      SerialPort.write(c);
      response += c;
    }
  }

  sendAT("AT+CIPCLOSE", 2000);
  return response;
}

void sendAT(const char* cmd, unsigned long timeout) {
  Serial1.println(cmd);
  unsigned long t = millis();
  while (millis() - t < timeout) {
    while (Serial1.available()) SerialPort.write(Serial1.read());
  }
}
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

#define SerialPort Serial

// ── ESP01 WiFi ───────────────────────────────────────────────────
#define ESP_EN    PA8
#define WIFI_SSID "Babanin1"
#define WIFI_PASS "babaanin03"
#define TV_IP     "192.168.100.106"
#define TV_PSK    "1234"

HardwareSerial Serial1(PA10, PA9); // RX=PA10, TX=PA9

void sendAT(const char* cmd, unsigned long timeout = 3000);
void httpPost(const char* host, const char* path, const char* psk, const char* body);
String httpPostRead(const char* host, const char* path, const char* psk, const char* body); // used if needed
void sendIRCC(const char* code);
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
// Помощна функция: чете разстоянието на даден сензор (single-shot)
// ════════════════════════════════════════════════════════════════
uint32_t readSensor(VL53L0X_X_NUCLEO_53L0A1 &sensor) {
  sensor.StartMeasurement();
  uint8_t ready = 0;
  VL53L0X_RangingMeasurementData_t data;
  do {
    sensor.GetMeasurementDataReady(&ready);
  } while (!ready);
  sensor.ClearInterruptMask(0);
  sensor.GetRangingMeasurementData(&data);
  return (data.RangeStatus == 0) ? data.RangeMilliMeter : 1200;
}

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
  sensor_left.StartMeasurement();
  sensor_right.StartMeasurement();
  sensor_center.StartMeasurement();

  int left_done = 0, right_done = 0, center_done = 0;
  uint8_t ready = 0;
  VL53L0X_RangingMeasurementData_t data;

  do {
    if (!left_done) {
      ready = 0;
      sensor_left.GetMeasurementDataReady(&ready);
      if (ready) {
        sensor_left.ClearInterruptMask(0);
        sensor_left.GetRangingMeasurementData(&data);
        distance_left = (data.RangeStatus == 0) ? data.RangeMilliMeter : 1200;
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
    if (!center_done) {
      ready = 0;
      sensor_center.GetMeasurementDataReady(&ready);
      if (ready) {
        sensor_center.ClearInterruptMask(0);
        sensor_center.GetRangingMeasurementData(&data);
        distance_center = (data.RangeStatus == 0) ? data.RangeMilliMeter : 1200;
        center_done = 1;
      }
    }
  } while (!left_done || !right_done || !center_done);
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
void printSwipeGesture(int gesture_code) {
  bool diag = (minL < HAND_PRESENT_MM)
           && (minR < HAND_PRESENT_MM)
           && ((uint32_t)abs((int32_t)minL - (int32_t)minR) > DIAG_THRESHOLD_MM);

  SerialPort.print(">>> ");
  if (gesture_code == GESTURES_SWIPE_LEFT_RIGHT) {
    if (!diag)            { currentMove = SWIPE_RIGHT;    SerialPort.println("SWIPE_RIGHT    ←→"); }
    else if (minL < minR) { currentMove = SWIPE_DIAG_UR;  SerialPort.println("SWIPE_DIAG_UR  ↗  долу-ляво → горе-дясно"); }
    else                  { currentMove = SWIPE_DIAG_DR;  SerialPort.println("SWIPE_DIAG_DR  ↘  горе-ляво → долу-дясно"); }
  } else {
    if (!diag)            { currentMove = SWIPE_LEFT;     SerialPort.println("SWIPE_LEFT     →←"); }
    else if (minR < minL) { currentMove = SWIPE_DIAG_UL;  SerialPort.println("SWIPE_DIAG_UL  ↖  долу-дясно → горе-ляво"); }
    else                  { currentMove = SWIPE_DIAG_DL;  SerialPort.println("SWIPE_DIAG_DL  ↙  горе-дясно → долу-ляво"); }
  }
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

  // Remap адреси
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
  SerialPort.println("WiFi ready.");
  SerialPort.println("Go!");

  SerialPort.println("\nReady.");
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
        SerialPort.print("DETECT L="); SerialPort.print(distance_left);
        SerialPort.print(" C="); SerialPort.print(distance_center);
        SerialPort.print(" R="); SerialPort.print(distance_right);
        SerialPort.print(" #"); SerialPort.println(handDetectCount);
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
        printSwipeGesture(gesture_code);
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
          SerialPort.println(">>> VERTICAL_UP    ↑  ръката се вдигна");
        } else if (delta < -(int32_t)VERTICAL_THRESHOLD_MM) {
          currentMove = VERTICAL_DOWN;
          SerialPort.println(">>> VERTICAL_DOWN  ↓  ръката се сниши");
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
              case ACTION_NONE: current_action = ACTION_TV;   SerialPort.println("Action: TV"); break;
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
            SerialPort.println("Action: NONE");
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
        SerialPort.println("Ready.");
      }
      break;
  }
}

// ════════════════════════════════════════════════════════════════
// ESP01 WiFi helpers
// ════════════════════════════════════════════════════════════════
void tvPower(bool on) {
  SerialPort.println(on ? "TV ON" : "TV OFF");
  if (on) {
    wakeTV();
  } else {
    const char* body = "{\"method\":\"setPowerStatus\",\"params\":[{\"status\":false}],\"id\":55,\"version\":\"1.0\"}";
    httpPost(TV_IP, "/sony/system", TV_PSK, body);
  }
}

void tvMute(bool mute) {
  SerialPort.println(mute ? "TV MUTE" : "TV UNMUTE");
  const char* body = mute
    ? "{\"method\":\"setAudioMute\",\"params\":[{\"status\":true}],\"id\":601,\"version\":\"1.0\"}"
    : "{\"method\":\"setAudioMute\",\"params\":[{\"status\":false}],\"id\":601,\"version\":\"1.0\"}";
  httpPost(TV_IP, "/sony/audio", TV_PSK, body);
}

void wakeTV() {
  uint8_t mac[6] = {0x3A, 0xFC, 0x08, 0x9C, 0xFB, 0x90};
  uint8_t packet[102];
  for (int i = 0; i < 6; i++) packet[i] = 0xFF;
  for (int i = 0; i < 16; i++)
    for (int j = 0; j < 6; j++)
      packet[6 + i*6 + j] = mac[j];

  sendAT("AT+CIPSTART=\"UDP\",\"192.168.100.255\",9", 3000);

  Serial1.println("AT+CIPSEND=102");
  unsigned long t = millis();
  while (millis() - t < 3000) {
    if (Serial1.available() && Serial1.read() == '>') break;
  }

  Serial1.write(packet, 102);

  t = millis();
  while (millis() - t < 3000) {
    while (Serial1.available()) SerialPort.write(Serial1.read());
  }

  sendAT("AT+CIPCLOSE", 2000);
}

void httpPost(const char* host, const char* path, const char* psk, const char* body) {
  String req = "";
  req += "POST "; req += path; req += " HTTP/1.1\r\n";
  req += "Host: ";            req += host;         req += "\r\n";
  req += "Content-Type: application/json\r\n";
  req += "X-Auth-PSK: ";      req += psk;          req += "\r\n";
  req += "Content-Length: ";  req += strlen(body); req += "\r\n";
  req += "Connection: close\r\n\r\n";
  req += body;

  String cipStart = "AT+CIPSTART=\"TCP\",\"";
  cipStart += host; cipStart += "\",80";
  sendAT(cipStart.c_str(), 5000);

  String cipSend = "AT+CIPSEND="; cipSend += req.length();
  Serial1.println(cipSend);

  // Чакай '>' преди изпращане
  unsigned long t = millis();
  while (millis() - t < 3000) {
    if (Serial1.available() && Serial1.read() == '>') break;
  }

  Serial1.print(req);

  t = millis();
  while (millis() - t < 5000) {
    while (Serial1.available()) SerialPort.write(Serial1.read());
  }

  sendAT("AT+CIPCLOSE", 2000);
}

void sendIRCC(const char* code) {
  String body = "<?xml version=\"1.0\"?>"
    "<s:Envelope xmlns:s=\"http://schemas.xmlsoap.org/soap/envelope/\""
    " s:encodingStyle=\"http://schemas.xmlsoap.org/soap/encoding/\">"
    "<s:Body><u:X_SendIRCC xmlns:u=\"urn:schemas-sony-com:service:IRCC:1\">"
    "<IRCCCode>";
  body += code;
  body += "</IRCCCode></u:X_SendIRCC></s:Body></s:Envelope>";

  String req = "POST /upnp/control/IRCC HTTP/1.1\r\n";
  req += "Host: "; req += TV_IP; req += ":52323\r\n";
  req += "Content-Type: text/xml; charset=UTF-8\r\n";
  req += "X-Auth-PSK: "; req += TV_PSK; req += "\r\n";
  req += "SOAPACTION: \"urn:schemas-sony-com:service:IRCC:1#X_SendIRCC\"\r\n";
  req += "Content-Length: "; req += body.length(); req += "\r\n";
  req += "Connection: close\r\n\r\n";
  req += body;

  String cipStart = "AT+CIPSTART=\"TCP\",\""; cipStart += TV_IP; cipStart += "\",52323";
  sendAT(cipStart.c_str(), 2000);

  String cipSend = "AT+CIPSEND="; cipSend += req.length();
  Serial1.println(cipSend);
  unsigned long t = millis();
  while (millis() - t < 1000) {
    if (Serial1.available() && Serial1.read() == '>') break;
  }
  Serial1.print(req);

  t = millis();
  while (millis() - t < 2000) {
    while (Serial1.available()) SerialPort.write(Serial1.read());
  }
  sendAT("AT+CIPCLOSE", 500);
}

void tvVolume(int delta) {
  SerialPort.print("Volume "); SerialPort.println(delta > 0 ? "+5" : "-5");
  char body[100];
  snprintf(body, sizeof(body),
    "{\"method\":\"setAudioVolume\",\"params\":[{\"volume\":\"%+d\"}],\"id\":601,\"version\":\"1.0\"}",
    delta);
  httpPost(TV_IP, "/sony/audio", TV_PSK, body);
}

String httpPostRead(const char* host, const char* path, const char* psk, const char* body) {
  String req = "";
  req += "POST "; req += path; req += " HTTP/1.1\r\n";
  req += "Host: ";            req += host;         req += "\r\n";
  req += "Content-Type: application/json\r\n";
  req += "X-Auth-PSK: ";      req += psk;          req += "\r\n";
  req += "Content-Length: ";  req += strlen(body); req += "\r\n";
  req += "Connection: close\r\n\r\n";
  req += body;

  String cipStart = "AT+CIPSTART=\"TCP\",\"";
  cipStart += host; cipStart += "\",80";
  sendAT(cipStart.c_str(), 5000);

  String cipSend = "AT+CIPSEND="; cipSend += req.length();
  Serial1.println(cipSend);

  unsigned long t = millis();
  while (millis() - t < 3000) {
    if (Serial1.available() && Serial1.read() == '>') break;
  }

  Serial1.print(req);

  String response = "";
  t = millis();
  while (millis() - t < 5000) {
    while (Serial1.available()) {
      char c = Serial1.read();
      SerialPort.write(c);
      response += c;
    }
  }

  sendAT("AT+CIPCLOSE", 2000);
  return response;
}

void sendAT(const char* cmd, unsigned long timeout) {
  Serial1.println(cmd);
  unsigned long t = millis();
  while (millis() - t < timeout) {
    while (Serial1.available()) SerialPort.write(Serial1.read());
  }
}
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

#define SerialPort Serial

// ── ESP01 WiFi ───────────────────────────────────────────────────
#define ESP_EN    PA8
#define WIFI_SSID "Babanin1"
#define WIFI_PASS "babaanin03"
#define TV_IP     "192.168.100.106"
#define TV_PSK    "1234"

HardwareSerial Serial1(PA10, PA9); // RX=PA10, TX=PA9

void sendAT(const char* cmd, unsigned long timeout = 3000);
void httpPost(const char* host, const char* path, const char* psk, const char* body);
String httpPostRead(const char* host, const char* path, const char* psk, const char* body); // used if needed
void sendIRCC(const char* code);
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
// Помощна функция: чете разстоянието на даден сензор (single-shot)
// ════════════════════════════════════════════════════════════════
uint32_t readSensor(VL53L0X_X_NUCLEO_53L0A1 &sensor) {
  sensor.StartMeasurement();
  uint8_t ready = 0;
  VL53L0X_RangingMeasurementData_t data;
  do {
    sensor.GetMeasurementDataReady(&ready);
  } while (!ready);
  sensor.ClearInterruptMask(0);
  sensor.GetRangingMeasurementData(&data);
  return (data.RangeStatus == 0) ? data.RangeMilliMeter : 1200;
}

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
  sensor_left.StartMeasurement();
  sensor_right.StartMeasurement();
  sensor_center.StartMeasurement();

  int left_done = 0, right_done = 0, center_done = 0;
  uint8_t ready = 0;
  VL53L0X_RangingMeasurementData_t data;

  do {
    if (!left_done) {
      ready = 0;
      sensor_left.GetMeasurementDataReady(&ready);
      if (ready) {
        sensor_left.ClearInterruptMask(0);
        sensor_left.GetRangingMeasurementData(&data);
        distance_left = (data.RangeStatus == 0) ? data.RangeMilliMeter : 1200;
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
    if (!center_done) {
      ready = 0;
      sensor_center.GetMeasurementDataReady(&ready);
      if (ready) {
        sensor_center.ClearInterruptMask(0);
        sensor_center.GetRangingMeasurementData(&data);
        distance_center = (data.RangeStatus == 0) ? data.RangeMilliMeter : 1200;
        center_done = 1;
      }
    }
  } while (!left_done || !right_done || !center_done);
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
void printSwipeGesture(int gesture_code) {
  bool diag = (minL < HAND_PRESENT_MM)
           && (minR < HAND_PRESENT_MM)
           && ((uint32_t)abs((int32_t)minL - (int32_t)minR) > DIAG_THRESHOLD_MM);

  SerialPort.print(">>> ");
  if (gesture_code == GESTURES_SWIPE_LEFT_RIGHT) {
    if (!diag)            { currentMove = SWIPE_RIGHT;    SerialPort.println("SWIPE_RIGHT    ←→"); }
    else if (minL < minR) { currentMove = SWIPE_DIAG_UR;  SerialPort.println("SWIPE_DIAG_UR  ↗  долу-ляво → горе-дясно"); }
    else                  { currentMove = SWIPE_DIAG_DR;  SerialPort.println("SWIPE_DIAG_DR  ↘  горе-ляво → долу-дясно"); }
  } else {
    if (!diag)            { currentMove = SWIPE_LEFT;     SerialPort.println("SWIPE_LEFT     →←"); }
    else if (minR < minL) { currentMove = SWIPE_DIAG_UL;  SerialPort.println("SWIPE_DIAG_UL  ↖  долу-дясно → горе-ляво"); }
    else                  { currentMove = SWIPE_DIAG_DL;  SerialPort.println("SWIPE_DIAG_DL  ↙  горе-дясно → долу-ляво"); }
  }
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

  // Remap адреси
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
  SerialPort.println("WiFi ready.");
  SerialPort.println("Go!");

  SerialPort.println("\nReady.");
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
        SerialPort.print("DETECT L="); SerialPort.print(distance_left);
        SerialPort.print(" C="); SerialPort.print(distance_center);
        SerialPort.print(" R="); SerialPort.print(distance_right);
        SerialPort.print(" #"); SerialPort.println(handDetectCount);
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
        printSwipeGesture(gesture_code);
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
          SerialPort.println(">>> VERTICAL_UP    ↑  ръката се вдигна");
        } else if (delta < -(int32_t)VERTICAL_THRESHOLD_MM) {
          currentMove = VERTICAL_DOWN;
          SerialPort.println(">>> VERTICAL_DOWN  ↓  ръката се сниши");
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
              case ACTION_NONE: current_action = ACTION_TV;   SerialPort.println("Action: TV"); break;
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
            SerialPort.println("Action: NONE");
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
        SerialPort.println("Ready.");
      }
      break;
  }
}

// ════════════════════════════════════════════════════════════════
// ESP01 WiFi helpers
// ════════════════════════════════════════════════════════════════
void tvPower(bool on) {
  SerialPort.println(on ? "TV ON" : "TV OFF");
  if (on) {
    wakeTV();
  } else {
    const char* body = "{\"method\":\"setPowerStatus\",\"params\":[{\"status\":false}],\"id\":55,\"version\":\"1.0\"}";
    httpPost(TV_IP, "/sony/system", TV_PSK, body);
  }
}

void tvMute(bool mute) {
  SerialPort.println(mute ? "TV MUTE" : "TV UNMUTE");
  const char* body = mute
    ? "{\"method\":\"setAudioMute\",\"params\":[{\"status\":true}],\"id\":601,\"version\":\"1.0\"}"
    : "{\"method\":\"setAudioMute\",\"params\":[{\"status\":false}],\"id\":601,\"version\":\"1.0\"}";
  httpPost(TV_IP, "/sony/audio", TV_PSK, body);
}

void wakeTV() {
  uint8_t mac[6] = {0x3A, 0xFC, 0x08, 0x9C, 0xFB, 0x90};
  uint8_t packet[102];
  for (int i = 0; i < 6; i++) packet[i] = 0xFF;
  for (int i = 0; i < 16; i++)
    for (int j = 0; j < 6; j++)
      packet[6 + i*6 + j] = mac[j];

  sendAT("AT+CIPSTART=\"UDP\",\"192.168.100.255\",9", 3000);

  Serial1.println("AT+CIPSEND=102");
  unsigned long t = millis();
  while (millis() - t < 3000) {
    if (Serial1.available() && Serial1.read() == '>') break;
  }

  Serial1.write(packet, 102);

  t = millis();
  while (millis() - t < 3000) {
    while (Serial1.available()) SerialPort.write(Serial1.read());
  }

  sendAT("AT+CIPCLOSE", 2000);
}

void httpPost(const char* host, const char* path, const char* psk, const char* body) {
  String req = "";
  req += "POST "; req += path; req += " HTTP/1.1\r\n";
  req += "Host: ";            req += host;         req += "\r\n";
  req += "Content-Type: application/json\r\n";
  req += "X-Auth-PSK: ";      req += psk;          req += "\r\n";
  req += "Content-Length: ";  req += strlen(body); req += "\r\n";
  req += "Connection: close\r\n\r\n";
  req += body;

  String cipStart = "AT+CIPSTART=\"TCP\",\"";
  cipStart += host; cipStart += "\",80";
  sendAT(cipStart.c_str(), 5000);

  String cipSend = "AT+CIPSEND="; cipSend += req.length();
  Serial1.println(cipSend);

  // Чакай '>' преди изпращане
  unsigned long t = millis();
  while (millis() - t < 3000) {
    if (Serial1.available() && Serial1.read() == '>') break;
  }

  Serial1.print(req);

  t = millis();
  while (millis() - t < 5000) {
    while (Serial1.available()) SerialPort.write(Serial1.read());
  }

  sendAT("AT+CIPCLOSE", 2000);
}

void sendIRCC(const char* code) {
  String body = "<?xml version=\"1.0\"?>"
    "<s:Envelope xmlns:s=\"http://schemas.xmlsoap.org/soap/envelope/\""
    " s:encodingStyle=\"http://schemas.xmlsoap.org/soap/encoding/\">"
    "<s:Body><u:X_SendIRCC xmlns:u=\"urn:schemas-sony-com:service:IRCC:1\">"
    "<IRCCCode>";
  body += code;
  body += "</IRCCCode></u:X_SendIRCC></s:Body></s:Envelope>";

  String req = "POST /upnp/control/IRCC HTTP/1.1\r\n";
  req += "Host: "; req += TV_IP; req += ":52323\r\n";
  req += "Content-Type: text/xml; charset=UTF-8\r\n";
  req += "X-Auth-PSK: "; req += TV_PSK; req += "\r\n";
  req += "SOAPACTION: \"urn:schemas-sony-com:service:IRCC:1#X_SendIRCC\"\r\n";
  req += "Content-Length: "; req += body.length(); req += "\r\n";
  req += "Connection: close\r\n\r\n";
  req += body;

  String cipStart = "AT+CIPSTART=\"TCP\",\""; cipStart += TV_IP; cipStart += "\",52323";
  sendAT(cipStart.c_str(), 2000);

  String cipSend = "AT+CIPSEND="; cipSend += req.length();
  Serial1.println(cipSend);
  unsigned long t = millis();
  while (millis() - t < 1000) {
    if (Serial1.available() && Serial1.read() == '>') break;
  }
  Serial1.print(req);

  t = millis();
  while (millis() - t < 2000) {
    while (Serial1.available()) SerialPort.write(Serial1.read());
  }
  sendAT("AT+CIPCLOSE", 500);
}

void tvVolume(int delta) {
  SerialPort.print("Volume "); SerialPort.println(delta > 0 ? "+5" : "-5");
  char body[100];
  snprintf(body, sizeof(body),
    "{\"method\":\"setAudioVolume\",\"params\":[{\"volume\":\"%+d\"}],\"id\":601,\"version\":\"1.0\"}",
    delta);
  httpPost(TV_IP, "/sony/audio", TV_PSK, body);
}

String httpPostRead(const char* host, const char* path, const char* psk, const char* body) {
  String req = "";
  req += "POST "; req += path; req += " HTTP/1.1\r\n";
  req += "Host: ";            req += host;         req += "\r\n";
  req += "Content-Type: application/json\r\n";
  req += "X-Auth-PSK: ";      req += psk;          req += "\r\n";
  req += "Content-Length: ";  req += strlen(body); req += "\r\n";
  req += "Connection: close\r\n\r\n";
  req += body;

  String cipStart = "AT+CIPSTART=\"TCP\",\"";
  cipStart += host; cipStart += "\",80";
  sendAT(cipStart.c_str(), 5000);

  String cipSend = "AT+CIPSEND="; cipSend += req.length();
  Serial1.println(cipSend);

  unsigned long t = millis();
  while (millis() - t < 3000) {
    if (Serial1.available() && Serial1.read() == '>') break;
  }

  Serial1.print(req);

  String response = "";
  t = millis();
  while (millis() - t < 5000) {
    while (Serial1.available()) {
      char c = Serial1.read();
      SerialPort.write(c);
      response += c;
    }
  }

  sendAT("AT+CIPCLOSE", 2000);
  return response;
}

void sendAT(const char* cmd, unsigned long timeout) {
  Serial1.println(cmd);
  unsigned long t = millis();
  while (millis() - t < timeout) {
    while (Serial1.available()) SerialPort.write(Serial1.read());
  }
}
