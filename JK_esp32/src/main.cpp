#include <Arduino.h>
#include <ArduinoJson.h>
#include <max6675.h>

// ── 핀 정의 ──────────────────────────────────────────────
#define PUMP1_PIN  18
#define PUMP2_PIN  19
#define PUMP3_PIN  21
#define PUMP4_PIN  22
#define PUMP5_PIN  23
#define PUMP6_PIN  25   // 탄산 펌프
#define RELAY_PIN  26   // 펠티어 릴레이

#define THERMO_SCK 14
#define THERMO_CS  13
#define THERMO_SO  12

// ── 설정 ─────────────────────────────────────────────────
#define SODA_TARGET_TEMP   15.0f  // 탄산수 목표 온도 (°C)
#define TEMP_HYSTERESIS     1.0f  // 히스테리시스 ±1°C
#define TEMP_READ_INTERVAL  2000  // 온도 읽기 주기 (ms)
#define STATUS_INTERVAL     1000  // 상태 전송 주기 (ms)
#define ML_PER_SEC          1.0f  // 펌프 유량 (ml/s) — 실측 후 조정

// ── 전역 변수 ─────────────────────────────────────────────
MAX6675 thermocouple(THERMO_SCK, THERMO_CS, THERMO_SO);

const uint8_t PUMP_PINS[6] = {
  PUMP1_PIN, PUMP2_PIN, PUMP3_PIN,
  PUMP4_PIN, PUMP5_PIN, PUMP6_PIN
};

struct Order {
  char     id[32];
  uint16_t amounts[6];  // [시럽1~5, 탄산]
  bool     valid;
};

#define QUEUE_MAX 20
Order    orderQueue[QUEUE_MAX];
int      queueHead = 0;
int      queueTail = 0;
int      queueSize = 0;

bool     isMaking       = false;
char     currentOrderId[32] = "";

float    currentTemp    = 0.0f;
bool     peltierOn      = false;

unsigned long lastTempRead   = 0;
unsigned long lastStatusSend = 0;

// ── 함수 선언 ─────────────────────────────────────────────
void enqueueOrder(const Order& o);
bool dequeueOrder(Order& o);
void startOrder(const Order& o);
void runPump(uint8_t pumpIndex, uint16_t ml);
void controlPeltier(float temp);
void sendStatus();
void parseIncoming(const String& line);

// ── Setup ─────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);

  for (int i = 0; i < 6; i++) {
    pinMode(PUMP_PINS[i], OUTPUT);
    digitalWrite(PUMP_PINS[i], LOW);
  }
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, LOW);

  delay(500); // MAX6675 안정화

  Serial.println("{\"event\":\"ready\",\"msg\":\"SodaBot ESP32 ready\"}");
}

// ── Loop ──────────────────────────────────────────────────
void loop() {
  // 시리얼 수신
  if (Serial.available()) {
    String line = Serial.readStringUntil('\n');
    line.trim();
    if (line.length() > 0) parseIncoming(line);
  }

  // 온도 읽기 & 펠티어 제어
  if (millis() - lastTempRead >= TEMP_READ_INTERVAL) {
    lastTempRead = millis();
    float t = thermocouple.readCelsius();
    if (!isnan(t)) {
      currentTemp = t;
      controlPeltier(t);
    }
  }

  // 상태 주기 전송
  if (millis() - lastStatusSend >= STATUS_INTERVAL) {
    lastStatusSend = millis();
    sendStatus();
  }
}

// ── 시리얼 파싱 ───────────────────────────────────────────
// 웹 → ESP32
//   주문 추가: {"cmd":"enqueue","id":"...","amounts":[s1,s2,s3,s4,s5,soda]}
//   시작:      {"cmd":"start","id":"..."}
void parseIncoming(const String& line) {
  StaticJsonDocument<512> doc;
  if (deserializeJson(doc, line) != DeserializationError::Ok) return;

  const char* cmd = doc["cmd"];
  if (!cmd) return;

  if (strcmp(cmd, "enqueue") == 0) {
    Order o;
    memset(&o, 0, sizeof(o));
    strlcpy(o.id, doc["id"] | "", sizeof(o.id));
    JsonArray arr = doc["amounts"].as<JsonArray>();
    for (int i = 0; i < 6 && i < (int)arr.size(); i++) {
      o.amounts[i] = arr[i].as<uint16_t>();
    }
    o.valid = true;
    enqueueOrder(o);

    StaticJsonDocument<128> resp;
    resp["event"]      = "queued";
    resp["id"]         = o.id;
    resp["queue_size"] = queueSize;
    serializeJson(resp, Serial);
    Serial.println();
  }
  else if (strcmp(cmd, "start") == 0) {
    if (isMaking) {
      Serial.println("{\"event\":\"error\",\"msg\":\"already_making\"}");
      return;
    }
    Order o;
    if (dequeueOrder(o)) {
      startOrder(o);
    } else {
      Serial.println("{\"event\":\"error\",\"msg\":\"queue_empty\"}");
    }
  }
}

// ── 큐 ────────────────────────────────────────────────────
void enqueueOrder(const Order& o) {
  if (queueSize >= QUEUE_MAX) return;
  orderQueue[queueTail] = o;
  queueTail = (queueTail + 1) % QUEUE_MAX;
  queueSize++;
}

bool dequeueOrder(Order& o) {
  if (queueSize == 0) return false;
  o = orderQueue[queueHead];
  queueHead = (queueHead + 1) % QUEUE_MAX;
  queueSize--;
  return true;
}

// ── 주문 실행 ─────────────────────────────────────────────
// 순서: 시럽1→2→3→4→5 순차 → 탄산(pump6) 마지막
void startOrder(const Order& o) {
  isMaking = true;
  strlcpy(currentOrderId, o.id, sizeof(currentOrderId));

  StaticJsonDocument<128> msg;
  msg["event"] = "making";
  msg["id"]    = o.id;
  serializeJson(msg, Serial);
  Serial.println();

  // 시럽 펌프 (pump1 ~ pump5)
  for (int i = 0; i < 5; i++) {
    if (o.amounts[i] == 0) continue;

    StaticJsonDocument<128> pMsg;
    pMsg["event"] = "pump";
    pMsg["pump"]  = i + 1;
    pMsg["ml"]    = o.amounts[i];
    serializeJson(pMsg, Serial);
    Serial.println();

    runPump(i, o.amounts[i]);
    delay(300);
  }

  // 탄산 펌프 (pump6)
  if (o.amounts[5] > 0) {
    StaticJsonDocument<128> pMsg;
    pMsg["event"] = "pump";
    pMsg["pump"]  = 6;
    pMsg["ml"]    = o.amounts[5];
    serializeJson(pMsg, Serial);
    Serial.println();

    runPump(5, o.amounts[5]);
  }

  StaticJsonDocument<128> doneMsg;
  doneMsg["event"] = "done";
  doneMsg["id"]    = o.id;
  serializeJson(doneMsg, Serial);
  Serial.println();

  isMaking = false;
  currentOrderId[0] = '\0';
}

// ── 펌프 구동 ─────────────────────────────────────────────
void runPump(uint8_t pumpIndex, uint16_t ml) {
  if (ml == 0) return;
  uint32_t ms = (uint32_t)((ml / ML_PER_SEC) * 1000.0f);
  digitalWrite(PUMP_PINS[pumpIndex], HIGH);
  delay(ms);
  digitalWrite(PUMP_PINS[pumpIndex], LOW);
}

// ── 펠티어 제어 (bang-bang + 히스테리시스) ────────────────
// 목표 15°C 이하 유지
//   temp > 15 + 1 (16°C) → 펠티어 ON
//   temp < 15 - 1 (14°C) → 펠티어 OFF
void controlPeltier(float temp) {
  if (!peltierOn && temp > SODA_TARGET_TEMP + TEMP_HYSTERESIS) {
    peltierOn = true;
    digitalWrite(RELAY_PIN, HIGH);
  } else if (peltierOn && temp < SODA_TARGET_TEMP - TEMP_HYSTERESIS) {
    peltierOn = false;
    digitalWrite(RELAY_PIN, LOW);
  }
}

// ── 상태 전송 ─────────────────────────────────────────────
// ESP32 → 웹 (1초마다)
// {"event":"temp","temp":13.5,"peltier":false,"target":15.0,"making":false,"queue":2}
void sendStatus() {
  StaticJsonDocument<192> doc;
  doc["event"]   = "temp";
  doc["temp"]    = serialized(String(currentTemp, 1));
  doc["peltier"] = peltierOn;
  doc["target"]  = SODA_TARGET_TEMP;
  doc["making"]  = isMaking;
  if (isMaking) doc["current_id"] = currentOrderId;
  doc["queue"]   = queueSize;
  serializeJson(doc, Serial);
  Serial.println();
}
