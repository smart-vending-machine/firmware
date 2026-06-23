#include <Arduino.h>
#include <ArduinoJson.h>
#include <max6675.h>
#include <ESP32Servo.h>

// ── 핀 정의 ──────────────────────────────────────────────
#define PUMP1_PIN  25
#define PUMP2_PIN  23
#define PUMP3_PIN  22
#define PUMP4_PIN  21
#define PUMP5_PIN  18
#define RELAY_PIN     26   // 펠티어 릴레이
#define COOLER_PIN    19   // 쿨러(팬) — 펠티어와 동시 작동
#define DISPENSER_PIN 27   // 디스펜서 SSR (active-HIGH)

#define PUMP_ON    HIGH
#define PUMP_OFF   LOW
#define RELAY_ON   HIGH
#define RELAY_OFF  LOW
#define SSR_ON     HIGH
#define SSR_OFF    LOW

#define DISPENSER_MS         3000
#define POST_DISPENSER_DELAY 2000

#define THERMO_SCK 14
#define THERMO_CS  13
#define THERMO_SO  12

// ── 설정 ─────────────────────────────────────────────────
#define SODA_TARGET_TEMP   20.0f
#define TEMP_HYSTERESIS     1.0f
#define TEMP_READ_INTERVAL  2000
#define STATUS_INTERVAL     1000
#define ML_PER_SEC          0.6f

#define PUMP_RUN_ANGLE     180
#define PUMP_STOP_ANGLE    90

// ── 동시 최대 펌프 수 ─────────────────────────────────────
#define MAX_CONCURRENT_PUMPS 3

// ── 전역 변수 ─────────────────────────────────────────────
MAX6675 thermocouple(THERMO_SCK, THERMO_CS, THERMO_SO);
Servo   pumpServos[5];

const uint8_t PUMP_PINS[5] = {
  PUMP1_PIN, PUMP2_PIN, PUMP3_PIN,
  PUMP4_PIN, PUMP5_PIN
};

struct Order {
  char     id[32];
  uint16_t amounts[5];
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
bool     coolingEnabled = true;
bool     stopped        = false;

unsigned long lastTempRead   = 0;
unsigned long lastStatusSend = 0;

// ── 함수 선언 ─────────────────────────────────────────────
void enqueueOrder(const Order& o);
bool dequeueOrder(Order& o);
void startOrder(const Order& o);
void runPump(uint8_t pumpIndex, uint16_t ml);
void runPumpsParallel(const uint8_t* indices, const uint16_t* mls, uint8_t count);
void controlPeltier(float temp);
void sendStatus();
void parseIncoming(const String& line);
bool interruptibleDelay(uint32_t ms);
void emergencyStop();
void washPumps(uint16_t mlPerPump);
void washSinglePump(uint8_t pumpIndex, uint16_t ml);

// ── Setup ─────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);

  for (int i = 0; i < 5; i++) {
    pumpServos[i].setPeriodHertz(50);
    pumpServos[i].attach(PUMP_PINS[i], 500, 2500);
    pumpServos[i].write(PUMP_STOP_ANGLE);
  }
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, RELAY_OFF);
  pinMode(COOLER_PIN, OUTPUT);
  digitalWrite(COOLER_PIN, RELAY_OFF);
  pinMode(DISPENSER_PIN, OUTPUT);
  digitalWrite(DISPENSER_PIN, SSR_OFF);

  delay(500);

  Serial.println("{\"event\":\"ready\",\"msg\":\"SodaBot ESP32 ready\"}");
}

// ── Loop ──────────────────────────────────────────────────
void loop() {
  if (Serial.available()) {
    String line = Serial.readStringUntil('\n');
    line.trim();
    if (line.length() > 0) parseIncoming(line);
  }

  if (!stopped && !isMaking && queueSize > 0) {
    Order o;
    if (dequeueOrder(o)) startOrder(o);
  }

  if (millis() - lastTempRead >= TEMP_READ_INTERVAL) {
    lastTempRead = millis();
    float t = thermocouple.readCelsius();
    if (!isnan(t)) {
      currentTemp = t;
      controlPeltier(t);
    }
  }

  if (millis() - lastStatusSend >= STATUS_INTERVAL) {
    lastStatusSend = millis();
    sendStatus();
  }
}

// ── 시리얼 파싱 ───────────────────────────────────────────
void parseIncoming(const String& line) {
  StaticJsonDocument<512> doc;
  if (deserializeJson(doc, line) != DeserializationError::Ok) return;

  const char* cmd = doc["cmd"];
  if (!cmd) return;

  // 주문 추가
  if (strcmp(cmd, "enqueue") == 0) {
    Order o;
    memset(&o, 0, sizeof(o));
    strlcpy(o.id, doc["id"] | "", sizeof(o.id));
    JsonArray arr = doc["amounts"].as<JsonArray>();
    for (int i = 0; i < 5 && i < (int)arr.size(); i++) {
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
  // 주문 시작
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
  // 단일 펌프 작동: {"cmd":"pour","pump":1,"ml":50}
  else if (strcmp(cmd, "pour") == 0) {
    if (isMaking || stopped) {
      Serial.println("{\"event\":\"error\",\"msg\":\"busy_or_stopped\"}");
      return;
    }
    int      p  = doc["pump"] | 0;  // 1~5
    uint16_t ml = doc["ml"]   | 0;
    if (p < 1 || p > 5 || ml == 0) return;

    isMaking = true;
    StaticJsonDocument<96> msg;
    msg["event"] = "pour";
    msg["pump"]  = p;
    msg["ml"]    = ml;
    serializeJson(msg, Serial);
    Serial.println();

    runPump(p - 1, ml);

    if (!stopped) Serial.println("{\"event\":\"pour_done\"}");
    isMaking = false;
  }
  // 세척
  // 전체 세척:  {"cmd":"wash"}  또는  {"cmd":"wash","ml":100}
  // 단일 세척:  {"cmd":"wash","pump":1,"ml":100}
  else if (strcmp(cmd, "wash") == 0) {
    if (isMaking || stopped) {
      Serial.println("{\"event\":\"error\",\"msg\":\"busy_or_stopped\"}");
      return;
    }
    uint16_t ml = doc["ml"] | 100;

    // pump 키가 있으면 단일 세척
    if (doc.containsKey("pump")) {
      int p = doc["pump"] | 0;
      if (p < 1 || p > 5) {
        Serial.println("{\"event\":\"error\",\"msg\":\"invalid_pump\"}");
        return;
      }
      washSinglePump(p - 1, ml);
    } else {
      // 전체 세척
      washPumps(ml);
    }
  }
  // 비상정지 / 재가동
  else if (strcmp(cmd, "stop") == 0) {
    emergencyStop();
    Serial.println("{\"event\":\"stopped\"}");
  }
  else if (strcmp(cmd, "resume") == 0) {
    stopped = false;
    Serial.println("{\"event\":\"resumed\"}");
  }
  // 냉각 수동 ON/OFF
  else if (strcmp(cmd, "cool") == 0) {
    coolingEnabled = doc["on"] | false;
    if (!coolingEnabled) {
      peltierOn = false;
      digitalWrite(RELAY_PIN,  RELAY_OFF);
      digitalWrite(COOLER_PIN, RELAY_OFF);
    }
    StaticJsonDocument<96> r;
    r["event"]   = "cool";
    r["enabled"] = coolingEnabled;
    serializeJson(r, Serial);
    Serial.println();
  }
  // GPIO 수동 토글
  else if (strcmp(cmd, "pin") == 0) {
    int gpio = doc["gpio"] | -1;
    bool on  = doc["on"]   | false;
    if (gpio < 0) return;

    int pumpCh = -1;
    for (int i = 0; i < 5; i++) if (PUMP_PINS[i] == gpio) { pumpCh = i; break; }
    if (pumpCh >= 0) {
      pumpServos[pumpCh].write(on ? PUMP_RUN_ANGLE : PUMP_STOP_ANGLE);
    } else {
      pinMode(gpio, OUTPUT);
      digitalWrite(gpio, on ? HIGH : LOW);
    }

    StaticJsonDocument<96> r;
    r["event"] = "pin";
    r["gpio"]  = gpio;
    r["state"] = on ? "HIGH" : "LOW";
    serializeJson(r, Serial);
    Serial.println();
  }
  // 펌프 테스트
  else if (strcmp(cmd, "test") == 0) {
    int p       = doc["pump"] | 0;
    uint32_t ms = doc["ms"]   | 3000;
    if (p < 1 || p > 5) return;
    StaticJsonDocument<96> r;
    r["event"] = "test";
    r["pump"]  = p;
    r["ms"]    = ms;
    serializeJson(r, Serial);
    Serial.println();
    pumpServos[p - 1].write(PUMP_RUN_ANGLE);
    delay(ms);
    pumpServos[p - 1].write(PUMP_STOP_ANGLE);
  }
}

// ── 인터럽트 가능 딜레이 ──────────────────────────────────
bool interruptibleDelay(uint32_t ms) {
  uint32_t start = millis();
  while (millis() - start < ms) {
    if (stopped) return false;
    if (Serial.available()) {
      String line = Serial.readStringUntil('\n');
      line.trim();
      if (line.length() > 0) parseIncoming(line);
      if (stopped) return false;
    }
    delay(5);
  }
  return !stopped;
}

// ── 단일 펌프 구동 ────────────────────────────────────────
// 해당 펌프만 정지 — 다른 펌프에 영향 없음
void runPump(uint8_t pumpIndex, uint16_t ml) {
  if (ml == 0) return;
  uint32_t ms     = (uint32_t)((ml / ML_PER_SEC) * 1000.0f);
  uint32_t endAt  = millis() + ms;

  pumpServos[pumpIndex].write(PUMP_RUN_ANGLE);

  while (millis() < endAt && !stopped) {
    if (Serial.available()) {
      String line = Serial.readStringUntil('\n');
      line.trim();
      if (line.length() > 0) parseIncoming(line);
    }
    delay(5);
  }

  // 이 펌프만 정지 (stopped여도 이 펌프는 반드시 멈춤)
  pumpServos[pumpIndex].write(PUMP_STOP_ANGLE);
}

// ── 동시 펌프 구동 (최대 MAX_CONCURRENT_PUMPS개) ──────────
// indices: 펌프 인덱스 배열 (0~4)
// mls:     각 펌프 ml 배열
// count:   펌프 수 (MAX_CONCURRENT_PUMPS 이하)
void runPumpsParallel(const uint8_t* indices, const uint16_t* mls, uint8_t count) {
  if (count == 0) return;

  // 각 펌프 종료 시각 계산
  uint32_t endTimes[MAX_CONCURRENT_PUMPS];
  uint32_t now = millis();
  for (int i = 0; i < count; i++) {
    uint32_t ms = (uint32_t)((mls[i] / ML_PER_SEC) * 1000.0f);
    endTimes[i] = now + ms;
    pumpServos[indices[i]].write(PUMP_RUN_ANGLE);
  }

  // 모든 펌프 끝날 때까지 대기, 각자 시간 되면 정지
  bool allDone = false;
  while (!allDone && !stopped) {
    allDone = true;
    now = millis();
    for (int i = 0; i < count; i++) {
      if (now < endTimes[i]) {
        allDone = false;
      } else {
        pumpServos[indices[i]].write(PUMP_STOP_ANGLE);
      }
    }
    // 시리얼 수신 처리
    if (Serial.available()) {
      String line = Serial.readStringUntil('\n');
      line.trim();
      if (line.length() > 0) parseIncoming(line);
    }
    delay(5);
  }

  // 비상정지면 모두 강제 정지
  if (stopped) {
    for (int i = 0; i < count; i++) {
      pumpServos[indices[i]].write(PUMP_STOP_ANGLE);
    }
  }
}

// ── 전체 세척 ─────────────────────────────────────────────
// 펌프 1~5를 MAX_CONCURRENT_PUMPS개씩 묶어서 동시 세척
void washPumps(uint16_t mlPerPump) {
  isMaking = true;
  strlcpy(currentOrderId, "wash_all", sizeof(currentOrderId));

  StaticJsonDocument<96> startMsg;
  startMsg["event"] = "wash_start";
  startMsg["ml"]    = mlPerPump;
  startMsg["mode"]  = "all";
  serializeJson(startMsg, Serial);
  Serial.println();

  int i = 0;
  while (i < 5 && !stopped) {
    uint8_t  batchIdx[MAX_CONCURRENT_PUMPS];
    uint16_t batchMl [MAX_CONCURRENT_PUMPS];
    uint8_t  batchCnt = 0;

    // MAX_CONCURRENT_PUMPS개씩 묶기
    while (batchCnt < MAX_CONCURRENT_PUMPS && i < 5) {
      batchIdx[batchCnt] = i;
      batchMl [batchCnt] = mlPerPump;
      batchCnt++;
      i++;
    }

    // 배치 정보 출력
    StaticJsonDocument<128> bMsg;
    bMsg["event"] = "wash_batch";
    JsonArray arr = bMsg.createNestedArray("pumps");
    for (int j = 0; j < batchCnt; j++) arr.add(batchIdx[j] + 1);
    bMsg["ml"] = mlPerPump;
    serializeJson(bMsg, Serial);
    Serial.println();

    runPumpsParallel(batchIdx, batchMl, batchCnt);
    if (stopped) break;
    if (!interruptibleDelay(500)) break;
  }

  if (!stopped) Serial.println("{\"event\":\"wash_done\",\"mode\":\"all\"}");
  isMaking = false;
  currentOrderId[0] = '\0';
}

// ── 단일 펌프 세척 ────────────────────────────────────────
// {"cmd":"wash","pump":1,"ml":100}
void washSinglePump(uint8_t pumpIndex, uint16_t ml) {
  isMaking = true;
  snprintf(currentOrderId, sizeof(currentOrderId), "wash_p%d", pumpIndex + 1);

  StaticJsonDocument<96> startMsg;
  startMsg["event"] = "wash_start";
  startMsg["pump"]  = pumpIndex + 1;
  startMsg["ml"]    = ml;
  startMsg["mode"]  = "single";
  serializeJson(startMsg, Serial);
  Serial.println();

  runPump(pumpIndex, ml);

  if (!stopped) {
    StaticJsonDocument<96> doneMsg;
    doneMsg["event"] = "wash_done";
    doneMsg["pump"]  = pumpIndex + 1;
    doneMsg["mode"]  = "single";
    serializeJson(doneMsg, Serial);
    Serial.println();
  }

  isMaking = false;
  currentOrderId[0] = '\0';
}

// ── 비상정지 ──────────────────────────────────────────────
void emergencyStop() {
  stopped = true;
  for (int i = 0; i < 5; i++) pumpServos[i].write(PUMP_STOP_ANGLE);
  digitalWrite(DISPENSER_PIN, SSR_OFF);
  digitalWrite(RELAY_PIN,     RELAY_OFF);
  digitalWrite(COOLER_PIN,    RELAY_OFF);
  peltierOn = false;
  isMaking  = false;
  currentOrderId[0] = '\0';
  queueHead = queueTail = queueSize = 0;
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
// 펌프를 MAX_CONCURRENT_PUMPS개씩 묶어서 동시 실행
void startOrder(const Order& o) {
  isMaking = true;
  strlcpy(currentOrderId, o.id, sizeof(currentOrderId));

  StaticJsonDocument<128> msg;
  msg["event"] = "making";
  msg["id"]    = o.id;
  serializeJson(msg, Serial);
  Serial.println();

  // 디스펜서 작동
  {
    StaticJsonDocument<128> dMsg;
    dMsg["event"] = "dispenser";
    dMsg["ms"]    = DISPENSER_MS;
    serializeJson(dMsg, Serial);
    Serial.println();

    digitalWrite(DISPENSER_PIN, SSR_ON);
    bool ok = interruptibleDelay(DISPENSER_MS);
    digitalWrite(DISPENSER_PIN, SSR_OFF);
    if (!ok) return;
  }

  if (!interruptibleDelay(POST_DISPENSER_DELAY)) return;

  // amounts > 0 인 펌프만 추려서 MAX_CONCURRENT_PUMPS개씩 묶어 실행
  uint8_t  activeIdx[5];
  uint16_t activeMl [5];
  uint8_t  activeCount = 0;

  for (int i = 0; i < 5; i++) {
    if (o.amounts[i] > 0) {
      activeIdx[activeCount] = i;
      activeMl [activeCount] = o.amounts[i];
      activeCount++;
    }
  }

  int i = 0;
  while (i < activeCount && !stopped) {
    uint8_t  batchIdx[MAX_CONCURRENT_PUMPS];
    uint16_t batchMl [MAX_CONCURRENT_PUMPS];
    uint8_t  batchCnt = 0;

    while (batchCnt < MAX_CONCURRENT_PUMPS && i < activeCount) {
      batchIdx[batchCnt] = activeIdx[i];
      batchMl [batchCnt] = activeMl [i];
      batchCnt++;
      i++;
    }

    // 배치 정보 출력
    StaticJsonDocument<200> pMsg;
    pMsg["event"] = "pump_batch";
    JsonArray arr = pMsg.createNestedArray("pumps");
    for (int j = 0; j < batchCnt; j++) {
      JsonObject p = arr.createNestedObject();
      p["pump"] = batchIdx[j] + 1;
      p["ml"]   = batchMl[j];
    }
    serializeJson(pMsg, Serial);
    Serial.println();

    runPumpsParallel(batchIdx, batchMl, batchCnt);
    if (stopped) return;
    if (!interruptibleDelay(300)) return;
  }

  StaticJsonDocument<128> doneMsg;
  doneMsg["event"] = "done";
  doneMsg["id"]    = o.id;
  serializeJson(doneMsg, Serial);
  Serial.println();

  isMaking = false;
  currentOrderId[0] = '\0';
}

// ── 펠티어 제어 ───────────────────────────────────────────
void controlPeltier(float temp) {
  if (!coolingEnabled || stopped) {
    if (peltierOn) {
      peltierOn = false;
      digitalWrite(RELAY_PIN,  RELAY_OFF);
      digitalWrite(COOLER_PIN, RELAY_OFF);
    }
    return;
  }

  if (!peltierOn && temp > SODA_TARGET_TEMP + TEMP_HYSTERESIS) {
    peltierOn = true;
    digitalWrite(RELAY_PIN,  RELAY_ON);
    digitalWrite(COOLER_PIN, RELAY_ON);
  } else if (peltierOn && temp < SODA_TARGET_TEMP - TEMP_HYSTERESIS) {
    peltierOn = false;
    digitalWrite(RELAY_PIN,  RELAY_OFF);
    digitalWrite(COOLER_PIN, RELAY_OFF);
  }
}

// ── 상태 전송 ─────────────────────────────────────────────
void sendStatus() {
  StaticJsonDocument<224> doc;
  doc["event"]   = "temp";
  doc["temp"]    = serialized(String(currentTemp, 1));
  doc["peltier"] = peltierOn;
  doc["cooling"] = coolingEnabled;
  doc["stopped"] = stopped;
  doc["target"]  = SODA_TARGET_TEMP;
  doc["making"]  = isMaking;
  if (isMaking) doc["current_id"] = currentOrderId;
  doc["queue"]   = queueSize;
  serializeJson(doc, Serial);
  Serial.println();
}