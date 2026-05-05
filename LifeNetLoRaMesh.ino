#include <Arduino.h>
#include <Wire.h>
#include <Preferences.h>
#include <string>
#include <NimBLEDevice.h>

// ─── Gateway Uplink Config ──────────────────────────────────────────────────
//
//  IS_GATEWAY = 1 SADECE Node 4'te aktif.
//  Node 2'de 0 olarak birak; WiFi/HTTP hic derlenmez.
//
#define IS_GATEWAY  1    // 1 = bu node backend'e gonderir | 0 = siradan node

#if IS_GATEWAY
  #include <WiFi.h>
  #include <HTTPClient.h>

  // -- Sadece bu bolumu degistir -----------------------------------------
  #define WIFI_SSID    "Berkay"         // ag adiniz
  #define WIFI_PASS    "brky1234"     // ag sifreniz
  #define BACKEND_IP   "192.168.1.107"    // bilgisayarin LAN IP'si
  #define BACKEND_PORT  5000
  #define GATEWAY_ID   "B0:CB:D8:EE:7E:46"        // backend'deki gateway id
  // -----------------------------------------------------------------------
#endif


#include "mesh_packet.h"
#include "lora_link.h"
#include "routing.h"
#include "store_forward.h"
#include "mesh_link.h"
#include "debug.h"

// ─── Mesh Config ────────────────────────────────────────────────────────────

#define LOCAL_ADDR  0x0004   // TODO: set per node
#define GATEWAY_ADDR 0x0004  // from PRD

#define PACKET_TYPE_DATA    0
#define PACKET_TYPE_RREQ    1
#define PACKET_TYPE_RREP    2
#define PACKET_TYPE_RERR    3
#define PACKET_TYPE_BEACON  4
#define PACKET_TYPE_ACK     6

#ifndef MAX_PAYLOAD_LEN
#define MAX_PAYLOAD_LEN 220  // LoRa SX127x practical limit per hop
#endif

static const uint32_t ACK_REPLY_DELAY_MS = 150;
static const uint32_t RREP_REPLY_DELAY_MS = 150;
static uint16_t local_seq_num = 0;

// ─── BLE / Activation Config ────────────────────────────────────────────────

static const char* DEVICE_NAME   = "ESP32_LifeNet_Node_4";
static const char* SERVICE_UUID  = "12345678-1234-1234-1234-123456789abc";
static const char* CHAR_RX_UUID  = "12345678-1234-1234-1234-123456789abd";
static const char* CHAR_TX_UUID  = "12345678-1234-1234-1234-123456789abe";

static const char*    ACTIVATION_PASSWORD = "ACTIVATE_2026";
static const char*    NVS_NAMESPACE       = "device_cfg";
static const char*    NVS_KEY_ACTIVATED   = "activated";
static const char*    NVS_KEY_FAIL_CNT    = "fail_cnt";
static const char*    NVS_KEY_LOCKED      = "locked";
static const uint8_t  MAX_FAILED_ATTEMPTS = 5;
static const uint32_t LOCKOUT_DURATION_MS = 60000;  // 1 minute

static Preferences nvs;
static bool     deviceActivated = false;
static uint8_t  failedAttempts  = 0;
static uint32_t lockoutStartMs  = 0;
static bool     isLockedOut     = false;

// ─── BLE Globals ────────────────────────────────────────────────────────────

static NimBLECharacteristic* pTxChar = nullptr;
static volatile uint8_t clientCount  = 0;
static volatile uint8_t txSubscribedCount = 0;

static const uint32_t NOTIFY_INTERVAL_MS = 50;
static uint32_t lastNotifyMs = 0;

// basit küçük TX kuyruğu
#define TX_BUF_SLOTS    8
#define TX_MSG_MAX_LEN  64

struct TxRingBuffer {
  char    data[TX_BUF_SLOTS][TX_MSG_MAX_LEN];
  uint8_t head;
  uint8_t tail;
  uint8_t count;
};
static TxRingBuffer txRing;
static portMUX_TYPE txRingMux = portMUX_INITIALIZER_UNLOCKED;

static void txRingInit(TxRingBuffer& rb) {
  rb.head = rb.tail = rb.count = 0;
}
static bool txRingPush(TxRingBuffer& rb, const char* msg) {
  portENTER_CRITICAL(&txRingMux);
  if (rb.count >= TX_BUF_SLOTS) {
    portEXIT_CRITICAL(&txRingMux);
    return false;
  }
  strncpy(rb.data[rb.head], msg, TX_MSG_MAX_LEN - 1);
  rb.data[rb.head][TX_MSG_MAX_LEN - 1] = '\0';
  rb.head = (rb.head + 1) % TX_BUF_SLOTS;
  rb.count++;
  portEXIT_CRITICAL(&txRingMux);
  return true;
}
static bool txRingPop(TxRingBuffer& rb, char* out, uint8_t maxLen) {
  portENTER_CRITICAL(&txRingMux);
  if (rb.count == 0) {
    portEXIT_CRITICAL(&txRingMux);
    return false;
  }
  strncpy(out, rb.data[rb.tail], maxLen - 1);
  out[maxLen - 1] = '\0';
  rb.tail = (rb.tail + 1) % TX_BUF_SLOTS;
  rb.count--;
  portEXIT_CRITICAL(&txRingMux);
  return true;
}
static uint8_t txRingCount(TxRingBuffer& rb) {
  portENTER_CRITICAL(&txRingMux);
  uint8_t c = rb.count;
  portEXIT_CRITICAL(&txRingMux);
  return c;
}
static void queueTxMessage(const char* msg) {
  if (!txRingPush(txRing, msg)) {
    Serial.println("[TX] Ring buffer full — message dropped");
  }
}

static bool isPrintablePayload(const Packet& p) {
  if (p.payload_len == 0) return false;
  for (uint8_t i = 0; i < p.payload_len; i++) {
    uint8_t c = p.payload[i];
    if (c == '\n' || c == '\r' || c == '\t') continue;
    if (c < 32 || c > 126) return false;
  }
  return true;
}

static void logPayloadHex(const Packet& p) {
  Serial.print("[APP_RX] payload hex:");
  for (uint8_t i = 0; i < p.payload_len; i++) {
    Serial.printf(" %02X", p.payload[i]);
  }
  Serial.println();
}

static bool logDisasterV4Payload(const Packet& p) {
  if (p.payload_len < 9 || p.payload[0] != 0xD0) return false;

  uint8_t checksum = 0;
  for (uint8_t i = 0; i < p.payload_len - 1; i++) {
    checksum ^= p.payload[i];
  }

  uint8_t msgLen = p.payload[5];
  uint16_t hhOffset = 6 + msgLen;
  if (hhOffset + 2 >= p.payload_len) {
    Serial.println("[APP_RX] Disaster v4 payload invalid length");
    return true;
  }

  uint16_t hhLen = ((uint16_t)p.payload[hhOffset] << 8) | p.payload[hhOffset + 1];
  uint16_t expectedLen = 9 + msgLen + hhLen;
  if (expectedLen != p.payload_len) {
    Serial.printf("[APP_RX] Disaster v4 length mismatch expected=%u actual=%u\n",
                  (unsigned)expectedLen, (unsigned)p.payload_len);
    return true;
  }

  bool checksumOk = checksum == p.payload[p.payload_len - 1];
  std::string text((const char*)&p.payload[6], msgLen);
  Serial.printf("[APP_RX] Disaster v4 message=\"%s\" health=%02X %02X %02X %02X household_len=%u checksum=%s\n",
                text.c_str(),
                p.payload[1], p.payload[2], p.payload[3], p.payload[4],
                (unsigned)hhLen,
                checksumOk ? "OK" : "BAD");
  return true;
}

static void logIncomingAppPayload(const Packet& p) {
  Serial.printf("[APP_RX] from=0x%04X len=%u\n", p.src_addr, (unsigned)p.payload_len);

  if (p.payload_len == 0) {
    Serial.println("[APP_RX] empty payload");
    return;
  }

  logPayloadHex(p);

  if (logDisasterV4Payload(p)) return;

  if (isPrintablePayload(p)) {
    std::string text((const char*)p.payload, p.payload_len);
    Serial.printf("[APP_RX] text=\"%s\"\n", text.c_str());
  } else {
    Serial.println("[APP_RX] binary payload is not a known app packet");
  }
}

// ─── Gateway WiFi Uplink ─────────────────────────────────────────────────────
//  Sadece IS_GATEWAY=1 olan node'da derlenir.
//  Disaster v4 paketi mesh üzerinden gelince backend'e HTTP POST atar.
#if IS_GATEWAY
static void gatewayUplink(const Packet& p, const std::string& msgText) {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("[UPLINK] WiFi yok, atlaniyor");
    return;
  }

  // JSON body oluştur (ArduinoJson yerine elle — bağımlılık azaltır)
  char body[512];
  char hexSrc[8];
  snprintf(hexSrc, sizeof(hexSrc), "0x%04X", p.src_addr);

  // msgText içindeki çift tırnakları ve backslash'ları escape et
  std::string safeMsg;
  for (char c : msgText) {
    if (c == '"')        safeMsg += "\\\"";
    else if (c == '\\') safeMsg += "\\\\";
    else                 safeMsg += c;
  }

  snprintf(body, sizeof(body),
    "{\"type\":\"manual_message\","
    "\"message\":\"%s\","
    "\"sentAt\":\"%lu\","
    "\"source\":\"mesh\","
    "\"meshHops\":%u,"
    "\"meshSrcAddr\":\"%s\","
    "\"meshMsgId\":\"%X\"}",
    safeMsg.c_str(),
    (unsigned long)millis(),
    (unsigned)p.hop_count,
    hexSrc,
    (unsigned int)p.msg_id
  );

  char url[128];
  snprintf(url, sizeof(url),
    "http://%s:%u/api/gateways/%s/disaster-events",
    BACKEND_IP, (unsigned)BACKEND_PORT, GATEWAY_ID);

  HTTPClient http;
  http.begin(url);
  http.addHeader("Content-Type", "application/json");
  http.addHeader("X-Source",   "mesh-uplink");
  http.addHeader("X-Mesh-Hops", String(p.hop_count));
  http.addHeader("X-Mesh-Src",  hexSrc);
  http.addHeader("X-Mesh-MsgId", String(p.msg_id, HEX));

  int code = http.POST(String(body));
  Serial.printf("[UPLINK] POST %s -> HTTP %d\n", url, code);
  http.end();
}
#endif
// ─── LoRa'ya forward helper ────────────────────────────────────────────────

static void forwardToLoRa(const std::string& msg) {
  Packet pkt;
  packet_init(pkt);
  pkt.type        = PACKET_TYPE_DATA;
  pkt.src_addr    = LOCAL_ADDR;
  pkt.dst_addr    = GATEWAY_ADDR;
  pkt.msg_id = esp_random();
  pkt.ttl         = 5;
  pkt.hop_count   = 0;
  pkt.payload_len = (msg.size() > MAX_PAYLOAD_LEN)
                    ? MAX_PAYLOAD_LEN
                    : (uint8_t)msg.size();
  memcpy(pkt.payload, msg.data(), pkt.payload_len);

  bool ok = mesh_send_unicast(pkt, LOCAL_ADDR, millis());
  Serial.printf("[LoRa] Forwarded BLE msg, status=%s\n", ok ? "OK" : "FAIL/QUEUED");
}

// ─── NVS helpers (sadece activated flag) ────────────────────────────────────

static bool nvsInit() {
  if (!nvs.begin(NVS_NAMESPACE, false)) {
    Serial.println("[NVS] ERROR — failed to open namespace");
    return false;
  }
  return true;
}
static bool nvsLoadActivated() {
  return nvs.getBool(NVS_KEY_ACTIVATED, false);
}
static bool nvsSaveActivated(bool val) {
  if (!nvs.putBool(NVS_KEY_ACTIVATED, val)) {
    Serial.println("[NVS] ERROR — failed to write activated flag");
    return false;
  }
  return true;
}
static void nvsSaveFailState(uint8_t attempts, bool locked) {
  nvs.putUChar(NVS_KEY_FAIL_CNT, attempts);
  nvs.putBool(NVS_KEY_LOCKED, locked);
}
static void nvsClearFailState() {
  nvs.putUChar(NVS_KEY_FAIL_CNT, 0);
  nvs.putBool(NVS_KEY_LOCKED, false);
}

// ─── BLE callbacks ─────────────────────────────────────────────────────────

static void startAdvertising();

static void trimTrailing(std::string& s) {
  while (!s.empty()) {
    char c = s.back();
    if (c == ' ' || c == '\n' || c == '\r' || c == '\t') s.pop_back();
    else break;
  }
}

class RxCallbacks : public NimBLECharacteristicCallbacks {
  void onWrite(NimBLECharacteristic* pChar, NimBLEConnInfo&) override {
    if (clientCount == 0) return;

    std::string rxValue = pChar->getValue();
    Serial.printf("[RX] \"%s\" (%u bytes)\n", rxValue.c_str(),
                  (unsigned)rxValue.length());

    // Detect binary: raw 0xD0-prefixed disaster packet OR hex-encoded "BIN:" payload
    bool isBinary = (!rxValue.empty() && (uint8_t)rxValue[0] == 0xD0)
                 || (rxValue.size() >= 4 && rxValue.substr(0, 4) == "BIN:");

    std::string trimmed = rxValue;
    if (!isBinary) {
      trimTrailing(trimmed);
    }

    if (!deviceActivated) {
      if (isBinary ||
          trimmed == "PING" ||
          trimmed == "GET_DEVICE_COUNT" ||
          trimmed == "FACTORY_RESET" ||
          trimmed.rfind("REGISTER:", 0) == 0) {
        queueTxMessage("NEED_ACTIVATION");
        return;
      }
      handleActivation(trimmed);
    } else {
      handleCommand(trimmed);
    }
  }

private:
  // ── Activation branch (device not yet activated) ─────────────────────────
  void handleActivation(const std::string& input) {
    // Lockout check: too many failed attempts
    if (isLockedOut) {
      uint32_t elapsed = millis() - lockoutStartMs;
      if (elapsed < LOCKOUT_DURATION_MS) {
        uint32_t remaining = (LOCKOUT_DURATION_MS - elapsed) / 1000;
        Serial.printf("[AUTH] Locked out — %u seconds remaining\n", remaining);
        char buf[48];
        snprintf(buf, sizeof(buf), "LOCKED_%u", remaining);
        queueTxMessage(buf);
        return;
      }
      // Lockout expired — clear and fall through
      isLockedOut    = false;
      failedAttempts = 0;
      nvsClearFailState();
    }

    if (input == ACTIVATION_PASSWORD) {
      Serial.println("[AUTH] Activation password CORRECT");
      if (nvsSaveActivated(true)) {
        deviceActivated = true;
        nvsClearFailState();
        queueTxMessage("ACTIVATED");
        Serial.println("[AUTH] Device activated");
      } else {
        queueTxMessage("NVS_ERROR");
        Serial.println("[AUTH] NVS write failed — not activating");
      }
    } else {
      failedAttempts++;
      Serial.printf("[AUTH] Wrong password (attempt %d/%d)\n",
                    failedAttempts, MAX_FAILED_ATTEMPTS);

      if (failedAttempts >= MAX_FAILED_ATTEMPTS) {
        isLockedOut    = true;
        lockoutStartMs = millis();
        nvsSaveFailState(failedAttempts, true);
        Serial.println("[AUTH] Too many attempts — locked for 60s");
        queueTxMessage("LOCKED_60");
      } else {
        nvsSaveFailState(failedAttempts, false);
        char buf[32];
        snprintf(buf, sizeof(buf), "WRONG_PW_%d",
                 MAX_FAILED_ATTEMPTS - failedAttempts);
        queueTxMessage(buf);
      }
    }
  }

  // ── Command branch (device activated) ────────────────────────────────────
  void handleCommand(const std::string& input) {
    Serial.printf("[CMD] \"%s\"\n", input.c_str());

    // 0a. BIN:<hex> — hex decode et, ham binary olarak ilet
    if (input.size() >= 4 && input.substr(0, 4) == "BIN:") {
      const std::string hexStr = input.substr(4);
      std::string binary;
      binary.reserve(hexStr.size() / 2);
      for (size_t i = 0; i + 1 < hexStr.size(); i += 2) {
        char hi = hexStr[i], lo = hexStr[i + 1];
        auto hexVal = [](char c) -> uint8_t {
          if (c >= '0' && c <= '9') return c - '0';
          if (c >= 'a' && c <= 'f') return c - 'a' + 10;
          if (c >= 'A' && c <= 'F') return c - 'A' + 10;
          return 0;
        };
        binary += (char)((hexVal(hi) << 4) | hexVal(lo));
      }
      if (binary.size() > MAX_PAYLOAD_LEN) {
        Serial.printf("[CMD] Payload too large (%u > %u) — rejecting\n",
                      (unsigned)binary.size(), (unsigned)MAX_PAYLOAD_LEN);
        queueTxMessage("MSG_BAD_LEN");
        return;
      }
      Serial.printf("[CMD] BIN decoded: %u bytes, first=0x%02X\n",
                    (unsigned)binary.size(), binary.empty() ? 0 : (uint8_t)binary[0]);
      // MSG_OK önce — forwardToLoRa blocking (max 6sn), mobil 5sn'de timeout yapar
      queueTxMessage("MSG_OK");
      forwardToLoRa(binary);
      return;
    }

    // 0b. Raw 0xD0-prefixed binary (doğrudan binary gönderen platformlar)
    if (!input.empty() && (uint8_t)input[0] == 0xD0) {
      if (input.size() > MAX_PAYLOAD_LEN) {
        Serial.printf("[CMD] Payload too large (%u > %u) — rejecting\n",
                      (unsigned)input.size(), (unsigned)MAX_PAYLOAD_LEN);
        queueTxMessage("MSG_BAD_LEN");
        return;
      }
      queueTxMessage("MSG_OK");
      forwardToLoRa(input);
      return;
    }

    // 1. Heartbeat — must not reach LoRa
    if (input == "PING") {
      queueTxMessage("PONG");
      return;
    }

    // 2. Device count — LoRa mesh has no registry; return 0
    if (input == "GET_DEVICE_COUNT") {
      queueTxMessage("DEVICE_COUNT_0");
      return;
    }

    // 3. Factory reset — clear NVS, do not forward to LoRa
    if (input == "FACTORY_RESET") {
      nvsSaveActivated(false);
      nvsClearFailState();
      deviceActivated = false;
      failedAttempts  = 0;
      isLockedOut     = false;
      queueTxMessage("RESET_OK");
      Serial.println("[CMD] Factory reset — device deactivated");
      return;
    }

    // 4. Device registration — acknowledge locally, skip LoRa flood
    if (input.rfind("REGISTER:", 0) == 0) {
      queueTxMessage("MSG_OK");
      Serial.printf("[CMD] REGISTER intercepted: %s\n", input.c_str());
      return;
    }

    // 5. All other messages — forward to LoRa mesh
    if (!input.empty()) {
      if (input.size() > MAX_PAYLOAD_LEN) {
        Serial.printf("[CMD] Payload too large (%u > %u) — rejecting\n",
                      (unsigned)input.size(), (unsigned)MAX_PAYLOAD_LEN);
        queueTxMessage("MSG_BAD_LEN");
        return;
      }
      // MSG_OK önce gönder (aynı sebep: forwardToLoRa blocking)
      queueTxMessage("MSG_OK");
      forwardToLoRa(input);
    }
  }
};

class TxCallbacks : public NimBLECharacteristicCallbacks {
  void onSubscribe(NimBLECharacteristic*, NimBLEConnInfo& info,
                   uint16_t subValue) override {
    if (subValue) {
      if (txSubscribedCount < clientCount) txSubscribedCount++;
      Serial.printf("[CCCD] Client subscribed (conn=%u)\n", info.getConnHandle());
      if (!deviceActivated) {
        queueTxMessage("NEED_ACTIVATION");
      }
    } else {
      if (txSubscribedCount > 0) txSubscribedCount--;
      Serial.printf("[CCCD] Client unsubscribed (conn=%u)\n", info.getConnHandle());
    }
  }
};

class ServerCallbacks : public NimBLEServerCallbacks {
  void onConnect(NimBLEServer*, NimBLEConnInfo& info) override {
    clientCount++;
    Serial.printf("[BLE] Client connected (%d total)\n", clientCount);
    if (clientCount < 3) {
      startAdvertising();
    }
  }
  void onDisconnect(NimBLEServer*, NimBLEConnInfo& info, int reason) override {
    if (clientCount > 0) clientCount--;
    if (clientCount == 0) {
      txSubscribedCount = 0;
    } else if (txSubscribedCount > clientCount) {
      txSubscribedCount = clientCount;
    }
    Serial.printf("[BLE] Disconnected (reason %d, %d remain)\n", reason, clientCount);
    startAdvertising();
  }
};

static void setupAdvertising() {
  NimBLEAdvertising* pAdv = NimBLEDevice::getAdvertising();
  pAdv->addServiceUUID(SERVICE_UUID);
  pAdv->setName(DEVICE_NAME);
  pAdv->enableScanResponse(true);
}
static void startAdvertising() {
  NimBLEAdvertising* pAdv = NimBLEDevice::getAdvertising();
  if (pAdv->isAdvertising()) return;
  if (pAdv->start()) {
    Serial.println("[ADV] Advertising started");
  } else {
    Serial.println("[ADV] ERROR — failed to start");
  }
}

// ─── Orijinal setup() + BLE ekle ───────────────────────────────────────────

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n==================================");
  Serial.println("Life-Net LoRa Mesh Node Starting...");
  Serial.printf("Local Address: 0x%04X\n", LOCAL_ADDR);
  Serial.println("==================================\n");

  lora_init();
  routing_init();
  dupdet_init();
  sf_init();

#if IS_GATEWAY
  // WiFi bağlantısı — backend uplink için gerekli
  Serial.printf("[WiFi] Connecting to %s ...\n", WIFI_SSID);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  uint32_t wifiStart = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - wifiStart < 15000) {
    delay(500);
    Serial.print('.');
  }
  Serial.println();
  if (WiFi.status() == WL_CONNECTED) {
    Serial.printf("[WiFi] Connected! IP: %s\n", WiFi.localIP().toString().c_str());
  } else {
    Serial.println("[WiFi] Connection FAILED — uplink disabled until reconnect");
  }
#endif

  uint32_t now = millis();
  routing_add_or_update(
      GATEWAY_ADDR,
      0x0004,         // Node 2 icin burasi boyle olacak Aracı / Next Hop (NODE 1)
      1,              // hop_count
      0,              // seq_num
      now + 604800000, // node 
      0,
      ROUTE_VALID
  );

  // BLE + NVS init (LoRa'yı yukarıda bıraktık)
  txRingInit(txRing);

  if (nvsInit()) {
    deviceActivated = nvsLoadActivated();
    Serial.printf("[NVS] Device %s\n", deviceActivated ? "ACTIVATED" : "NOT ACTIVATED");

    // Restore fail/lockout state so a power-cycle cannot bypass the lockout
    failedAttempts = nvs.getUChar(NVS_KEY_FAIL_CNT, 0);
    isLockedOut    = nvs.getBool(NVS_KEY_LOCKED, false);
    if (isLockedOut) {
      lockoutStartMs = millis();  // restart lockout timer from boot
      Serial.println("[AUTH] Lockout restored from NVS — 60s lockout restarted");
    } else if (failedAttempts > 0) {
      Serial.printf("[AUTH] %u failed attempt(s) restored from NVS\n", failedAttempts);
    }
  } else {
    Serial.println("[NVS] Init failed — running in UNACTIVATED mode");
    deviceActivated = false;
  }

  NimBLEDevice::init(DEVICE_NAME);
  NimBLEDevice::setPower(ESP_PWR_LVL_P3);

  NimBLEServer* pServer = NimBLEDevice::createServer();
  pServer->setCallbacks(new ServerCallbacks());

  NimBLEService* pService = pServer->createService(SERVICE_UUID);

  NimBLECharacteristic* pRxChar = pService->createCharacteristic(
    CHAR_RX_UUID,
    NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR
  );
  pRxChar->setCallbacks(new RxCallbacks());

  pTxChar = pService->createCharacteristic(
    CHAR_TX_UUID,
    NIMBLE_PROPERTY::NOTIFY | NIMBLE_PROPERTY::READ
  );
  pTxChar->setCallbacks(new TxCallbacks());

  pService->start();
  setupAdvertising();
  startAdvertising();

  Serial.println("----------------------------------");
  Serial.printf("  Service  : %s\n", SERVICE_UUID);
  Serial.printf("  RX Char  : %s\n", CHAR_RX_UUID);
  Serial.printf("  TX Char  : %s\n", CHAR_TX_UUID);
  Serial.printf("  Status   : %s\n", deviceActivated ? "READY" : "AWAITING ACTIVATION");
  Serial.println("----------------------------------");
}

// ─── Orijinal loop() + en sona BLE TX drain ────────────────────────────────

void loop() {
  uint32_t now_ms = millis();
  Packet p;

  static uint32_t last_hb = 0;
  if (now_ms - last_hb >= 5000) {
      last_hb = now_ms;
      DBG_PRINTF("[HB] Node alive, millis=%u\n", now_ms);
  }

  // Try to receive a packet with a small timeout to not block too long
  if (lora_receive_packet(p, 50)) {
      DBG_PRINTF("[RX] type=%d src=0x%04X dst=0x%04X prev=0x%04X next=0x%04X ttl=%d hop=%d\n",
                 p.type, p.src_addr, p.dst_addr, p.prev_hop, p.next_hop, p.ttl, p.hop_count);
      
      if (p.type == PACKET_TYPE_RREQ || p.type == PACKET_TYPE_RREP || p.type == PACKET_TYPE_RERR) {
          ControlAction action;
          action.type = CTRL_DROP;
          
          if (p.type == PACKET_TYPE_RREQ) {
              action = handle_rreq(p, LOCAL_ADDR, now_ms);
          } else if (p.type == PACKET_TYPE_RREP) {
              action = handle_rrep(p, LOCAL_ADDR, now_ms);
          } else if (p.type == PACKET_TYPE_RERR) {
              action = handle_rerr(p, LOCAL_ADDR, now_ms);
          }

          if (action.type == CTRL_FORWARD_BROADCAST) {
              p.next_hop = 0xFFFF;
              lora_send_packet(p);
          } else if (action.type == CTRL_FORWARD_UNICAST) {
              p.next_hop = action.next_hop;
              lora_send_packet(p);
          } else if (action.type == CTRL_GENERATE_RREP) {
              Packet rrep;
              packet_init(rrep);
              rrep.msg_id = esp_random();
              rrep.src_addr = LOCAL_ADDR;
              rrep.dst_addr = p.src_addr;
              rrep.prev_hop = LOCAL_ADDR;
              rrep.next_hop = action.next_hop;
              rrep.ttl      = 7;
              rrep.hop_count   = 0;
              rrep.seq_num     = ++local_seq_num;
              rrep.type        = PACKET_TYPE_RREP;
              rrep.ai_priority = 1;
              rrep.payload_len = 0;
              Serial.printf("[ROUTING] Sending RREP to 0x%04X for origin=0x%04X\n",
                            rrep.next_hop, rrep.dst_addr);
              delay(RREP_REPLY_DELAY_MS);
              bool rrepOk = lora_send_packet(rrep);
              Serial.printf("[ROUTING] RREP send status=%s\n", rrepOk ? "OK" : "FAIL");
          }
      } else if (p.type == PACKET_TYPE_DATA || p.type == PACKET_TYPE_ACK) {
          if (p.type == PACKET_TYPE_DATA) {
              if (p.dst_addr == LOCAL_ADDR) {
                  logIncomingAppPayload(p);
#if IS_GATEWAY
                  // Disaster v4 paketi mi? Varsa backend'e ilet.
                  if (p.payload_len >= 9 && p.payload[0] == 0xD0) {
                      uint8_t msgLen = p.payload[5];
                      std::string msgText((const char*)&p.payload[6], msgLen);
                      gatewayUplink(p, msgText);
                  }
#endif
              }
              // ACK_REPLY_DELAY_MS artık mesh_link.cpp içinde send_data_ack'te uygulanıyor
              // Burada ekstra delay olmadan mesh_handle_incoming'e bırakıyoruz
          }
          mesh_handle_incoming(p, LOCAL_ADDR, now_ms);
      }
  }

  // Periodically process the store-and-forward queue
  sf_process(now_ms, [](Packet& pkt) -> bool {
      return mesh_retry_queued_packet(pkt, LOCAL_ADDR, millis());
  });

  // Simple Serial CLI
  if (Serial.available()) {
      char cmd = Serial.read();
      if (cmd == 'h') {
          DBG_PRINTLN("Commands:\n  h - help\n  r - dump routing table\n  q - dump store-and-forward queue\n  s - send test DATA to gateway");
      } else if (cmd == 'r') {
          routing_debug_dump(now_ms);
      } else if (cmd == 'q') {
          sf_debug_dump(now_ms);
      } else if (cmd == 's') {
          Packet pkt;
          packet_init(pkt);
          pkt.type      = PACKET_TYPE_DATA;
          pkt.src_addr  = LOCAL_ADDR;
          pkt.dst_addr  = GATEWAY_ADDR;
          pkt.msg_id = esp_random();
          pkt.ttl       = 7;
          const char* msg = "HELLO";
          pkt.payload_len = 5;
          for (int i = 0; i < 5; i++) pkt.payload[i] = msg[i];

          DBG_PRINTLN("[CLI] Sending test DATA to Gateway...");
          if (mesh_send_unicast(pkt, LOCAL_ADDR, now_ms)) {
              DBG_PRINTLN("[CLI] Send success (ACK received).");
          } else {
              DBG_PRINTLN("[CLI] Send failed (enqueued or no route).");
          }
      }
  }

  // BLE TX kuyruğunu ara ara boşalt
  if (clientCount > 0 && txSubscribedCount > 0 && pTxChar && txRingCount(txRing) > 0) {
    if (now_ms - lastNotifyMs >= NOTIFY_INTERVAL_MS) {
      char msg[TX_MSG_MAX_LEN];
      if (txRingPop(txRing, msg, sizeof(msg))) {
        pTxChar->setValue((uint8_t*)msg, strlen(msg));
        if (pTxChar->notify()) {
          Serial.printf("[TX] \"%s\"\n", msg);
        } else {
          Serial.println("[TX] notify() failed");
        }
        lastNotifyMs = now_ms;
      }
    }
  }

  delay(1); // FreeRTOS watchdog
}
