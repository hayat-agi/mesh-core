// LifeNetLoRaMeshLite.ino
//
// Bluetooth-free, gateway-free LoRa mesh node firmware.
// Stripped down from LifeNetLoRaMesh.ino to reduce CPU/radio load and
// avoid thermal stress on the ESP32.
//
// What is gone vs. the full sketch:
//   - All NimBLE code (no phone connection, no GATT server, no advertising)
//   - All NVS activation / lockout state
//   - All gateway WiFi + HTTP uplink + heartbeat
//   - BLE TX ring buffer (no consumer)
//
// What is kept:
//   - Full LoRa mesh: routing (AODV), duplicate detection, store-and-forward,
//     ACK-based unicast delivery
//   - Disaster v4 payload logging on inbound DATA packets
//   - Serial CLI for testing (h / r / q / s)
//
// Heat reduction levers applied:
//   - CPU clock dropped to 80 MHz (vs. 240 MHz default)
//   - Bluetooth controller memory released back to the heap (~80 KB)
//     and the BT radio is guaranteed to stay off
//   - WiFi never initialized, so its radio also stays powered down
//
// Set LOCAL_ADDR per node before flashing.

#include <Arduino.h>
#include <esp_bt.h>
#include <string>

#include "mesh_packet.h"
#include "lora_link.h"
#include "routing.h"
#include "store_forward.h"
#include "mesh_link.h"
#include "debug.h"

// ─── Mesh Config ────────────────────────────────────────────────────────────

#define LOCAL_ADDR    0x0002   // TODO: set per node
#define GATEWAY_ADDR  0x0004   // from PRD

#define PACKET_TYPE_DATA    0
#define PACKET_TYPE_RREQ    1
#define PACKET_TYPE_RREP    2
#define PACKET_TYPE_RERR    3
#define PACKET_TYPE_BEACON  4
#define PACKET_TYPE_ACK     6

#ifndef MAX_PAYLOAD_LEN
#define MAX_PAYLOAD_LEN 220
#endif

static const uint32_t RREP_REPLY_DELAY_MS = 150;
static uint16_t local_seq_num = 0;

// ─── Payload Logging Helpers ────────────────────────────────────────────────

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

// ─── Setup ──────────────────────────────────────────────────────────────────

void setup() {
  // Drop CPU to 80 MHz before bringing up peripherals: UART + LoRa work fine
  // at this clock and the chip runs noticeably cooler than at 240 MHz.
  setCpuFrequencyMhz(80);

  // This sketch never uses Bluetooth — release the BT controller's RAM back
  // to the heap (~80 KB) and guarantee the radio stays off.
  esp_bt_controller_mem_release(ESP_BT_MODE_BTDM);

  Serial.begin(115200);
  delay(500);
  Serial.println("\n==================================");
  Serial.println("Life-Net LoRa Mesh Node (Lite) Starting...");
  Serial.printf("Local Address: 0x%04X\n", LOCAL_ADDR);
  Serial.printf("CPU clock    : %u MHz\n", (unsigned)getCpuFrequencyMhz());
  Serial.println("BLE / WiFi   : disabled");
  Serial.println("==================================\n");

  lora_init();
  routing_init();
  dupdet_init();
  sf_init();

  Serial.println("----------------------------------");
  Serial.println("Status: READY (mesh only, no BLE)");
  Serial.println("----------------------------------");
}

// ─── Loop ───────────────────────────────────────────────────────────────────

void loop() {
  uint32_t now_ms = millis();
  Packet p;

  static uint32_t last_hb = 0;
  if (now_ms - last_hb >= 5000) {
    last_hb = now_ms;
    DBG_PRINTF("[HB] Node alive, millis=%u\n", now_ms);
  }

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
        rrep.src_addr = p.dst_addr;
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
      if (p.type == PACKET_TYPE_DATA && p.dst_addr == LOCAL_ADDR) {
        logIncomingAppPayload(p);
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
      pkt.msg_id    = esp_random();
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

  delay(1); // FreeRTOS watchdog
}
