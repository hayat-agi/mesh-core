#include <Arduino.h>
#include "mesh_packet.h"
#include "lora_link.h"
#include "routing.h"
#include "store_forward.h"
#include "mesh_link.h"
#include "debug.h"

#define LOCAL_ADDR 0x0004   // Gateway node
#define GATEWAY_ADDR 0x0004 // from PRD

// Packet types matching PRD
#define PACKET_TYPE_DATA   0
#define PACKET_TYPE_RREQ   1
#define PACKET_TYPE_RREP   2
#define PACKET_TYPE_RERR   3
#define PACKET_TYPE_BEACON 4
#define PACKET_TYPE_ACK    6

// Global sequence number for local node (simplified)
static uint16_t local_seq_num = 0;
static const uint32_t ACK_REPLY_DELAY_MS = 150;

static void logPayloadHex(const Packet& p) {
    Serial.print("[APP_RX] payload hex:");
    for (uint8_t i = 0; i < p.payload_len; i++) {
        Serial.printf(" %02X", p.payload[i]);
    }
    Serial.println();
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

    char text[181];
    uint8_t copyLen = msgLen < sizeof(text) ? msgLen : sizeof(text) - 1;
    memcpy(text, &p.payload[6], copyLen);
    text[copyLen] = '\0';

    bool checksumOk = checksum == p.payload[p.payload_len - 1];
    Serial.printf("[APP_RX] Disaster v4 message=\"%s\" health=%02X %02X %02X %02X household_len=%u checksum=%s\n",
                  text,
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
        char text[221];
        uint8_t copyLen = p.payload_len < sizeof(text) ? p.payload_len : sizeof(text) - 1;
        memcpy(text, p.payload, copyLen);
        text[copyLen] = '\0';
        Serial.printf("[APP_RX] text=\"%s\"\n", text);
    } else {
        Serial.println("[APP_RX] binary payload is not a known app packet");
    }
}

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

    // Pre-install a route to the gateway for quick testing (V1 hack)
    uint32_t now = millis();
    routing_add_or_update(
        GATEWAY_ADDR,
        /*next_hop=*/GATEWAY_ADDR,  // direct for tiny test network
        /*hop_count=*/1,
        /*seq_num=*/0,
        now + 120000,
        /*link_rssi=*/0,
        ROUTE_VALID
    );
}

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
        
        // Dispatch based on packet type
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

            // Execute the decided action
            if (action.type == CTRL_FORWARD_BROADCAST) {
                p.next_hop = 0xFFFF; // Broadcast address
                lora_send_packet(p);
            } else if (action.type == CTRL_FORWARD_UNICAST) {
                p.next_hop = action.next_hop;
                lora_send_packet(p);
            } else if (action.type == CTRL_GENERATE_RREP) {
                Packet rrep;
                packet_init(rrep);
                
                // We are responding to an RREQ. 
                // The source of the original RREQ becomes the destination of our RREP.
                rrep.msg_id = (uint32_t)random(0xFFFFFFFF);
                rrep.src_addr = LOCAL_ADDR;
                rrep.dst_addr = p.src_addr;
                rrep.prev_hop = LOCAL_ADDR;
                rrep.next_hop = action.next_hop;
                rrep.ttl = 7; // Max hops
                rrep.hop_count = 0;
                rrep.seq_num = ++local_seq_num;
                rrep.type = PACKET_TYPE_RREP;
                rrep.ai_priority = 1;
                rrep.payload_len = 0;
                
                lora_send_packet(rrep);
            } else if (action.type == CTRL_DROP) {
                // Packet is consumed/discarded, do nothing
            }
            
        } else if (p.type == PACKET_TYPE_DATA || p.type == PACKET_TYPE_ACK) {
            if (p.type == PACKET_TYPE_DATA) {
                if (p.dst_addr == LOCAL_ADDR) {
                    logIncomingAppPayload(p);
                }
                if (p.dst_addr == LOCAL_ADDR || p.next_hop == LOCAL_ADDR) {
                    delay(ACK_REPLY_DELAY_MS);
                }
            }
            // Let the mesh_link module handle ACKs and local DATA consumption
            mesh_handle_incoming(p, LOCAL_ADDR, now_ms);
            
            // Note: In V1, forwarding of non-local DATA packets is left for future implementation.
        }
    }

    // Periodically process the store-and-forward queue
    sf_process(now_ms, [](Packet& pkt) -> bool {
        // Best-effort forwarding callback for queued DATA
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
            pkt.type = PACKET_TYPE_DATA;
            pkt.src_addr = LOCAL_ADDR;
            pkt.dst_addr = GATEWAY_ADDR;
            pkt.msg_id = (uint32_t)random(0xFFFFFFFF);
            const char* msg = "HELLO";
            pkt.payload_len = 5;
            for(int i=0; i<5; i++) pkt.payload[i] = msg[i];
            
            DBG_PRINTLN("[CLI] Sending test DATA to Gateway...");
            if (mesh_send_unicast(pkt, LOCAL_ADDR, now_ms)) {
                DBG_PRINTLN("[CLI] Send success (ACK received).");
            } else {
                DBG_PRINTLN("[CLI] Send failed (enqueued or no route).");
            }
        }
    }

    // Yield to FreeRTOS watchdog
    delay(1);
}
