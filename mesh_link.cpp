#include "mesh_link.h"
#include "lora_link.h"
#include "routing.h"
#include "store_forward.h"
#include "debug.h"
#include <Arduino.h>

#define ACK_TIMEOUT_MS 2000
#define MAX_TX_RETRIES 3
#define ACK_REPLY_DELAY_MS 150

// --- PAKET TİPLERİ ---
#define PACKET_TYPE_DATA   0
#define PACKET_TYPE_RREQ   1
#define PACKET_TYPE_RREP   2
#define PACKET_TYPE_RERR   3
#define PACKET_TYPE_BEACON 4
#define PACKET_TYPE_ACK    6

#define RREQ_RATE_LIMIT_MS 5000

struct RreqRateLimit {
    uint16_t dst;
    uint32_t last_time;
};
static RreqRateLimit rreq_limits[4];
static uint8_t rreq_limit_idx = 0;

static void send_data_ack(const Packet& p, uint16_t local_addr, const char* reason) {
    Packet ack;
    packet_init(ack);
    ack.msg_id      = p.msg_id;
    ack.src_addr    = local_addr;
    ack.dst_addr    = p.prev_hop;
    ack.prev_hop    = local_addr;
    ack.next_hop    = p.prev_hop;
    ack.type        = PACKET_TYPE_ACK;
    ack.ttl         = 1;
    ack.hop_count   = 0;
    ack.payload_len = 0;

    DBG_PRINTF("[MESH_RX] DATA %s, sending ACK to 0x%04X\n", reason, ack.dst_addr);
    delay(ACK_REPLY_DELAY_MS);
    lora_send_packet(ack);
}

bool can_send_rreq(uint16_t dst, uint32_t now_ms) {
    for(int i = 0; i < 4; i++) {
        if(rreq_limits[i].dst == dst) {
            if(now_ms - rreq_limits[i].last_time < RREQ_RATE_LIMIT_MS) return false;
            rreq_limits[i].last_time = now_ms;
            return true;
        }
    }
    rreq_limits[rreq_limit_idx].dst = dst;
    rreq_limits[rreq_limit_idx].last_time = now_ms;
    rreq_limit_idx = (rreq_limit_idx + 1) % 4;
    return true;
}

// ACK bekleme süresinde gelen ama ACK olmayan paketleri tutan geçici buffer
#define SIDE_PKT_BUF_SIZE 4
static Packet s_side_pkts[SIDE_PKT_BUF_SIZE];
static uint8_t s_side_pkt_count = 0;

static bool mesh_send_unicast_internal(Packet& p, uint16_t local_addr, uint32_t now_ms, bool allow_enqueue) {
    RouteEntry route;

    DBG_PRINTF("[MESH_TX] dst=0x%04X msg_id=%08X now=%u allow_enqueue=%d\n",
               p.dst_addr, (unsigned int)p.msg_id, now_ms, (int)allow_enqueue);

    if (!routing_lookup(p.dst_addr, route, now_ms)) {
        DBG_PRINTLN("[MESH_TX] no valid route, triggering RREQ and enqueueing");

        if (allow_enqueue) {
            sf_enqueue(p, now_ms);
        }
        if (can_send_rreq(p.dst_addr, now_ms)) {
            Packet rreq;
            packet_init(rreq);
            rreq.type      = PACKET_TYPE_RREQ;
            rreq.src_addr  = local_addr;      // Rota isteğini başlatan (Originator)
            rreq.dst_addr  = p.dst_addr;      // Aranan hedef
            rreq.prev_hop  = local_addr;
            rreq.next_hop  = 0xFFFF;          // 0xFFFF: Broadcast (Herkese duyur)
            rreq.msg_id    = esp_random();    // Eşsiz RREQ kimliği
            rreq.ttl       = 7;               // Maksimum sekme ömrü
            rreq.hop_count = 0;

            DBG_PRINTF("[MESH_TX] Broadcasted RREQ for dst=0x%04X\n", p.dst_addr);
            lora_send_packet(rreq);
        }
        return false;
    }

    DBG_PRINTF("[MESH_TX] route lookup -> next=0x%04X hop=%u state=%d\n",
               route.next_hop, route.hop_count, (int)route.state);

    p.prev_hop = local_addr;
    p.next_hop = route.next_hop;

    bool ack_received = false;
    for (int attempt = 0; attempt < MAX_TX_RETRIES && !ack_received; ++attempt) {
        DBG_PRINTF("[MESH_TX] attempt %d to next=0x%04X\n", attempt + 1, route.next_hop);

        if (!lora_send_packet(p)) {
            DBG_PRINTLN("[MESH_TX] lora_send_packet failed locally, retrying");
            continue;
        }

        // ACK_TIMEOUT_MS süresi içinde sadece ACK'i bekle.
        // Diğer gelen paketleri hemen işlemek yerine buffer'a al;
        // çünkü lora_send_packet (TX) henuz bitmiş olabilir ve SX127x'te
        // RX için tekrar UART döngüsüne girmeyi bekliyoruz.
        uint32_t ack_deadline = millis() + ACK_TIMEOUT_MS;
        while ((int32_t)(ack_deadline - millis()) > 0) {
            Packet rx;
            uint32_t remaining = (uint32_t)(ack_deadline - millis());
            if (remaining == 0) break;
            // Küçük pencerelerle polla — uzun blocking'den kaçın
            uint32_t poll_ms = (remaining > 50) ? 50 : remaining;
            if (!lora_receive_packet(rx, poll_ms)) continue;

            DBG_PRINTF("[MESH_TX] got packet while waiting for ACK: type=%d src=0x%04X dst=0x%04X msg_id=%08X\n",
                       rx.type, rx.src_addr, rx.dst_addr, (unsigned int)rx.msg_id);

            if (rx.type == PACKET_TYPE_ACK &&
                rx.msg_id == p.msg_id &&
                rx.src_addr == route.next_hop &&
                rx.dst_addr == local_addr) {
                DBG_PRINTLN("[MESH_TX] ACK matched, send success");
                ack_received = true;
                break;
            } else {
                // ACK değil — kaybolmasın diye buffer'a al, sonra işle
                if (s_side_pkt_count < SIDE_PKT_BUF_SIZE) {
                    s_side_pkts[s_side_pkt_count++] = rx;
                } else {
                    DBG_PRINTLN("[MESH_TX] side-packet buffer full, dropping");
                }
            }
        }

        if (!ack_received) {
            DBG_PRINTLN("[MESH_TX] ACK timeout expired");
        }
    }

    // ACK döngüsü bitti — önce buffer'daki yan paketleri işle
    for (uint8_t i = 0; i < s_side_pkt_count; i++) {
        mesh_handle_incoming(s_side_pkts[i], local_addr, millis());
    }
    s_side_pkt_count = 0;

    if (ack_received) return true;

    DBG_PRINTLN("[MESH_TX] all retries failed, invalidating route and considering enqueue");
    routing_invalidate(p.dst_addr);

    if (allow_enqueue) {
        bool qok = sf_enqueue(p, now_ms);
        DBG_PRINTF("[MESH_TX] sf_enqueue (after retries) result=%d\n", (int)qok);
    }

    return false;
}

bool mesh_send_unicast(Packet& p, uint16_t local_addr, uint32_t now_ms) {
    return mesh_send_unicast_internal(p, local_addr, now_ms, true);
}

bool mesh_retry_queued_packet(Packet& p, uint16_t local_addr, uint32_t now_ms) {
    return mesh_send_unicast_internal(p, local_addr, now_ms, false);
}

bool mesh_handle_incoming(Packet& p, uint16_t local_addr, uint32_t now_ms) {
    if (p.type == PACKET_TYPE_DATA) {
        if (dupdet_is_duplicate(p.msg_id, p.src_addr)) {
            DBG_PRINTF("[MESH_RX] duplicate DATA received src=0x%04X msg_id=%08X\n",
                       p.src_addr, (unsigned int)p.msg_id);
            if (p.dst_addr == local_addr || p.next_hop == local_addr) {
                send_data_ack(p, local_addr, "duplicate received");
            }
            return false;
        }
    }

    // 1. ACK GÖNDERME: Hedef biz isek VEYA bir sonraki kurye biz isek ACK atmalıyız!
    if (p.type == PACKET_TYPE_DATA && (p.dst_addr == local_addr || p.next_hop == local_addr)) {
        send_data_ack(p, local_addr, "received");

        // Eğer nihai hedef BİZ isek (Gateway), veriyi işledik demektir.
        if (p.dst_addr == local_addr) {
            DBG_PRINTLN("[MESH_RX] Packet arrived at FINAL DESTINATION!");
            return true;
        }
    }

    // 2. KURYELİK (FORWARDING): Hedef biz değilsek ama paket bize emanet edilmişse
    if (p.type == PACKET_TYPE_DATA && p.dst_addr != local_addr && p.next_hop == local_addr) {
        if (p.ttl <= 1) {
            DBG_PRINTLN("[MESH_FWD] dropping DATA due to TTL<=1");
            return false;
        }

        p.ttl      -= 1;
        p.hop_count += 1;

        RouteEntry route;
        if (!routing_lookup(p.dst_addr, route, now_ms)) {
            DBG_PRINTLN("[MESH_FWD] no route for transit DATA, not forwarding");
            return false;
        }

        DBG_PRINTF("[MESH_FWD] forwarding transit DATA to next=0x%04X\n", route.next_hop);
        return mesh_send_unicast(p, local_addr, now_ms);
    }

    if (p.type == PACKET_TYPE_ACK && p.dst_addr == local_addr) {
        DBG_PRINTLN("[MESH_RX] stray ACK for us, consuming");
        return true;
    }

    return false;
}
