#include "routing.h"
#include "store_forward.h"
#include "lora_link.h"
#include "debug.h"

#define ROUTE_LIFETIME_MS 3600000

extern void mesh_retry_queued_packet(uint16_t dst_addr);

static RouteEntry routing_table[ROUTING_TABLE_SIZE];

void routing_init() {
    for (int i = 0; i < ROUTING_TABLE_SIZE; ++i) {
        routing_table[i].in_use = false;
    }
}

void routing_purge_expired(uint32_t now_ms) {
    for (int i = 0; i < ROUTING_TABLE_SIZE; ++i) {
        if (routing_table[i].in_use) {
            if ((int32_t)(now_ms - routing_table[i].lifetime) >= 0) {
                routing_table[i].in_use = false;
            }
        }
    }
}

bool routing_lookup(uint16_t dst, RouteEntry& out, uint32_t now_ms) {
    routing_purge_expired(now_ms); 
    
    for (int i = 0; i < ROUTING_TABLE_SIZE; ++i) {
        if (routing_table[i].in_use && routing_table[i].destination == dst) {
            if (routing_table[i].state == ROUTE_VALID) {
                out = routing_table[i];
                return true;
            }
        }
    }
    return false;
}

bool routing_add_or_update(uint16_t dst, uint16_t next_hop, uint8_t hop_count, uint16_t seq_num, uint32_t lifetime_ms, int8_t link_rssi, RouteState state) {
    int update_idx = -1;
    int free_idx = -1;
    int invalid_idx = -1;

    for (int i = 0; i < ROUTING_TABLE_SIZE; ++i) {
        if (routing_table[i].in_use) {
            if (routing_table[i].destination == dst) {
                update_idx = i;
                break; 
            } else if (routing_table[i].state == ROUTE_INVALID && invalid_idx == -1) {
                invalid_idx = i; 
            }
        } else if (free_idx == -1) {
            free_idx = i; 
        }
    }

    int target_idx = -1;
    if (update_idx != -1) target_idx = update_idx;
    else if (free_idx != -1) target_idx = free_idx;
    else if (invalid_idx != -1) target_idx = invalid_idx;
    else return false; 

    // BUG 4 fix: do not overwrite a valid route with stale information
    if (update_idx != -1) {
        RouteEntry& existing = routing_table[update_idx];
        if (existing.state == ROUTE_VALID) {
            if (seq_num < existing.seq_num) return false;
            if (seq_num == existing.seq_num && hop_count >= existing.hop_count) return false;
        }
    }

    routing_table[target_idx].destination = dst;
    routing_table[target_idx].next_hop = next_hop;
    routing_table[target_idx].hop_count = hop_count;
    routing_table[target_idx].seq_num = seq_num;
    routing_table[target_idx].lifetime = lifetime_ms; 
    routing_table[target_idx].link_rssi = link_rssi;
    routing_table[target_idx].state = state;
    routing_table[target_idx].in_use = true;

    return true;
}

void routing_invalidate(uint16_t dst) {
    for (int i = 0; i < ROUTING_TABLE_SIZE; ++i) {
        if (routing_table[i].in_use && routing_table[i].destination == dst) {
            routing_table[i].state = ROUTE_INVALID;
            break;
        }
    }
}

// ==========================================
// Duplicate Detection Implementation
// ==========================================
static DupEntry dup_buffer[DUP_BUFFER_SIZE];
static int dup_head = 0;

void dupdet_init() {
    for (int i = 0; i < DUP_BUFFER_SIZE; ++i) dup_buffer[i].in_use = false;
    dup_head = 0;
}

bool dupdet_is_duplicate(uint32_t msg_id, uint16_t src_addr) {
    for (int i = 0; i < DUP_BUFFER_SIZE; ++i) {
        if (dup_buffer[i].in_use && dup_buffer[i].msg_id == msg_id && dup_buffer[i].src_addr == src_addr) return true; 
    }
    dup_buffer[dup_head].msg_id = msg_id;
    dup_buffer[dup_head].src_addr = src_addr;
    dup_buffer[dup_head].in_use = true;
    dup_head = (dup_head + 1) % DUP_BUFFER_SIZE;
    return false; 
}

// ==========================================
// AODV Control Message Handlers
// ==========================================
ControlAction handle_rreq(Packet& p, uint16_t local_addr, uint32_t now_ms) {
    ControlAction action;
    action.type = CTRL_DROP;
    action.next_hop = 0;

    // Kendi yayınladığımız RREQ'i echo olarak alabiliyoruz — yoksay.
    if (p.src_addr == local_addr) return action;

    if (dupdet_is_duplicate(p.msg_id, p.src_addr)) return action;

    uint8_t new_hop_count = p.hop_count + 1;
    routing_add_or_update(p.src_addr, p.prev_hop, new_hop_count, p.seq_num, now_ms + ROUTE_LIFETIME_MS, 0, ROUTE_VALID);

    if (p.dst_addr == local_addr) {
        action.type = CTRL_GENERATE_RREP;
        action.next_hop = p.prev_hop;
        return action;
    }

    RouteEntry route;
    if (routing_lookup(p.dst_addr, route, now_ms)) {
        action.type = CTRL_GENERATE_RREP;
        action.next_hop = p.prev_hop; 
        return action;
    }

    if (p.ttl <= 1) return action;

    p.ttl -= 1;
    p.prev_hop = local_addr;
    p.hop_count = new_hop_count; 
    action.type = CTRL_FORWARD_BROADCAST;
    return action;
}

ControlAction handle_rrep(Packet& p, uint16_t local_addr, uint32_t now_ms) {
    ControlAction action;
    action.type = CTRL_DROP;
    action.next_hop = 0;

    // Güvenlik: kendi adresimizden gelen RREP'i yoksay.
    // Bu durum genellikle kendi RREQ'imizin echo olarak geri dönmesi ve
    // yanlış type parse edilmesinden kaynaklanır.
    if (p.src_addr == local_addr) {
        DBG_PRINTF("[ROUTING] RREP from self (0x%04X) ignored\n", p.src_addr);
        return action;
    }

    if (dupdet_is_duplicate(p.msg_id, p.src_addr)) return action;

    uint8_t new_hop_count = p.hop_count + 1;
    routing_add_or_update(p.src_addr, p.prev_hop, new_hop_count, p.seq_num, now_ms + ROUTE_LIFETIME_MS, 0, ROUTE_VALID);

    if (p.dst_addr == local_addr) {
        DBG_PRINTF("[ROUTING] RREP received! Route established to 0x%04X\n", p.src_addr);
        mesh_retry_queued_packet(p.src_addr); 
        return action; 
    }

    RouteEntry reverse_route;
    if (routing_lookup(p.dst_addr, reverse_route, now_ms)) {
        if (p.ttl <= 1) return action;
        p.ttl -= 1;
        p.prev_hop = local_addr;
        p.hop_count = new_hop_count;
        action.type = CTRL_FORWARD_UNICAST;
        action.next_hop = reverse_route.next_hop;
        return action;
    }
    return action; 
}

ControlAction handle_rerr(Packet& p, uint16_t local_addr, uint32_t now_ms) {
    ControlAction action;
    action.type = CTRL_DROP;
    action.next_hop = 0;

    if (dupdet_is_duplicate(p.msg_id, p.src_addr)) return action;

    routing_invalidate(p.src_addr);

    if (p.payload_len >= 2) {
        uint16_t unreachable_dst = p.payload[0] | (p.payload[1] << 8);
        routing_invalidate(unreachable_dst);
    }

    if (p.dst_addr == local_addr) return action; 

    if (p.dst_addr == 0xFFFF) {
        if (p.ttl > 1) {
            p.ttl -= 1;
            p.prev_hop = local_addr;
            action.type = CTRL_FORWARD_BROADCAST;
        }
    } else {
        RouteEntry route;
        if (routing_lookup(p.dst_addr, route, now_ms)) {
            if (p.ttl > 1) {
                p.ttl -= 1;
                p.prev_hop = local_addr;
                action.type = CTRL_FORWARD_UNICAST;
                action.next_hop = route.next_hop;
            }
        }
    }
    return action;
}

void routing_debug_dump(uint32_t now_ms) {
#if DEBUG_ENABLED
    routing_purge_expired(now_ms);
    DBG_PRINTLN("=== Routing Table ===");
    for (int i = 0; i < ROUTING_TABLE_SIZE; ++i) {
        if (routing_table[i].in_use) {
            uint32_t remaining = routing_table[i].lifetime - now_ms;
            DBG_PRINTF("dst: 0x%04X | next: 0x%04X | hop: %d | state: %d | rem: %u ms\n", 
                       routing_table[i].destination, routing_table[i].next_hop, 
                       routing_table[i].hop_count, routing_table[i].state, remaining);
        }
    }
#endif
}