#pragma once
#include <stdint.h>
#include "mesh_packet.h"

struct StoredMessage {
    Packet   packet;        // full mesh packet (including ai_priority)
    uint32_t enqueue_time;  // ms since boot
    uint8_t  retry_count;   // how many times forwarding has been attempted
    bool     in_use;
    uint32_t last_attempt_time;  // <--- BU SATIRI EKLE
};

// V1 queue parameters
#define QUEUE_MAX_MESSAGES   10
#define QUEUE_MAX_AGE_MS     300000  // 300 s
#define QUEUE_MAX_RETRIES    5

// Callback type: provided by the caller to try forwarding one packet.
// Should return true on successful forwarding (ACK ok), false on failure.
typedef bool (*StoreForwardSendFn)(Packet& p);

void sf_init();

// Enqueue a DATA packet for later forwarding.
// Returns true on success, false if the queue is full (oldest entry may be dropped).
bool sf_enqueue(const Packet& p, uint32_t now_ms);

// Periodic processing function to be called from a low-priority task.
// It will:
//  - drop expired messages,
//  - attempt to forward using the provided send callback,
//  - update retry_count, and drop messages exceeding QUEUE_MAX_RETRIES.
void sf_process(uint32_t now_ms, StoreForwardSendFn send_fn);
void sf_debug_dump(uint32_t now_ms);

// RREP alındığında routing.cpp tarafından çağrılır.
// dst_addr hedefine ait kuyruklanmış paketlerin exponential backoff süresini sıfırlar,
// böylece sf_process bir sonraki döngüde yeniden deneme yapar.
void sf_reset_backoff_for_dst(uint16_t dst_addr);
