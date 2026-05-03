#pragma once
// lora_rx_reset(): RX state machine'i sıfırlar.
// lora_send_packet'ten sonra stale baytları temizlemek için kullanılabilir.
void lora_rx_reset();
#include <Arduino.h>
#include "mesh_packet.h"

void lora_init();

// Best-effort send: returns true if the packet was serialized and
// written to Serial1 without local errors.
bool lora_send_packet(const Packet& p);

// Blocking receive with timeout in milliseconds.
// Returns true and fills `out` if a valid Packet (CRC OK) was received
// before timeout; otherwise returns false.
bool lora_receive_packet(Packet& out, uint32_t timeout_ms);
