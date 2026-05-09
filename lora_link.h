#pragma once
#include <Arduino.h>
#include "mesh_packet.h"

// ── Breadboard prototype pin tanımları ──────────────────
// AUX pini GPIO 4'e bağlı; lora_send_packet TX-complete handshake'i kullanır.
#ifndef LORA_AUX_PIN
#define LORA_AUX_PIN  4   // GPIO 4 = AUX from LoRa E22 module
#endif

// lora_rx_reset(): RX state machine'i sıfırlar.
void lora_rx_reset();

void lora_init();

// Best-effort send: returns true if the packet was serialized and
// written to Serial1 without local errors.
bool lora_send_packet(const Packet& p);

// Blocking receive with timeout in milliseconds.
// Returns true and fills `out` if a valid Packet (CRC OK) was received
// before timeout; otherwise returns false.
bool lora_receive_packet(Packet& out, uint32_t timeout_ms);