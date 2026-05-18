#pragma once
#include <Arduino.h>
#include "mesh_packet.h"

// ── PCB (Node 2 / Node 4) pin tanımları ──────────────────
// AUX pini LoRa modülünün hazır sinyali. GPIO 4'e bağlı.
#ifndef LORA_AUX_PIN
#define LORA_AUX_PIN  4
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