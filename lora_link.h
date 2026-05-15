#pragma once
#include <Arduino.h>
#include "mesh_packet.h"

// ── AUX Pin Konfigürasyonu ──────────────────────────────────
// Breadboard : AUX bağlı → LORA_AUX_PIN = 4  (gerçek sinyal kullanılır)
// PCB        : AUX bağlı değil → LORA_AUX_PIN = -1 (sabit delay kullanılır)
//
// PCB derlemesi için Arduino IDE'de build flag ekle:
//   -DPCB_NODE
// ya da bu dosyanın başına doğrudan #define PCB_NODE yaz.

#ifdef PCB_NODE
  #define LORA_AUX_PIN  -1   // PCB: AUX bağlı değil, floating → sabit delay
#else
  #ifndef LORA_AUX_PIN
  #define LORA_AUX_PIN   4   // Breadboard: ESP32 Pin 4 → LoRa AUX
  #endif
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