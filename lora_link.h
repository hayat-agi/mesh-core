#pragma once
#include <Arduino.h>
#include "mesh_packet.h"

// ─── Hardware Variant ────────────────────────────────────────────────────────
// 1 = Breadboard → Node 3 only  (LoRa: M0=25,M1=26,TX=17,RX=16,AUX=4 | MPU: SDA=32,SCL=33)
// 0 = PCB        → Node 1,2,4  (LoRa: M0=32,M1=33,TX=27,RX=35,AUX=-1 | MPU: SDA=21,SCL=22)
#define BREADBOARD_NODE 0  // TODO: 1 for Node 3, 0 for Node 1/2/4

#if BREADBOARD_NODE
  #define LORA_TX_PIN   17
  #define LORA_RX_PIN   16
  #define LORA_M0_PIN   25
  #define LORA_M1_PIN   26
  #define LORA_AUX_PIN  4
#else
  #define LORA_TX_PIN   27
  #define LORA_RX_PIN   35
  #define LORA_M0_PIN   32
  #define LORA_M1_PIN   33
  #define LORA_AUX_PIN  (-1)
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