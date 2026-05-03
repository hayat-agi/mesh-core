#include "lora_link.h"
#include <Arduino.h>

// PRD Section 1: Hardware Mapping
#define LORA_TX_PIN 27  // ESP32 TX1 (connected to module RX)
#define LORA_RX_PIN 35  // ESP32 RX1 (connected to module TX)
#define LORA_M0_PIN 32
#define LORA_M1_PIN 33

void lora_init() {
    pinMode(LORA_M0_PIN, OUTPUT);
    pinMode(LORA_M1_PIN, OUTPUT);

    digitalWrite(LORA_M0_PIN, LOW);
    digitalWrite(LORA_M1_PIN, LOW);

    Serial1.begin(9600, SERIAL_8N1, LORA_RX_PIN, LORA_TX_PIN);
    delay(100);
}

bool lora_send_packet(const Packet& p) {
    uint8_t buf[128]; 
    
    size_t serialized_len = packet_serialize(p, &buf[3], sizeof(buf) - 3);
    if (serialized_len == 0 || serialized_len > 85) {
        return false; 
    }

    buf[0] = 0xAA; // SOF1
    buf[1] = 0x55; // SOF2
    buf[2] = (uint8_t)serialized_len;

    size_t total_len = serialized_len + 3;
    size_t written = Serial1.write(buf, total_len);
    Serial1.flush(); 
    
    return (written == total_len);
}

enum RxState {
    WAIT_SOF1,
    WAIT_SOF2,
    WAIT_LEN,
    READ_PACKET_BYTES
};

bool lora_receive_packet(Packet& out, uint32_t timeout_ms) {
    uint32_t start_time = millis();
    uint8_t buf[128];
    size_t bytes_read = 0;
    uint8_t expected_len = 0;
    RxState state = WAIT_SOF1;

    while ((millis() - start_time) < timeout_ms) {
        if (Serial1.available()) {
            uint8_t c = Serial1.read();

            switch (state) {
                case WAIT_SOF1:
                    if (c == 0xAA) state = WAIT_SOF2;
                    break;
                case WAIT_SOF2:
                    if (c == 0x55) {
                        state = WAIT_LEN;
                    } else if (c == 0xAA) {
                        state = WAIT_SOF2; 
                    } else {
                        state = WAIT_SOF1;
                    }
                    break;
                case WAIT_LEN:
                    if (c == 0 || c > 85) {
                        state = WAIT_SOF1; 
                    } else {
                        expected_len = c;
                        bytes_read = 0;
                        state = READ_PACKET_BYTES;
                    }
                    break;
                case READ_PACKET_BYTES:
                    buf[bytes_read++] = c;
                    if (bytes_read == expected_len) {
                        if (packet_deserialize(out, buf, expected_len)) {
                            return true; // Paket başarıyla ayrıştırıldı!
                        }
                        state = WAIT_SOF1;
                    }
                    break;
            }
        } else {
            delay(1);
        }
    }
    return false;
}