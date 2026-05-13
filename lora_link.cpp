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

    // LoRa modülü RF TX'i tamamlayana kadar bekle.
    // AUX pini varsa (HIGH = hazır) onu kullan, yoksa sabit süre bekle.
    // 9600 baud + RF overhead: tipik 50-120ms.
    if (LORA_AUX_PIN >= 0) {
        uint32_t t = millis();
        while (digitalRead(LORA_AUX_PIN) == LOW && (millis() - t) < 200) {
            delay(1);
        }
        delay(5);  // stabilizasyon
    } else {
        delay(120); // AUX pini yok: sabit bekleme (SX1262 RF TX için yeterli marj)
    }

    // NOT: lora_rx_reset() burada kasıtlı olarak ÇAĞRILMIYOR.
    // TX biter bitmez karşı taraf ACK göndermeye başlayabilir; sıfırlama
    // kısmen alınmış ACK byte'larını siler ve ACK timeout'a yol açar.
    // RX state machine'in SOF1/SOF2 sync mekanizması gürültüyü zaten temizler.

    return (written == total_len);
}

enum RxState {
    WAIT_SOF1,
    WAIT_SOF2,
    WAIT_LEN,
    READ_PACKET_BYTES
};

// Kalıcı RX state machine — çağrılar arasında kısmi alınan baytlar korunur.
// Böylece ACK polling döngüsünde kısa timeout'larla bile paket kaybı olmaz.
static RxState  s_rx_state        = WAIT_SOF1;
static uint8_t  s_rx_buf[128];
static size_t   s_rx_bytes_read   = 0;
static uint8_t  s_rx_expected_len = 0;

void lora_rx_reset() {
    s_rx_state        = WAIT_SOF1;
    s_rx_bytes_read   = 0;
    s_rx_expected_len = 0;
}

bool lora_receive_packet(Packet& out, uint32_t timeout_ms) {
    uint32_t start_time = millis();

    while ((millis() - start_time) < timeout_ms) {
        if (Serial1.available()) {
            uint8_t c = Serial1.read();

            switch (s_rx_state) {
                case WAIT_SOF1:
                    if (c == 0xAA) s_rx_state = WAIT_SOF2;
                    break;
                case WAIT_SOF2:
                    if (c == 0x55) {
                        s_rx_state = WAIT_LEN;
                    } else if (c == 0xAA) {
                        s_rx_state = WAIT_SOF2;
                    } else {
                        s_rx_state = WAIT_SOF1;
                    }
                    break;
                case WAIT_LEN:
                    if (c == 0 || c > 85) {
                        s_rx_state = WAIT_SOF1;
                    } else {
                        s_rx_expected_len = c;
                        s_rx_bytes_read   = 0;
                        s_rx_state        = READ_PACKET_BYTES;
                    }
                    break;
                case READ_PACKET_BYTES:
                    s_rx_buf[s_rx_bytes_read++] = c;
                    if (s_rx_bytes_read == s_rx_expected_len) {
                        bool ok = packet_deserialize(out, s_rx_buf, s_rx_expected_len);
                        lora_rx_reset(); // Bir sonraki paket için state'i sıfırla
                        if (ok) return true;
                    }
                    break;
            }
        } else {
            delay(1);
        }
    }
    return false;
}