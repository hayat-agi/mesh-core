#include "lora_link.h"
#include <Arduino.h>

// PRD Section 1: Hardware Mapping
#define LORA_TX_PIN 17  // ESP32 TX (connected to module RXD)
#define LORA_RX_PIN 16  // ESP32 RX (connected to module TXD)
#define LORA_M0_PIN 25
#define LORA_M1_PIN 26

// App-layer cap on a single serialized Packet on the wire. The SX127x/E22
// PHY supports up to ~255 bytes per frame; we cap below that to leave
// margin for SOF1/SOF2/len framing and to keep airtime predictable.
// Raised from 85 to 128 to fit the new hop_path field in the header.
#define LORA_MAX_FRAME 128

void lora_init() {
    if (LORA_AUX_PIN >= 0) {
        pinMode(LORA_AUX_PIN, INPUT);
    }
    pinMode(LORA_M0_PIN, OUTPUT);
    pinMode(LORA_M1_PIN, OUTPUT);

    digitalWrite(LORA_M0_PIN, LOW);
    digitalWrite(LORA_M1_PIN, LOW);

    Serial1.begin(9600, SERIAL_8N1, LORA_RX_PIN, LORA_TX_PIN);
    delay(100);
}

bool lora_send_packet(const Packet& p) {
    uint8_t buf[LORA_MAX_FRAME + 3];  // +3 for SOF1, SOF2, len

    size_t serialized_len = packet_serialize(p, &buf[3], sizeof(buf) - 3);
    if (serialized_len == 0 || serialized_len > LORA_MAX_FRAME) {
        return false;
    }

    buf[0] = 0xAA; // SOF1
    buf[1] = 0x55; // SOF2
    buf[2] = (uint8_t)serialized_len;

    size_t total_len = serialized_len + 3;
    size_t written = Serial1.write(buf, total_len);
    Serial1.flush();

    // LoRa modülü RF TX'i tamamlayana kadar bekle.
    //
    // ⚠️ E22-900T22D (SX1262) ÖNEMLİ FARK:
    //   E32 (SX1278)'de AUX=HIGH → RF üzerinden iletim TAMAMEN bitti demektir.
    //   E22 (SX1262)'de AUX=HIGH → Dahili buffer RF çipine aktarıldı demektir,
    //   ancak RF üzerinden iletim HÂLÂ DEVAM EDİYOR OLABİLİR!
    //   Bu yüzden AUX=HIGH sonrası ekstra hava süresi (air-time) bekliyoruz.
    //
    // E22-900T22D varsayılan Air Data Rate: 2.4 kbps
    //   SF/BW modülün firmware'i tarafından seçilir, doğrudan kontrol edilemez.
    //   33-byte paket için RF on-air ≈ 300-600ms arası olabilir.
    //   AUX HIGH geldikten sonra kalan tahmini on-air için 500ms bekliyoruz.
    if (LORA_AUX_PIN >= 0) {
        delay(10);   // AUX=LOW gecikmesini bekle (flush→AUX arası race: ~2-5ms)
        uint32_t t = millis();
        while (digitalRead(LORA_AUX_PIN) == LOW && (millis() - t) < 700) {
            delay(2);
        }
        // E22 (SX1262): AUX=HIGH = buffer→RF chip tamam, ama hava iletimi sürebilir.
        // 2.4 kbps air rate varsayımıyla güvenli bekleme:
        delay(500);
    } else {
        delay(700); // AUX pini yok: E22 için sabit tam bekleme (RF air-time dahil)
    }

    // TX tamamlandı: RX state machine'i sıfırla.
    // FIFO'yu temizlemiyoruz — gerçek gelen paketler korunuyor.
    lora_rx_reset();

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
static uint8_t  s_rx_buf[LORA_MAX_FRAME];
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
                    if (c == 0 || c > LORA_MAX_FRAME) {
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