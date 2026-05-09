// BreadboardWiringCheck.ino
//
// Hayat Ağı breadboard prototipi için donanım self-test sketch'i.
// Flash atınca Serial Monitor'da (115200 baud) her bir alt-sistem için
// PASS/FAIL satırı yazılır. Tüm satırlar PASS olmadan ana firmware'i
// (LifeNetLoRaMesh.ino) flash atma — kabloda eksik/yanlış var demektir.
//
// Test edilen donanım:
//   LoRa E22:  TX=GPIO17  RX=GPIO16  M0=GPIO25  M1=GPIO26  AUX=GPIO4
//   MPU6050:   SDA=GPIO32 SCL=GPIO33
//   Her iki modül de 3V3 + GND'den besleniyor.
//
// Test sırası:
//   1. I2C scan (32/33 üzerinde)             → MPU pinleri ve gücü
//   2. MPU6050 WHO_AM_I (0x68 → 0x68/0x70)   → MPU çipi gerçek mi
//   3. MPU6050 ivme örneği (~1g büyüklüğü)   → MPU register/clock'u sağlam mı
//   4. LoRa AUX baseline (mode 0 sonrası HIGH) → AUX kablosu + LoRa gücü
//   5. LoRa parametre readback (mode 3'te 0xC1×3 → cevap) → UART TX/RX + M0/M1

#include <Arduino.h>
#include <Wire.h>

// ── Pin assignments (breadboard layout) ─────────────────────────────────────

#define PIN_LORA_TX   17
#define PIN_LORA_RX   16
#define PIN_LORA_M0   25
#define PIN_LORA_M1   26
#define PIN_LORA_AUX   4

#define PIN_MPU_SDA   32
#define PIN_MPU_SCL   33

#define MPU_ADDR     0x68

// ── State ───────────────────────────────────────────────────────────────────

static int passed = 0;
static int failed = 0;
static int skipped = 0;

static void recordResult(const char* name, bool ok, const char* hint = nullptr) {
  Serial.printf("  [%s] %s", ok ? "PASS" : "FAIL", name);
  if (!ok && hint) Serial.printf(" — %s", hint);
  Serial.println();
  if (ok) passed++; else failed++;
}

static void recordSkip(const char* name, const char* reason) {
  Serial.printf("  [SKIP] %s — %s\n", name, reason);
  skipped++;
}

// ── 1. I2C scan ─────────────────────────────────────────────────────────────

static bool i2cScan() {
  Serial.println("\n[1/5] I2C scan on GPIO 32 (SDA) / GPIO 33 (SCL):");
  Wire.begin(PIN_MPU_SDA, PIN_MPU_SCL);
  Wire.setClock(100000);
  delay(50);

  int found = 0;
  bool foundMpu = false;
  for (uint8_t addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      Serial.printf("    -> device at 0x%02X\n", addr);
      found++;
      if (addr == MPU_ADDR) foundMpu = true;
    }
  }
  if (found == 0) {
    Serial.println("    -> no devices responded");
  }

  recordResult("MPU6050 detected at 0x68", foundMpu,
               "check VCC/GND or SDA<->SCL swap");
  return foundMpu;
}

// ── 2. WHO_AM_I ─────────────────────────────────────────────────────────────

static bool mpuWhoAmI() {
  Serial.println("\n[2/5] MPU6050 WHO_AM_I (register 0x75):");
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x75);
  if (Wire.endTransmission(false) != 0) {
    recordResult("WHO_AM_I read", false, "I2C write failed");
    return false;
  }
  if (Wire.requestFrom((int)MPU_ADDR, 1) != 1) {
    recordResult("WHO_AM_I read", false, "no byte returned");
    return false;
  }
  uint8_t whoami = Wire.read();
  Serial.printf("    -> WHO_AM_I = 0x%02X (expected 0x68 / 0x70 / 0x72)\n", whoami);
  bool ok = (whoami == 0x68 || whoami == 0x70 || whoami == 0x72);
  recordResult("WHO_AM_I value valid", ok, "wrong chip or bus error");
  return ok;
}

// ── 3. Accel sanity ─────────────────────────────────────────────────────────

static bool mpuAccelSanity() {
  Serial.println("\n[3/5] MPU6050 accelerometer sanity:");

  // Wake from sleep (PWR_MGMT_1 = 0)
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);
  Wire.write(0x00);
  if (Wire.endTransmission() != 0) {
    recordResult("Accel read", false, "wake-up write failed");
    return false;
  }
  delay(50);

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);  // ACCEL_XOUT_H
  if (Wire.endTransmission(false) != 0) {
    recordResult("Accel read", false, "register select failed");
    return false;
  }
  if (Wire.requestFrom((int)MPU_ADDR, 6) != 6) {
    recordResult("Accel read", false, "did not get 6 bytes");
    return false;
  }
  int16_t ax = (Wire.read() << 8) | Wire.read();
  int16_t ay = (Wire.read() << 8) | Wire.read();
  int16_t az = (Wire.read() << 8) | Wire.read();

  // ±2g range default → 16384 LSB/g
  float gx = ax / 16384.0f;
  float gy = ay / 16384.0f;
  float gz = az / 16384.0f;
  float mag = sqrtf(gx*gx + gy*gy + gz*gz);
  Serial.printf("    -> accel = (%.2f, %.2f, %.2f) g, |a| = %.2f g\n",
                gx, gy, gz, mag);

  bool ok = (mag > 0.7f && mag < 1.3f);
  recordResult("Accel magnitude ~1g (sensor must be at rest)", ok,
               "if sensor is moving this is expected; otherwise suspect MPU clock");
  return ok;
}

// ── 4. LoRa AUX baseline ────────────────────────────────────────────────────

static bool loraAuxBaseline() {
  Serial.println("\n[4/5] LoRa E22 AUX baseline (mode 0 — normal):");
  pinMode(PIN_LORA_M0, OUTPUT);
  pinMode(PIN_LORA_M1, OUTPUT);
  pinMode(PIN_LORA_AUX, INPUT);

  digitalWrite(PIN_LORA_M0, LOW);
  digitalWrite(PIN_LORA_M1, LOW);
  delay(200);

  int aux = digitalRead(PIN_LORA_AUX);
  Serial.printf("    -> AUX = %s\n", aux ? "HIGH (idle/ready)" : "LOW (busy or disconnected)");
  recordResult("AUX HIGH after entering mode 0", aux == HIGH,
               "check AUX wire to GPIO 4 or VCC to E22");
  return aux == HIGH;
}

// ── 5. LoRa parameter readback ──────────────────────────────────────────────

static bool loraParamReadback() {
  Serial.println("\n[5/5] LoRa E22 parameter readback (mode 3 — sleep/config):");

  Serial1.begin(9600, SERIAL_8N1, PIN_LORA_RX, PIN_LORA_TX);

  digitalWrite(PIN_LORA_M0, HIGH);
  digitalWrite(PIN_LORA_M1, HIGH);
  delay(200);

  while (Serial1.available()) Serial1.read();

  // E22-900T22D parameter read: 0xC1 0xC1 0xC1 → returns config bytes
  uint8_t cmd[3] = { 0xC1, 0xC1, 0xC1 };
  Serial1.write(cmd, 3);
  Serial1.flush();

  uint8_t resp[16];
  size_t n = 0;
  uint32_t start = millis();
  while (millis() - start < 500 && n < sizeof(resp)) {
    if (Serial1.available()) {
      resp[n++] = Serial1.read();
    }
  }

  Serial.printf("    -> received %u bytes:", (unsigned)n);
  for (size_t i = 0; i < n; i++) Serial.printf(" %02X", resp[i]);
  Serial.println();

  // Restore normal mode
  digitalWrite(PIN_LORA_M0, LOW);
  digitalWrite(PIN_LORA_M1, LOW);

  // Valid response is >= 6 bytes starting with 0xC1 (newer fw) or 0xC0 (older).
  bool ok = (n >= 6 && (resp[0] == 0xC1 || resp[0] == 0xC0));
  recordResult("Parameter roundtrip (UART TX/RX + M0/M1 + power)", ok,
               "TX<->RX swap, baud mismatch, missing VCC, or M0/M1 stuck");
  return ok;
}

// ── Setup ───────────────────────────────────────────────────────────────────

void setup() {
  Serial.begin(115200);
  delay(800);
  Serial.println("\n=========================================");
  Serial.println("  Hayat Agi — Breadboard Pin Wiring Check");
  Serial.println("=========================================");
  Serial.println("  LoRa E22:  TX=17  RX=16  M0=25  M1=26  AUX=4");
  Serial.println("  MPU6050:   SDA=32 SCL=33");
  Serial.println("  Power:     both modules from 3V3 + GND");
  Serial.println("-----------------------------------------");

  bool i2cOk = i2cScan();

  bool whoOk = false;
  if (i2cOk) {
    whoOk = mpuWhoAmI();
  } else {
    Serial.println("\n[2/5] MPU6050 WHO_AM_I:");
    recordSkip("WHO_AM_I read", "no I2C device responded");
  }

  if (whoOk) {
    mpuAccelSanity();
  } else {
    Serial.println("\n[3/5] MPU6050 accelerometer sanity:");
    recordSkip("Accel read", "WHO_AM_I did not pass");
  }

  loraAuxBaseline();
  loraParamReadback();

  Serial.println("\n=========================================");
  Serial.printf("  RESULT: %d passed, %d failed, %d skipped\n",
                passed, failed, skipped);
  if (failed == 0 && skipped == 0) {
    Serial.println("  -> WIRING OK. Safe to flash main firmware.");
  } else {
    Serial.println("  -> Fix the FAIL/SKIP lines above before flashing main firmware.");
  }
  Serial.println("=========================================\n");
}

void loop() {
  delay(10000);
}
