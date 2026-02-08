#include <Wire.h>
// ===== MOTOR CONTROL =====
const int MOTOR_PWM_PIN = 9;   // D9 (~PWM)
int motorPWM = 0;              // 0–255
bool motorEnabled = false;

void motorApply() {
  if (!motorEnabled) {
    analogWrite(MOTOR_PWM_PIN, 0);
  } else {
    analogWrite(MOTOR_PWM_PIN, motorPWM);
  }
}
// ---------- Addresses ----------
static const uint8_t TCA_ADDR = 0x70;   // TCA9548A
static const uint8_t MPU_ADDR = 0x68;   // MPU6050 (AD0 low)

// ---------- MPU6050 Registers ----------
static const uint8_t REG_PWR_MGMT_1 = 0x6B;
static const uint8_t REG_SMPLRT_DIV = 0x19;
static const uint8_t REG_CONFIG     = 0x1A;
static const uint8_t REG_GYRO_CONFIG  = 0x1B;
static const uint8_t REG_ACCEL_CONFIG = 0x1C;
static const uint8_t REG_INT_ENABLE = 0x38;
static const uint8_t REG_ACCEL_XOUT_H = 0x3B;

// ---------- Streaming config ----------
volatile bool streaming = false;
uint16_t sampleRateHz = 200; // per sensor loop iteration rate (each loop reads all sensors)
uint8_t sensorCount = 3;     // 2 or 3 recommended

// Fixed-rate scheduler
uint32_t nextTickUs = 0;

// Simple gyro offsets per sensor (raw units)
struct Offsets {
  int16_t gx0 = 0, gy0 = 0, gz0 = 0;
};
Offsets gyroOffsets[3];

// ---------- Helpers ----------
void tcaSelect(uint8_t channel) {
  if (channel > 7) return;
  Wire.beginTransmission(TCA_ADDR);
  Wire.write(1 << channel);
  Wire.endTransmission();
}

bool i2cWriteByte(uint8_t addr, uint8_t reg, uint8_t val) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write(val);
  return Wire.endTransmission() == 0;
}

bool i2cReadBytes(uint8_t addr, uint8_t reg, uint8_t *buf, uint8_t len) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false; // repeated start
  uint8_t n = Wire.requestFrom((uint8_t)addr, (uint8_t)len, (uint8_t)1);
  if (n != len) return false;
  for (uint8_t i = 0; i < len; i++) buf[i] = Wire.read();
  return true;
}

bool mpuPing() {
  Wire.beginTransmission(MPU_ADDR);
  return (Wire.endTransmission() == 0);
}

bool mpuInitOne(uint8_t ch) {
  tcaSelect(ch);
  if (!mpuPing()) return false;

  // Wake up
  if (!i2cWriteByte(MPU_ADDR, REG_PWR_MGMT_1, 0x00)) return false;

  // Config:
  // DLPF ~ 42Hz (CONFIG=3) -> helps noisy vibrations
  i2cWriteByte(MPU_ADDR, REG_CONFIG, 0x03);

  // Gyro range: ±250 dps (GYRO_CONFIG=0)
  i2cWriteByte(MPU_ADDR, REG_GYRO_CONFIG, 0x00);

  // Accel range: ±2g (ACCEL_CONFIG=0)
  i2cWriteByte(MPU_ADDR, REG_ACCEL_CONFIG, 0x00);

  // Sample rate divider:
  // Internal gyro rate is 8kHz, but with DLPF enabled it's 1kHz.
  // We'll set SMPLRT_DIV so internal sampling is ~1kHz/(div+1)
  // For stability, keep internal at 250Hz (div=3), while we read at 200Hz externally.
  i2cWriteByte(MPU_ADDR, REG_SMPLRT_DIV, 3);

  // Disable interrupts for simplicity
  i2cWriteByte(MPU_ADDR, REG_INT_ENABLE, 0x00);

  return true;
}

bool mpuReadRaw(uint8_t ch, int16_t &ax, int16_t &ay, int16_t &az,
                int16_t &gx, int16_t &gy, int16_t &gz) {
  tcaSelect(ch);
  uint8_t buf[14];
  if (!i2cReadBytes(MPU_ADDR, REG_ACCEL_XOUT_H, buf, 14)) return false;

  ax = (int16_t)((buf[0] << 8) | buf[1]);
  ay = (int16_t)((buf[2] << 8) | buf[3]);
  az = (int16_t)((buf[4] << 8) | buf[5]);

  gx = (int16_t)((buf[8] << 8) | buf[9]);
  gy = (int16_t)((buf[10] << 8) | buf[11]);
  gz = (int16_t)((buf[12] << 8) | buf[13]);

  // Apply gyro offsets
  if (ch < 3) {
    gx -= gyroOffsets[ch].gx0;
    gy -= gyroOffsets[ch].gy0;
    gz -= gyroOffsets[ch].gz0;
  }
  return true;
}

void calibrateGyro(uint8_t samples = 200) {
  // Make sure bridge is still during this
  for (uint8_t s = 0; s < 3; s++) {
    gyroOffsets[s] = Offsets();
  }

  int32_t sumGx[3] = {0,0,0}, sumGy[3] = {0,0,0}, sumGz[3] = {0,0,0};
  uint16_t count[3] = {0,0,0};

  for (uint8_t i = 0; i < samples; i++) {
    for (uint8_t sid = 0; sid < sensorCount; sid++) {
      int16_t ax,ay,az,gx,gy,gz;
      if (mpuReadRaw(sid, ax,ay,az, gx,gy,gz)) {
        // IMPORTANT: during calibration, mpuReadRaw already subtracts offsets,
        // but offsets are currently zero, so it's fine.
        sumGx[sid] += gx;
        sumGy[sid] += gy;
        sumGz[sid] += gz;
        count[sid]++;
      }
    }
    delay(5);
  }

  for (uint8_t sid = 0; sid < sensorCount; sid++) {
    if (count[sid] == 0) continue;
    gyroOffsets[sid].gx0 = (int16_t)(sumGx[sid] / (int32_t)count[sid]);
    gyroOffsets[sid].gy0 = (int16_t)(sumGy[sid] / (int32_t)count[sid]);
    gyroOffsets[sid].gz0 = (int16_t)(sumGz[sid] / (int32_t)count[sid]);
  }

  Serial.println("CAL_OK");
  for (uint8_t sid = 0; sid < sensorCount; sid++) {
    Serial.print("GYRO_OFF,"); Serial.print(sid); Serial.print(",");
    Serial.print(gyroOffsets[sid].gx0); Serial.print(",");
    Serial.print(gyroOffsets[sid].gy0); Serial.print(",");
    Serial.println(gyroOffsets[sid].gz0);
  }
}

void printHelp() {
  Serial.println("CMDS: START | STOP | RATE <hz> | SENS <2|3> | CAL | STATUS");
}

// ---------- Setup / loop ----------
void setup() {
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000); // fast I2C
  pinMode(MOTOR_PWM_PIN, OUTPUT);
  motorApply();  // ensure motor is OFF at boot

  delay(200);
  Serial.println("BOOT");

  // Init sensors on channels 0..2
  for (uint8_t sid = 0; sid < 3; sid++) {
    bool ok = mpuInitOne(sid);
    Serial.print("MPU_INIT,"); Serial.print(sid); Serial.print(",");
    Serial.println(ok ? "OK" : "FAIL");
  }

  printHelp();

  nextTickUs = micros();
}

String cmdLine;

void handleCommand(const String &line) {
  String s = line;
  s.trim();
  if (s.length() == 0) return;

  // Normalize once for cleaner parsing
  String u = s;
  u.toUpperCase();

  // ===== MOTOR COMMANDS (put EARLY so it works even when not streaming) =====
  if (u == "MOTORON") {
    motorEnabled = true;
    motorApply();
    Serial.println("MOTOR_ON");
    return;
  }

  if (u == "MOTOROFF") {
    motorEnabled = false;
    motorApply();
    Serial.println("MOTOR_OFF");
    return;
  }

  if (u.startsWith("MOTOR ")) {
    int val = s.substring(6).toInt();      // use original 's' to keep spacing/indexing
    val = constrain(val, 0, 255);
    motorPWM = val;
    motorEnabled = true;
    motorApply();
    Serial.print("MOTOR_PWM=");
    Serial.println(motorPWM);
    return;
  }

  // ===== EXISTING COMMANDS =====
  if (u == "START") {
    streaming = true;
    Serial.println("STREAM_ON");
    return;
  }

  if (u == "STOP") {
    streaming = false;
    Serial.println("STREAM_OFF");
    return;
  }

  if (u == "CAL") {
    Serial.println("CAL_BEGIN");
    calibrateGyro(250);
    return;
  }

  if (u == "STATUS") {
    Serial.print("STATUS,streaming="); Serial.print(streaming ? "1":"0");
    Serial.print(",rate="); Serial.print(sampleRateHz);
    Serial.print(",sensors="); Serial.println(sensorCount);
    return;
  }

  if (u.startsWith("RATE")) {
    int sp = s.indexOf(' ');
    if (sp > 0) {
      int hz = s.substring(sp + 1).toInt();
      if (hz >= 20 && hz <= 500) {
        sampleRateHz = (uint16_t)hz;
        Serial.print("RATE_OK,"); Serial.println(sampleRateHz);
      } else {
        Serial.println("RATE_ERR");
      }
    } else {
      Serial.println("RATE_ERR");
    }
    return;
  }

  if (u.startsWith("SENS")) {
    int sp = s.indexOf(' ');
    if (sp > 0) {
      int n = s.substring(sp + 1).toInt();
      if (n == 2 || n == 3) {
        sensorCount = (uint8_t)n;
        Serial.print("SENS_OK,"); Serial.println(sensorCount);
      } else {
        Serial.println("SENS_ERR");
      }
    } else {
      Serial.println("SENS_ERR");
    }
    return;
  }

  if (u == "HELP") {
    printHelp();
    return;
  }

  Serial.println("UNKNOWN_CMD");
}
void loop() {
  // Read commands
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n') {
      handleCommand(cmdLine);
      cmdLine = "";
    } else if (c != '\r') {
      cmdLine += c;
      if (cmdLine.length() > 80) cmdLine = ""; // prevent runaway
    }
  }

  if (!streaming) return;

  // fixed-rate tick
  uint32_t now = micros();
  uint32_t periodUs = (uint32_t)(1000000UL / (uint32_t)sampleRateHz);
  if ((int32_t)(now - nextTickUs) < 0) return;
  nextTickUs += periodUs;

  // Read & print each sensor (one CSV line per sensor)
  uint32_t tms = millis();
  for (uint8_t sid = 0; sid < sensorCount; sid++) {
    int16_t ax,ay,az,gx,gy,gz;
    if (mpuReadRaw(sid, ax,ay,az, gx,gy,gz)) {
      Serial.print(tms); Serial.print(",");
      Serial.print(sid); Serial.print(",");
      Serial.print(ax); Serial.print(",");
      Serial.print(ay); Serial.print(",");
      Serial.print(az); Serial.print(",");
      Serial.print(gx); Serial.print(",");
      Serial.print(gy); Serial.print(",");
      Serial.println(gz);
    }
  }
}
