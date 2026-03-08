#include "MPC_01.h"


MPC_01::MPC_01()
: stepperX(AccelStepper::DRIVER, STEP1, DIR1),
  stepperY(AccelStepper::DRIVER, STEP2, DIR2),
  stepperZ(AccelStepper::DRIVER, STEP3, DIR3)
{
}

void MPC_01::begin(uint16_t rms_mA, uint8_t microsteps){

    pinMode(NCXpin, INPUT_PULLUP);
    pinMode(NCYpin, INPUT_PULLUP);
    pinMode(NCZpin, INPUT_PULLUP);
    
    tmc.beginTMC2209();
    tmc.setupTMC2209(rms_mA, microsteps);

    stepperX.setMaxSpeed(maxSpeedSteps);
    stepperY.setMaxSpeed(maxSpeedSteps);
    stepperZ.setMaxSpeed(maxSpeedSteps);

    stepperX.setAcceleration(accelSteps);
    stepperY.setAcceleration(accelSteps);
    stepperZ.setAcceleration(accelSteps);

    i2c.setDebug(true);
    i2c.begin(100000);
    i2c.scan();

    rs485.begin(19200);

}

void MPC_01::setCoordMotion(CoordMotion mode) {
    motionMode = mode;
}
void MPC_01::setAxisTarget(int x, int y, int z) {
  if (x > maxX) return;
  if (y > maxY) return;
  if (z > maxZ) return;

  int targetX = x * xConstant;
  int targetY = y * yConstant;
  int targetZ = z * zConstant;

  switch (motionMode) {
    case CoordMotion::LinearXY:
        motor1 = targetX;
        motor2 = targetY;
        motor3 = targetZ;
        Serial.println("Motion System LinearXY selected!");
        break;

    case CoordMotion::CoreXY:
        motor1 = targetX + targetY;
        motor2 = targetX - targetY;
        motor3 = targetZ;
        Serial.println("Motion System CoreXY selected!");
        break;
    
    default:
        Serial.println("No Motion System selected!");
        break;
    }
    moveToSync();
}
void MPC_01::setMoveSpeed(int speed){
  if (speed < 1) speed = 1;
  maxSpeedSteps = speed;

  stepperX.setMaxSpeed(maxSpeedSteps);
  stepperY.setMaxSpeed(maxSpeedSteps);
  stepperZ.setMaxSpeed(maxSpeedSteps);
}
void MPC_01::setMoveAcceleration(int accel){
  if (accel < 1) accel = 1;
  accelSteps = accel;

  stepperX.setAcceleration(accelSteps);
  stepperY.setAcceleration(accelSteps);
  stepperZ.setAcceleration(accelSteps);  
}

void MPC_01::applyMotionStepper(){
  updateMotion();
  isMoving();

}
void MPC_01::updateMotion(){
  if (isEStopActive()) {
    return;
  }

  stepperX.run();
  stepperY.run();
  stepperZ.run();
}
bool MPC_01::isMoving(){
  return stepperX.distanceToGo() != 0 ||
         stepperY.distanceToGo() != 0 ||
         stepperZ.distanceToGo() != 0;

}
void MPC_01::moveToSync(){
  long d1 = labs(motor1 - stepperX.currentPosition());
  long d2 = labs(motor2 - stepperX.currentPosition());
  long d3 = labs(motor3 - stepperX.currentPosition());

  long maxDist = d1;
  if (d2 > maxDist) maxDist = d2;
  if (d3 > maxDist) maxDist = d3;

  if (maxDist == 0) return;

  stepperX.setMaxSpeed(maxSpeedSteps * (float)d1 / maxDist);
  stepperY.setMaxSpeed(maxSpeedSteps * (float)d2 / maxDist);
  stepperZ.setMaxSpeed(maxSpeedSteps * (float)d3 / maxDist);

  stepperX.setAcceleration(accelSteps * (float)d1 / maxDist);
  stepperY.setAcceleration(accelSteps * (float)d2 / maxDist);
  stepperZ.setAcceleration(accelSteps * (float)d3 / maxDist);

  stepperX.moveTo(motor1);
  stepperY.moveTo(motor2);
  stepperZ.moveTo(motor3);
  
  applyMotionStepper();

}

void MPC_01::homing(){
  // fast seek
  // backoff
  // slow seek
  // backoff
}

void MPC_01::calibrate(int8_t Multiplikator){
    
}

uint8_t MPC_01::getMaxX(){    
    return maxX;
}
uint8_t MPC_01::getMaxY(){    
    return maxY;
}
uint8_t MPC_01::getMaxZ(){    
    return maxZ;
}

bool MPC_01::setStatus(uint8_t mode) {
  return i2c.writeBytes(0x10, &mode, 1);
}

void MPC_01::sendRS485Text(const char* text) {
    rs485.sendText(text);
}
void MPC_01::updateRS485() {
    while (rs485.available()) {
        int c = rs485.read();
        if (c >= 0) {
            Serial.write((char)c);   // zum Testen im Serial Monitor ausgeben
        }
    }
}

// - - - - - - - - - - TMC2209Manager - - - - - - - - - -

void TMC2209Manager::beginTMC2209(){
    // Pins initialisieren
    pinMode(STEP1, OUTPUT); pinMode(DIR1, OUTPUT);
    pinMode(STEP2, OUTPUT); pinMode(DIR2, OUTPUT);
    pinMode(STEP3, OUTPUT); pinMode(DIR3, OUTPUT);

    pinMode(EN_STATUS_PIN, INPUT);
    
    tmcSerial.begin(115200);
    delay(50);
}
bool TMC2209Manager::setupTMC2209(uint16_t rms_mA, uint8_t microsteps){
    setupOneDriver(drv0, rms_mA, microsteps);
    setupOneDriver(drv1, rms_mA, microsteps);
    setupOneDriver(drv2, rms_mA, microsteps);

    bool ok = testTMC();
    if (ok) {
        // NICHT Hardware-EN schalten! Nur software "arm"
        arm();
        Serial.println(F("TMC UART OK, drivers armed (software)."));
    } else {
        disarm();
        Serial.println(F("UART problem: wiring/pullup/MS1-MS2 address."));
    }
    return ok;

}
void TMC2209Manager::setupOneDriver(TMC2209Stepper &d, uint16_t rms_mA, uint8_t microsteps) {
  d.begin();
  d.pdn_disable(true);
  d.I_scale_analog(false);

  d.toff(4);
  d.blank_time(24);

  d.rms_current(rms_mA);
  d.microsteps(microsteps);

  d.en_spreadCycle(false);
  d.pwm_autoscale(true);
  // optional stabil/konservativ
  d.TPWMTHRS(0);
}
bool TMC2209Manager::commOK(TMC2209Stepper &d, const char* name) {
  uint32_t ioin = d.IOIN();
  Serial.print(name);
  Serial.print(" IOIN=0x");
  Serial.println(ioin, HEX);

  return (ioin != 0x00000000UL && ioin != 0xFFFFFFFFUL);
}
bool TMC2209Manager::testTMC() {
  bool ok0 = commOK(drv0, "DRV0");
  bool ok1 = commOK(drv1, "DRV1");
  bool ok2 = commOK(drv2, "DRV2");

  Serial.print("COMM OK: ");
  Serial.print(ok0); Serial.print(ok1); Serial.print(ok2);

  return ok0 && ok1 && ok2;
}
void TMC2209Manager::arm() {
  drv0.toff(4);
  drv1.toff(4);
  drv2.toff(4);
}
void TMC2209Manager::disarm() {
  drv0.toff(0);
  drv1.toff(0);
  drv2.toff(0);
}

//- - - - - - - - - - I2CManager - - - - - - - - - -

void I2CManager::begin(uint32_t clockHz) {
  Wire.begin();                // SDA/SCL automatisch (A4/A5 am ATmega328P)
  Wire.setClock(clockHz);      // 100k oder 400k

  if (debug_) {
    Serial.print(F("I2C started, clock="));
    Serial.println(clockHz);
  }
}
void I2CManager::setDebug(bool on) {
  debug_ = on;
}
void I2CManager::dbgPrint_(const __FlashStringHelper* s) {
  if (!debug_) return;
  Serial.print(s);
}
void I2CManager::dbgPrintHex_(uint8_t v) {
  if (!debug_) return;
  if (v < 16) Serial.print('0');
  Serial.print(v, HEX);
}
bool I2CManager::ping(uint8_t addr) {
  Wire.beginTransmission(addr);
  return (Wire.endTransmission() == 0);
}
void I2CManager::scan() {
  if (!debug_) return;

  Serial.println(F("I2C scan..."));
  uint8_t found = 0;

  for (uint8_t addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    uint8_t err = Wire.endTransmission();

    if (err == 0) {
      Serial.print(F("Found 0x"));
      dbgPrintHex_(addr);
      Serial.println();
      found++;
    }
  }

  Serial.print(F("Devices found: "));
  Serial.println(found);
}
bool I2CManager::writeBytes(uint8_t addr, const uint8_t* data, size_t len) {
  Wire.beginTransmission(addr);
  for (size_t i = 0; i < len; i++) Wire.write(data[i]);
  return (Wire.endTransmission() == 0);
}
bool I2CManager::readBytes(uint8_t addr, uint8_t* data, size_t len) {
  size_t got = Wire.requestFrom((int)addr, (int)len);
  if (got != len) return false;

  for (size_t i = 0; i < len; i++) data[i] = Wire.read();
  return true;
}
bool I2CManager::writeReg8(uint8_t addr, uint8_t reg, uint8_t value) {
  uint8_t buf[2] = { reg, value };
  return writeBytes(addr, buf, sizeof(buf));
}
bool I2CManager::writeReg16(uint8_t addr, uint8_t reg, uint16_t value, bool msbFirst) {
  uint8_t buf[3];
  buf[0] = reg;
  if (msbFirst) {
    buf[1] = (uint8_t)(value >> 8);
    buf[2] = (uint8_t)(value & 0xFF);
  } else {
    buf[1] = (uint8_t)(value & 0xFF);
    buf[2] = (uint8_t)(value >> 8);
  }
  return writeBytes(addr, buf, sizeof(buf));
}
bool I2CManager::readReg8(uint8_t addr, uint8_t reg, uint8_t &value) {
  // register pointer setzen
  Wire.beginTransmission(addr);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false; // repeated start

  // 1 Byte lesen
  if (Wire.requestFrom((int)addr, 1) != 1) return false;
  value = Wire.read();
  return true;
}
bool I2CManager::readRegBytes(uint8_t addr, uint8_t reg, uint8_t* data, size_t len) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;

  size_t got = Wire.requestFrom((int)addr, (int)len);
  if (got != len) return false;

  for (size_t i = 0; i < len; i++) data[i] = Wire.read();
  return true;
}

//- - - - - - - - - - RS485 - - - - - - - - - -

RS485Manager::RS485Manager(uint8_t rxPin, uint8_t txPin, uint8_t deRePin)
    : rs485Serial(rxPin, txPin), deRePin_(deRePin)
{
}
void RS485Manager::begin(unsigned long baud) {
    pinMode(deRePin_, OUTPUT);
    setReceiveMode();
    rs485Serial.begin(baud);
}
void RS485Manager::setTransmitMode() {
    digitalWrite(deRePin_, HIGH);
}
void RS485Manager::setReceiveMode() {
    digitalWrite(deRePin_, LOW);
}
void RS485Manager::sendByte(uint8_t data) {
    setTransmitMode();
    delayMicroseconds(20);

    rs485Serial.write(data);
    rs485Serial.flush();

    delayMicroseconds(20);
    setReceiveMode();
}
void RS485Manager::sendBytes(const uint8_t* data, size_t len) {
    setTransmitMode();
    delayMicroseconds(20);

    for (size_t i = 0; i < len; i++) {
        rs485Serial.write(data[i]);
    }
    rs485Serial.flush();

    delayMicroseconds(20);
    setReceiveMode();
}
void RS485Manager::sendText(const char* text) {
    setTransmitMode();
    delayMicroseconds(20);

    rs485Serial.print(text);
    rs485Serial.flush();

    delayMicroseconds(20);
    setReceiveMode();
}
bool RS485Manager::available() {
    return rs485Serial.available() > 0;
}
int RS485Manager::read() {
    return rs485Serial.read();
}


