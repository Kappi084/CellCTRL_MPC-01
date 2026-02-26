#include "MPC_01.h"







void MPC_01::begin(uint16_t rms_mA, uint8_t microsteps){

    pinMode(END_X, INPUT_PULLUP);
    pinMode(END_Y, INPUT_PULLUP);
    pinMode(END_Z, INPUT_PULLUP);

    const int Servo1pin = 2; //Servo 1 (Turning Unit)
    const int Servo2pin = 3; //Servo 2 (Gripper)

    const int NCXpin = A2;  //Endstop X
    const int NCYpin = A1;  //Endstop Y
    const int NCZpin = A0;  //Endstop Z

    const int PGOOD = 0;


    const int RS485_RX = A5;
    const int RS485_TX = A4;
    const int RS485_DERE = A3;
    
    tmc.beginTMC2209();
    tmc.setupTMC2209(rms_mA, microsteps);

}

void MPC_01::setCoordMotion(CoordMotion mode) {
    motionMode = mode;
}

void MPC_01::setAxisTarget(int x, int y, int z, int a) {
    int motor1 = 0;
    int motor2 = 0;

    switch (motionMode)
    {
    case CoordMotion::LinearXY:
        motor1 = x;
        motor2 = y;
        break;

    case CoordMotion::CoreXY:
        motor1 = x + y;
        motor2 = x - y;
        break;
    
    default:
        Serial.println("No Motion System selected!");
        break;
    }
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

// - - - - - - - - - - TMC2209Manager - - - - - - - - - -

void TMC2209Manager::beginTMC2209(){
    // Pins initialisieren
    pinMode(STEP1, OUTPUT); pinMode(DIR1, OUTPUT);
    pinMode(STEP2, OUTPUT); pinMode(DIR2, OUTPUT);
    pinMode(STEP3, OUTPUT); pinMode(DIR3, OUTPUT);
    pinMode(STEP4, OUTPUT); pinMode(DIR4, OUTPUT);

    pinMode(EN_STATUS_PIN, INPUT);
    
    tmcSerial.begin(115200);
    delay(50);
}
bool TMC2209Manager::setupTMC2209(uint16_t rms_mA, uint8_t microsteps){
    setupOneDriver(drv0, rms_mA, microsteps);
    setupOneDriver(drv1, rms_mA, microsteps);
    setupOneDriver(drv2, rms_mA, microsteps);
    setupOneDriver(drv3, rms_mA, microsteps);

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
  bool ok3 = commOK(drv3, "DRV3");

  Serial.print("COMM OK: ");
  Serial.print(ok0); Serial.print(ok1); Serial.print(ok2); Serial.println(ok3);

  return ok0 && ok1 && ok2 && ok3;
}
void TMC2209Manager::arm() {
  drv0.toff(4);
  drv1.toff(4);
  drv2.toff(4);
  drv3.toff(4);
}
void TMC2209Manager::disarm() {
  drv0.toff(0);
  drv1.toff(0);
  drv2.toff(0);
  drv3.toff(0);
}
