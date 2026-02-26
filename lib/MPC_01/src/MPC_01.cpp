#include "MPC_01.h"



void MPC_01::begin(uint16_t rms_mA, uint8_t microsteps){
    // Pins initialisieren
    pinMode(STEP1, OUTPUT); pinMode(DIR1, OUTPUT);
    pinMode(STEP2, OUTPUT); pinMode(DIR2, OUTPUT);
    pinMode(STEP3, OUTPUT); pinMode(DIR3, OUTPUT);
    pinMode(STEP4, OUTPUT); pinMode(DIR4, OUTPUT);


    pinMode(EN_PIN, OUTPUT);
    digitalWrite(EN_PIN, HIGH);

    pinMode(END_X, INPUT_PULLUP);
    pinMode(END_Y, INPUT_PULLUP);
    pinMode(END_Z, INPUT_PULLUP);
    setupTMC2209(rms_mA, microsteps);

    const int Servo1pin = 2; //Servo 1 (Turning Unit)
    const int Servo2pin = 3; //Servo 2 (Gripper)

    const int NCXpin = A2;  //Endstop X
    const int NCYpin = A1;  //Endstop Y
    const int NCZpin = A0;  //Endstop Z

    const int PGOOD = 0;


    const int RS485_RX = 8;
    const int RS485_TX = 9;
    const int RS485_DERE = A3;
    
}

bool MPC_01::setupTMC2209(uint16_t rms_mA, uint8_t microsteps){

    // Debug-Serial nur starten, wenn nicht schon im Sketch gemacht
    // (Wenn du es im Sketch machst, kannst du das hier weglassen)
//    if (!Serial) {
//        Serial.begin(115200);
//        delay(200);
//    }

    tmcSerial.begin(115200);
    delay(50);

    setupOneDriver(drv0, rms_mA, microsteps);
    setupOneDriver(drv1, rms_mA, microsteps);
    setupOneDriver(drv2, rms_mA, microsteps);
    setupOneDriver(drv3, rms_mA, microsteps);

    bool ok = testTMC();
    if (ok) {
        enableDrivers(true);
        Serial.println("All drivers enabled.");
    } else {
        Serial.println("UART problem: check wiring, pullup, MS1/MS2 addresses.");
    }
    return ok;

}

void MPC_01::enableDrivers(bool on) {
  // active LOW
  digitalWrite(EN_PIN, on ? LOW : HIGH);
}

void MPC_01::setupOneDriver(TMC2209Stepper &d, uint16_t rms_mA, uint8_t microsteps) {
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

    // Kommunikationstest: IOIN liefert sinnvolle Werte wenn Treiber antwortet
bool MPC_01::commOK(TMC2209Stepper &d, const char* name) {
  uint32_t ioin = d.IOIN();
  Serial.print(name);
  Serial.print(" IOIN=0x");
  Serial.println(ioin, HEX);

  return (ioin != 0x00000000UL && ioin != 0xFFFFFFFFUL);
}

bool MPC_01::testTMC() {
  bool ok0 = commOK(drv0, "DRV0");
  bool ok1 = commOK(drv1, "DRV1");
  bool ok2 = commOK(drv2, "DRV2");
  bool ok3 = commOK(drv3, "DRV3");

  Serial.print("COMM OK: ");
  Serial.print(ok0); Serial.print(ok1); Serial.print(ok2); Serial.println(ok3);

  return ok0 && ok1 && ok2 && ok3;
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




