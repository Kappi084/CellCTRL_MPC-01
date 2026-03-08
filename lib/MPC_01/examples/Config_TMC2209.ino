#include "MPC_01.h"

MPC_01 mpc;

void setup() {
  Serial.begin(9600); //Serielle Verbindung zur Konsole (zum Debuggen)
  mpc.begin(800, 16);   // 800mA RMS (Steppermotor Datenblatt), 16 Microsteps
}

void loop() {
  // hier später Motion/Stepgen
}