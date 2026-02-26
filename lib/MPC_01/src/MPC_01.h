#pragma once
#include <Arduino.h>
#include <AltSoftSerial.h>
#include <TMCStepper.h>


class MPC_01{
    public:
    /*
        MPC_01(uint8_t mX, uint8_t mY, uint8_t mZ, uint8_t mA, uint8_t mS)
            : maxX(mX), maxY(mY), maxZ(mZ), maxAcceleration(mA), maxSpeed(mS)
        {}
            */

        void begin(uint16_t rms_mA = 800, uint8_t microsteps = 16);
        bool setupTMC2209(uint16_t rms_mA, uint8_t microsteps);
        void enableDrivers(bool on);

        // Debug / Diagnose
        bool testTMC();

        uint8_t getMaxX();
        uint8_t getMaxY();
        uint8_t getMaxZ();

        enum class CoordMotion : uint8_t {   
            CoreXY, LinearXY
        };

        void calibrate(int8_t Multiplikator);

        void setCoordMotion();
        void setAxisTarget(int x, int y, int z, int a);
        void setMoveSpeed(int speed);

        void setGripper(bool state);
        void setTurnValue(int Turnval);
        void setTurnSpeed(int speed);


    private:
        uint8_t maxAcceleration; //0-100%
        uint8_t maxSpeed;   //0-100%

        uint8_t currentSpeed = 0;
        uint8_t currentAcceleration = 0;
        int8_t currentPositionX = -1;
        int8_t currentPositionY = -1;
        int8_t currentPositionZ = -1;
        int8_t currentPositionA = -1;
    //    int8_t currentStateGripper = -1;
    //    int8_t currentStateTurnunit = -1;

    // ----- Pin Mapping (als Member/constexpr) -----
        static constexpr uint8_t STEP1 = 13; // X
        static constexpr uint8_t DIR1  = 12;

        static constexpr uint8_t STEP2 = 11; // Y
        static constexpr uint8_t DIR2  = 10;

        static constexpr uint8_t STEP3 = 5;  // Z
        static constexpr uint8_t DIR3  = 4;

        static constexpr uint8_t STEP4 = 7;  // A
        static constexpr uint8_t DIR4  = 6;

        // WICHTIG: NICHT 0/1 verwenden. Nimm einen freien Pin.
        static constexpr uint8_t EN_PIN = 1;

        static constexpr uint8_t SERVO1 = 2;
        static constexpr uint8_t SERVO2 = 3;

        static constexpr uint8_t END_X = A2;
        static constexpr uint8_t END_Y = A1;
        static constexpr uint8_t END_Z = A0;

        static constexpr float R_SENSE = 0.11f; // ggf. 0.075f (BTT-Shunt prüfen)
        
        // ----- UART & Treiber-Objekte müssen dauerhaft existieren -----
        AltSoftSerial tmcSerial;

        TMC2209Stepper drv0;
        TMC2209Stepper drv1;
        TMC2209Stepper drv2;
        TMC2209Stepper drv3;

        // interne Helfer
        void setupOneDriver(TMC2209Stepper &d, uint16_t rms_mA, uint8_t microsteps);
        bool commOK(TMC2209Stepper &d, const char* name);

        uint8_t maxX = 0, maxY = 0, maxZ = 0;

};

