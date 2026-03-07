#pragma once
#include <Arduino.h>
#include <AltSoftSerial.h>
#include <TMCStepper.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

class TMC2209Manager;
class I2CManager;

class MPC_01{
    public:
        MPC_01();
        void begin(uint16_t rms_mA, uint8_t microsteps);
        
        uint8_t setMaxX();
        uint8_t setMaxY();
        uint8_t setMaxZ();

        uint8_t getMaxX();
        uint8_t getMaxY();
        uint8_t getMaxZ();

        enum class CoordMotion : uint8_t {   
            CoreXY, 
            LinearXY
        };
        CoordMotion motionMode = CoordMotion::LinearXY;
        void setCoordMotion(CoordMotion mode);
        void calibrate(int8_t Multiplikator);

        void setAxisTarget(int x, int y, int z, int a);
        void setMoveSpeed(int speed);
        void setGripper(bool state);
        void setTurnValue(int Turnval);
        void setTurnSpeed(int speed);

        bool setStatus(uint8_t mode);
        void setservo(uint8_t ch, uint16_t pulse);

        void beginServo();



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

        static constexpr uint8_t SERVO1 = 2;
        static constexpr uint8_t SERVO2 = 3;

        static constexpr uint8_t END_X = A2;
        static constexpr uint8_t END_Y = A1;
        static constexpr uint8_t END_Z = A0;

        uint8_t maxX = 0, maxY = 0, maxZ = 0;
        TMC2209Manager tmc;
        I2CManager i2c;
        Adafruit_PWMServoDriver pca = Adafruit_PWMServoDriver(0x40);
};

class TMC2209Manager{
    public:
        TMC2209Manager();
        void beginTMC2209();
        bool setupTMC2209(uint16_t rms_mA, uint8_t microsteps);
        void arm();     //toff(4)
        void disarm();   //toff(0)
        // Debug / Diagnose
        bool testTMC();

    private:
        // ----- Pin Mapping (als Member/constexpr) -----
        static constexpr uint8_t STEP1 = 13, DIR1 = 12; //X-Achse
        static constexpr uint8_t STEP2 = 11, DIR2 = 10; //Y-Achse
        static constexpr uint8_t STEP3 = 5,  DIR3 = 4;  //Z-Achse
        static constexpr uint8_t STEP4 = 7,  DIR4 = 6;  //Reserve

        static constexpr uint8_t EN_STATUS_PIN = 12;
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
};

class I2CManager{
    public:
        void begin(uint32_t clockHz = 100000);
        void setDebug(bool on);
        void scan();
        bool ping(uint8_t addr);
        bool writeBytes(uint8_t addr, const uint8_t* data, size_t len);
        bool readBytes(uint8_t addr, uint8_t* data, size_t len);

        // --- Register helpers (typisch für I2C-Module) ---
        bool writeReg8(uint8_t addr, uint8_t reg, uint8_t value);
        bool writeReg16(uint8_t addr, uint8_t reg, uint16_t value, bool msbFirst = true);

        bool readReg8(uint8_t addr, uint8_t reg, uint8_t &value);
        bool readRegBytes(uint8_t addr, uint8_t reg, uint8_t* data, size_t len);
        
    private:
      bool debug_ = false;

        // Helfer: Ausgabe nur wenn debug_ aktiv ist
        void dbgPrint_(const __FlashStringHelper* s);
        void dbgPrintHex_(uint8_t v);

};

class RS485Manager{
    public:

    private:


};



