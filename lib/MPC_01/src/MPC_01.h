#pragma once
#include <Arduino.h>
#include <AltSoftSerial.h>
#include <TMCStepper.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <SoftwareSerial.h>
#include <AccelStepper.h>

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
        void homing();

        void setAxisTarget(int x, int y, int z);
        void setMoveSpeed(int speed);
        void setMoveAcceleration(int accel);

        void setGripper(bool state);
        void setTurnValue(int Turnval);
        void setTurnSpeed(int speed);

        void applyMotionStepper();
        void updateMotion();
        bool isMoving();
        void moveToSync();

        bool setStatus(uint8_t mode);
        void setservo(uint8_t ch, uint16_t pulse);

        void sendRS485Text(const char* text);
        void updateRS485();


    private:
        uint8_t maxAcceleration; //0-100%
        uint8_t maxSpeed;   //0-100%
        uint8_t xConstant;
        uint8_t yConstant;
        uint8_t zConstant;
        long motor1;
        long motor2;
        long motor3;
        
        uint8_t currentSpeed = 0;
        uint8_t currentAcceleration = 0;
        int8_t currentPositionX = -1;
        int8_t currentPositionY = -1;
        int8_t currentPositionZ = -1;
        int8_t currentPositionA = -1;
    //    int8_t currentStateGripper = -1;
    //    int8_t currentStateTurnunit = -1;

        static constexpr uint8_t NCXpin = A2;   //Endstop X
        static constexpr uint8_t NCYpin = A1;   //Endstop Y
        static constexpr uint8_t NCZpin = A0;   //Endstop Z

        uint8_t maxX = 0, maxY = 0, maxZ = 0;

        TMC2209Manager tmc;
        I2CManager i2c;
        Adafruit_PWMServoDriver pca = Adafruit_PWMServoDriver(0x40);
        RS485Manager rs485 {11, 10, A3};

        AccelStepper stepperX;
        AccelStepper stepperY;
        AccelStepper stepperZ;


        // ----- Pin Mapping (als Member/constexpr) -----
        static constexpr uint8_t STEP1 = 7, DIR1 = 6; //(Motor 1) X-Achse
        static constexpr uint8_t STEP2 = 5, DIR2 = 4; //(Motor 2) Y-Achse
        static constexpr uint8_t STEP3 = 3,  DIR3 = 2;  //(Motor 3) Z-Achse

        float maxSpeedSteps = 800.0;
        float accelSteps = 400.0;
        int8_t isEStopActive;

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

        static constexpr uint8_t EN_STATUS_PIN = 12;
        static constexpr float R_SENSE = 0.11f; // ggf. 0.075f (BTT-Shunt prüfen)
        // ----- UART & Treiber-Objekte müssen dauerhaft existieren -----
        AltSoftSerial tmcSerial;
        TMC2209Stepper drv0;
        TMC2209Stepper drv1;
        TMC2209Stepper drv2;

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
        RS485Manager(uint8_t rxPin, uint8_t txPin, uint8_t deRePin);

        void begin(unsigned long baud = 19200);

        void sendByte(uint8_t data);
        void sendBytes(const uint8_t* data, size_t len);
        void sendText(const char* text);

        bool available();
        int read();

    private:
        SoftwareSerial rs485Serial;
        uint8_t deRePin_;

        void setTransmitMode();
        void setReceiveMode();
};



