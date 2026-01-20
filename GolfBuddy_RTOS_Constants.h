// GolfBuddy_RTOS_Constants.h
//Autor: Niklas Krakhofer
//Project: Golf Buddy
//TODO: -

#ifndef _GOLFBUDDY_RTOS_CONSTANTS_h
#define _GOLFBUDDY_RTOS_CONSTANTS_h

///////////////////////////////////////////////////////////////////////////////////////////////
// PIN Declaration
///////////////////////////////////////////////////////////////////////////////////////////////

#define RaspberryPIRXPin 20
#define RaspberryPITXPin 19

#define MotorLeftSpeedPin 21
#define MotorLeftPWMPin 47
#define MotorLeftBreakPin 48
#define MotorLeftDrivingDirectionPin 17

#define MotorRightSpeedPin 18
#define MotorRightPWMPin 35
#define MotorRightBreakPin 36
#define MotorRightDrivingDirectionPin 37

#define HC12RXPin 38
#define HC12TXPin 39
#define HC12SetPin 40

#define HCSR04TrigPin2 41
#define HCSR04EchoPin2 42

#define HCSR04TrigPin1 2
#define HCSR04EchoPin1 1

#define HCSR04TrigPin0 13
#define HCSR04EchoPin0 14

#define SDA 11
#define SCL 12

//#define GPSRXPin 9
//#define GPSTXPin 10
#define GPSRXPin 10
#define GPSTXPin 9
#define TouchSensorLeft 8
#define TouchSensorRight 3

#define AkkuVoltageMeasurePin 7

///////////////////////////////////////////////////////////////////////////////////////////////
// Constants
///////////////////////////////////////////////////////////////////////////////////////////////

// --- BME280 Defines ---
#define BME280_ADDR     0x76
#define REG_CALIB00     0x88
#define REG_CHIPID      0xD0
#define REG_CTRL_MEAS   0xF4
#define REG_STATUS      0xF3
#define REG_TEMP_MSB    0xFA

// --- GY271 Defines ---
#define ADDR  0x0d
#define MAG_ADDRESS 0x0D
#define XOUT_LSB 0x00

//values for the QMC5883 control register 1
//operating mode
#define Mode_Standby    0b00000000
#define Mode_Continuous 0b00000001
//Output data rate
#define ODR_10Hz        0b00000000
#define ODR_50Hz        0b00000100
#define ODR_100Hz       0b00001000
#define ODR_200Hz       0b00001100
//Measure range
#define RNG_2G          0b00000000
#define RNG_8G          0b00010000
//Over sampling rate
#define OSR_512         0b00000000
#define OSR_256         0b01000000
#define OSR_128         0b10000000
#define OSR_64          0b11000000

// --- Defines ---
#define HCSR04VorneLinks 1
#define HCSR04VorneRechts 2
#define HCSR04Hinten 0

#define BaudRate_9600 9600
#define Delay_1000 1000

#define AccelerationStepDelay_100 100
#define AccelerationSteps 10
#define pulsesPerTurn 48
#define wheelDiameter 0.175 // in meters

#define MotorLeft 0
#define MotorRight 1

#define DrivingDirectionForwards 0
#define DrivingDirectionBackwards 1

#define RPM_0 0
#define RPM_10 10

#define EmergencyBreaking 255

#define QMC_ADDR 0x0D
#define REG_DATA 0x00
#define REG_CTRL1 0x09
#define REG_CTRL2 0x0B

// CalibrationTimeGY271 (ms)
#define CALIB_TIME 10000UL

#define BUFFER_Coordinates_SIZE 1000

#define RASPI_RES_BUFFER_SIZE 4096  

#endif

