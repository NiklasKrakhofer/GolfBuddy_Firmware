// GolfBuddy_RTOS_Declaration.cpp
// Autor: Niklas Krakhofer
// Project: Golf Buddy

#include "GolfBuddy_RTOS_Declaration.h"

TinyGPSPlus gps;
MPU9250 mpu;
QMC5883P mag;

// Serial ports
HardwareSerial piSerial(1);
HardwareSerial gpsSerial(2);
SoftwareSerial funkSerial = SoftwareSerial(HC12TXPin, HC12RXPin);

GPSCoordinates trolleyCoords;

// BME / temperature / humidity
uint16_t u16Dig_T1;
int16_t i16Dig_T2;
int16_t i16Dig_T3;
int32_t i32T_fine;

// Transmit data struct
TransmitDataToPi RaspPI_transmitData;

// GPS buffer
std::vector<GPSCoordinates> targetCoordsBuffer;

// Motor variables
volatile unsigned long vulPulseCountMotorLeft = 0;
volatile unsigned long vulPulseCountMotorRight = 0;
float fMotorLeftRPM = 0;
float fMotorRightRPM = 0;
float fMotorLeftSpeed_ms = 0;
float fMotorRightSpeed_ms = 0;
float fMotorLeftSpeed_kmh = 0;
float fMotorRightSpeed_kmh = 0;
float fSollMotorLeftRPM = 0;
float fSollMotorRightRPM = 0;
int MotorPwmLeft = 0;
int MotorPwmRight = 0;

// Base speed and sensors
float fBaseSpeedSetting = 200;
char cSensorIDArray[3] = { HCSR04VorneLinks, HCSR04VorneRechts, HCSR04Hinten };
float fMeasuredDistances[3];

// Other flags / offsets
bool bDogingactive = false;
int iDrivingDirectionMotorLeft = 0;
int iDrivingDirectionMotorRight = 0;
int trackerTrackingFlag = 0;

// Golf trolley control
bool bIsPlayerTrackingActivated = false;
bool bIsMotorSupportActivated = false;

int iTrackingRPM = 75;

// Break variables
const uint32_t brakeDuration = 3000;
bool bIsBreakingActive = false;