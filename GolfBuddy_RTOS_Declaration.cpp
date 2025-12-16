//GolfBuddy_RTOS_Declaration.cpp
//Autor: Niklas Krakhofer
//Project: Golf Buddy
//TODO: ms umbennen in m/s

#include "GolfBuddy_RTOS_Declaration.h"

// Hardware Serials
TinyGPSPlus gps;
HardwareSerial piSerial(1);
HardwareSerial funkSerial(2);
SoftwareSerial gpsSerial = SoftwareSerial(GPSRXPin, GPSTXPin);
GPSCoordinates trolleyCoords;

// Motor encoder
volatile unsigned long vulPulseCountMotorLeft = 0;
volatile unsigned long vulPulseCountMotorRight = 0;
unsigned long ulPulsesMotorLeft = 0;
unsigned long ulPulsesMotorRight = 0;
unsigned long sulLastPulseCountMotorLeft = 0;
unsigned long sulLastPulseCountMotorRight = 0;

// Sensor calibration / offsets
int16_t xOffset = 3213;
int16_t yOffset = -2415;
int16_t zOffset = -3795;

// BME / temperature / humidity
uint16_t u16Dig_T1;
int16_t i16Dig_T2;
int16_t i16Dig_T3;
int32_t i32T_fine;

// Transmit data struct
TransmitDataToPi RaspPI_transmitData;

// GPS buffer
std::vector<GPSCoordinates> targetCoordsBuffer;

// Motor / speed
float fMotorLeftRPM = 0;
float fMotorRightRPM = 0;
float fMotorLeftSpeed_ms = 0;
float fMotorRightSpeed_ms = 0;
float fMotorLeftSpeed_kmh = 0;
float fMotorRightSpeed_kmh = 0;
float fSollMotorLeftRPM = 0;
float fSollMotorRightRPM = 0;

// PID Motor regulator
unsigned long ulTimestampRegulator = 0;
unsigned long ulLastUpdateMotorRegulator = 0;
float fKpMotorRegulator = 0.1;
float fKiMotorRegulator = 0.02;
float fIntegralMotorRegulator = 0;

// Base speed and sensors
float fBaseSpeedSetting = 100;
char cSensorIDArray[3] = { HCSR04VorneLinks, HCSR04VorneRechts, HCSR04Hinten };
float fMeasuredDistances[3];

// PID driving
float Kp = 1.0;
float Ki = 0.0;
float Kd = 0.2;
float integral = 0;
float lastError = 0;
unsigned long lastTime = 0;

// Other flags / offsets
bool bDogingactive = false;
int iDrivingDirectionMotorLeft = 0;
int iDrivingDirectionMotorRight = 0;
int16_t x;
int16_t y;
int16_t z;

// Income tracker
String sCurrentIncomeTrackerDataField = "";
String sIncomeTrackerDataFields[10];
int iIncomeTrackerFieldIndex = 0;

// Golf trolley control
bool bvDriveAroundonRightwithCheck = false;
bool bvDriveAroundonLeftwithCheck = false;
bool bDogeRight = false;
bool bDodgeLeft = false;
bool bIsPlayerTrackingActivated = false;
bool bIsMotorSupportActivated = false;

long x_min = -1092, x_max = 1507;
long y_min = -331, y_max = 986;
long z_min = -1011, z_max = 2023;
long x_offset = 207, y_offset = 327, z_offset = 506;

int iTrackingRPM;

const uint32_t brakeDuration = 3000;
bool bIsBreakingActive = false;
int iBreakIntensityMotorLeft;
int iBreakIntensityMotorRight;
