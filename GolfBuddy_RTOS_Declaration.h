// GolfBuddy_RTOS_Declaration.h
// Autor: Niklas Krakhofer
// Project: Golf Buddy

#ifndef _GOLFBUDDY_RTOS_DECLARATION_h
#define _GOLFBUDDY_RTOS_DECLARATION_h

#include <Arduino.h>
#include <TinyGPSPlus.h>
#include <HardwareSerial.h>
#include <SoftwareSerial.h>
#include "freertos/semphr.h"
#include "GolfBuddy_RTOS_Constants.h"
#include "MPU9250.h"
#include <qmc5883p.h>

extern TinyGPSPlus gps;
extern MPU9250 mpu;
extern QMC5883P mag;

// Serial ports
extern HardwareSerial piSerial;
extern HardwareSerial gpsSerial;
extern SoftwareSerial funkSerial;

struct GPSCoordinates 
{
    double dGolfTrolley_latitude = 0.0;
    double dGolfTrolley_longitude = 0.0;
};
extern GPSCoordinates trolleyCoords;

// BME Compensate Variables
extern uint16_t u16Dig_T1;
extern int16_t i16Dig_T2;
extern int16_t i16Dig_T3;
extern int32_t i32T_fine;

// Transmit data struct
struct TransmitDataToPi 
{
    float   iBatteryLevel = 0.0;
    double  dLatitudeGolfBuddy = 0.0;
    double  dLongitudeGolfBuddy = 0.0;
    float   fFacingDirection = 0.0;
    double  latitudeGolfPlayer = 0.0;
    double  longitudeGolfPlayer = 0.0;
    float   swingSpeed = 0.0;
};
extern TransmitDataToPi RaspPI_transmitData;

// GPS buffer
extern std::vector<GPSCoordinates> targetCoordsBuffer;

// Motor variables
extern volatile unsigned long vulPulseCountMotorLeft;
extern volatile unsigned long vulPulseCountMotorRight;
extern float fMotorLeftRPM;
extern float fMotorRightRPM;
extern float fMotorLeftSpeed_ms;
extern float fMotorRightSpeed_ms;
extern float fMotorLeftSpeed_kmh;
extern float fMotorRightSpeed_kmh;
extern float fSollMotorLeftRPM;
extern float fSollMotorRightRPM;
extern int MotorPwmLeft;
extern int MotorPwmRight;

// Base speed
extern float fBaseSpeedSetting;

//HCSR04
extern char cSensorIDArray[3];
extern float fMeasuredDistances[3];

// Other flags / offsets
extern int iDrivingDirectionMotorLeft;
extern int iDrivingDirectionMotorRight;
extern int trackerTrackingFlag;

// Golf trolley control
extern bool bIsPlayerTrackingActivated;
extern bool bIsMotorSupportActivated;

extern int iTrackingRPM;

// Break variables
extern const uint32_t brakeDuration;
extern bool bIsBreakingActive;

struct HCSR04_average 
{
    static constexpr int n = 4;
    float data[n] = { 0 };
    int index = 0;
    int count = 0;
    float summ = 0;
    float average = 0;

    void add(float distance) 
    {
        summ -= data[index];
        data[index] = distance;
        summ += distance;
        index = (index + 1) % n;
        if (count < n)
        {
            count++;
        }
        average = summ / count;
    }
};

#endif

