// GolfBuddy_RTOS_Declaration.h
//Autor: Niklas Krakhofer
//Project: Golf Buddy
//TODO: -

#ifndef _GOLFBUDDY_RTOS_DECLARATION_h
#define _GOLFBUDDY_RTOS_DECLARATION_h

#include <Arduino.h>
#include <TinyGPSPlus.h>
#include <HardwareSerial.h>
#include <SoftwareSerial.h>
#include "freertos/semphr.h"
#include "GolfBuddy_RTOS_Constants.h"
#include "MPU9250.h"

// Hardware Serials
extern TinyGPSPlus gps;
extern HardwareSerial piSerial;
extern HardwareSerial funkSerial;
extern SoftwareSerial gpsSerial;

struct GPSCoordinates {
    double dGolfTrolley_latitude = 48.28659492421564;
    double dGolfTrolley_longitude = 16.497860077134096;
};
extern GPSCoordinates trolleyCoords;

// Motor encoder
extern volatile unsigned long vulPulseCountMotorLeft;
extern volatile unsigned long vulPulseCountMotorRight;
extern unsigned long ulPulsesMotorLeft;
extern unsigned long ulPulsesMotorRight;
extern unsigned long sulLastPulseCountMotorLeft;
extern unsigned long sulLastPulseCountMotorRight;

// GY271 calibration / offsets
extern int16_t xOffset;
extern int16_t yOffset;
extern int16_t zOffset;

// BME Compensate Variables
extern uint16_t u16Dig_T1;
extern int16_t i16Dig_T2;
extern int16_t i16Dig_T3;
extern int32_t i32T_fine;

// Transmit data struct
struct TransmitDataToPi {
    int     iBatteryLevel = 0;
    double  dLatitudeGolfBuddy = 0.0;
    double  dLongitudeGolfBuddy = 0.0;
    float   fFacingDirection = 0.0;
};
extern TransmitDataToPi RaspPI_transmitData;

// GPS buffer
extern std::vector<GPSCoordinates> targetCoordsBuffer;

// Motor / speed
extern float fMotorLeftRPM;
extern float fMotorRightRPM;
extern float fMotorLeftSpeed_ms;
extern float fMotorRightSpeed_ms;
extern float fMotorLeftSpeed_kmh;
extern float fMotorRightSpeed_kmh;
extern float fSollMotorLeftRPM;
extern float fSollMotorRightRPM;

// PID Motor regulator
extern unsigned long ulTimestampRegulator;
extern unsigned long ulLastUpdateMotorRegulator;
extern float fKpMotorRegulator;
extern float fKiMotorRegulator;
extern float fIntegralMotorRegulator;

// Base speed
extern float fBaseSpeedSetting;

//HCSR04
extern char cSensorIDArray[3];
extern float fMeasuredDistances[3];

// PID regulator
extern float Kp;
extern float Ki;
extern float Kd;
extern float integral;
extern float lastError;
extern unsigned long lastTime;

// Other flags / offsets
extern bool bDogingactive;
extern int iDrivingDirectionMotorLeft;
extern int iDrivingDirectionMotorRight;
extern int16_t x;
extern int16_t y;
extern int16_t z;

// Income tracker
extern String sCurrentIncomeTrackerDataField;
extern String sIncomeTrackerDataFields[10];
extern int iIncomeTrackerFieldIndex;

// Golf trolley control
extern bool bvDriveAroundonRightwithCheck;
extern bool bvDriveAroundonLeftwithCheck;
extern bool bDogeRight;
extern bool bDodgeLeft;
extern bool bIsPlayerTrackingActivated;
extern bool bIsMotorSupportActivated;

extern int iTrackingRPM;

extern const uint32_t brakeDuration;
extern bool bIsBreakingActive;
extern int iBreakIntensityMotorLeft;
extern int iBreakIntensityMotorRight;

struct HCSR04_average {
    static constexpr int n = 4;
    float data[n] = { 0 };
    int index = 0;
    int count = 0;
    float summ = 0;
    float average = 0;

    void add(float distance) {
        summ -= data[index];
        data[index] = distance;
        summ += distance;
        index = (index + 1) % n;
        if (count < n) count++;
        average = summ / count;
    }
};

extern MPU9250 mpu;
extern bool trackerTrackingFlag;

#endif

