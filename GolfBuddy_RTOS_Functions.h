// GolfBuddy_RTOS_Functions.h
// Autor: Niklas Krakhofer
// Project: Golf Buddy

#ifndef _GOLFBUDDY_RTOS_FUNCTIONS_h
#define _GOLFBUDDY_RTOS_FUNCTIONS_h

#include <Arduino.h>       
#include <Wire.h>        
#include <cstdint>        
#include "GolfBuddy_RTOS_Constants.h"
#include "GolfBuddy_RTOS_Declaration.h"

void IRAM_ATTR vCountPulseMotorLeft();
void IRAM_ATTR vCountPulseMotorRight();
void vResetBME280();
uint8_t ui8ReadRegister(uint8_t ui8Reg);
void vWriteRegister(uint8_t ui8Adress, uint8_t ui8Reg, uint8_t ui8Value);
void vReadCalibrationDataBME280();
int32_t i32ReadRawTemperatureBME280();
float fCompensateTemperatureBME280(int32_t i32Adc_T);
void updateGetHeadingWithGPS();
void vPlayerTracking(const GPSCoordinates& targetCoords, const GPSCoordinates& currentCoords);
double dGetTargetHeading(GPSCoordinates from, GPSCoordinates to);
void vUpdateHeadingControl(float fHeading, float fHeadingTarget, int iBaseSpeed, float fTurnSpeedFactor);
double dCalculateHaversine(double dLat1, double dLon1, double dLat2, double dLon2);
void vRegulateMotorLeftRPM(int iSollRPM);
void vRegulateMotorRightRPM(int iSollRPM);
float fPIRegulate(float fSetpoint, float fActual);
void vParking();
float fGetMessuredDistanceofHCSR04(int cSensorID);
void vSetDrivingdirectionMotorLeft(int iDirection);
void vSetDrivingdirectionMotorRight(int iDirection);
void print_MPU9250_calibration();
float mapFloat(float x, float in_min, float in_max, float out_min, float out_max);
uint32_t crc24q(const uint8_t* data, uint16_t len);

#endif

