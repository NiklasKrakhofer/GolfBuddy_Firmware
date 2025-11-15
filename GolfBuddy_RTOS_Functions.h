// GolfBuddy_RTOS_Functions.h
//Autor: Niklas Krakhofer
//Project: Golf Buddy
//TODO: 

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
void vSetCtrlRegisterGY271(uint8_t ui8OverSampling, uint8_t ui8Range, uint8_t ui8DataRate, uint8_t ui8Mode);
void softReset(); //
void vSendTransmitdataToPi();
void vPushGPSData(float latitude, float longitude);
int32_t i32ReadRawTemperatureBME280();
float fCompensateTemperatureBME280(int32_t i32Adc_T);
void vUpdateMeausuredMotorSpeed();
void vWriteRegGY271(uint8_t ui8Reg, uint8_t ui8Value);
void vResetAndInitGY271();
bool bReadRawDataGY271(int16_t& i16RawX, int16_t& i16RawY, int16_t& i16RawZ);
double dGetHeading();
bool bGetOldestGPSCoordinate(GPSCoordinates& dataOut);
void vPlayerTracking(const GPSCoordinates& targetCoords, const GPSCoordinates& currentCoords);
double dGetTargetHeading(GPSCoordinates from, GPSCoordinates to);
void vUpdateHeadingControl(float fHeading, float fHeadingTarget, int iBaseSpeed, float fTurnSpeedFactor);
bool bGetNewestGPSCoordinates(GPSCoordinates& dataOut);
double dCalculateHaversine(double dLat1, double dLon1, double dLat2, double dLon2);
void vRegulateMotorLeftRPM(int iSollRPM);
void vRegulateMotorRightRPM(int iSollRPM);
float fPIRegulate(float fSetpoint, float fActual);
void vMotorSupport();
void vParking();
void vBreakMotorLeft(int iBreakIntensity);
void vBreakMotorRight(int iBreakIntensity);
void vCheckSurrounding();
float fGetMessuredDistanceofHCSR04(int cSensorID);
void vDodge();
void vSetDrivingdirectionMotorLeft(int iDirection);
void vSetDrivingdirectionMotorRight(int iDirection);
void vMakeASetBack();
void vMakeASetForward();
void vSpinToHeading(float fTargetHeading);
void vAccelarate(int iIntensity);
void vUpdateCalibrationDataGY271(int16_t i16X, int16_t i16Y);
void vCalculateOffsetsGY271();
void vPrintCalibrationDataGY271();

#endif

