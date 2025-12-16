//GolfBuddy_RTOS_Functions.cpp
//Autor: Niklas Krakhofer
//Project: Golf Buddy
//TODO: 

#include "GolfBuddy_RTOS_Functions.h"

void IRAM_ATTR vCountPulseMotorLeft() {
    vulPulseCountMotorLeft++;
}

void IRAM_ATTR vCountPulseMotorRight() {
    vulPulseCountMotorRight++;
}

void vResetBME280() {
    Wire.beginTransmission(BME280_ADDR);
    Wire.write(0xE0);
    Wire.write(0xB6);
    Wire.endTransmission();
    delay(10);
}

uint8_t ui8ReadRegister(uint8_t ui8Reg) {
    Wire.beginTransmission(BME280_ADDR);
    Wire.write(ui8Reg);
    Wire.endTransmission();
    Wire.requestFrom(BME280_ADDR, 1);
    return Wire.read();
}

void vWriteRegister(uint8_t ui8Adress, uint8_t ui8Reg, uint8_t ui8Value) {
    Wire.beginTransmission(ui8Adress);
    Wire.write(ui8Reg);
    Wire.write(ui8Value);
    Wire.endTransmission();
}

void vReadCalibrationDataBME280() {
    Wire.beginTransmission(BME280_ADDR);
    Wire.write(REG_CALIB00);
    Wire.endTransmission();
    Wire.requestFrom(BME280_ADDR, 6);

    u16Dig_T1 = (uint16_t)(Wire.read() | (Wire.read() << 8));
    i16Dig_T2 = (int16_t)(Wire.read() | (Wire.read() << 8));
    i16Dig_T3 = (int16_t)(Wire.read() | (Wire.read() << 8));
}

void vSetCtrlRegisterGY271(uint8_t ui8OverSampling, uint8_t ui8Range, uint8_t ui8DataRate, uint8_t ui8Mode) {
    vWriteRegister(ADDR, 9, ui8OverSampling | ui8Range | ui8DataRate | ui8Mode);
}

void softReset() {
    vWriteRegister(ADDR, 0x0a, 0x80);
    vWriteRegister(ADDR, 0x0b, 0x01);
}

void vSendTransmitdataToPi() {
    String out =
        String(RaspPI_transmitData.iBatteryLevel) + ";" +
        String(RaspPI_transmitData.dLatitudeGolfBuddy) + ";" +
        String(RaspPI_transmitData.dLongitudeGolfBuddy) + ";" +
        String(RaspPI_transmitData.fFacingDirection) + ";" +
        String(bIsPlayerTrackingActivated) +
        "\n";

    piSerial.write(out.c_str());
}

int32_t i32ReadRawTemperatureBME280() {
    Wire.beginTransmission(BME280_ADDR);
    Wire.write(REG_TEMP_MSB);
    Wire.endTransmission();
    Wire.requestFrom(BME280_ADDR, 3);

    int32_t adc_T = ((uint32_t)Wire.read() << 12);
    adc_T |= ((uint32_t)Wire.read() << 4);
    adc_T |= (Wire.read() >> 4);

    return adc_T;
}

float fCompensateTemperatureBME280(int32_t i32Adc_T) {
    int32_t var1, var2;
    var1 = ((((i32Adc_T >> 3) - ((int32_t)u16Dig_T1 << 1))) * ((int32_t)i16Dig_T2)) >> 11;
    var2 = (((((i32Adc_T >> 4) - ((int32_t)u16Dig_T1)) *
        ((i32Adc_T >> 4) - ((int32_t)u16Dig_T1))) >> 12) *
        ((int32_t)i16Dig_T3)) >> 14;
    i32T_fine = var1 + var2;
    float T = (i32T_fine * 5 + 128) >> 8;
    return T / 100.0f;
}

void vUpdateMeausuredMotorSpeed() {
    static unsigned long sulLastPulseCountMotorLeft = 0;
    static unsigned long sulLastPulseCountMotorRight = 0;

    unsigned long now = millis();
    float deltaTime = (now - ulLastUpdateMotorRegulator) / 1000.0; // Zeit in Sekunden
    if (deltaTime <= 0) return;

    // Pulsdifferenz seit letzter Messung
    unsigned long ulPulsesMotorLeft = vulPulseCountMotorLeft - sulLastPulseCountMotorLeft;
    unsigned long ulPulsesMotorRight = vulPulseCountMotorRight - sulLastPulseCountMotorRight;

    sulLastPulseCountMotorLeft = vulPulseCountMotorLeft;
    sulLastPulseCountMotorRight = vulPulseCountMotorRight;

    // Umdrehungen pro Sekunde
    float turnsPerSecondLeft = (float)ulPulsesMotorLeft / pulsesPerTurn / deltaTime;
    float turnsPerSecondRight = (float)ulPulsesMotorRight / pulsesPerTurn / deltaTime;

    // RPM
    fMotorLeftRPM = turnsPerSecondLeft * 60.0;
    fMotorRightRPM = turnsPerSecondRight * 60.0;

    // Geschwindigkeit
    fMotorLeftSpeed_ms = turnsPerSecondLeft * wheelDiameter * M_PI; // [m/s]
    fMotorRightSpeed_ms = turnsPerSecondRight * wheelDiameter * M_PI;

    fMotorLeftSpeed_kmh = fMotorLeftSpeed_ms * 3.6; // [km/h]
    fMotorRightSpeed_kmh = fMotorRightSpeed_ms * 3.6;

    ulLastUpdateMotorRegulator = now;
}

void vWriteRegGY271(uint8_t ui8Reg, uint8_t ui8Val) {
    Wire.beginTransmission(QMC_ADDR);
    Wire.write(ui8Reg);
    Wire.write(ui8Val);
    Wire.endTransmission();
}

void vResetAndInitGY271() {
    // Soft reset
    vWriteRegGY271(REG_CTRL2, 0x80);
    delay(10);
    vWriteRegGY271(REG_CTRL1, 0x1D);
    delay(10);
}

bool bReadRawDataGY271(int16_t& i16RawX, int16_t& i16RawY, int16_t& i16RawZ) {
    Wire.beginTransmission(QMC_ADDR);
    Wire.write(REG_DATA);
    if (Wire.endTransmission(false) != 0) return false; // no ack

    Wire.requestFrom(QMC_ADDR, (uint8_t)6);
    if (Wire.available() < 6) return false;

    uint8_t xl = Wire.read();
    uint8_t xh = Wire.read();
    uint8_t yl = Wire.read();
    uint8_t yh = Wire.read();
    uint8_t zl = Wire.read();
    uint8_t zh = Wire.read();

    // QMC5883L uses LSB then MSB
    i16RawX = (int16_t)((xh << 8) | xl);
    i16RawY = (int16_t)((yh << 8) | yl);
    i16RawZ = (int16_t)((zh << 8) | zl);
    return true;
}

double dGetHeading() {
    int16_t xr, yr, zr;
    if (!bReadRawDataGY271(xr, yr, zr)) {
        Serial.println("I2C read failed");
        delay(200);
    }

    // --- Offset-Korrektur ---
    double y_cal = (double)yr - y_offset;
    double z_cal = (double)zr - z_offset;

    // --- Soft-Iron Skalierung ---
    const double y_scale = (y_max - y_min) / 2.0;
    const double z_scale = (z_max - z_min) / 2.0;
    const double avg_scale = (y_scale + z_scale) / 2.0;

    double y_norm = y_cal * (avg_scale / y_scale);
    double z_norm = z_cal * (avg_scale / z_scale);

    // --- Heading (Y/Z angenommen horizontal) ---
    double heading = atan2(y_norm, z_norm) * 180.0 / M_PI;
    if (heading < 0) heading += 360.0;
    RaspPI_transmitData.fFacingDirection = heading;
    return heading;
}

void updateGetHeadingWithGPS()
{
    GPSCoordinates oldCoords;
    double φ1 = oldCoords.dGolfTrolley_latitude * M_PI / 180.0;
    double φ2 = trolleyCoords.dGolfTrolley_latitude * M_PI / 180.0;
    double Δλ = (trolleyCoords.dGolfTrolley_longitude - oldCoords.dGolfTrolley_longitude) * M_PI / 180.0;

    double y = sin(Δλ) * cos(φ2);
    double x = cos(φ1) * sin(φ2) - sin(φ1) * cos(φ2) * cos(Δλ);

    double θ = atan2(y, x);
    double heading = θ * 180.0 / M_PI;

    if (heading < 0) heading += 360.0;
    RaspPI_transmitData.fFacingDirection = heading;

    oldCoords.dGolfTrolley_latitude = trolleyCoords.dGolfTrolley_latitude;
    oldCoords.dGolfTrolley_longitude = trolleyCoords.dGolfTrolley_longitude;
}

void vPlayerTracking(const GPSCoordinates& targetCoords, const GPSCoordinates& currentCoords) {
	// Determine tracking speed based on buffer status
 //   if () {
 //       iTrackingRPM = 200; // faster
 //   }
 //   else if () {
	//	iTrackingRPM = 100; // slower
 //   }
	//else if () { // TODO: check if this case is possible
 //       iTrackingRPM = 0; // stehen bleiben -> ziel erreicht
 //   }
    
	// Update heading control to face target coordinate
    vUpdateHeadingControl(RaspPI_transmitData.fFacingDirection, dGetTargetHeading(currentCoords, targetCoords), iTrackingRPM, 1);
    Serial.println(RaspPI_transmitData.fFacingDirection);
    Serial.println(currentCoords.dGolfTrolley_latitude);
    Serial.println(currentCoords.dGolfTrolley_longitude);
    Serial.println(fSollMotorLeftRPM);
    Serial.println(fSollMotorRightRPM);
	// Regulate motors to desired RPM
    vRegulateMotorLeftRPM(fSollMotorLeftRPM);
    vRegulateMotorRightRPM(fSollMotorRightRPM);
   
	// If within 2 meters, mark coordinate as reached and remove it from buffer
    if (dCalculateHaversine(currentCoords.dGolfTrolley_latitude, currentCoords.dGolfTrolley_longitude, targetCoords.dGolfTrolley_latitude, targetCoords.dGolfTrolley_longitude) < 2) {
        if (!targetCoordsBuffer.empty()) {
            targetCoordsBuffer.erase(targetCoordsBuffer.begin());
        }
    }
}

double dGetTargetHeading(GPSCoordinates from, GPSCoordinates to) {
    double φ1 = from.dGolfTrolley_latitude * M_PI / 180.0;
    double φ2 = to.dGolfTrolley_latitude * M_PI / 180.0;
    double Δλ = (to.dGolfTrolley_longitude - from.dGolfTrolley_longitude) * M_PI / 180.0;

    double y = sin(Δλ) * cos(φ2);
    double x = cos(φ1) * sin(φ2) - sin(φ1) * cos(φ2) * cos(Δλ);

    double θ = atan2(y, x);
    double heading = θ * 180.0 / M_PI;

    if (heading < 0) heading += 360.0;

    return heading;
}

void vUpdateHeadingControl(float fHeading, float fHeadingTarget, int iBaseSpeed, float fTurnSpeedFactor) {
    unsigned long now = millis();
    float dt = (now - lastTime) / 1000.0; // Zeit in Sekunden
    if (dt <= 0) dt = 0.001; // Vermeidung von Division durch 0
    lastTime = now;

    // Fehler berechnen (zwischen -180° und 180°)
    float error = fHeadingTarget - fHeading;
    if (error > 180) error -= 360;
    if (error < -180) error += 360;

    // Integral- und Differentialanteil
    integral += error * dt;
    float derivative = (error - lastError) / dt;
    lastError = error;

    // PID-Ausgang
    float turn = Kp * error + Ki * integral + Kd * derivative;

    // Begrenzung von turn in Abhängigkeit von baseSpeed
    float maxTurn = iBaseSpeed * fTurnSpeedFactor;

    if (turn > maxTurn) turn = maxTurn;
    if (turn < -maxTurn) turn = -maxTurn;

    // Motorgeschwindigkeiten berechnen
    fSollMotorRightRPM = iBaseSpeed - turn;
    fSollMotorLeftRPM = iBaseSpeed + turn;
}

double dCalculateHaversine(double dLat1, double dLon1, double dLat2, double dLon2) {
    const double R = 6371000; // Erdradius in Metern
    double φ1 = dLat1 * M_PI / 180.0;
    double φ2 = dLat2 * M_PI / 180.0;
    double Δφ = (dLat2 - dLat1) * M_PI / 180.0;
    double Δλ = (dLon2 - dLon1) * M_PI / 180.0;
    double a = sin(Δφ / 2) * sin(Δφ / 2) + cos(φ1) * cos(φ2) * sin(Δλ / 2) * sin(Δλ / 2);
    return 2 * R * atan2(sqrt(a), sqrt(1 - a));
}

void vRegulateMotorLeftRPM(int iSollRPM) {
    float pwmValue = fPIRegulate(iSollRPM, fMotorLeftRPM);
    analogWrite(MotorLeftPWMPin, pwmValue);
}

void vRegulateMotorRightRPM(int iSollRPM) {
    float pwmValue = fPIRegulate(iSollRPM, fMotorRightRPM);
    analogWrite(MotorRightPWMPin, pwmValue);
}

float fPIRegulate(float fSetpoint, float fAactual) {
    unsigned long now = millis();
    float dt = (now - ulTimestampRegulator) / 1000.0;  // Sekunden
    if (dt <= 0.0f) dt = 0.001f;  // Schutz

    ulTimestampRegulator = now;

    // Optional: Messwert glätten (einfaches PT1)
    static float actual_filtered = 0;
    const float tau = 0.05; // Filterzeitkonstante in s (anpassen)
    actual_filtered += (fAactual - actual_filtered) * (dt / (tau + dt));

    float error = fSetpoint - actual_filtered;

    // Integral (Anti-Windup: Begrenzung)
    fIntegralMotorRegulator += error * dt;
    const float I_MAX = 1000.0; // anpassen (sinnvoller Bereich)
    if (fIntegralMotorRegulator > I_MAX) fIntegralMotorRegulator = I_MAX;
    if (fIntegralMotorRegulator < -I_MAX) fIntegralMotorRegulator = -I_MAX;

    float output = fKpMotorRegulator * error + fKiMotorRegulator * fIntegralMotorRegulator;

    // Falls du Richtung brauchst: begrenze symmetrisch, sonst 0..255
    const float OUT_MAX = 255.0;
    if (output > OUT_MAX) output = OUT_MAX;
    if (output < 0.0)    output = 0.0;

    return output;
}

void vMotorSupport() {
    if (digitalRead(TouchSensorLeft) || digitalRead(TouchSensorRight)) {
        vRegulateMotorLeftRPM(fBaseSpeedSetting);
        vRegulateMotorRightRPM(fBaseSpeedSetting);
    }
    else {
        vParking();
    }
}

void vParking() {
    vRegulateMotorLeftRPM(RPM_0);
    vRegulateMotorRightRPM(RPM_0);
    if (fMotorLeftRPM < RPM_10 && fMotorRightRPM < RPM_10) {
        vBreakMotorLeft(255);
        vBreakMotorRight(255);
    }
}

void vBreakMotorLeft(int iBreakInansity) {
    analogWrite(MotorLeftBreakPin, iBreakInansity);
}

void vBreakMotorRight(int iBreakInansity) {
    analogWrite(MotorRightBreakPin, iBreakInansity);
}

void vCheckSurrounding() {
    #ifdef ObsticaleDetectionWithDogeing
    for (int i = 0; i <= 1; i++) {
        float fDistance = fGetMessuredDistanceofHCSR04(cSensorIDArray[i]);
        if (!bDogingactive) {
            if (fDistance < 75) {
                bDogingactive = true;
                bIsBreakingActive = true;
                iBreakIntensityMotorLeft = EmergencyBreaking;
                iBreakIntensityMotorRight = EmergencyBreaking;
                vDodge();
                return true;
            }
            else if (fDistance < 250) {
                bDogingactive = true;
                vDodge();
                return true;
            }
        }
        else if (bDogingactive) {
            if (fDistance > 75) {
                bIsBreakingActive = true;
                iBreakIntensityMotorLeft = EmergencyBreaking;
                iBreakIntensityMotorRight = EmergencyBreaking;

                return true;
            }
        }
    }
    if (fGetMessuredDistanceofHCSR04(cSensorIDArray[3]) < 75) {
        bIsBreakingActive = true;
        iBreakIntensityMotorLeft = EmergencyBreaking;
        iBreakIntensityMotorRight = EmergencyBreaking;
    }
    return false;
    #endif

    static unsigned int sensorIndex = 0;
    sensorIndex = (sensorIndex + 1) % 3;

    float fDistance = fGetMessuredDistanceofHCSR04(cSensorIDArray[sensorIndex]);

    if (fDistance < 100) {
        bIsBreakingActive = true;
        iBreakIntensityMotorLeft = EmergencyBreaking;
        iBreakIntensityMotorRight = EmergencyBreaking;
    }
}

HCSR04_average HCSR04_0;
HCSR04_average HCSR04_1;
HCSR04_average HCSR04_2;

float fGetMessuredDistanceofHCSR04(int cSensorID) {
    int iActiveHCSR04TrigPin;
    int iActiveHCSR04EchoPin;
    HCSR04_average* activeSensor = nullptr;

    if (cSensorID == HCSR04VorneLinks) {
        iActiveHCSR04TrigPin = HCSR04TrigPin0;
        iActiveHCSR04EchoPin = HCSR04EchoPin0;
        activeSensor = &HCSR04_0;
    }

    else if (cSensorID == HCSR04VorneRechts) {
        iActiveHCSR04TrigPin = HCSR04TrigPin1;
        iActiveHCSR04EchoPin = HCSR04EchoPin1;
        activeSensor = &HCSR04_1;
    }

    else if (cSensorID == HCSR04Hinten) {
        iActiveHCSR04TrigPin = HCSR04TrigPin2;
        iActiveHCSR04EchoPin = HCSR04EchoPin2;
        activeSensor = &HCSR04_2;
    }

    digitalWrite(iActiveHCSR04TrigPin, LOW);
    delayMicroseconds(20);
    digitalWrite(iActiveHCSR04TrigPin, HIGH);
    delayMicroseconds(100);
    digitalWrite(iActiveHCSR04TrigPin, LOW);

    float duration = pulseIn(iActiveHCSR04EchoPin, HIGH);
    float distance = duration * 0.0343 / 2;

    activeSensor->add(distance);
     
    return activeSensor->average;
}

#ifdef ObsticaleDetectionWithDogeing
void vDodge() {
    for (int i = 0; i <= 2; i++) {
        fMeasuredDistances[i] = fGetMessuredDistanceofHCSR04(cSensorIDArray[i]);
    }
    if (iDrivingDirectionMotorLeft == DrivingDirectionForwards || iDrivingDirectionMotorRight == DrivingDirectionForwards) {
        if (fMeasuredDistances[HCSR04VorneLinks] < 100 && fMeasuredDistances[HCSR04VorneRechts] < 100) {
            Serial.println("bvDriveAroundonRightwithCheck");
            bvDriveAroundonRightwithCheck = true;
        }
        else if (fMeasuredDistances[HCSR04VorneLinks] < 100 && fMeasuredDistances[HCSR04VorneRechts] > 100) {
            Serial.println("bvDriveAroundonLeftwithCheck");
            bvDriveAroundonLeftwithCheck = true;
        }
        else if (fMeasuredDistances[HCSR04VorneLinks] > 100 && fMeasuredDistances[HCSR04VorneRechts] < 100) {
            Serial.println("bvDriveAroundonRightwithCheck");
            bvDriveAroundonRightwithCheck = true;
        }
        // wenn das obstacle weiter weg ist nur vorbeifahren und nicht extra zurückschieben
        else if (fMeasuredDistances[HCSR04VorneLinks] < 200 && fMeasuredDistances[HCSR04VorneRechts] < 200) {
            Serial.println("Dodge right");
            bDogeRight = true;

        }
        else if (fMeasuredDistances[HCSR04VorneLinks] < 200 && fMeasuredDistances[HCSR04VorneRechts] > 200) {
            Serial.println("Dodge left");
            bDodgeLeft = true;

        }
        else if (fMeasuredDistances[HCSR04VorneLinks] > 200 && fMeasuredDistances[HCSR04VorneRechts] < 200) {
            Serial.println("Dodge right");
            bDogeRight = true;

        }
    }
}
#endif // ObsticaleDetectionWithDogeing

void vSetDrivingdirectionMotorLeft(int iDirection) {
    if (iDirection == DrivingDirectionForwards) {
        iDrivingDirectionMotorLeft = DrivingDirectionForwards;
        digitalWrite(MotorLeftDrivingDirectionPin, HIGH);
    }
    else if (iDirection == DrivingDirectionBackwards) {
        iDrivingDirectionMotorLeft = DrivingDirectionBackwards;
        digitalWrite(MotorLeftDrivingDirectionPin, LOW);
    }
}

void vSetDrivingdirectionMotorRight(int iDirection) {
    if (iDirection == DrivingDirectionForwards) {
        iDrivingDirectionMotorRight = DrivingDirectionForwards;
        digitalWrite(MotorRightDrivingDirectionPin, LOW);
    }
    else if (iDirection == DrivingDirectionBackwards) {
        iDrivingDirectionMotorRight = DrivingDirectionBackwards;
        digitalWrite(MotorRightDrivingDirectionPin, HIGH);
    }
}

void vMakeASetBack() {
    vSetDrivingdirectionMotorLeft(DrivingDirectionBackwards);
    vSetDrivingdirectionMotorRight(DrivingDirectionBackwards);

    vRegulateMotorLeftRPM(60);
    vRegulateMotorRightRPM(60);
}

void vMakeASetForward() {
    vSetDrivingdirectionMotorLeft(DrivingDirectionForwards);
    vSetDrivingdirectionMotorRight(DrivingDirectionForwards);

    vRegulateMotorLeftRPM(60);
    vRegulateMotorRightRPM(60);
}

void vSpinToHeading(float fTargetHeading) {
    vUpdateHeadingControl(dGetHeading(), fTargetHeading, 100, 2);

    vRegulateMotorLeftRPM(fSollMotorLeftRPM);
    vRegulateMotorRightRPM(fSollMotorRightRPM);
}

void vAccelarate(int iItensity) {

}

void vUpdateCalibrationDataGY271(int16_t i16X, int16_t i16Y, int16_t i16Z) {
    if (i16X < x_min) x_min = i16X;
    if (i16X > x_max) x_max = i16X;
    if (i16Y < y_min) y_min = i16Y;
    if (i16Y > y_max) y_max = i16Y;
	if (i16Z < z_min) z_min = i16Z;
	if (i16Z > z_max) z_max = i16Z;
}

void vCalculateOffsetsGY271() {
    x_offset = (x_max + x_min) / 2;
    y_offset = (y_max + y_min) / 2;
	z_offset = (z_max + z_min) / 2;
    Serial.print("x_offset = "); Serial.println(x_offset);
    Serial.print("y_offset = "); Serial.println(y_offset);
    Serial.print("y_offset = "); Serial.println(z_offset);
}

void vPrintCalibrationDataGY271() {
    int duration_ms = 20000;
    Serial.println("Starte Kalibrierung. Bitte Modul 360° drehen...");
    unsigned long start = millis();
    int16_t xr, yr, zr;

    while (millis() - start < duration_ms) {
        if (bReadRawDataGY271(xr, yr, zr)) {
            vUpdateCalibrationDataGY271(xr, yr, zr);
        }
        delay(50);
    }

    vCalculateOffsetsGY271();
    Serial.println("Kalibrierung beendet!");
    Serial.print("x_min = "); Serial.println(x_min);
    Serial.print("x_max = "); Serial.println(x_max);
    Serial.print("y_min = "); Serial.println(y_min);
    Serial.print("y_max = "); Serial.println(y_max);
	Serial.print("z_min = "); Serial.println(z_min);
	Serial.print("z_max = "); Serial.println(z_max);
}