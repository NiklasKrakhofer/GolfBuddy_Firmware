// GolfBuddy_RTOS_Functions.cpp
// Autor: Niklas Krakhofer
// Project: Golf Buddy

#include "GolfBuddy_RTOS_Functions.h"

// Counts up vulPulseCountMotorLeft if Motor is spinning
void IRAM_ATTR vCountPulseMotorLeft()
{
	vulPulseCountMotorLeft++;
}

// Counts up vulPulseCountMotorRight if Motor is spinning
void IRAM_ATTR vCountPulseMotorRight()
{
	vulPulseCountMotorRight++;
}

//@brief: Resets the BME280
//@param: -
//@return: -
void vResetBME280()
{
	Wire.beginTransmission(BME280_ADDR);
	Wire.write(0xE0);
	Wire.write(0xB6);
	Wire.endTransmission();
	delay(10);
}

//@brief: Support funktion to read out the value of an register of an I2C address.
//@param: -
//@return: -
uint8_t ui8ReadRegister(uint8_t ui8Reg)
{
	Wire.beginTransmission(BME280_ADDR);
	Wire.write(ui8Reg);
	Wire.endTransmission();
	Wire.requestFrom(BME280_ADDR, 1);
	return Wire.read();
}

//@brief: Support funktion to change the value of a register based of the I2C address.
//@param: -
//@return: -
void vWriteRegister(uint8_t ui8Adress, uint8_t ui8Reg, uint8_t ui8Value)
{
	Wire.beginTransmission(ui8Adress);
	Wire.write(ui8Reg);
	Wire.write(ui8Value);
	Wire.endTransmission();
}

//@brief: Reads and sets BME280 calibration values
//@param: -
//@return: -
void vReadCalibrationDataBME280()
{
	Wire.beginTransmission(BME280_ADDR);
	Wire.write(REG_CALIB00);
	Wire.endTransmission();
	Wire.requestFrom(BME280_ADDR, 6);

	u16Dig_T1 = (uint16_t)(Wire.read() | (Wire.read() << 8));
	i16Dig_T2 = (int16_t)(Wire.read() | (Wire.read() << 8));
	i16Dig_T3 = (int16_t)(Wire.read() | (Wire.read() << 8));
}

//@brief: Reads the raw temperature from BME280.
//@param: -
//@return: raw Temperature
int32_t i32ReadRawTemperatureBME280()
{
	Wire.beginTransmission(BME280_ADDR);
	Wire.write(REG_TEMP_MSB);
	Wire.endTransmission();
	Wire.requestFrom(BME280_ADDR, 3);

	int32_t adc_T = ((uint32_t)Wire.read() << 12);
	adc_T |= ((uint32_t)Wire.read() << 4);
	adc_T |= (Wire.read() >> 4);

	return adc_T;
}

//@brief: Compensates measured temperature
//@param: i32Adc_T : measured temperature
//@return: compensated temperature
float fCompensateTemperatureBME280(int32_t i32Adc_T)
{
	int32_t var1, var2;
	var1 = ((((i32Adc_T >> 3) - ((int32_t)u16Dig_T1 << 1))) * ((int32_t)i16Dig_T2)) >> 11;
	var2 = (((((i32Adc_T >> 4) - ((int32_t)u16Dig_T1)) *
		((i32Adc_T >> 4) - ((int32_t)u16Dig_T1))) >> 12) *
		((int32_t)i16Dig_T3)) >> 14;
	i32T_fine = var1 + var2;
	float T = (i32T_fine * 5 + 128) >> 8;
	return T / 100.0f;
}

//@brief: Calculates the heading of the GolfBuddy based on its coordinate, by calculating the angle between to coordinates.
//        This is only working, when the GolfBuddy is in motion.
//@param: -
//@return: -
void updateGetHeadingWithGPS()
{
	static GPSCoordinates oldCoords;

	// Convert latitudes from degrees to radians
	double phi1 = oldCoords.dGolfTrolley_latitude * M_PI / 180.0;
	double phi2 = trolleyCoords.dGolfTrolley_latitude * M_PI / 180.0;

	// Longitude difference in radians
	double deltaLambda = (trolleyCoords.dGolfTrolley_longitude - oldCoords.dGolfTrolley_longitude) * M_PI / 180.0;

	// Bearing calculation components
	double y = sin(deltaLambda) * cos(phi2);
	double x = cos(phi1) * sin(phi2)
		- sin(phi1) * cos(phi2) * cos(deltaLambda);

	// Convert bearing in degrees
	double heading = atan2(y, x) * 180.0 / M_PI;
	if (heading < 0)
	{
		heading += 360.0;
	}

	RaspPI_transmitData.fFacingDirection = heading;

	oldCoords.dGolfTrolley_latitude = trolleyCoords.dGolfTrolley_latitude;
	oldCoords.dGolfTrolley_longitude = trolleyCoords.dGolfTrolley_longitude;
}

//@brief: Moves the GolfBuddy to the targetCoord.
//@param: targetCoord  : The resulting position of the GolfBuddy
//        currentCoord : The current coordinate of the GolfBuddy
//@return: -
void vPlayerTracking(const GPSCoordinates& targetCoord, const GPSCoordinates& currentCoord)
{
	vSetDrivingdirectionMotorLeft(DrivingDirectionForwards);
	vSetDrivingdirectionMotorRight(DrivingDirectionForwards);

	// Calcualte Motors RPM to turn to target coordinate
	vUpdateHeadingControl(RaspPI_transmitData.fFacingDirection, dGetTargetHeading(currentCoord, targetCoord), iTrackingRPM, 0.75);

	//// DEBUG
    Serial.println(dGetTargetHeading(currentCoord, targetCoord));
	Serial.println(RaspPI_transmitData.fFacingDirection);
	Serial.println(fSollMotorLeftRPM);
	Serial.println(fSollMotorRightRPM);

	// Regulate motors to desired RPM
	vRegulateMotorLeftRPM(fSollMotorLeftRPM);
	vRegulateMotorRightRPM(fSollMotorRightRPM + 15);

	// If GolfBuddy within 4 meters of targetCoord, remove targetCoord from buffer
	if (dCalculateHaversine(currentCoord.dGolfTrolley_latitude, currentCoord.dGolfTrolley_longitude, targetCoord.dGolfTrolley_latitude, targetCoord.dGolfTrolley_longitude) < 4)
	{
		if (!targetCoordsBuffer.empty())
		{
			targetCoordsBuffer.erase(targetCoordsBuffer.begin());
		}
	}
}

//@brief: Calculates the angle between to coordinates.
//@param: from : Coordinate 1
//        to   : Coordinate 2
//@return: Angle between to coordinates
double dGetTargetHeading(GPSCoordinates from, GPSCoordinates to)
{
	// Convert latitudes from degrees to radians
	double phi1 = from.dGolfTrolley_latitude * M_PI / 180.0;
	double phi2 = to.dGolfTrolley_latitude * M_PI / 180.0;

	// Longitude difference in radians
	double deltaLambda =
		(to.dGolfTrolley_longitude - from.dGolfTrolley_longitude) * M_PI / 180.0;

	// Bearing calculation components
	double y = sin(deltaLambda) * cos(phi2);
	double x = cos(phi1) * sin(phi2)
		- sin(phi1) * cos(phi2) * cos(deltaLambda);

	// Bearing in degrees
	double angle = atan2(y, x) * 180.0 / M_PI;
	if (angle < 0) angle += 360.0;

	return angle;
}

//@brief: Calculates the RPM of both motors to turn and face the target coordinate. The RPMs are calculated based on the current GolfBuddy heading and the heading from the GolfBuddy to the target
//        coordinate. The average speed is defined by baseSpeed and the speed of turning is defined by the turnSpeedFactor. The system is regulated with a PID-Regulator.
//@param: heading         : Current heading of the GolfBuddy
//        headingTarget   : Angle between GolfBuddy and target coordinate
//        baseSpeed       : Average RPM of the GolfBuddy
//        turnFactor : Defines how sharp the GolfBuddy turns. From 0 to 1 0 = no turn
//                                                                             1 = turning at the same place possible
//@return: -
void vUpdateHeadingControl(float heading, float headingTarget, int baseSpeed, float turnFactor)
{
	// PID controller gains
	const float KP = 1.0;
	const float KI = 0.0;
	const float KD = 0.2;

	// PID state variables
	static float integral = 0;
	static float lastError = 0;
	static unsigned long lastTime = 0;

	// Time step in seconds
	unsigned long now = millis();
	float dt = (now - lastTime) / 1000.0;
	if (dt <= 0) dt = 0.001;
	lastTime = now;

	// Heading error normalized to -180..180 degrees
	float error = headingTarget - heading;
	if (error > 180)  error -= 360;
	if (error < -180) error += 360;

	// PID calculations
	integral += error * dt;
	float derivative = (error - lastError) / dt;
	lastError = error;

	// PID output (turn rate)
	float turn = KP * error + KI * integral + KD * derivative;

	// Limit turn rate relative to base speed
	float maxTurn = baseSpeed * turnFactor;
	if (turn > maxTurn) turn = maxTurn;
	if (turn < -maxTurn) turn = -maxTurn;

	// Differential motor speed control
	fSollMotorRightRPM = baseSpeed - turn;
	fSollMotorLeftRPM = baseSpeed + turn;
}

//@brief: Calculates distance between two GPS Coordinates.
//@param: lat1 : Latitude Coordinate 1 [±ddd.ddddd°]
//        lon1 : Longitude Coordinate 1 [±ddd.ddddd°]
//        lat2 : Latitude Coordinate 2 [±ddd.ddddd°]
//        lon1 : Longitude Coordinate 2 [±ddd.ddddd°]
//@return: Distance bettween the Coordinates [m]
double dCalculateHaversine(double lat1, double lon1, double lat2, double lon2)
{
	const double R = 6371000; // Earthradius in [m]
	double phi1 = lat1 * M_PI / 180.0;
	double phi2 = lat2 * M_PI / 180.0;
	double deltaPhi = (lat2 - lat1) * M_PI / 180.0;
	double deltaLambda = (lon2 - lon1) * M_PI / 180.0;
	double a = sin(deltaPhi / 2) * sin(deltaPhi / 2) + cos(phi1) * cos(phi2) * sin(deltaLambda / 2) * sin(deltaLambda / 2);
	return 2 * R * atan2(sqrt(a), sqrt(1 - a));
}

//@brief: Regulates the RPM of the left motor to sollRPM.
//@param: sollRPM : The to be reached RPM value of the left motor.
//@return: -
void vRegulateMotorLeftRPM(int sollRPM)
{
	analogWrite(MotorLeftPWMPin, fPIRegulate(sollRPM, fMotorLeftRPM));
}

//@brief: Regulates the RPM of the right motor to sollRPM.
//@param: sollRPM : The to be reached RPM value of the right motor.
//@return: -
void vRegulateMotorRightRPM(int sollRPM)
{
	analogWrite(MotorRightPWMPin, fPIRegulate(sollRPM, fMotorRightRPM));
}

//@brief: Returns an 8-Bit PWM value based on setpoint and actual with an PI-Regulator.
//@param: setpoint : The to be reached value
//        actual   : The current value
//@return: 8-Bit PWM value
float fPIRegulate(float setpoint, float actual)
{
	// PI controller gains
	const float KP = 0.1;
	const float KI = 0.02;

	// PI state variables
	static float integral = 0;
	static unsigned long lastUpdate = 0;

	// Time step in seconds
	unsigned long now = millis();
	float dt = (now - lastUpdate) / 1000.0;
	if (dt <= 0.0f) dt = 0.001f;
	lastUpdate = now;

	// Low-pass filter for measured value
	static float actual_filtered = 0;
	const float tau = 0.05;
	actual_filtered += (actual - actual_filtered) * (dt / (tau + dt));

	// Control error
	float error = setpoint - actual_filtered;

	// Integral term with anti-windup
	integral += error * dt;
	const float I_MAX = 1000.0;
	if (integral > I_MAX) integral = I_MAX;
	if (integral < -I_MAX) integral = -I_MAX;

	// PI controller output
	float output = KP * error + KI * integral;

	// Output saturation
	const float OUT_MAX = 255.0;
	if (output > OUT_MAX) output = OUT_MAX;
	if (output < 0.0f)    output = 0.0f;

	return output;
}

//@brief: Parking sets the motorrpm to 0 and breaks both motors. Should be called if GolfBuddy is not moving.
//@param: -
//@return: -
void vParking()
{
	analogWrite(MotorLeftPWMPin, 0);
	analogWrite(MotorRightPWMPin, 0);
	// TODO: Add breaking
}

HCSR04_average HCSR04_0;
HCSR04_average HCSR04_1;
HCSR04_average HCSR04_2;

//@brief: Returns average measured distance of an HCSR04
//@param: sensorID : ID of the to be measured sensor 
//                   0 -> HCSR04Hinten
//                   1 -> HCSR04VorneLinks
//                   2 -> HCSR04VorneRechts
//@return: The average measured distance of an HCSR04
float fGetMessuredDistanceofHCSR04(int sensorID)
{
	int iActiveHCSR04TrigPin;
	int iActiveHCSR04EchoPin;
	HCSR04_average* activeSensor = nullptr;

	// Determine the sensor to be measured
	if (sensorID == HCSR04VorneLinks)
	{
		iActiveHCSR04TrigPin = HCSR04TrigPin2;
		iActiveHCSR04EchoPin = HCSR04EchoPin2;
		activeSensor = &HCSR04_0;
	}
	else if (sensorID == HCSR04VorneRechts)
	{
		iActiveHCSR04TrigPin = HCSR04TrigPin1;
		iActiveHCSR04EchoPin = HCSR04EchoPin1;
		activeSensor = &HCSR04_1;
	}
	else if (sensorID == HCSR04Hinten)
	{
		iActiveHCSR04TrigPin = HCSR04TrigPin0;
		iActiveHCSR04EchoPin = HCSR04EchoPin0;
		activeSensor = &HCSR04_2;
	}

	// Start measure
	digitalWrite(iActiveHCSR04TrigPin, LOW);
	delayMicroseconds(20);
	digitalWrite(iActiveHCSR04TrigPin, HIGH);
	delayMicroseconds(100);
	digitalWrite(iActiveHCSR04TrigPin, LOW);
	// Await positive flank
	float duration = pulseIn(iActiveHCSR04EchoPin, HIGH);
	float distance = duration * 0.0343 / 2; // Calculate the distance based on ultrasonic travel time

	activeSensor->add(distance); // Add measured distance to sensor array

	return activeSensor->average; // Return average measured distance
}

//@brief: Changes the drivingdirection of the left motor
//@param: direction : 0 -> DrivingDirectionForwards
//                    1 -> DrivingDirectionBackwards
//@return: -
void vSetDrivingdirectionMotorLeft(int iDirection)
{
	if (iDirection == DrivingDirectionForwards)
	{
		iDrivingDirectionMotorLeft = DrivingDirectionForwards;
		digitalWrite(MotorLeftDrivingDirectionPin, HIGH);
	}
	else if (iDirection == DrivingDirectionBackwards)
	{
		iDrivingDirectionMotorLeft = DrivingDirectionBackwards;
		digitalWrite(MotorLeftDrivingDirectionPin, LOW);
	}
}

//@brief: Changes the drivingdirection of the right motor
//@param: direction : 0 -> DrivingDirectionForwards
//                    1 -> DrivingDirectionBackwards
//@return: -
void vSetDrivingdirectionMotorRight(int direction)
{
	if (direction == DrivingDirectionForwards)
	{
		iDrivingDirectionMotorRight = DrivingDirectionForwards;
		digitalWrite(MotorRightDrivingDirectionPin, LOW);
	}
	else if (direction == DrivingDirectionBackwards)
	{
		iDrivingDirectionMotorRight = DrivingDirectionBackwards;
		digitalWrite(MotorRightDrivingDirectionPin, HIGH);
	}
}

//@brief: Prints out calibration values after MPU9250 calibration for calibration.
//@param: -
//@return: -
void print_MPU9250_calibration()
{
	Serial.println("< calibration parameters >");
	Serial.println("accel bias [g]: ");
	Serial.print(mpu.getAccBiasX() * 1000.f / (float)MPU9250::CALIB_ACCEL_SENSITIVITY);
	Serial.print(", ");
	Serial.print(mpu.getAccBiasY() * 1000.f / (float)MPU9250::CALIB_ACCEL_SENSITIVITY);
	Serial.print(", ");
	Serial.print(mpu.getAccBiasZ() * 1000.f / (float)MPU9250::CALIB_ACCEL_SENSITIVITY);
	Serial.println();
	Serial.println("gyro bias [deg/s]: ");
	Serial.print(mpu.getGyroBiasX() / (float)MPU9250::CALIB_GYRO_SENSITIVITY);
	Serial.print(", ");
	Serial.print(mpu.getGyroBiasY() / (float)MPU9250::CALIB_GYRO_SENSITIVITY);
	Serial.print(", ");
	Serial.print(mpu.getGyroBiasZ() / (float)MPU9250::CALIB_GYRO_SENSITIVITY);
	Serial.println();
	Serial.println("mag bias [mG]: ");
	Serial.print(mpu.getMagBiasX());
	Serial.print(", ");
	Serial.print(mpu.getMagBiasY());
	Serial.print(", ");
	Serial.print(mpu.getMagBiasZ());
	Serial.println();
	Serial.println("mag scale []: ");
	Serial.print(mpu.getMagScaleX());
	Serial.print(", ");
	Serial.print(mpu.getMagScaleY());
	Serial.print(", ");
	Serial.print(mpu.getMagScaleZ());
	Serial.println();
}

//@brief: Performs linearization of a value with float
//@param: x       : the value to be linearized (input)
//        in_min  : input minimum
//        in_max  : input maximum
//        out_min : output minimum
//        out_max : output maximum
//@return: linearized value type float
float mapFloat(float x, float in_min, float in_max, float out_min, float out_max)
{
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

//@brief: Calculates rtcm crc
//@param: data : recieved rtcm frame
//        len  : length of the recieved rtcm frame
//@return: calculated crc
uint32_t crc24q(const uint8_t* data, uint16_t len)
{
	uint32_t crc = 0;
	for (uint16_t i = 0; i < len; i++)
	{
		crc ^= ((uint32_t)data[i]) << 16;
		for (uint8_t j = 0; j < 8; j++)
		{
			crc <<= 1;
			if (crc & 0x1000000)
			{
				crc ^= 0x1864CFB;
			}
		}
	}
	return crc & 0xFFFFFF;
}

//@brief: 
//@param: 
//@return: 
float fuseHeading2(float deg1, float deg2, float w1, float w2)
{
	float sum = w1 + w2;
	if (sum <= 0.0f) return deg1;
	w1 /= sum;
	w2 /= sum;

	float rad1 = radians(deg1);
	float rad2 = radians(deg2);

	float x =
		w1 * cos(rad1) +
		w2 * cos(rad2);

	float y =
		w1 * sin(rad1) +
		w2 * sin(rad2);

	float fusedRad = atan2(y, x);
	float fusedDeg = degrees(fusedRad);

	if (fusedDeg < 0) fusedDeg += 360.0f;

	return fusedDeg;
}

//@brief: 
//@param: 
//@return: 
float fuseHeading3(float gpsDeg, float mpuDeg, float qmcDeg, float wGps, float wMpu, float wQmc) 
{
	float sum = wGps + wMpu + wQmc;
	if (sum <= 0.0f) return gpsDeg;
	wGps /= sum;
	wMpu /= sum;
	wQmc /= sum;

	float gpsRad = radians(gpsDeg);
	float mpuRad = radians(mpuDeg);
	float qmcRad = radians(qmcDeg);

	float x =
		wGps * cos(gpsRad) +
		wMpu * cos(mpuRad) +
		wQmc * cos(qmcRad);

	float y =
		wGps * sin(gpsRad) +
		wMpu * sin(mpuRad) +
		wQmc * sin(qmcRad);

	float fusedRad = atan2(y, x);
	float fusedDeg = degrees(fusedRad);

	if (fusedDeg < 0) fusedDeg += 360.0f;

	return fusedDeg;
}
