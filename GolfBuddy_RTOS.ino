// GolfBuddy_RTOS.ino
// Autor: Niklas Krakhofer
// Project: Golf Buddy

#include "GolfBuddy_RTOS_Declaration.h"
#include "GolfBuddy_RTOS_Functions.h"
#include "GolfBuddy_RTOS_Constants.h"
#include <Arduino.h>
#include <TinyGPSPlus.h>
#include "MPU9250.h"
#include <HardwareSerial.h>
#include <cstdint>
#include <cstring>
#include <Wire.h> 
#include <math.h>
#include <freertos/semphr.h>
#include <qmc5883p.h>

//////////////////////////////////////////////////////////////////////////////
// Tasks
//////////////////////////////////////////////////////////////////////////////

TaskHandle_t xHandleReceiveDataFromRaspberryPI;
TaskHandle_t xHandleTransmittDataToRaspberryPI;
TaskHandle_t xHandleReadGPSData;
TaskHandle_t xHandleReadTemperature;
TaskHandle_t xHandleMeasureMotorSpeed;
TaskHandle_t xHandlePlayerTracking;
TaskHandle_t xHandleMotorSupport;
TaskHandle_t xHandleCheckSurrounding;
TaskHandle_t xHandleReceiveDataFromTracker;
TaskHandle_t xHandleBrakeTaskHandle;
TaskHandle_t xHandleMeasureHeading;
TaskHandle_t xHandleMeasureAkkuVoltage;


// @brief: The tasks reads all revieced Data from the Raspberry PI and filters out RTCM-Data and Control Data.
//		   RTCM-Data is forwarded to the GPS-Modul. Control Data is processed.
// @param: -
// @return: -
void ReceiveDataFromRaspPI(void* pvParameters)
{
	enum class RxState
	{
		WAIT_SYNC,
		READ_RTCM_HEADER,
		READ_RTCM_PAYLOAD,
		READ_RTCM_CRC,
		WAIT_CONTROL_BYTE,
		READ_CONTROL
	};

	RxState state = RxState::WAIT_SYNC;

	uint8_t rtcm_header[3];
	uint16_t rtcm_payload_len = 0;
	uint16_t rtcm_total_len = 0;
	uint8_t rtcm_frame[4096];
	uint16_t rtcm_index = 0;

	uint8_t ctrl_byte = 0;
	bool ctrl_wait_stop = false;

	uint32_t lastByteTime = 0;

	while (1)
	{

		// Reconects to proctocoll if reading failed after timout.
		if (state != RxState::WAIT_SYNC && (millis() - lastByteTime) > FRAME_TIMEOUT_MS)
		{
			state = RxState::WAIT_SYNC; // Return to Wait and wait for starcondition 
			rtcm_index = 0; // Reset receiving array
			ctrl_wait_stop = false;
		}

		// The following proceeds on byte at a time. Depedning on startbyte, the follwing bytes are proceeded. 
		while (piSerial.available() > 0)
		{
			uint8_t byte1 = piSerial.read();
			lastByteTime = millis();

			switch (state)
			{
			case RxState::WAIT_SYNC: // Waits for a startcondition and switches to resulting state.
				if (byte1 == 0xD3)  // Startbyte for rtcm
				{
					rtcm_header[0] = byte1; // 
					state = RxState::READ_RTCM_HEADER;
					rtcm_index = 1;
				}
				else if (byte1 == 0xAA) // Startbyte for controll
				{
					state = RxState::READ_CONTROL;
					ctrl_wait_stop = false;
				}
				break;

			case RxState::READ_RTCM_HEADER: // Reads the RTCM header
				rtcm_header[rtcm_index++] = byte1;
				if (rtcm_index == 3)
				{
					rtcm_payload_len = ((uint16_t)rtcm_header[1] << 8 | rtcm_header[2]) & 0x03FF; // Calculate payload length
					rtcm_total_len = 3 + rtcm_payload_len + 3;
					if (rtcm_total_len > sizeof(rtcm_frame))
					{
						state = RxState::WAIT_SYNC;
						break;
					}
					memcpy(rtcm_frame, rtcm_header, 3);
					rtcm_index = 3;
					state = RxState::READ_RTCM_PAYLOAD;
				}
				break;

			case RxState::READ_RTCM_PAYLOAD: // Reads the RTCM Payload
				rtcm_frame[rtcm_index++] = byte1;

				if (rtcm_index >= sizeof(rtcm_frame))
				{
					state = RxState::WAIT_SYNC;
					rtcm_index = 0;
					break;
				}

				if (rtcm_index == 3 + rtcm_payload_len)
				{
					state = RxState::READ_RTCM_CRC;
				}
				break;

			case RxState::READ_RTCM_CRC: // Reads the CRC
				rtcm_frame[rtcm_index++] = byte1;
				if (rtcm_index == rtcm_total_len)
				{
					uint32_t calc_crc = crc24q(rtcm_frame, 3 + rtcm_payload_len);
					uint32_t recv_crc = (rtcm_frame[3 + rtcm_payload_len] << 16) | (rtcm_frame[3 + rtcm_payload_len + 1] << 8) | rtcm_frame[3 + rtcm_payload_len + 2];

					//// Debug
					// uint16_t msg_id = ((uint16_t)rtcm_frame[3] << 4) | ((rtcm_frame[4] & 0xF0) >> 4);
					// Serial.printf("[RTCM] ID:%u Len:%u CRC:%s\n", msg_id, rtcm_total_len, (calc_crc == recv_crc) ? "OK" : "FEHLER");

					if (calc_crc == recv_crc) // Only forward if calculated and recieved CRC are maching.
					{
						gpsSerial.write(rtcm_frame, rtcm_total_len);

						// Read the controll data, which are always coming after rtcm frame.
						if (piSerial.available() >= 3)
						{
							uint8_t ctrl_sync = piSerial.read();
							uint8_t Trolley_Mode = piSerial.read();
							uint8_t stopbyte = piSerial.read();

							if (ctrl_sync == 0xAA && stopbyte == 0xFF)
							{
								if (Trolley_Mode == 0)
								{
									bIsPlayerTrackingActivated = false;
									bIsMotorSupportActivated = false;
								}
								else if (Trolley_Mode == 1)
								{
									bIsPlayerTrackingActivated = true;
									bIsMotorSupportActivated = false;
								}
								else if (Trolley_Mode == 2)
								{
									bIsPlayerTrackingActivated = false;
									bIsMotorSupportActivated = true;
								}
								//Serial.println(bIsPlayerTrackingActivated);
								//Serial.println(bIsMotorSupportActivated);
							}
							else
							{
								Serial.println("[CTRL] Error reading control Byte!");
							}
						}
					}
					state = RxState::WAIT_SYNC;
				}
				break;

			case RxState::READ_CONTROL: // If somehow no rtcm frames are recieved, the controldata is still transmitted. This state proceeds them.
				if (!ctrl_wait_stop)
				{
					ctrl_byte = byte1;
					ctrl_wait_stop = true;
				}
				else
				{
					if (byte1 == 0xFF)
					{
						if (ctrl_byte == 0)
						{
							bIsPlayerTrackingActivated = false;
							bIsMotorSupportActivated = false;
						}
						else if (ctrl_byte == 1)
						{
							bIsPlayerTrackingActivated = true;
							bIsMotorSupportActivated = false;
						}
						else if (ctrl_byte == 2)
						{
							bIsPlayerTrackingActivated = false;
							bIsMotorSupportActivated = true;
						}
					}
					else
					{
						Serial.println("[CTRL] STOPBYTE Error!");
					}
					state = RxState::WAIT_SYNC;
				}
				break;

			}
		}
		vTaskDelay(1);
	}
}


// @brief: The tasks transmitt a controllstring every second to the Raspberry PI. Including:
//				iBatteryLevel: Batterylevel
//				dLatitudeGolfBuddy: Latitude from GolfBuddy
//				dLongitudeGolfBuddy: Longitude from GolfBuddy
//				fFacingDirection: Facing direction from GolfBuddy
//				trackerTrackingFlag: Changes its condition if tracking start/stop button is pressed on transmitter. 
// @param: -
// @return: -
void TransmittDataToRaspPI(void* pvParameters)
{
	while (1)
	{
		String out =
			String(RaspPI_transmitData.iBatteryLevel) + ";" +
			String(RaspPI_transmitData.dLatitudeGolfBuddy, 6) + ";" +
			String(RaspPI_transmitData.dLongitudeGolfBuddy, 6) + ";" +
			String(RaspPI_transmitData.fFacingDirection) + ";" +
			String(trackerTrackingFlag) + ";" +
			String(RaspPI_transmitData.latitudeGolfPlayer, 6) + ";" +
			String(RaspPI_transmitData.longitudeGolfPlayer, 6) + ";" +
			String(RaspPI_transmitData.swingSpeed) +
			"\n";

		piSerial.write(out.c_str());

		vTaskDelay(1000 / portTICK_PERIOD_MS);
	}
}

// @brief: Read the NMEA-String from GPS-Modul and encodes it to get latitude and longitude from GolfBuddy.
// @param: -
// @return: -
void ReadGPSData(void* pvParameters)
{
	unsigned long lastUpdate = millis();
	while (1)
	{
		if (gps.location.isUpdated() && (millis() - lastUpdate > 100))
		{
			lastUpdate = millis();
			trolleyCoords.dGolfTrolley_latitude = gps.location.lat();
			trolleyCoords.dGolfTrolley_longitude = gps.location.lng();

			Serial.println(trolleyCoords.dGolfTrolley_latitude, 6);
			Serial.println(trolleyCoords.dGolfTrolley_longitude, 6);

			RaspPI_transmitData.dLatitudeGolfBuddy = trolleyCoords.dGolfTrolley_latitude;
			RaspPI_transmitData.dLongitudeGolfBuddy = trolleyCoords.dGolfTrolley_longitude;
		}
		while (gpsSerial.available() > 0)
		{
			gps.encode(gpsSerial.read());

			/*char c = gpsSerial.read();
			Serial.print(c);*/
		}
		vTaskDelay(1 / portTICK_PERIOD_MS);
	}
}

// @brief: Read the temperature from temperature sensor. 
// @param: -
// @return: -
void ReadTemperature(void* pvParameter)
{
	while (1)
	{
		int32_t i32RawTemp = i32ReadRawTemperatureBME280();
		float fTemperature = fCompensateTemperatureBME280(i32RawTemp);
		vTaskDelay(10000 / portTICK_PERIOD_MS);
	}
}

// @brief: Measures the motorspeed of each motor and calculates the following values for each motor: RPM, Speed in [m/s] and Speed in [km/h]
// @param: -
// @return: -
void MeassureMotorSpeed(void* pvParameter)
{
	unsigned long ulPulsesMotorLeft = 0;
	unsigned long ulPulsesMotorRight = 0;
	unsigned long sulLastPulseCountMotorLeft = 0;
	unsigned long sulLastPulseCountMotorRight = 0;
	unsigned long lastUpdate = 0;

	while (1)
	{
		unsigned long now = millis();
		float deltaTime = (now - lastUpdate) / 1000.0;
		if (deltaTime <= 0)
		{
			return;
		}

		// Pulse difference since last measurement
		unsigned long ulPulsesMotorLeft = vulPulseCountMotorLeft - sulLastPulseCountMotorLeft;
		unsigned long ulPulsesMotorRight = vulPulseCountMotorRight - sulLastPulseCountMotorRight;

		sulLastPulseCountMotorLeft = vulPulseCountMotorLeft;
		sulLastPulseCountMotorRight = vulPulseCountMotorRight;

		// Rotation per second
		float turnsPerSecondLeft = (float)ulPulsesMotorLeft / pulsesPerTurn / deltaTime;
		float turnsPerSecondRight = (float)ulPulsesMotorRight / pulsesPerTurn / deltaTime;

		// Rotation per minute
		fMotorLeftRPM = turnsPerSecondLeft * 60.0;
		fMotorRightRPM = turnsPerSecondRight * 60.0;

		// Speed in [m/s]
		fMotorLeftSpeed_ms = turnsPerSecondLeft * wheelDiameter * M_PI;
		fMotorRightSpeed_ms = turnsPerSecondRight * wheelDiameter * M_PI;

		// Speed in [km/h]
		fMotorLeftSpeed_kmh = fMotorLeftSpeed_ms * 3.6;
		fMotorRightSpeed_kmh = fMotorRightSpeed_ms * 3.6;

		lastUpdate = now;

		vTaskDelay(100 / portTICK_PERIOD_MS);
	}
}

// @brief: If playertracking is activated, this task calculates the driving route and controlls the motors to follow the golfplayer.
// @param: -
// @return: -
void PlayerTracking(void* pvParameter)
{
	GPSCoordinates targetCoords;

	// Block task until GPS-Koordinates are available.
	while (trolleyCoords.dGolfTrolley_latitude == 0.0 || trolleyCoords.dGolfTrolley_longitude == 0.0)
	{
		vTaskDelay(500 / portTICK_PERIOD_MS);
	}

	while (true)
	{
		if (bIsPlayerTrackingActivated && !bIsBreakingActive)
		{
			if (targetCoordsBuffer.empty()) // If no target coordinate is in buffer, no need to follow.
			{
				vParking();
			}
			else
			{
				vPlayerTracking(targetCoordsBuffer[0], trolleyCoords);
			}
		}
		vTaskDelay(10 / portTICK_PERIOD_MS);
	}
}

// @brief: This task accelerates the motors, if motorsupport is active and the button is pressed.
// @param: -
// @return: -
void MotorSupport(void* pvParameter)
{
	int motorSupportPWM = 0;
	const int motorSupportTargetPWM = 50;

	unsigned long lastMotorRampTime = 0;
	const unsigned long motorRampInterval = 20;

	while (1)
	{
		if (bIsMotorSupportActivated)
		{
			vSetDrivingdirectionMotorLeft(DrivingDirectionBackwards);
			vSetDrivingdirectionMotorRight(DrivingDirectionBackwards);

			if (digitalRead(TouchSensorLeft) || digitalRead(TouchSensorRight))
			{
				//// Accelartes the motors to a certain pwm.
				//unsigned long now = millis();
				//if (motorSupportPWM < motorSupportTargetPWM && now - lastMotorRampTime >= motorRampInterval)
				//{

				//	lastMotorRampTime = now;
				//	motorSupportPWM++;
				//}

				//analogWrite(MotorLeftPWMPin, motorSupportPWM);
				//analogWrite(MotorRightPWMPin, motorSupportPWM);
				vRegulateMotorLeftRPM(200);
				vRegulateMotorRightRPM(218);
			}
			else
			{
				motorSupportPWM = 0; // reset accelaration ramp
				vParking();
			}
		}

		if (!bIsPlayerTrackingActivated && !bIsMotorSupportActivated)
		{
			vParking();
		}

		vTaskDelay(1 / portTICK_PERIOD_MS);
	}
}

//@brief: This task reads out the measured distances of each distance sensor and breaks the motors if one of the distances is closer than 150 cm. 
//@param: -
//@return: -
void CheckSurrounding(void* pvParameter)
{
	while (1)
	{
		//if (!bIsMotorSupportActivated)
		//{
		//	static unsigned int sensorIndex = 0;
		//	sensorIndex = (sensorIndex + 1) % 2;
		//	
		//	float fDistance = fGetMessuredDistanceofHCSR04(cSensorIDArray[sensorIndex]);
		//	Serial.println(fDistance);

		//	if (fDistance < 200)
		//	{
		//		bIsBreakingActive = true;
		//	}
		//}
		vTaskDelay(10 / portTICK_PERIOD_MS);
	}
}

//@brief: This task recieves the data from the tracker from the radio module, including position and trackingbutton condition.
//@param: -
//@return: -
void ReceiveDataFromTracker(void* pvParameter)
{
	unsigned long lastUpdate = 0;
	const unsigned long interval = 5000;

	String sCurrentIncomeTrackerDataField = "";
	String sIncomeTrackerDataFields[10];
	int iIncomeTrackerFieldIndex = 0;

	while (1)
	{
		// Reads the received data from radio module and splits the string into data fields seperated with ";".
		while (funkSerial.available())
		{
			char c = funkSerial.read();
			//Serial.println(c);
			if (c == ';')
			{
				sIncomeTrackerDataFields[iIncomeTrackerFieldIndex] = sCurrentIncomeTrackerDataField;
				iIncomeTrackerFieldIndex++;
				sCurrentIncomeTrackerDataField = "";
			}
			else if (c == '\n')
			{
				sIncomeTrackerDataFields[iIncomeTrackerFieldIndex] = sCurrentIncomeTrackerDataField;

				double latitude = sIncomeTrackerDataFields[0].toDouble();
				double longitude = sIncomeTrackerDataFields[1].toDouble();

				RaspPI_transmitData.latitudeGolfPlayer = latitude;
				RaspPI_transmitData.longitudeGolfPlayer = longitude;

				if (sIncomeTrackerDataFields[2] == "1")
				{
					trackerTrackingFlag = true;
				}
				else
				{
					trackerTrackingFlag = false;
				}

				RaspPI_transmitData.swingSpeed = sIncomeTrackerDataFields[3].toFloat();

				if (millis() - lastUpdate >= interval)
				{
					lastUpdate = millis();

					if (latitude != 0.0 && longitude != 0.0 && bIsPlayerTrackingActivated)
					{
						targetCoordsBuffer.push_back({ latitude, longitude });
					}

				}

				sCurrentIncomeTrackerDataField = "";
				iIncomeTrackerFieldIndex = 0;
			}
			else
			{
				sCurrentIncomeTrackerDataField += c;
			}
		}
		vTaskDelay(1 / portTICK_PERIOD_MS);
	}
}

//@brief: This task breaks both motors for the duration of brakeDuration if bIsBreakingActive is active.
//@param: -
//@return: -
void BreakMotors(void* parameter)
{
	static bool brakeInProgress = false;
	unsigned long startTime = millis();

	while (1)
	{
		if (bIsBreakingActive)
		{
			if (!brakeInProgress)
			{
				startTime = millis();
				brakeInProgress = true;
			}

			analogWrite(MotorLeftPWMPin, 0);
			analogWrite(MotorRightPWMPin, 0);
			digitalWrite(MotorLeftBreakPin, LOW);
			digitalWrite(MotorRightBreakPin, LOW);

			// Stops the breaking after brakeDuration
			if (millis() - startTime > brakeDuration)
			{
				bIsBreakingActive = false;
				brakeInProgress = false;
			}
		}
		else
		{
			digitalWrite(MotorLeftBreakPin, HIGH);
			digitalWrite(MotorRightBreakPin, HIGH);
		}
		vTaskDelay(1 / portTICK_PERIOD_MS);
	}
}

//@brief: This task gets the heading of the golftrolley from the compass module
//@param: -
//@return: -
void MeasureHeading(void* parameter)
{
	//const float SCALE_AVG = 0.196f;
	//const float SCALE_X = 0.189f;
	//const float SCALE_Y = 0.204f;

	//float minX = 1e6, maxX = -1e6;
	//float minY = 1e6, maxY = -1e6;
	//unsigned long t0;
	//t0 = millis();

	GPSCoordinates from;
	GPSCoordinates to;
	bool gpsMeasureFlag = false;

	float mpuHeading = 0;
	float qmcHeading = 0;
	float gpsHeading = 0;

	while (1)
	{
		if (mpu.update()) {
			static uint32_t prev_ms = millis();
			if (millis() > prev_ms + 25) {
				mpuHeading = mpu.getYaw();
				mpu.getPitch();
				mpu.getRoll();

				if (mpuHeading < 0)
				{
					mpuHeading += 360;
				}
				mpuHeading += 101;
				if (mpuHeading >= 360.0)
				{
					mpuHeading -= 360.0;
				}
				RaspPI_transmitData.fFacingDirection = mpuHeading;
				//Serial.println(RaspPI_transmitData.fFacingDirection);
				prev_ms = millis();
			}
		}

		//float xyz[3];
		//if (mag.readXYZ(xyz)) {
		//	// Apply soft-iron correction
		//	xyz[0] *= SCALE_AVG / SCALE_X;
		//	xyz[1] *= SCALE_AVG / SCALE_Y;
		//}
		//qmcHeading = mag.getHeadingDeg(5.2833); // Adjust declination
		//RaspPI_transmitData.fFacingDirection = qmcHeading;
		////Serial.println(RaspPI_transmitData.fFacingDirection);

		///* Stop after 30 seconds */
		//if (millis() - t0 > 30000) {
		//	float offX = (maxX + minX) / 2.0f;
		//	float offY = (maxY + minY) / 2.0f;
		//	float scaleX = (maxX - minX) / 2.0f;
		//	float scaleY = (maxY - minY) / 2.0f;
		//	float avg = (scaleX + scaleY) / 2.0f;

		//	Serial.println("\n=== CALIBRATION RESULTS ===");
		//	Serial.printf("Offset X = %.3f µT\n", offX);
		//	Serial.printf("Offset Y = %.3f µT\n", offY);
		//	Serial.printf("Scale  X = %.3f µT\n", scaleX);
		//	Serial.printf("Scale  Y = %.3f µT\n", scaleY);
		//	Serial.printf("Average  = %.3f µT\n", avg);

		//	Serial.println("\nCopy these lines into your main sketch:");
		//	Serial.println("mag.setHardIronOffsets(" + String(offX, 3) +
		//		"f, " + String(offY, 3) + "f);");
		//	Serial.println("// Soft-Iron:");
		//	Serial.println("const float SCALE_AVG = " + String(avg, 3) + "f;");
		//	Serial.println("const float SCALE_X   = " + String(scaleX, 3) + "f;");
		//	Serial.println("const float SCALE_Y   = " + String(scaleY, 3) + "f;");
		//}

		//if (!gpsMeasureFlag)
		//{
		//	gpsMeasureFlag = true;
		//	from.dGolfTrolley_latitude = trolleyCoords.dGolfTrolley_latitude;
		//	from.dGolfTrolley_longitude = trolleyCoords.dGolfTrolley_longitude;
		//}
		//if (dCalculateHaversine(from.dGolfTrolley_latitude, from.dGolfTrolley_longitude, trolleyCoords.dGolfTrolley_latitude, trolleyCoords.dGolfTrolley_longitude) > 2)
		//{
		//	gpsHeading = dGetTargetHeading({ from.dGolfTrolley_latitude, from.dGolfTrolley_longitude },{ trolleyCoords.dGolfTrolley_latitude, trolleyCoords.dGolfTrolley_longitude });
		//	gpsMeasureFlag = false;
		//}

		//float fusedHeading = fuseHeading3(gpsHeading, mpuHeading, qmcHeading, 0.2, 0.4, 0.4);
		//RaspPI_transmitData.fFacingDirection = fusedHeading;
		//Serial.println(fusedHeading);

		vTaskDelay(1 / portTICK_PERIOD_MS);
	}
}

//@brief: This task measures the akku voltage from the golftrolley with adc and maps it to a voltage between 33 V and 42 V. 
//@param: -
//@return: -
void MeasureAkkuVoltage(void* parameter)
{
	while (1)
	{
		RaspPI_transmitData.iBatteryLevel = mapFloat(analogRead(AkkuVoltageMeasurePin), 2703, 3660, 33, 42);
		if (RaspPI_transmitData.iBatteryLevel < 33)
		{
			RaspPI_transmitData.iBatteryLevel = 33;
		}
		vTaskDelay(1000 / portTICK_PERIOD_MS);
	}
}

//////////////////////////////////////////////////////////////////////////////
// INIT Functions
//////////////////////////////////////////////////////////////////////////////

//@brief: Initializes PINs, serial ports and I2C
//@param: -
//@return: -
void init()
{
	Serial.begin(115200);
	piSerial.setRxBufferSize(2048);
	piSerial.begin(115200, SERIAL_8N1, RaspberryPIRXPin, RaspberryPITXPin);
	Wire.begin(SDA, SCL);

	pinMode(MotorLeftPWMPin, OUTPUT);
	pinMode(MotorRightPWMPin, OUTPUT);

	pinMode(MotorLeftBreakPin, OUTPUT);
	pinMode(MotorRightBreakPin, OUTPUT);

	analogWriteFrequency(MotorLeftPWMPin, 20000);
	analogWriteFrequency(MotorRightPWMPin, 20000);

	pinMode(MotorLeftSpeedPin, INPUT);
	pinMode(MotorRightSpeedPin, INPUT);

	attachInterrupt(digitalPinToInterrupt(MotorLeftSpeedPin), vCountPulseMotorLeft, RISING);
	attachInterrupt(digitalPinToInterrupt(MotorRightSpeedPin), vCountPulseMotorRight, RISING);

	pinMode(MotorLeftDrivingDirectionPin, OUTPUT);
	pinMode(MotorRightDrivingDirectionPin, OUTPUT);

	pinMode(TouchSensorLeft, INPUT);
	pinMode(TouchSensorRight, INPUT);

	pinMode(AkkuVoltageMeasurePin, INPUT);
}

//@brief: Initializes temperature sensor.
//@param: -
//@return: -
void initBME280()
{
	vResetBME280();
	delay(10);
	uint8_t chipID = ui8ReadRegister(REG_CHIPID);
	if (chipID != 0x60)
	{
		Serial.print("Kein BME280 gefunden! Chip-ID: 0x");
		Serial.println(chipID, HEX);
		return;
	}

	vReadCalibrationDataBME280();

	vWriteRegister(BME280_ADDR, REG_CTRL_MEAS, 0x27);
}

//@brief: Initializes GPS-Module
//@param: -
//@return: -
void initGPSModule()
{
	gpsSerial.begin(19200, SERIAL_8N1, GPSRXPin, GPSTXPin);
}

//@brief: Initializes radio module and sets the radio chanell to 437.0 MHz.
//@param: -
//@return: -
void initHC12()
{
	funkSerial.begin(9600);

	pinMode(HC12SetPin, OUTPUT);
	digitalWrite(HC12SetPin, LOW);
	delay(2000);

	Serial.println("Sende AT+BAUD4 (9600 Baud)");
	funkSerial.println("AT+BAUD4");
	delay(500);

	while (funkSerial.available())
	{
		Serial.write(funkSerial.read());
	}

	Serial.println("Setze Kanal 10 (437.0 MHz)...");
	funkSerial.print("AT+C003\r\n");
	delay(500);

	while (funkSerial.available())
	{
		Serial.write(funkSerial.read());
	}

	Serial.println("Sende AT...");
	funkSerial.println("AT");
	delay(2000);

	while (funkSerial.available())
	{
		Serial.write(funkSerial.read());
	}

	digitalWrite(HC12SetPin, HIGH);
}

//@brief: Initializes all three distance sensors.
//@param: -
//@return: -
void initHCSR04()
{
	pinMode(HCSR04TrigPin0, OUTPUT);
	pinMode(HCSR04EchoPin0, INPUT);
	pinMode(HCSR04TrigPin1, OUTPUT);
	pinMode(HCSR04EchoPin1, INPUT);
	pinMode(HCSR04TrigPin2, OUTPUT);
	pinMode(HCSR04EchoPin2, INPUT);
}

//@brief: Initializes kompass sensor and sets calibration data.
//@param: -
//@return: -
void initMPU9250()
{
	if (!mpu.setup(0x68)) {  // change to your own address
		while (1) {
			Serial.println("MPU connection failed. Please check your connection with `connection_check` example.");
			delay(5000);
		}
	}

	//mpu.ahrs(true);

	//mpu.selectFilter(QuatFilterSel::MADGWICK);
	//mpu.setFilterIterations(15);

	//mpu.setMagneticDeclination(5.2833);

	/*Serial.println("Accel Gyro calibration will start in 5sec.");
	Serial.println("Please leave the device still on the flat plane.");
	mpu.verbose(true);
	delay(5000);
	mpu.calibrateAccelGyro();

	Serial.println("Mag calibration will start in 5sec.");
	Serial.println("Please Wave device in a figure eight until done.");
	delay(5000);
	mpu.calibrateMag();

	print_MPU9250_calibration();
	mpu.verbose(false);*/

	mpu.setAccBias(142.83, -46.81, 24.25);
	mpu.setGyroBias(-8.73, 2.52, -0.13);
	mpu.setMagBias(573.59, 331.93, 8.67);
	mpu.setMagScale(0.88, 1.16, 1.00);
}

//@brief: Initializes kompass sensor backup
//@param: -
//@return: -
void initQMC5883P()
{
	if (!mag.begin())
	{
		Serial.println("Initialization of QMC5883P failed!");
		while (true);
	}

	//mag.setHardIronOffsets(0.004f, -0.310f);
}

//////////////////////////////////////////////////////////////////////////////
// Setup
//////////////////////////////////////////////////////////////////////////////
void setup()
{
	init();
	initBME280();
	initGPSModule();
	initHC12();
	initHCSR04();
	initMPU9250();
	//initQMC5883P();

	//targetCoordsBuffer.push_back({ 48.191787768768535, 16.397051539658563 }); //Kalibrierkoordinate

	///*targetCoordsBuffer.push_back({ 48.19167695312045, 16.397048079899708 });
	//targetCoordsBuffer.push_back({ 48.19162106865085, 16.397131909809133 });
	//targetCoordsBuffer.push_back({ 48.19173417875418, 16.39730091090654 });
	//targetCoordsBuffer.push_back({ 48.191790510174954, 16.39721976355422 });*/

	//targetCoordsBuffer.push_back({ 48.19176949766811, 16.39721775163639 });
	//targetCoordsBuffer.push_back({ 48.19173686120418, 16.397243235928855 });
	//targetCoordsBuffer.push_back({ 48.1917042247195, 16.397225799307694 });
	//targetCoordsBuffer.push_back({ 48.19170556594531, 16.397174160083487 });
	//targetCoordsBuffer.push_back({ 48.19167918849799, 16.397099719123915 });
	//targetCoordsBuffer.push_back({ 48.19162196280286, 16.397129227252034 });
	//targetCoordsBuffer.push_back({ 48.1918714305996, 16.39717348944421 });

	xTaskCreatePinnedToCore(
		ReceiveDataFromRaspPI,        // Funktion
		"ReceiveSerialDataFromRaspberryPI",// Name
		10000,                              // Stack
		NULL,                               // Parameter
		2,                                  // Priorität
		&xHandleReceiveDataFromRaspberryPI,// TaskHandle
		1                                   // Core
	);

	xTaskCreatePinnedToCore(
		TransmittDataToRaspPI,
		"TransmittSerialDataToRaspberryPI",
		10000,
		NULL,
		2,
		&xHandleTransmittDataToRaspberryPI,
		1
	);

	xTaskCreatePinnedToCore(
		ReadGPSData,
		"ReadGPSCoordinatesFromGPSModule",
		10000,
		NULL,
		2,
		&xHandleReadGPSData,
		1
	);

	xTaskCreatePinnedToCore(
		ReadTemperature,
		"ReadTemperatureFromBME",
		10000,
		NULL,
		2,
		&xHandleReadTemperature,
		1
	);

	xTaskCreatePinnedToCore(
		MeassureMotorSpeed,
		"MeassureMotorSpeed",
		10000,
		NULL,
		2,
		&xHandleMeasureMotorSpeed,
		1
	);

	xTaskCreatePinnedToCore(
		PlayerTracking,
		"Follow Player if enabled",
		10000,
		NULL,
		2,
		&xHandlePlayerTracking,
		1
	);

	xTaskCreatePinnedToCore(
		MotorSupport,
		"Handle Motor Support if enabled",
		10000,
		NULL,
		2,
		&xHandleMotorSupport,
		1
	);

	xTaskCreatePinnedToCore(
		CheckSurrounding,
		"Check if an Obsticle is blocking the way",
		10000,
		NULL,
		2,
		&xHandleCheckSurrounding,
		1
	);

	xTaskCreatePinnedToCore(
		ReceiveDataFromTracker,
		"Receive Data from Tracker",
		10000,
		NULL,
		2,
		&xHandleReceiveDataFromTracker,
		1
	);

	xTaskCreatePinnedToCore(
		BreakMotors,
		"Emergency Braking",
		10000,
		NULL,
		2,
		&xHandleBrakeTaskHandle,
		1
	);

	xTaskCreatePinnedToCore(
		MeasureHeading,
		"Measure Heading",
		10000,
		NULL,
		2,
		&xHandleMeasureHeading,
		1
	);

	xTaskCreatePinnedToCore(
		MeasureAkkuVoltage,
		"Measure Akku Voltage",
		10000,
		NULL,
		2,
		&xHandleMeasureAkkuVoltage,
		1
	);
}

void loop()
{

}
