//GolfBuddy_RTOS.ino
//Autor: Niklas Krakhofer
//Project: Golf Buddy
//TODO: Auslesen und Berechnen des Heading muss überprüft werden im Bezug auf Verdrehung und Paramter &Übergabe und Globale Parameter
//      Eventuell wenn möglich Globale Variablen durch returns der Funktionen ersetzen
//      vTaskReceiveDataFromTracker ausprogrammieren
//      Ausweichen durchtesten

//#define ObsticaleDetectionWithDogeing

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

#ifdef ObsticaleDetectionWithDogeing
TaskHandle_t xHandleDriveArroundwithCheck;
TaskHandle_t xHandleDodge;
#endif // ObsticaleDetectionWithDogeing

//Task: Recieves serial Data from Uart including: {DriveMode;} DriveMode = 0 -> no DriveMode active
//                                                             DriveMode = 1 -> Tracking activated 
//                                                             DriveMode = 2 -> MotorSupport activated
//      Decodes Received Massage and sets Variables.
//Param: -
enum class RxState {
	WAIT_SYNC,
	READ_RTCM_HEADER,
	READ_RTCM_PAYLOAD,
	READ_RTCM_CRC,
	WAIT_CONTROL_BYTE,
	READ_CONTROL
};

void ReceiveDataFromRaspPI(void* pvParameters) {
	static RxState state = RxState::WAIT_SYNC;

	static uint8_t rtcm_header[3];
	static uint16_t rtcm_payload_len = 0;
	static uint16_t rtcm_total_len = 0;
	static uint8_t rtcm_frame[4096];
	static uint16_t rtcm_index = 0;

	static uint8_t ctrl_byte = 0;
	static bool ctrl_wait_stop = false;

	static uint32_t lastByteTime = 0;
#define FRAME_TIMEOUT_MS 50

	while (1) {

		if (state != RxState::WAIT_SYNC &&
			(millis() - lastByteTime) > FRAME_TIMEOUT_MS)
		{
			state = RxState::WAIT_SYNC;
			rtcm_index = 0;
			ctrl_wait_stop = false;
		}

		while (piSerial.available() > 0) {
			uint8_t byte1 = piSerial.read();
			lastByteTime = millis();

			switch (state) {
			case RxState::WAIT_SYNC:
				if (byte1 == 0xD3) { rtcm_header[0] = byte1; state = RxState::READ_RTCM_HEADER; rtcm_index = 1; }
				else if (byte1 == 0xAA) { state = RxState::READ_CONTROL; ctrl_wait_stop = false; }
				break;

			case RxState::READ_RTCM_HEADER:
				rtcm_header[rtcm_index++] = byte1;
				if (rtcm_index == 3) {
					rtcm_payload_len = ((uint16_t)rtcm_header[1] << 8 | rtcm_header[2]) & 0x03FF;
					rtcm_total_len = 3 + rtcm_payload_len + 3;
					if (rtcm_total_len > sizeof(rtcm_frame)) { state = RxState::WAIT_SYNC; break; }
					memcpy(rtcm_frame, rtcm_header, 3);
					rtcm_index = 3;
					state = RxState::READ_RTCM_PAYLOAD;
				}
				break;

			case RxState::READ_RTCM_PAYLOAD:
				rtcm_frame[rtcm_index++] = byte1;

				if (rtcm_index >= sizeof(rtcm_frame)) {
					state = RxState::WAIT_SYNC;
					rtcm_index = 0;
					break;
				}

				if (rtcm_index == 3 + rtcm_payload_len)
					state = RxState::READ_RTCM_CRC;
				break;

			case RxState::READ_RTCM_CRC:
				rtcm_frame[rtcm_index++] = byte1;
				if (rtcm_index == rtcm_total_len) {
					uint32_t calc_crc = crc24q(rtcm_frame, 3 + rtcm_payload_len);
					uint32_t recv_crc = (rtcm_frame[3 + rtcm_payload_len] << 16) |
						(rtcm_frame[3 + rtcm_payload_len + 1] << 8) |
						rtcm_frame[3 + rtcm_payload_len + 2];

					uint16_t msg_id = ((uint16_t)rtcm_frame[3] << 4) | ((rtcm_frame[4] & 0xF0) >> 4);

					// Debug nur Header + CRC
					//Serial.printf("[RTCM] ID:%u Len:%u CRC:%s\n", msg_id, rtcm_total_len,
					//	(calc_crc == recv_crc) ? "OK" : "FEHLER");

					if (calc_crc == recv_crc) {
						gpsSerial.write(rtcm_frame, rtcm_total_len);

						// --- DIREKT STEUERDATEN AUSLESEN ---
						if (piSerial.available() >= 3) { // Stelle sicher, dass alle 3 Bytes da sind
							uint8_t ctrl_sync = piSerial.read();
							uint8_t Trolley_Mode = piSerial.read();
							uint8_t stopbyte = piSerial.read();

							if (ctrl_sync == 0xAA && stopbyte == 0xFF) {
								//Serial.printf("[CTRL] Trolley_Mode (direkt nach RTCM): %u\n", Trolley_Mode);
								if (Trolley_Mode == 0) {
									bIsPlayerTrackingActivated = false;
									bIsMotorSupportActivated = false;
								}
								else if (Trolley_Mode == 1) {
									bIsPlayerTrackingActivated = true;
									bIsMotorSupportActivated = false;
								}
								else if (Trolley_Mode == 2) {
									bIsPlayerTrackingActivated = false;
									bIsMotorSupportActivated = true;
								}
								//Serial.println(bIsPlayerTrackingActivated);
								//Serial.println(bIsMotorSupportActivated);
							}
							else {
								//Serial.println("[CTRL] Fehler beim direkten Auslesen der Steuerdaten!");
							}
						}
					}

					state = RxState::WAIT_SYNC;
				}
				break;

			case RxState::READ_CONTROL:
				if (!ctrl_wait_stop) { ctrl_byte = byte1; ctrl_wait_stop = true; }
				else {
					if (byte1 == 0xFF) Serial.printf("[CTRL] Trolley_Mode: %u\n", ctrl_byte);
					else Serial.println("[CTRL] STOPBYTE FEHLER!");
					state = RxState::WAIT_SYNC;
				}
				break;

			}
		}
		vTaskDelay(1);
	}
}

//Task: Sends Serial Data to Raspberry Pi every 1s including: iBatteryLevel
//                                                            dLatitudeGolfBuddy
//                                                            dLongitudeGolfBuddy
//                                                            fFacingDirection
//Param: -
void TransmittDataToRaspPI(void* pvParameters) {
	while (1) {
		vSendTransmitdataToPi();
		vTaskDelay(1000 / portTICK_PERIOD_MS);
	}
}

//Task: Reads GPS Coordinates from GPS Modul every 1.5s
//Param: -
void ReadGPSData(void* pvParameters) {
	unsigned long lastUpdate = millis();
	while (1) {
		if (gps.location.isUpdated() && (millis() - lastUpdate > 100)) {
			lastUpdate = millis();
			trolleyCoords.dGolfTrolley_latitude = gps.location.lat();
			trolleyCoords.dGolfTrolley_longitude = gps.location.lng();

			//Serial.println(trolleyCoords.dGolfTrolley_latitude, 6);
			//Serial.println(trolleyCoords.dGolfTrolley_longitude, 6);

			RaspPI_transmitData.dLatitudeGolfBuddy = trolleyCoords.dGolfTrolley_latitude;
			RaspPI_transmitData.dLongitudeGolfBuddy = trolleyCoords.dGolfTrolley_longitude;
		}
		while (gpsSerial.available() > 0) {
			gps.encode(gpsSerial.read());

			/*char c = gpsSerial.read();
			Serial.print(c);*/
		}
		vTaskDelay(1 / portTICK_PERIOD_MS);
	}
}

//Task: Reads Temperature from BME280 every 1s
//Param: -
void ReadTemperature(void* pvParameter) {
	while (1) {
		int32_t i32RawTemp = i32ReadRawTemperatureBME280();
		float fTemperature = fCompensateTemperatureBME280(i32RawTemp);
		vTaskDelay(10000 / portTICK_PERIOD_MS);
	}
}

//Task: Measures MotorSpeed of both Motors and sets the following Variables with the Calculated Values:
//  fMotorLeftRPM
//  fMotorRightRPM
//  fMotorLeftSpeed_ms
//  fMotorRightSpeed_ms
//  fMotorLeftSpeed_kmh
//  fMotorRightSpeed_kmh
// 
//Param: -
void MeassureMotorSpeed(void* pvParameter) {
	while (1) {
		vUpdateMeausuredMotorSpeed();
		vTaskDelay(100 / portTICK_PERIOD_MS);
	}
}

//Task: If targetCoordinates are in Queue and Tracking is enabled, then the Trolley drives to the target Coordinate.
//      Else the Trolley stays in Parking and is not moving.
//Param: -
void PlayerTracking(void* pvParameter) {
	GPSCoordinates targetCoords;

	// Warten, bis gültige GPS-Daten vorhanden sind
	while (trolleyCoords.dGolfTrolley_latitude == 0.0 ||
		trolleyCoords.dGolfTrolley_longitude == 0.0) {
		vTaskDelay(500 / portTICK_PERIOD_MS);
	}
	while (true) {
		if (!bIsBreakingActive) {
			if (bIsPlayerTrackingActivated && !bDogingactive) {
				if (targetCoordsBuffer.empty())
				{
					vParking();
				}
				else {
					vPlayerTracking(targetCoordsBuffer[0], trolleyCoords);
				}
			}
		}
		vTaskDelay(10 / portTICK_PERIOD_MS);
	}
}

//Task: If bIsMotorSupportActivated is activated and Touchsensor/s is/are triggerd both Motors are regulated to BaseSpeed. 
//Param: -
void MotorSupport(void* pvParameter) {
	while (1) {
		if (bIsMotorSupportActivated && !bDogingactive) {
			vMotorSupport();
		}

		if (!bIsPlayerTrackingActivated && !bIsMotorSupportActivated) {
			vParking();
		}

		vTaskDelay(1 / portTICK_PERIOD_MS);
	}
}

//@brief: ifdef ObsticaleDetectionWithDogeing:  If an Objects is to close to the Trolley a Doging algorythm is triggered. 
//                                              If Doging is finished the previous Driving mode continious.
//        ifndef ObsticaleDetectionWithDogeing: The Trolley is breaking if an obsticle detected by distance sensors. 
//@param: -
//@return: -
void CheckSurrounding(void* pvParameter) {
	while (1) {
		if (!bIsMotorSupportActivated)
		{
			vCheckSurrounding();
		}
		vTaskDelay(100 / portTICK_PERIOD_MS);
	}
}

//Task: !
//Param: -
void ReceiveDataFromTracker(void* pvParameter) {

	static unsigned long lastUpdate = 0;
	const unsigned long interval = 5000;

	while (1) {
		while (funkSerial.available()) {
			char c = funkSerial.read();
			//Serial.println(c);
			if (c == ';') {
				sIncomeTrackerDataFields[iIncomeTrackerFieldIndex] = sCurrentIncomeTrackerDataField;
				iIncomeTrackerFieldIndex++;
				sCurrentIncomeTrackerDataField = "";
			}
			else if (c == '\n') {
				sIncomeTrackerDataFields[iIncomeTrackerFieldIndex] = sCurrentIncomeTrackerDataField;

				double latitude = sIncomeTrackerDataFields[0].toDouble();
				double longitude = sIncomeTrackerDataFields[1].toDouble();

				if (sIncomeTrackerDataFields[2] == "1") // Muss kontrolliert werden ob funktioniert wegen 1\n kann vielleicht auch mit 1;\n gefixt werden
				{
					trackerTrackingFlag = true;
				}
				else
				{
					trackerTrackingFlag = false;
				}

				if (millis() - lastUpdate >= interval) {
					lastUpdate = millis();

					if (latitude != 0.0 && longitude != 0.0 && bIsPlayerTrackingActivated) {
						targetCoordsBuffer.push_back({ latitude, longitude });
					}

				}

				sCurrentIncomeTrackerDataField = "";
				iIncomeTrackerFieldIndex = 0;
			}
			else {
				sCurrentIncomeTrackerDataField += c;
			}
		}
		vTaskDelay(1 / portTICK_PERIOD_MS);
	}
}

#ifdef ObsticaleDetectionWithDogeing
//Task: !
//Param: -
void vDriveAroundwithCheck(void* pvParameter) {
	float fTargetHeading;
	while (bvDriveAroundonRightwithCheck) {
		Serial.println("RightStarted");
		vMakeASetBack();

		vTaskDelay(5000 / portTICK_PERIOD_MS);

		vRegulateMotorLeftRPM(0);
		vRegulateMotorRightRPM(0);

		fTargetHeading = dGetHeading() - 90;
		while (dGetHeading() - fTargetHeading <= 5) {
			vSpinToHeading(fTargetHeading);
		}

		vMakeASetForward();

		vTaskDelay(5000 / portTICK_PERIOD_MS);

		vRegulateMotorLeftRPM(0);
		vRegulateMotorRightRPM(0);

		fTargetHeading = dGetHeading() + 90;
		while (dGetHeading() - fTargetHeading <= 5) {
			vSpinToHeading(fTargetHeading);
		}

		if (!vCheckSurrounding()) {
			bDogingactive = false;
			bvDriveAroundonRightwithCheck = false;
		}
		vTaskDelay(1 / portTICK_PERIOD_MS);
	}
	while (bvDriveAroundonLeftwithCheck) {
		Serial.println("LeftStarted");
		vMakeASetBack();

		vTaskDelay(5000 / portTICK_PERIOD_MS);

		vRegulateMotorLeftRPM(0);
		vRegulateMotorRightRPM(0);

		fTargetHeading = dGetHeading() + 90;
		while (dGetHeading() - fTargetHeading <= 5) {
			vSpinToHeading(fTargetHeading);
		}

		vMakeASetForward();

		vTaskDelay(5000 / portTICK_PERIOD_MS);

		vRegulateMotorLeftRPM(0);
		vRegulateMotorRightRPM(0);

		fTargetHeading = dGetHeading() - 90;
		while (dGetHeading() - fTargetHeading <= 5) {
			vSpinToHeading(fTargetHeading);
		}

		if (!vCheckSurrounding()) {
			bDogingactive = false;
			bvDriveAroundonRightwithCheck = false;
		}
		vTaskDelay(1 / portTICK_PERIOD_MS);
	}
	vTaskDelete(NULL);
}

//Task: !
//Param: -
void vDodgeObsticle(void* pvParameter) {
	while (bDogeRight) {
		vRegulateMotorLeftRPM(100);
		vRegulateMotorRightRPM(50);

		vTaskDelay(5000 / portTICK_PERIOD_MS);

		vRegulateMotorLeftRPM(100);
		vRegulateMotorRightRPM(100);

		vTaskDelay(5000 / portTICK_PERIOD_MS);

		bDogeRight = false;

		vTaskDelay(1 / portTICK_PERIOD_MS);
	}
	while (bDodgeLeft) {
		vRegulateMotorLeftRPM(50);
		vRegulateMotorRightRPM(100);

		vTaskDelay(5000 / portTICK_PERIOD_MS);

		vRegulateMotorLeftRPM(100);
		vRegulateMotorRightRPM(100);

		vTaskDelay(5000 / portTICK_PERIOD_MS);

		bDodgeLeft = false;

		vTaskDelay(1 / portTICK_PERIOD_MS);
	}
	vTaskDelete(NULL);
}
#endif

//Task: Breaks both Motors with a setable Intensity and Duration
//Param: -
void BreakMotors(void* parameter) {
	static bool brakeInProgress = false;
	unsigned long startTime = millis();

	while (1) {
		if (bIsBreakingActive) {
			if (!brakeInProgress) {
				startTime = millis();
				brakeInProgress = true;
			}

			analogWrite(MotorLeftPWMPin, 0);
			analogWrite(MotorRightPWMPin, 0);
			digitalWrite(MotorLeftBreakPin, LOW);
			digitalWrite(MotorRightBreakPin, LOW);

			if (millis() - startTime > brakeDuration && fGetMessuredDistanceofHCSR04(cSensorIDArray[0]) > 10 && fGetMessuredDistanceofHCSR04(cSensorIDArray[1]) > 10 && fGetMessuredDistanceofHCSR04(cSensorIDArray[2]) > 10) {
				bIsBreakingActive = false; // Nur wenn die distanz größer ist
				brakeInProgress = false;
			}
		}
		else {
			digitalWrite(MotorLeftBreakPin, HIGH);
			digitalWrite(MotorRightBreakPin, HIGH);
		}
		vTaskDelay(1 / portTICK_PERIOD_MS);
	}
}

void MeasureHeading(void* parameter) {
	/*const float SCALE_AVG = 0.411f;
	const float SCALE_X = 0.390f;
	const float SCALE_Y = 0.432f;*/
	while (1) {
		if (mpu.update()) {
			float heading = mpu.getYaw();
			if (heading < 0) heading += 360;
			heading += 15;
			if (heading >= 360.0) heading -= 360.0;
			RaspPI_transmitData.fFacingDirection = heading;
			//Serial.println(RaspPI_transmitData.fFacingDirection);
		}

		vTaskDelay(10 / portTICK_PERIOD_MS);



		//float xyz[3];
		//if (mag.readXYZ(xyz)) {
		//	// Apply soft-iron correction
		//	xyz[0] *= SCALE_AVG / SCALE_X;
		//	xyz[1] *= SCALE_AVG / SCALE_Y;

		//	float heading = mag.getHeadingDeg(5.2833); // Adjust declination
		//}
		//vTaskDelay(10 / portTICK_PERIOD_MS);
	}
}

void MeasureAkkuVoltage(void* parameter) {
	while (1) {
		RaspPI_transmitData.iBatteryLevel = mapFloat(analogRead(AkkuVoltageMeasurePin), 2703, 3660, 33, 42);
		if (RaspPI_transmitData.iBatteryLevel < 33) {
			RaspPI_transmitData.iBatteryLevel = 33;
		}
		vTaskDelay(1000 / portTICK_PERIOD_MS);
	}
}

//////////////////////////////////////////////////////////////////////////////
// INIT Functions
//////////////////////////////////////////////////////////////////////////////

//Initialise uart interfaces and pins
//Parameter: -
//return: -
void init() {
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

	pinMode(AkkuVoltageMeasurePin, INPUT); //Akkuvoltage measurepin

	vSetDrivingdirectionMotorLeft(DrivingDirectionForwards);
}

void initBME280() {
	vResetBME280();
	delay(10);
	uint8_t chipID = ui8ReadRegister(REG_CHIPID);
	if (chipID != 0x60) {
		Serial.print("Kein BME280 gefunden! Chip-ID: 0x");
		Serial.println(chipID, HEX);
		return;
	}

	vReadCalibrationDataBME280();

	vWriteRegister(BME280_ADDR, REG_CTRL_MEAS, 0x27);
}

void initGPSModule() {
	gpsSerial.begin(19200, SERIAL_8N1, GPSRXPin, GPSTXPin);
}

void initHC12() {
	funkSerial.begin(9600);

	pinMode(HC12SetPin, OUTPUT);
	digitalWrite(HC12SetPin, LOW);
	delay(2000);

	Serial.println("Sende AT+BAUD4 (9600 Baud)");
	funkSerial.println("AT+BAUD4");
	delay(500);

	while (funkSerial.available()) {
		Serial.write(funkSerial.read());
	}

	Serial.println("Setze Kanal 10 (437.0 MHz)...");
	funkSerial.print("AT+C003\r\n");
	delay(500);

	while (funkSerial.available()) {
		Serial.write(funkSerial.read());
	}

	Serial.println("Sende AT...");
	funkSerial.println("AT");
	delay(2000);

	while (funkSerial.available()) {
		Serial.write(funkSerial.read());
	}

	digitalWrite(HC12SetPin, HIGH);
}

void initHCSR04() {
	pinMode(HCSR04TrigPin0, OUTPUT);
	pinMode(HCSR04EchoPin0, INPUT);
	pinMode(HCSR04TrigPin1, OUTPUT);
	pinMode(HCSR04EchoPin1, INPUT);
	pinMode(HCSR04TrigPin2, OUTPUT);
	pinMode(HCSR04EchoPin2, INPUT);
}

void initMPU9250() {
	if (!mpu.setup(0x68)) {
		while (1) {
			Serial.println("MPU connection failed!");
			delay(5000);
		}
	}

	// AHRS aktivieren (Tilt Compensation)
	mpu.ahrs(true);

	// Filter auswählen und Iterationen erhöhen
	mpu.selectFilter(QuatFilterSel::MADGWICK);
	mpu.setFilterIterations(15);

	mpu.setMagneticDeclination(5.2833);

	// Kalibrierung der Sensoren
	//Serial.println("Accel Gyro calibration will start in 5sec.");
	//Serial.println("Please leave the device still on the flat plane.");
	//mpu.verbose(true);
	//delay(5000);
	//mpu.calibrateAccelGyro();

	//Serial.println("Mag calibration will start in 5sec.");
	//Serial.println("Please Wave device in a figure eight until done.");
	//delay(5000);
	//mpu.calibrateMag();

	//print_MPU9250_calibration();
	//mpu.verbose(false);

	mpu.setAccBias(81.02, -942.55, -652.85);
	mpu.setGyroBias(-6.64, 2.20, -0.03);
	mpu.setMagBias(575.39, 164.16, -220.18);
	mpu.setMagScale(0.74, 1.11, 1.32);
}

void initQMC5883P() {
	if (!mag.begin()) {
		Serial.println("Initialization of QMC5883P failed!");
		while (true);
	}

	mag.setHardIronOffsets(0.257f, -0.131f);
}

//////////////////////////////////////////////////////////////////////////////
// Setup
//////////////////////////////////////////////////////////////////////////////
void setup() {
	init();
	initBME280();
	initGPSModule();
	initHC12();
	initHCSR04();
	initMPU9250();
	//initQMC5883P();

	//targetCoordsBuffer.push_back({ 48.191787768768535, 16.397051539658563 }); //Kalibrierkoordinate

	/*targetCoordsBuffer.push_back({ 48.19167695312045, 16.397048079899708 });
	targetCoordsBuffer.push_back({ 48.19162106865085, 16.397131909809133 });
	targetCoordsBuffer.push_back({ 48.19173417875418, 16.39730091090654 });
	targetCoordsBuffer.push_back({ 48.191790510174954, 16.39721976355422 });*/

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
		TransmittDataToRaspPI,        // Funktion
		"TransmittSerialDataToRaspberryPI",// Name
		10000,                              // Stack
		NULL,                               // Parameter
		2,                                  // Priorität
		&xHandleTransmittDataToRaspberryPI,// TaskHandle
		1                                   // Core
	);

	xTaskCreatePinnedToCore(
		ReadGPSData,        // Funktion
		"ReadGPSCoordinatesFromGPSModule",// Name
		10000,                              // Stack
		NULL,                               // Parameter
		2,                                  // Priorität
		&xHandleReadGPSData,// TaskHandle
		1                                   // Core
	);

	xTaskCreatePinnedToCore(
		ReadTemperature,        // Funktion
		"ReadTemperatureFromBME",// Name
		10000,                              // Stack
		NULL,                               // Parameter
		2,                                  // Priorität
		&xHandleReadTemperature,// TaskHandle
		1                                   // Core
	);

	xTaskCreatePinnedToCore(
		MeassureMotorSpeed,        // Funktion
		"MeassureMotorSpeed",// Name
		10000,                              // Stack
		NULL,                               // Parameter
		2,                                  // Priorität
		&xHandleMeasureMotorSpeed,// TaskHandle
		1                                   // Core
	);

	xTaskCreatePinnedToCore(
		PlayerTracking,        // Funktion
		"Follow Player if enabled",// Name
		10000,                              // Stack
		NULL,                               // Parameter
		2,                                  // Priorität
		&xHandlePlayerTracking,// TaskHandle
		1                                   // Core
	);

	xTaskCreatePinnedToCore(
		MotorSupport,        // Funktion
		"Handle Motor Support if enabled",// Name
		10000,                              // Stack
		NULL,                               // Parameter
		2,                                  // Priorität
		&xHandleMotorSupport,// TaskHandle
		1                                   // Core
	);

	xTaskCreatePinnedToCore(
		CheckSurrounding,        // Funktion
		"Check if an Obsticle is blocking the way",// Name
		10000,                              // Stack
		NULL,                               // Parameter
		2,                                  // Priorität
		&xHandleCheckSurrounding,// TaskHandle
		1                                   // Core
	);

#ifdef ObsticaleDetectionWithDogeing
	xTaskCreatePinnedToCore(
		vDriveAroundwithCheck,        // Funktion
		"Drive Around an Object closer then 100cm",// Name
		10000,                              // Stack
		NULL,                               // Parameter
		2,                                  // Priorität
		&xHandleDriveArroundwithCheck,// TaskHandle
		1                                   // Core
	);

	xTaskCreatePinnedToCore(
		vDodgeObsticle,        // Funktion
		" an Obsticle without setback",// Name
		10000,                              // Stack
		NULL,                               // Parameter
		2,                                  // Priorität
		&xHandleDodge,// TaskHandle
		1                                   // Core
	);
#endif // ObsticaleDetectionWithDogeing

	xTaskCreatePinnedToCore(
		ReceiveDataFromTracker,        // Funktion
		"Receive Data from Tracker",// Name
		10000,                              // Stack
		NULL,                               // Parameter
		2,                                  // Priorität
		&xHandleReceiveDataFromTracker,// TaskHandle
		1                                   // Core
	);

	xTaskCreatePinnedToCore(
		BreakMotors,        // Funktion
		"Emergency Braking",// Name
		10000,                              // Stack
		NULL,                               // Parameter
		2,                                  // Priorität
		&xHandleBrakeTaskHandle,// TaskHandle
		1                                   // Core
	);

	xTaskCreatePinnedToCore(
		MeasureHeading,        // Funktion
		"Measure Heading",// Name
		10000,                              // Stack
		NULL,                               // Parameter
		2,                                  // Priorität
		&xHandleMeasureHeading,// TaskHandle
		1                                   // Core
	);

	xTaskCreatePinnedToCore(
		MeasureAkkuVoltage,        // Funktion
		"Measure Akku Voltage",// Name
		10000,                              // Stack
		NULL,                               // Parameter
		2,                                  // Priorität
		&xHandleMeasureAkkuVoltage,// TaskHandle
		1                                   // Core
	);
}

void loop() {

}
