//GolfBuddy_RTOS.ino
//Autor: Niklas Krakhofer
//Project: Golf Buddy
//TODO: Auslesen und Berechnen des Heading muss überprüft werden im Bezug auf Verdrehung und Paramter &Übergabe und Globale Parameter
//      Eventuell wenn möglich Globale Variablen durch returns der Funktionen ersetzen
//      vTaskReceiveDataFromTracker ausprogrammieren
//      Ausweichen durchtesten

//Defines for different Code usage
//#define ObsticaleDetectionWithDogeing

#include "GolfBuddy_RTOS_Declaration.h"
#include "GolfBuddy_RTOS_Functions.h"
#include "GolfBuddy_RTOS_Constants.h"
#include <Arduino.h>
#include <TinyGPSPlus.h>
#include <HardwareSerial.h>
#include <cstdint>
#include <cstring>
#include <Wire.h>
#include <math.h>
#include <freertos/semphr.h>

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

#ifdef ObsticaleDetectionWithDogeing
TaskHandle_t xHandleDriveArroundwithCheck;
TaskHandle_t xHandleDodge;
#endif // ObsticaleDetectionWithDogeing

//Task: Recieves serial Data from Uart including: {DriveMode;} DriveMode = 0 -> no DriveMode active
//                                                             DriveMode = 1 -> Tracking activated 
//                                                             DriveMode = 2 -> MotorSupport activated
//      Decodes Received Massage and sets Variables.
//Param: -
void vReceiveDataFromRaspberryPI(void* pvParameters) {
    while (1) {   
        if (piSerial.available() > 0) {
            String sReceivedData = piSerial.readStringUntil('\n');
            Serial.println(sReceivedData);

            String sDataSegments[10];
            int iDataSegmentIndex = 0;
            String sCurrentDataSegment = "";

            for (int i = 0; i < sReceivedData.length(); i++) {
                char c = sReceivedData[i];
                if (c == ';') {
                    sDataSegments[iDataSegmentIndex++] = sCurrentDataSegment;
                    sCurrentDataSegment = "";
                }
                else {
                    sCurrentDataSegment += c;
                }
            }

            if (sCurrentDataSegment.length() > 0) {
                sDataSegments[iDataSegmentIndex++] = sCurrentDataSegment;
            }
            
            if (sDataSegments[0].toInt() == 0) {
                bIsPlayerTrackingActivated = false;
                bIsMotorSupportActivated = false;
            }
            else if (sDataSegments[0].toInt() == 1) {
                bIsPlayerTrackingActivated = true;
                bIsMotorSupportActivated = false;
            }
            else if (sDataSegments[0].toInt() == 2) {
                bIsPlayerTrackingActivated = false;
                bIsMotorSupportActivated = true;
            }
        }
        vTaskDelay(1 / portTICK_PERIOD_MS);
    }
}

//Task: Sends Serial Data to Raspberry Pi every 1s including: u8StartByte
//                                                            iBatteryLevel
//                                                            iSolarpanelRecuperationLevel
//                                                            fOutsideTemperature
//                                                            fOutsideHumidity
//                                                            u8ObstacleInWay
//                                                            dLatitudeGolfBuddy
//                                                            dLongitudeGolfBuddy
//                                                            fFacingDirection
//Param: -
void vTransmittDataToRaspberryPI(void* pvParameters) {
    while (1) {
        vSendTransmitdataToPi();
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

//Task: Reads GPS Coordinates from GPS Modul every 1.5s
//Param: -
void vReadGPSData(void* pvParameters) {
    unsigned long lastUpdate = millis();
    while (1) {
        if (gps.location.isUpdated() && (millis() - lastUpdate > 1500)) {
            lastUpdate = millis();
            trolleyCoords.dGolfTrolley_latitude = gps.location.lat();
            trolleyCoords.dGolfTrolley_longitude = gps.location.lng();   

            RaspPI_transmitData.dLatitudeGolfBuddy = trolleyCoords.dGolfTrolley_latitude;
            RaspPI_transmitData.dLongitudeGolfBuddy = trolleyCoords.dGolfTrolley_longitude;
        }
        while (gpsSerial.available() > 0) {
            gps.encode(gpsSerial.read());
        }
        vTaskDelay(1 / portTICK_PERIOD_MS);
    }
}

//Task: Reads Temperature from BME280 every 1s
//Param: -
void vReadTemperature(void* pvParameter) {
    while (1) {
        int32_t i32RawTemp = i32ReadRawTemperatureBME280();
        float fTemperature = fCompensateTemperatureBME280(i32RawTemp);

        vTaskDelay(1000 / portTICK_PERIOD_MS);
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
void vMeassureMotorSpeed(void* pvParameter) {
    while (1) {
        vUpdateMeausuredMotorSpeed();
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

//Task: If targetCoordinates are in Queue and Tracking is enabled, then the Trolley drives to the target Coordinate.
//      Else the Trolley stays in Parking and is not moving.
//Param: -
void vPlayerTracking(void* pvParameter) {
    GPSCoordinates targetCoords;
    while (1) {
        if (bIsPlayerTrackingActivated && !bDogingactive && !bIsBreakingActive) {
            if (iBuffer_Coordinates_ReadIndex == iBuffer_Coordinates_WriteIndex)
            {
                vParking();
            }
            else if (bGetOldestGPSCoordinate(targetCoords) && iBuffer_Coordinates_ReadIndex != iBuffer_Coordinates_WriteIndex) {
                vPlayerTracking(targetCoords, trolleyCoords);
            }
        }
        vTaskDelay(1 / portTICK_PERIOD_MS);
    }
}

//Task: If bIsMotorSupportActivated is activated and Touchsensor/s is/are triggerd both Motors are regulated to BaseSpeed. 
//Param: -
void vTaskMotorSupport(void* pvParameter) {
    while (1) {
        if (bIsMotorSupportActivated && !bDogingactive) {
            vMotorSupport();
        }
        vTaskDelay(1 / portTICK_PERIOD_MS);
    }
}

//@brief: ifdef ObsticaleDetectionWithDogeing:  If an Objects is to close to the Trolley a Doging algorythm is triggered. 
//                                              If Doging is finished the previous Driving mode continious.
//        ifndef ObsticaleDetectionWithDogeing: The Trolley is breaking if an obsticle detected by distance sensors. 
//@param: -
//@return: -
void vTaskCheckSurrounding(void* pvParameter) {
    while (1) {
        vCheckSurrounding();    
        vTaskDelay(100  / portTICK_PERIOD_MS);
    }
}

//Task: !
//Param: -
void vTaskReceiveDataFromTracker(void* pvParameter) {
    while (1) {
        while (funkSerial.available()) {
            char c = funkSerial.read();

            if (c == ';') {
                sIncomeTrackerDataFields[iIncomeTrackerFieldIndex] = sCurrentIncomeTrackerDataField;
                iIncomeTrackerFieldIndex++;
                sCurrentIncomeTrackerDataField = "";
            }
            else if (c == '\n') {
                sIncomeTrackerDataFields[iIncomeTrackerFieldIndex] = sCurrentIncomeTrackerDataField;

                double dlatitude = sIncomeTrackerDataFields[0].toDouble();
                double longitude = sIncomeTrackerDataFields[1].toDouble();
                bTrolleyStartStop = (sIncomeTrackerDataFields[3] == "true");
                
                if (bTrolleyStartStop) {
                    vPushGPSData(dlatitude, longitude);
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
void vTaskBreakMotors(void* parameter) {
    static bool brakeInProgress = false;
    unsigned long startTime = millis();

    while (1) {
        if (bIsBreakingActive) {
            if (!brakeInProgress) {
                startTime = millis();
                brakeInProgress = true;
            }
            //Testen ob das passt mit digital, wenn nicht pwm
            digitalWrite(MotorLeftBreakPin, HIGH);
            digitalWrite(MotorRightBreakPin, HIGH);
            //vBreakMotorLeft(iBreakIntensityMotorLeft);
            //vBreakMotorRight(iBreakIntensityMotorRight);

            if (millis() - startTime > brakeDuration) {
                bIsBreakingActive = false;
                brakeInProgress = false;
            }
        }
        else {
            digitalWrite(MotorLeftBreakPin, LOW);
            digitalWrite(MotorRightBreakPin, LOW);
        }
        vTaskDelay(1 / portTICK_PERIOD_MS);
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
    piSerial.begin(BaudRate_9600, SERIAL_8N1, RaspberryPIRXPin, RaspberryPITXPin);
    Wire.begin(SDA, SCL);

    pinMode(MotorLeftPWMPin, OUTPUT);
    pinMode(MotorRightPWMPin, OUTPUT);

    pinMode(MotorLeftBreakPin, OUTPUT);
    pinMode(MotorRightBreakPin, OUTPUT);

    analogWriteFrequency(MotorLeftPWMPin, 20000);
    analogWriteFrequency(MotorRightPWMPin, 20000);

    analogWriteFrequency(MotorLeftBreakPin, 20000);
    analogWriteFrequency(MotorRightBreakPin, 20000);

    pinMode(MotorLeftSpeedPin, INPUT_PULLUP);
    pinMode(MotorRightSpeedPin, INPUT_PULLUP);

    attachInterrupt(digitalPinToInterrupt(MotorLeftSpeedPin), vCountPulseMotorLeft, RISING);
    attachInterrupt(digitalPinToInterrupt(MotorRightSpeedPin), vCountPulseMotorRight, RISING);

    pinMode(MotorLeftDrivingDirectionPin, OUTPUT);
    pinMode(MotorRightDrivingDirectionPin, OUTPUT);
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
    gpsSerial.begin(BaudRate_9600);
}

void initGY271() {
    //softReset(); //Wird glaube ich nicht genutzt weil qmcResetAndInit sowieso resetet muss aber überprüft werden
    //vSetCtrlRegisterGY271(OSR_128, RNG_2G, ODR_100Hz, Mode_Continuous); //Wird glaube ich nicht genutzt weil qmcResetAndInit sowieso resetet muss aber überprüft werden
    vResetAndInitGY271();
}

void initHC12() {
    funkSerial.begin(BaudRate_9600, SERIAL_8N1, HC12RXPin, HC12TXPin);
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

//////////////////////////////////////////////////////////////////////////////
// Setup
//////////////////////////////////////////////////////////////////////////////

void setup() {
    init();
    initBME280();
    initGPSModule();
    initGY271();
    initHC12();
    initHCSR04();

    //vPrintCalibrationDataGY271();

    bufferMutex = xSemaphoreCreateMutex();

    xTaskCreatePinnedToCore(
        vReceiveDataFromRaspberryPI,        // Funktion
        "ReceiveSerialDataFromRaspberryPI",// Name
        10000,                              // Stack
        NULL,                               // Parameter
        2,                                  // Priorität
        &xHandleReceiveDataFromRaspberryPI,// TaskHandle
        1                                   // Core
    );

    xTaskCreatePinnedToCore(
        vTransmittDataToRaspberryPI,        // Funktion
        "TransmittSerialDataToRaspberryPI",// Name
        10000,                              // Stack
        NULL,                               // Parameter
        2,                                  // Priorität
        &xHandleTransmittDataToRaspberryPI,// TaskHandle
        1                                   // Core
    );

    xTaskCreatePinnedToCore(
        vReadGPSData,        // Funktion
        "ReadGPSCoordinatesFromGPSModule",// Name
        10000,                              // Stack
        NULL,                               // Parameter
        2,                                  // Priorität
        &xHandleReadGPSData,// TaskHandle
        1                                   // Core
    );

    xTaskCreatePinnedToCore(
        vReadTemperature,        // Funktion
        "ReadTemperatureFromBME",// Name
        10000,                              // Stack
        NULL,                               // Parameter
        2,                                  // Priorität
        &xHandleReadTemperature,// TaskHandle
        1                                   // Core
    );

    xTaskCreatePinnedToCore(
        vMeassureMotorSpeed,        // Funktion
        "MeassureMotorSpeed",// Name
        10000,                              // Stack
        NULL,                               // Parameter
        2,                                  // Priorität
        &xHandleMeasureMotorSpeed,// TaskHandle
        1                                   // Core
    );

    xTaskCreatePinnedToCore(
        vPlayerTracking,        // Funktion
        "Follow Player if enabled",// Name
        10000,                              // Stack
        NULL,                               // Parameter
        2,                                  // Priorität
        &xHandlePlayerTracking,// TaskHandle
        1                                   // Core
    );

    xTaskCreatePinnedToCore(
        vTaskMotorSupport,        // Funktion
        "Handle Motor Support if enabled",// Name
        10000,                              // Stack
        NULL,                               // Parameter
        2,                                  // Priorität
        &xHandleMotorSupport,// TaskHandle
        1                                   // Core
    );

    xTaskCreatePinnedToCore(
        vTaskCheckSurrounding,        // Funktion
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
        vTaskReceiveDataFromTracker,        // Funktion
        "Receive Data from Tracker",// Name
        10000,                              // Stack
        NULL,                               // Parameter
        2,                                  // Priorität
        &xHandleReceiveDataFromTracker,// TaskHandle
        1                                   // Core
    );

    xTaskCreatePinnedToCore(
        vTaskBreakMotors,        // Funktion
        "Emergency Braking",// Name
        10000,                              // Stack
        NULL,                               // Parameter
        2,                                  // Priorität
        &xHandleBrakeTaskHandle,// TaskHandle
        1                                   // Core
    );    
}

void loop() {

}
