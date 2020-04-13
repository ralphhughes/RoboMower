#include "Arduino.h"

// I2C is always loaded
#include <Wire.h>   
void i2cScan();

// For testing when not everything is connected:
#define COMPASS_ENABLED true
#define INA219_ENABLED true
#define MOTOR_DRIVER_ENABLED true
#define SONAR_ENABLED true
#define WIFI_ENABLED true

// I2C addresses:
//    0x30    Dual motor driver
//    0x0D    QMC5883L compass
//    0x40   INA219 current monitor
//
// MCU connections:
//    Wemos_D1_Mini.A0 ->     Analogue
//    Wemos_D1_Mini.D0 ->     BladeMotorRelay1, BladeMotorRelay2  (Int pulldown, No PWM\interrupt etc)
//    Wemos_D1_Mini.D1 ->	INA219.SCL, MotorShield.SCL, DigitalCompass.SCL
//    Wemos_D1_Mini.D2 ->	INA219.SDA, MotorShield.SDA, DigitalCompass.SDA
//    Wemos_D1_Mini.D3 ->	LeftSonar\LeftBumper                (Ext pullup, also run\flash code mode)
//    Wemos_D1_Mini.D4 ->	LeftEncoder                         (also built in LED, inverted)
//    Wemos_D1_Mini.D5 ->	BodyBumper (4 switches in series)
//    Wemos_D1_Mini.D6 ->	RightSonar\RightBumper
//    Wemos_D1_Mini.D7 ->	RightEncoder
//    Wemos_D1_Mini.D8 ->                                          (Ext pull down)
//    Wemos_D1_Mini.TX ->     Serial debug
//    Wemos_D1_Mini.RX ->     Serial debug
int BLADE_RELAY_PIN = D0;
int BODY_SWITCH_PIN = D5;
int LEFT_SONAR_PIN = D3;
int RIGHT_SONAR_PIN = D6;

// State machine for mower functions
String stateLabels[] = {"CALIBRATE_COMPASS","IDLE","MOWING","TURN_LEFT","TURN_RIGHT","EMERG_STOP"};

enum STATES {
    CALIBRATE_COMPASS,
    IDLE,
    MOWING,
    TURN_LEFT,
    TURN_RIGHT,
    EMERG_STOP
};

// Globals (with default erroneous values)
int currentState = IDLE;
float currentHeading = -1.0f;
float previousHeading = -1.0f;
unsigned long leftDist = 0;
unsigned long rightDist = 0;
float busvoltage = 0;
float current_mA = 0;
float power_mW = 0;

// Wifi
#if WIFI_ENABLED
    #include "ESP8266WiFi.h"
    #include <WiFiClient.h>
    #include <ESP8266WebServer.h>
    #include <ESP8266mDNS.h>
    #include "auth.h"

    const char* ssid = SSID;    // From auth.h (gitignored)
    const char* password = PSK; // From auth.h (gitignored)

    ESP8266WebServer server(80);
    
#endif

// Compass
#if COMPASS_ENABLED
    #include <MechaQMC5883.h>
    MechaQMC5883 compass;
    float magneticDeclination;
    float getTrueHeading();
    float differenceBetweenAngles(float angle1, float angle2);
#endif
    
#if INA219_ENABLED
    #include <Adafruit_INA219.h>
    Adafruit_INA219 ina219;
#endif

// Motor shield
#if MOTOR_DRIVER_ENABLED
    #include <LOLIN_I2C_MOTOR.h>
    LOLIN_I2C_MOTOR motorShield; //I2C address 0x30
    const byte intPinMotorA = D6;
    const byte intPinMotorB = D7;
#endif

// Serial
char receivedChar;
boolean newSerialData = false;
void recvOneChar();
void showNewData();


// Start setup sonar
#if SONAR_ENABLED
    #include <NewPingESP8266.h>
    #define MAX_DISTANCE 200 // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.
    NewPingESP8266 leftSonar(LEFT_SONAR_PIN, LEFT_SONAR_PIN, MAX_DISTANCE); // trig, echo, max dist
    NewPingESP8266 rightSonar(RIGHT_SONAR_PIN, RIGHT_SONAR_PIN, MAX_DISTANCE); // trig, echo, max dist
#endif




void setup() {
    Serial.begin(74880); // baud chosen to match the boot status debug output of the ESP
    Serial.print( F("\nStarting up...\nCode compiled: "));
    Serial.print( F(__DATE__));
    Serial.print( F(", "));
    Serial.println( F(__TIME__));

    // Startup I2C (has debug println statements in i2cscan)
    Wire.begin();
    i2cScan();    
    
    Serial.print("MAC: ");
    Serial.println(WiFi.macAddress());

    #if WIFI_ENABLED
        WiFi.mode(WIFI_STA);
        WiFi.begin(ssid, password);
        Serial.print("Connecting to AP");

        // Wait for connection
        while (WiFi.status() != WL_CONNECTED) {
            delay(500);
            Serial.print(".");
        }
        Serial.println("");
        Serial.print("Connected to ");
        Serial.println(ssid);
        Serial.print("IP address: ");
        Serial.println(WiFi.localIP());

        if (MDNS.begin("esp8266")) {
            Serial.println("MDNS responder started");
        }


        server.on("/", []() {
            String message = "Status\n\n";
            message += "currentState: ";    message += stateLabels[currentState];    message += "\n";
            message += "busVoltage: ";      message += busvoltage;      message += "\n";
            message += "power_mW: ";        message += power_mW;        message += "\n";
            message += "leftDist: ";        message += leftDist;        message += "\n";
            message += "rightDist: ";       message += rightDist;       message += "\n";
            message += "previousHeading: "; message += previousHeading; message += "\n";
            message += "currentHeading: ";  message += currentHeading;  message += "\n";
            
            server.send(200, "text/plain", message);
        });
        
        server.on("/stop", []() {
            currentState = EMERG_STOP;
            server.send(200, "text/plain", "Stop signal received.");
        });
        
        server.on("/mow", []() {
            currentState = MOWING;
            server.send(200, "text/plain", "Will start mowing.");
        });
        
        server.begin();
        Serial.println("HTTP server started");
    #endif

    
    // Start calibrate compass
    #if COMPASS_ENABLED
        compass.init();
        // This is for the GY-271 magnetometer while mounted on the breadboard on my desk. ie probably bollocks.
        compass.setMinMaxCalibration(-1256, -1042, -728, 2071, 2016, 2428);
        //http://www.magnetic-declination.com/ for Llandudno:
        // Magnetic Declination: -1° 31' - Declination is NEGATIVE (WEST)
        // Inclination: 67° 49', Magnetic field strength: 49311.2 nT
        magneticDeclination = -1.516667f;
    #endif
    
    #if INA219_ENABLED
        // Initialize the INA219.
        // By default the initialization will use the largest range (32V, 2A).  However
        // you can call a setCalibration function to change this range (see comments).
        ina219.begin();
        // To use a slightly lower 32V, 1A range (higher precision on amps):
        // ina219.setCalibration_32V_1A();
    #endif
    
    // Start setup motor shield
    #if MOTOR_DRIVER_ENABLED
        while (motorShield.PRODUCT_ID != PRODUCT_ID_I2C_MOTOR) { //wait until motor shield ready.
            //Serial.print("motorShield.getInfo(): ");
            Serial.println(motorShield.getInfo()); // This is NOT the same as PRODUCT_ID
            //Serial.print("Waiting for '2': ");
            //Serial.println(motorShield.PRODUCT_ID);
        }
        motorShield.changeFreq(MOTOR_CH_BOTH, 7500); //Change A & B 's Frequency to 7.5kHz to match NXT freq.
        pinMode(intPinMotorA, INPUT);
        pinMode(intPinMotorB, INPUT);
    #endif
    // End setup motor shield
    
    
    // Start setup pin modes
    pinMode(BLADE_RELAY_PIN, OUTPUT);    // BladeRelay
    // End setup pin modes
    
    currentState = IDLE;
}

void loop() {
    delay(200); // Why? For debug?
    
    // Things that need to run in the background
    #if WIFI_ENABLED
        server.handleClient();
        MDNS.update();
    #endif
    // USB Serial is always enabled
    recvOneChar();

        
    // Get values from sensors
    #if SONAR_ENABLED
        leftDist = leftSonar.ping_cm();
        delay(29); // 29ms should be the shortest delay between pings.
        rightDist = rightSonar.ping_cm();
        Serial.print("Left: ");
        Serial.print(leftDist);
        Serial.print("cm\tRight: ");
        Serial.print(rightDist);
        Serial.println("cm");
    #endif
    #if INA219_ENABLED
        float shuntvoltage = 0;
        //float busvoltage = 0;
        //float current_mA = 0;
        //float loadvoltage = 0;
        //float power_mW = 0;

        shuntvoltage = ina219.getShuntVoltage_mV();
        busvoltage = ina219.getBusVoltage_V();
        current_mA = ina219.getCurrent_mA();
        power_mW = ina219.getPower_mW();
        //loadvoltage = busvoltage + (shuntvoltage / 1000);

        //Serial.print("Bus Voltage:   "); Serial.print(busvoltage); Serial.println(" V");
        //Serial.print("Shunt Voltage: "); Serial.print(shuntvoltage); Serial.println(" mV");
        //Serial.print("Load Voltage:  "); Serial.print(loadvoltage); Serial.println(" V");
        //Serial.print("Current:       "); Serial.print(current_mA); Serial.println(" mA");
        //Serial.print("Power:         "); Serial.print(power_mW); Serial.println(" mW");
        //Serial.println("");


    #endif
    #if COMPASS_ENABLED
        Serial.print("previousHeading: ");
        Serial.print(previousHeading);
        currentHeading = getTrueHeading();
        Serial.print("\tcurrentHeading: ");
        Serial.println(currentHeading);
    #endif

    // If any of these sensors should interrupt and trigger a state change, then update the currentState variable
    if (leftDist > 0 && leftDist < 20) {
        previousHeading = currentHeading;
        currentState = TURN_RIGHT;
    }
    if (rightDist > 0 && rightDist < 20) {
        previousHeading = currentHeading;
        currentState = TURN_LEFT;
    }
        
        
    if (newSerialData == true) {
        Serial.print("Received serial debug command: ");
        Serial.println(receivedChar);
        
        switch(receivedChar) {
            case '0': 
                digitalWrite(BLADE_RELAY_PIN, LOW);
                break;
            case '1':
                digitalWrite(BLADE_RELAY_PIN, HIGH);
                break;
            case 'm':
                currentState = MOWING;
                break;
            case 'i':
                currentState = IDLE;
                break;
            case ' ':
                currentState = EMERG_STOP;
                break;
        }
        newSerialData = false;
    }    
        
        
    // Update outputs (motors, blade relay) based on current state    
    #if MOTOR_DRIVER_ENABLED
    switch (currentState) {
        case CALIBRATE_COMPASS:
            motorShield.changeDuty(MOTOR_CH_BOTH, 50);
            motorShield.changeStatus(MOTOR_CH_A, MOTOR_STATUS_CW);
            motorShield.changeStatus(MOTOR_CH_B, MOTOR_STATUS_CCW);
            // loopCompassCalibration();
            break;
        case IDLE:
            motorShield.changeStatus(MOTOR_CH_BOTH, MOTOR_STATUS_STOP);
            Serial.println("LeftMotor: Stop\tRightMotor: Stop");
            break;
        case MOWING:
            motorShield.changeDuty(MOTOR_CH_BOTH, 100);
            motorShield.changeStatus(MOTOR_CH_BOTH, MOTOR_STATUS_CW);
            Serial.println("LeftMotor: CW\tRightMotor: CW");
            break;
        case TURN_LEFT:
            motorShield.changeStatus(MOTOR_CH_A, MOTOR_STATUS_STOP);
            motorShield.changeStatus(MOTOR_CH_B, MOTOR_STATUS_CW);
            Serial.println("LeftMotor: Stop\tRightMotor: CW");
            break;
        case TURN_RIGHT:
            motorShield.changeStatus(MOTOR_CH_A, MOTOR_STATUS_CW);
            motorShield.changeStatus(MOTOR_CH_B, MOTOR_STATUS_STOP);
            Serial.println("LeftMotor: CW\tRightMotor: Stop");
            break;
        case EMERG_STOP:
            digitalWrite(BLADE_RELAY_PIN, LOW);
            motorShield.changeStatus(MOTOR_CH_BOTH, MOTOR_STATUS_SHORT_BRAKE);
            Serial.println("LeftMotor: Brake\tRightMotor: Brake");
            break;
            
    } // end switch
    
    #endif

    // Process any exit conditions for the current state
    switch (currentState) {
        case TURN_LEFT:
        case TURN_RIGHT:
            if (differenceBetweenAngles(previousHeading, currentHeading) > 90) {
                previousHeading = -1.0f;
                currentState = MOWING;
            }
            break;
    }
    
}



#if COMPASS_ENABLED
float getTrueHeading() {
    int trueX, trueY, trueZ;
    int err = compass.readCorrected(&trueX, &trueY, &trueZ);
    if (err) {
        return err;
    } else {
        float azimuth = compass.azimuth(&trueY, &trueX);
        azimuth = azimuth + magneticDeclination;
        return azimuth;
    }
}
float differenceBetweenAngles(float angle1, float angle2) {
    float a = angle1 - angle2;
    if (a > 180)
        a -= 360;
    if (a < -180)
        a+= 360;
    return abs(a);
}
#endif

void recvOneChar() {
    if (Serial.available() > 0) {
        receivedChar = Serial.read();
        newSerialData = true;
    }
}


void i2cScan() {
    byte error, address;
    int nDevices;

    Serial.println("Scanning i2c bus...");

    nDevices = 0;
    bool addresses[127];
    for (address = 1; address < 127; address++) {
        // The i2c_scanner uses the return value of
        // the Write.endTransmission to see if
        // a device did acknowledge to the address.
        Wire.beginTransmission(address);
        error = Wire.endTransmission();

        if (error == 0) {
            //Serial.print("I2C device found at address 0x");
            addresses[address] = true;
            //if (address < 16)
            //    Serial.print("0");
            //Serial.println(address, HEX);
            nDevices++;
        } else if (error == 4) {
            //Serial.print("Unknown error at address 0x");
            addresses[address] = false;
            //if (address < 16)
            //    Serial.print("0");
            //Serial.println(address, HEX);
        } else {
            addresses[address] = false;
        }
    }
    if (nDevices == 0) {
        Serial.println("No I2C devices found.");
    } else {
        Serial.println("Finished scan.");
    }
    if(addresses[0x30]) {
        #if MOTOR_DRIVER_ENABLED
            Serial.println("Info: Motor driver found and enabled.");
        #else
            Serial.println("Warn: Motor driver found but not enabled.");
        #endif
    } else {
        #if MOTOR_DRIVER_ENABLED
            Serial.println("ERROR: Motor driver enabled but not connected!");
        #endif
    }
    if(addresses[0x0D]) {
        #if COMPASS_ENABLED
            Serial.println("Info: Compass found and enabled.");
        #else
            Serial.println("Warn: Compass found but not enabled.");
        #endif
    } else {
        #if COMPASS_ENABLED
            Serial.println("ERROR: Compass enabled but not connected!");
        #endif
    }
    if(addresses[0x40]) {
        #if INA219_ENABLED
            Serial.println("Info: INA219 found and enabled.");
        #else
            Serial.println("Warn: INA219 found but not enabled.");
        #endif
    } else {
        #if INA219_ENABLED
            Serial.println("ERROR: INA219 enabled but not connected!");
        #endif
    }
}