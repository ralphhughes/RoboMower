/*
 * Baterry Monitor
 * by Flavius Bindea
 * this sketch uses averaging and check of ready status
 */
#include <Wire.h>
#include <INA219.h>

// Current sensor and shunt used 
INA219 ina219;

#define R_SHUNT 0.00375
#define V_SHUNT_MAX 0.075
#define V_BUS_MAX 16
#define I_MAX_EXPECTED 20

boolean led=true;

// current and voltage readings
float shuntvoltage = 0;
float busvoltage = 0;
float current_A = 0;
float batvoltage = 0;
float power = 0;
float Ah = 0;
unsigned long lastread = 0; // used to calculate Ah
unsigned long tick;         // current read time - last read

// different intervals for each Task
int intervalReadData = 50;
int intervalDisplay = 1000;

// last taks call
unsigned long previousMillisReadData = 0;
unsigned long previousMillisDisplay = 0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(57600);
  Serial.println("Hello - Arduino_INA219 !");

  ina219.begin();
  // configure INA219 for averaging at 16 samples (8.51ms)
  ina219.configure(INA219::RANGE_16V, INA219::GAIN_2_80MV, INA219::ADC_16SAMP, INA219::ADC_16SAMP, INA219::CONT_SH_BUS);
  // configure INA219 for averaging at 128 samples
  //ina219.configure(INA219::RANGE_16V, INA219::GAIN_2_80MV, INA219::ADC_128SAMP, INA219::ADC_128SAMP, INA219::CONT_SH_BUS);
  lastread = millis();
  // calibrate INA219 with out shunt values
  ina219.calibrate(R_SHUNT, V_SHUNT_MAX, V_BUS_MAX, I_MAX_EXPECTED);

  // for led blinking
  pinMode(13, OUTPUT);

/*
  // use timer 1 to launch current reading
  // this is a test only and not sure it works
  Timer1.initialize(READFREQ); // 100ms reading interval
  Timer1.attachInterrupt(readCurrent); 
*/
  delay(1000);
}

void loop() {
  
  // get current time stamp
  // only need one for both if-statements
  unsigned long currentMillis = millis();  

  if ((unsigned long)(currentMillis - previousMillisReadData) >= intervalReadData) {
    previousMillisReadData = millis();
    readCurrent();
    Serial.print("tick:   "); Serial.print(tick); Serial.println(" ms");
  }
  if ((unsigned long)(currentMillis - previousMillisDisplay) >= intervalDisplay) {
    previousMillisDisplay = millis();
    // displays data
    Serial.print("Bus Voltage:   "); Serial.print(busvoltage,3); Serial.println(" V");
    Serial.print("Shunt Voltage: "); Serial.print(shuntvoltage,3); Serial.println(" mV");
    Serial.print("Bat Voltage:   "); Serial.print(batvoltage,3); Serial.println(" V");
    Serial.print("Current:       "); Serial.print(current_A,3); Serial.println(" A");
    Serial.print("Power:         "); Serial.print(power,3); Serial.println(" W");
    Serial.print("Ah:         ");    Serial.print(Ah,3); Serial.println(" Ah");
    Serial.println("");
  }
  // blink led
  digitalWrite(13, led);
  led = !led;
  
  delay(10);
}

void readCurrent() {
  uint32_t count = 0;
  unsigned long newtime;

//  Serial.println("waiting data ready");

  // reads busVoltage
  busvoltage = ina219.busVoltage();
  // waits for conversion ready
  while(!ina219.ready() && count < 500) {
    count++;
    delay(1);
    busvoltage = ina219.busVoltage();  
  }
  
//  Serial.print("Count:   "); Serial.println(count);

  // read the other values
  shuntvoltage = ina219.shuntVoltage() * 1000;
  current_A = ina219.shuntCurrent();
  batvoltage = busvoltage + (shuntvoltage / 1000);
  power = ina219.busPower();
  newtime = millis();
  tick = newtime - lastread;
  Ah += (current_A * tick)/3600000.0;
  lastread = newtime;

  // prepare for next read -- this is security just in case the ina219 is reset by transient curent
  ina219.recalibrate();
  ina219.reconfig();
}

