// Lovesac_FootOnly.ino
// Nottingham Spirk, October 17, 2023
//
// This sketch uses the switch inputs to run the FootRest motor 
// forward and reverse.
//
// This sketch is targetted to a Adafruit Feather 328P board.
//    https://www.adafruit.com/product/3458
//    https://learn.adafruit.com/adafruit-feather-328p-atmega328-atmega328p
// Select "Arduino Pro or Pro Mini" in the boards manager.
//

#include <Wire.h>  // I2C communication

// *******************************************************************
// Hardware Pins
// *******************************************************************
// #define SP_HOME_PIN     0   // input, reserved for Debug UART
// #define SP_LIMIT_PIN    1   // input, reserved for Debug UART
#define SW1_PIN         2   // input, high = closed
#define SW2_PIN         3   // input, high = closed
#define unused_4_PIN    4
#define SP_MOT_PH_PIN   5   // output
#define SP_MOT_EN_PIN   6   // output
#define SP_MOT_SLP_PIN  7   // output, low = sleep
#define FR_MOT_PH_PIN   9   // output
#define FR_MOT_EN_PIN  10   // output
#define FR_MOT_SLP_PIN 11   // output, low = sleep
#define FR_HOME_PIN    12   // input
#define FR_LIMIT_PIN   13   // input
#define SP_CUR_PIN     A0   // analog input, 1V = tbd A
#define FR_CUR_PIN     A1   // analog input
#define SW3_PIN        16   // input, high = closed
#define SW4_PIN        17   // input, high = closed

// *******************************************************************
// Constants
// *******************************************************************
#define ADC_VREF  3.3       // default AREF (V)
#define ADC_COUNT 1024.0    // 10-bit ADC
#define ADC_SAMPLES 10      // samples for average
#define MOTOR_CUR  4.762    // motor driver SO output scaling (A/V)
#define CUR_THRESH 0.020    // limit for "0" current (A)
#define CUR_OBSTR  8.500    // limit for "obstructed" current (A)
#define BAT_ADDR  0x12      // battery manager I2C address ***tbd
#define BAT_VOLT  0x34      // battery voltage register address ***tbd
#define PRESSED      1      // input switch state
#define NOT_PRESSED  0      // input switch state
#define AT_LIMIT     1      // limit position sensor state
#define NOT_AT_LIMIT 0      // limit position sensor state
#define AT_HOME      1      // home position sensor state
#define NOT_AT_HOME  0      // home position sensor state
#define PH_FWD       1      // motor driver PH input = forward
#define PH_REV       0      // motor driver PH input = reverse
#define ACTIVE       1      // motor driver nSLEEP input = active/run
#define SLEEP        0      // motor driver nSLEEP input = sleep/off

// *******************************************************************
// Global Variables
// *******************************************************************
char  buf[24];        // char buffer for generating printouts
float SP_current;     // offset-corrected ADC reading scaled to current
int   SP_offset;      // ADC reading with 0 motor current
float FR_current;
int   FR_offset;
int   forward  = NOT_PRESSED;
int   reverse  = NOT_PRESSED;
int   SP_limit = NOT_AT_LIMIT;
int   FR_limit = NOT_AT_LIMIT;
int   SP_home  = NOT_AT_HOME;
int   FR_home  = NOT_AT_HOME;
int   timeout  = 0;

// *******************************************************************
// Function Prototypes
// *******************************************************************
void  getSeatpanOffset(void);
void  getFootrestOffset(void);
float readSeatpanCurrent(void);
float readFootrestCurrent(void);
void  readBattery(void);


// *******************************************************************
// Setup Code, runs only once
// *******************************************************************
void setup() {

  while (!Serial) { delay(10); }
  Serial.begin(115200);
  Serial.println(F("*************************************"));
  Serial.println(F("Lovesac FootRest Only Demo"));
  Serial.println(__DATE__ "  Compiler Version: " __VERSION__);
  Serial.println(F("*************************************"));

  Wire.begin();

  // pinMode(SP_HOME_PIN,  INPUT);
  // pinMode(SP_LIMIT_PIN, INPUT);
  pinMode(FR_HOME_PIN,  INPUT);
  pinMode(FR_LIMIT_PIN, INPUT);
  pinMode(SW1_PIN, INPUT);
  pinMode(SW2_PIN, INPUT);
  pinMode(SW3_PIN, INPUT);
  pinMode(SW4_PIN, INPUT);

  pinMode(SP_MOT_PH_PIN,  OUTPUT);
  pinMode(SP_MOT_EN_PIN,  OUTPUT);
  pinMode(SP_MOT_SLP_PIN, OUTPUT);
  pinMode(FR_MOT_PH_PIN,  OUTPUT);
  pinMode(FR_MOT_EN_PIN,  OUTPUT);
  pinMode(FR_MOT_SLP_PIN, OUTPUT);

  digitalWrite(SP_MOT_PH_PIN,  PH_REV);
  digitalWrite(SP_MOT_EN_PIN,  LOW);
  digitalWrite(SP_MOT_SLP_PIN, SLEEP);
  digitalWrite(FR_MOT_PH_PIN,  PH_REV);
  digitalWrite(FR_MOT_EN_PIN,  LOW);
  digitalWrite(FR_MOT_SLP_PIN, SLEEP);

  getSeatpanOffset();
  getFootrestOffset();

}  // setup()


// *******************************************************************
// Main Loop, continuously runs
// *******************************************************************
void loop() {
  float tempFloat;

  digitalWrite(FR_MOT_SLP_PIN, SLEEP);
  analogWrite(FR_MOT_EN_PIN,   0);

  forward  = digitalRead(SW1_PIN);
  FR_limit = digitalRead(FR_LIMIT_PIN);
  reverse  = digitalRead(SW2_PIN);
  FR_home  = digitalRead(FR_HOME_PIN);
  timeout  = 200;    // 200 * 50 = 10 sec 

  while ( (forward==PRESSED) &&
          (FR_limit==NOT_AT_LIMIT) &&
          (timeout>0) ) {

        // read the inputs  
        forward  = digitalRead(SW1_PIN);
        FR_limit = digitalRead(FR_LIMIT_PIN);
        timeout--;

        // run the motor forward
        digitalWrite(FR_MOT_SLP_PIN, ACTIVE);
        digitalWrite(FR_MOT_PH_PIN,  PH_FWD);
        analogWrite(FR_MOT_EN_PIN,   255);
        tempFloat = readFootrestCurrent();
        dtostrf(tempFloat, 4, 3, buf);
        Serial.print(F("FR = "));
        Serial.print(buf);
        Serial.println("A");
        delay(50);
        }

   while ( (reverse==PRESSED) &&
           (FR_home==NOT_AT_HOME) &&
           (timeout>0) ) {

        // read the inputs  
        reverse = digitalRead(SW2_PIN);
        FR_home = digitalRead(FR_HOME_PIN);
        timeout--;

        // run the motor reverse
        digitalWrite(FR_MOT_SLP_PIN, ACTIVE);
        digitalWrite(FR_MOT_PH_PIN,  PH_REV);
        analogWrite(FR_MOT_EN_PIN,   255);
        tempFloat = readFootrestCurrent();
        dtostrf(tempFloat, 4, 3, buf);
        Serial.print(F("FR = "));
        Serial.print(buf);
        Serial.println("A");
        delay(50);
        }

  delay(100);

}  // loop()




// *******************************************************************
// Get the current measurement offset voltage for each motor
// driver.  The offset is in ADC counts and will be saved in
// global variables for later use by the measurement functions.
// Note that this must be done with the motor awake but disabled.
// *******************************************************************
void getSeatpanOffset() {
 
  // wake the motor driver
  digitalWrite(SP_MOT_SLP_PIN, ACTIVE);

  // take 10 readings and sum them up
  SP_offset = 0;
  for (int i = ADC_SAMPLES; i > 0; i--) {
    SP_offset += analogRead(SP_CUR_PIN);
    delay(10);
  }

  // put the motor driver back to sleep
  digitalWrite(SP_MOT_SLP_PIN, SLEEP);

  // get the average
  SP_offset /= ADC_SAMPLES;

  Serial.print(F("SeatPan Offset  = "));
  Serial.println(SP_offset);
}

void getFootrestOffset() {

  // wake the motor driver
  digitalWrite(FR_MOT_SLP_PIN, ACTIVE);

  // take 10 readings and sum them up
  FR_offset = 0;
  for (int i = ADC_SAMPLES; i > 0; i--) {
    FR_offset += analogRead(FR_CUR_PIN);
    delay(10);
  }

  // put the motor driver back to sleep
  digitalWrite(FR_MOT_SLP_PIN, SLEEP);

  // get the average
  FR_offset /= ADC_SAMPLES;

  Serial.print(F("FootRest Offset = "));
  Serial.println(FR_offset);
}


// *******************************************************************
// Read the current for each motor.  The output is offset-corrected
// and scaled to return current in Amperes.
// ADC * 4.762A/V * 3.3Vref / 1024 = 15.35mA/bit resolution
// *******************************************************************
float readSeatpanCurrent() {
  float scaled = 0.0;
  int ADCValue = 0;
 
  for (int i = ADC_SAMPLES; i > 0; i--) {
    ADCValue += analogRead(SP_CUR_PIN);
    delay(1);
  }

  ADCValue /= ADC_SAMPLES;          // get the average (int)
  scaled = (ADCValue - SP_offset);  // eliminate the motor driver offset
  scaled *= MOTOR_CUR;              // multiply by motor driver A/V gain
  scaled *= ADC_VREF;               // multiply by ADC full scale range
  scaled /= ADC_COUNT;              // divide by ADC resolution
  return scaled;
}

float readFootrestCurrent() {
  float scaled = 0.0;
  int ADCValue = 0;
 
  for (int i = ADC_SAMPLES; i > 0; i--) {
    ADCValue += analogRead(FR_CUR_PIN);
    delay(1);
  }

  ADCValue /= ADC_SAMPLES;          // get the average (int)
  scaled = (ADCValue - FR_offset);  // eliminate the motor driver offset
  scaled *= MOTOR_CUR;              // multiply by motor driver A/V gain
  scaled *= ADC_VREF;               // multiply by ADC full scale range
  scaled /= ADC_COUNT;              // divide by ADC resolution
  return scaled;
}


// *******************************************************************
// Use the SMBus (I2C) interface to read the current battery status.
//
// T B D
//
// *******************************************************************
void readBattery() {
  Wire.requestFrom(BAT_ADDR, 6);  // request 6 bytes from slave device
  while (Wire.available()) {      // slave may send less than requested
    char c = Wire.read();         // receive a byte as character
    Serial.print(c);              // print the character
  }
  Serial.println(" ");  // finish the line
}
