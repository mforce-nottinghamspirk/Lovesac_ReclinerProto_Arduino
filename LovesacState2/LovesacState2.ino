// LovesacState2.ino
// Nottingham Spirk, October 13, 2023
//
// This sketch implements a simplified state machine.
//   no timeout monitoring
//   no input debouncing
//   no other error handling
//
// This sketch is targetted to a Adafruit Feather 328P board.
//    https://www.adafruit.com/product/3458
//    https://learn.adafruit.com/adafruit-feather-328p-atmega328-atmega328p
// Select "Arduino Pro or Pro Mini" in the boards manager.
//

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
#define MOTOR_CUR 1.623     // motor driver SO output scaling (A/V)
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
#define STATE_INIT	    0
#define STATE_IDLE	    1
#define STATE_REC_FR    2
#define STATE_REC_SP    3
#define STATE_REC_WAIT  4
#define STATE_HOME_SP   5
#define STATE_HOME_FR   6
#define STATE_HOME_WAIT 7
#define STATE_OBSTRUCT  20


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
int   state    = STATE_INIT;


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
  Serial.println(F("Lovesac Simple State Machine"));
  Serial.print(F("Analog Samples = "));
  Serial.println(ADC_SAMPLES);
  Serial.println(__DATE__ "  Compiler Version: " __VERSION__);
  Serial.println(F("*************************************"));

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
  
  state = STATE_INIT;

}  // setup()


// *******************************************************************
// Main Loop, continuously runs
// *******************************************************************
void loop() {

  // read the operator switches
  forward = digitalRead(SW1_PIN);
  reverse = digitalRead(SW2_PIN);

  // read the limit switches
  // SP_limit = digitalRead(SP_LIMIT_PIN);
  // SP_home  = digitalRead(SP_HOME_PIN);
  FR_limit = digitalRead(FR_LIMIT_PIN);
  FR_home  = digitalRead(FR_HOME_PIN);

  // read the motor currents
  SP_current = readSeatpanCurrent();
  FR_current = readFootrestCurrent();

  switch (state) {

    case STATE_INIT:
      digitalWrite(SP_MOT_PH_PIN, PH_REV);
      analogWrite(SP_MOT_EN_PIN,  0);
      digitalWrite(SP_MOT_SLP_PIN, SLEEP);
      digitalWrite(FR_MOT_PH_PIN, PH_REV);
      analogWrite(FR_MOT_EN_PIN,  0);
      digitalWrite(FR_MOT_SLP_PIN, SLEEP);
      state = STATE_IDLE;
      Serial.println("State => Idle");
      break;

    case STATE_IDLE:
      digitalWrite(SP_MOT_PH_PIN, PH_REV);
      analogWrite(SP_MOT_EN_PIN, 0);
      digitalWrite(SP_MOT_SLP_PIN, ACTIVE);
      digitalWrite(FR_MOT_PH_PIN, PH_REV);
      analogWrite(FR_MOT_EN_PIN, 0);
      digitalWrite(FR_MOT_SLP_PIN, ACTIVE);

      if (forward==PRESSED) {
        state = STATE_REC_FR;
        Serial.println("State => Recline Footrest");
      }
      else if (reverse==PRESSED) {
        state = STATE_HOME_SP;
        Serial.println("State => Home Seatpan");
      }
      break;

    case STATE_REC_FR:
      digitalWrite(SP_MOT_PH_PIN, PH_FWD);
      analogWrite(SP_MOT_EN_PIN, 0);
      digitalWrite(FR_MOT_PH_PIN, PH_FWD);
      analogWrite(FR_MOT_EN_PIN, 255);

      if (forward == NOT_PRESSED) {
        state = STATE_IDLE;
        Serial.println("State => Idle");
      }
      else if (FR_limit == AT_LIMIT) {
        state = STATE_REC_SP;
        Serial.println("State => Recline Seatpan");
      }
      break;

    case STATE_REC_SP:
      digitalWrite(SP_MOT_PH_PIN, PH_FWD);
      analogWrite(SP_MOT_EN_PIN, 255);
      digitalWrite(FR_MOT_PH_PIN, PH_FWD);
      analogWrite(FR_MOT_EN_PIN, 0);

      if (forward == NOT_PRESSED) {
        state = STATE_IDLE;
        Serial.println("State => Idle");
      }
      else if (SP_limit == AT_LIMIT) {
        state = STATE_REC_WAIT;
        Serial.println("State => Recline Wait");
      }
      break;

    case STATE_REC_WAIT:
      digitalWrite(SP_MOT_PH_PIN, PH_FWD);
      analogWrite(SP_MOT_EN_PIN, 0);
      digitalWrite(FR_MOT_PH_PIN, PH_FWD);
      analogWrite(FR_MOT_EN_PIN, 0);

      if (forward == NOT_PRESSED) {
        state = STATE_IDLE;
        Serial.println("State => Idle");
      }
      break;

    case STATE_HOME_SP:
      digitalWrite(SP_MOT_PH_PIN, PH_REV);
      analogWrite(SP_MOT_EN_PIN, 255);
      digitalWrite(FR_MOT_PH_PIN, PH_REV);
      analogWrite(FR_MOT_EN_PIN, 0);

      if (reverse==NOT_PRESSED) {
        state = STATE_IDLE;
        Serial.println("State => Idle");
      }
      else if (SP_home == AT_HOME) {
        state = STATE_HOME_FR;
        Serial.println("State => Home Footrest");
      }
      else if (SP_current > CUR_OBSTR) {
        state = STATE_OBSTRUCT;
        Serial.println("State => Obstruction");
      }
      break;

    case STATE_HOME_FR:
      digitalWrite(SP_MOT_PH_PIN, PH_REV);
      analogWrite(SP_MOT_EN_PIN, 0);
      digitalWrite(FR_MOT_PH_PIN, PH_REV);
      analogWrite(FR_MOT_EN_PIN, 255);

      if (reverse==NOT_PRESSED) {
        state = STATE_IDLE;
        Serial.println("State => Idle");
      }
      else if (FR_home == AT_HOME) {
        state = STATE_HOME_WAIT;
        Serial.println("State => Home Wait");
       }
      else if (FR_current > CUR_OBSTR) {
        state = STATE_OBSTRUCT;
        Serial.println("State => Obstruction");
      }
      break;

    case STATE_HOME_WAIT:
      digitalWrite(SP_MOT_PH_PIN, PH_REV);
      analogWrite(SP_MOT_EN_PIN, 0);
      digitalWrite(FR_MOT_PH_PIN, PH_REV);
      analogWrite(FR_MOT_EN_PIN, 0);

      if (reverse==NOT_PRESSED) {
        state = STATE_IDLE;
        Serial.println("State => Idle");
      }
      break;

    case STATE_OBSTRUCT:
      digitalWrite(SP_MOT_PH_PIN, PH_REV);
      analogWrite(SP_MOT_EN_PIN, 0);
      digitalWrite(FR_MOT_PH_PIN, PH_REV);
      analogWrite(FR_MOT_EN_PIN, 0);

      if ((reverse==NOT_PRESSED) & (forward == NOT_PRESSED)) {
        state = STATE_IDLE;
        Serial.println("State => Idle");
      }
      break;

    default:
      state = STATE_IDLE;
      break;

  }  // switch(state)

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
// ADC * 1.623A/V * 3.3Vref / 1024 = 5.23mA/bit resolution
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
