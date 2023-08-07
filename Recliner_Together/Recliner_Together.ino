// Motor 1 (Seat) Variables:
int SeatA=6;
int SeatB=5;
int SeatFWD_A=0;
int SeatFWD_B=255;     // Seat Forward Speed 0-255:
int SeatREV_A=255;    // Seat Reverse Speed 0-255:
int SeatREV_B=0;
// Motor 2 (Foot) variables:
int FootA=9;
int FootB=10;
int FootFWD_A=0;
int FootFWD_B=255;   // Foot forward speed 0-255:
int FootREV_A=255;   // Foot Reverse speed 0-255:
int FootREV_B=0;
// Controls:
int FWDpin=2;
int FWD;
int REVpin=4;
int REV;
int WFWDpin=12;
int WFWD;
int WREVpin=13;
int WREV;
// Limit switches:
int SeatOUTpin=3;
int SeatOUT;
int SeatINpin=7;
int SeatIN;
int FootOUTpin=11;
int FootOUT;
int FootINpin=8;
int FootIN;
int MotorStop=0;


void setup() {
  // put your setup code here, to run once:
Serial.begin(9600);
pinMode(SeatA,OUTPUT);
pinMode(SeatB,OUTPUT);
pinMode(FootA,OUTPUT);
pinMode(FootB,OUTPUT);
pinMode(FWDpin,INPUT_PULLUP);
pinMode(REVpin,INPUT_PULLUP);
pinMode(WFWDpin,INPUT_PULLUP);
pinMode(WREVpin,INPUT_PULLUP);
pinMode(SeatOUTpin,INPUT_PULLUP);
pinMode(SeatINpin,INPUT_PULLUP);
pinMode(FootOUTpin,INPUT_PULLUP);
pinMode(FootINpin,INPUT_PULLUP);

}

void loop() {
  // 
  // Read Inputs:
FWD=digitalRead(FWDpin);
REV=digitalRead(REVpin);
WFWD=digitalRead(WFWDpin);
WREV=digitalRead(WREVpin);
SeatOUT=digitalRead(SeatOUTpin);
SeatIN=digitalRead(SeatINpin);
FootOUT=digitalRead(FootOUTpin);
FootIN=digitalRead(FootINpin);  
analogWrite(SeatA,MotorStop);
analogWrite(SeatB,MotorStop);
analogWrite(FootA,MotorStop);
analogWrite(FootB,MotorStop);

// Forward Button is Pressed:
while (FWD==0||WFWD==0){
  FWD=digitalRead(FWDpin);
  WFWD=digitalRead(WFWDpin);
  SeatOUT=digitalRead(SeatOUTpin);
  FootOUT=digitalRead(FootOUTpin);
  if(SeatOUT==0){
    analogWrite(SeatA,SeatFWD_A);
    analogWrite(SeatB,SeatFWD_B);
  }
  else {
    analogWrite(SeatA,MotorStop);
    analogWrite(SeatB,MotorStop);
  }
  if(FootOUT==0){
    analogWrite(FootA,FootFWD_A);
    analogWrite(FootB,FootFWD_B);
  }
  else {
    analogWrite(FootA,MotorStop);
    analogWrite(FootB,MotorStop);
  }
}

// Reverse button is pressed:
while(REV==0||WREV==0){
  REV=digitalRead(REVpin);
  WREV=digitalRead(WREVpin);
  SeatIN=digitalRead(SeatINpin);
  FootIN=digitalRead(FootINpin);
    if(FootIN==0){
    analogWrite(FootA,FootREV_A);
    analogWrite(FootB,FootREV_B);
  }
  else{
    analogWrite(FootA,MotorStop);
    analogWrite(FootB,MotorStop);
  }
  if(SeatIN==0){
    analogWrite(SeatA,SeatREV_A);
    analogWrite(SeatB,SeatREV_B);
  }
  else{
    analogWrite(SeatA,MotorStop);
    analogWrite(SeatB,MotorStop);
  }
}

}
