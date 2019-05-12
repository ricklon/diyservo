/*
  Read RC Signal and move servo
    Uses input capture to decode incoming signal
   Then generators and output to command the servo to move to position
   for Fubarino Mini
*/
#include <Arduino.h>


//These lines are for the input capture for pwm read off RC
#define RC_INPUT_STR_PIN 0
#define RC_INPUT_COUNT 1
volatile uint16_t pulseHighTime[RC_INPUT_COUNT];
volatile uint16_t pulseLowTime[RC_INPUT_COUNT];


//const int cmdPosPin = A9;
const int posPin = A7;
const int gainPin = A8;
const int scalePin = A9;
const int offsetPin = A10;

const int minTurnPos = 23;
const int maxTurnPos = 1000;
const int maxVel = 125;
const int deadband=30;
const int maxError=500;

const int GainDivisor=205;

//servo control range. outside of this is considered bad data
int rcMax = 2100;
int rcMin =  900;


int cmdPosPot;
int curPot;
int offSet;
int gGain;
int gScale = 20;

int binsize;

int pwmVal;
int dr1;
int pwm1;
int en1;
const int PINdr1 = 2;
const int PINpwm1 = 4;
const int PINen1 = 3;



//This function pulls the data being populated by the input capture interrupts.
//it corrects for the timer restarting.
inline int pulseRead(int RCindex) {
  return (pulseHighTime[RCindex] > 0) ? (int)(4.0 * pulseHighTime[RCindex] / 3.0) : (int)(4.0 * pulseHighTime[RCindex] / 3.0 + 0xFFFF);
}
//inline int pulseRead(int RCindex){return (int)(0.8*pulseHighTime[RCindex]);}

//interrupt service routine for first input capture module
void __USER_ISR InputCaptureSTR_ISR(void) {
  static uint16_t risingEdgeTime = 0;
  static uint16_t fallingEdgeTime = 0;

  clearIntFlag(_INPUT_CAPTURE_1_IRQ);
  if (IC1CONbits.ICBNE == 1)
  {
    if (digitalRead(RC_INPUT_STR_PIN) == HIGH)
    {
      risingEdgeTime = IC1BUF;
      pulseLowTime[0] = risingEdgeTime - fallingEdgeTime;
    }
    else
    {
      fallingEdgeTime = IC1BUF;
      pulseHighTime[0] = fallingEdgeTime - risingEdgeTime;
    }
  }
}


void move(int cmdPos, int servoPos) {
  int velocity;
  int error = cmdPos - servoPos; // if 0 we're on target
  bool dir = (error < 0) ? 1 : 0;
  Serial.print("dir: ");
  Serial.println(dir);
  
  error = abs(error);
  if (error > maxError) {
    error = maxError;
  }
  if(error<deadband){
    error=deadband;
  }
  velocity= gGain*(1+(error-deadband)/binsize)/GainDivisor; //scale factor

  velocity=(velocity>maxVel)?maxVel:velocity;


  // if in a max bad place make speed zero
  // Allow movement in direction less than the max bad place
  if ((servoPos > maxTurnPos) && (cmdPos>servoPos))
  {
    Serial.print("1111111111");
    stop();
  } else  if ((servoPos < minTurnPos) && (cmdPos<servoPos)) {
    Serial.print("1111111111");
    stop();
  } else {
    Serial.print("GO: ");
    Serial.println(velocity);
    //move normally
    digitalWrite(PIN_LED1, LOW);
    digitalWrite(PINen1, 1);
    digitalWrite(PINdr1, 1-dir);
    analogWrite(PINpwm1, velocity);

  }
}

void stop() {
  digitalWrite(PINen1, 0);
  digitalWrite(PINdr1, 1);
  // analogWrite(PINpwm1, 0);
}



void setup() {
  Serial.begin(9600);
  pinMode(PINdr1, OUTPUT);
  pinMode(PINen1, OUTPUT);
  stop();

  mapPps(0, PPS_IN_IC1);

  //setup input capture modules one and two
  IC1CON = 0;
  IC1CONbits.ICM = 1;   // Capture an interrupt on every rising and falling edge
  IC1CONbits.ICTMR = 0; // Set to user Timer3
  IC1CONbits.ON = 1;    // Turn IC1 on


  /*We're using timer2 for the input capture. This shouldn't interfere with pwm
    output, which uses timers 3-5.
  */
  PR3 = 0xFFFF;         // This tells timer 3 to count up to 0xFFFF, after which it will restart at 0
  T3CONbits.TCKPS = 6;  // 1:64 prescale, which means 48MHz/64 or 0.75MHz clock rate
  T3CONbits.TON = 1;    // Turn on Timer3

  pinMode(RC_INPUT_STR_PIN, INPUT);
  //pinMode(PIN_LED1, OUTPUT);
  //digitalWrite(PIN_LED1, LOW);

  //these lines set up the interrupt functions to trigger
  setIntVector(_INPUT_CAPTURE_1_VECTOR, InputCaptureSTR_ISR);
  setIntPriority(_INPUT_CAPTURE_1_VECTOR, 4, 0);
  clearIntFlag(_INPUT_CAPTURE_1_IRQ);
  setIntEnable(_INPUT_CAPTURE_1_IRQ);

  binsize=(maxVel-deadband)/gScale;
}

void loop() {
  curPot = analogRead(posPin); //where i am
  gGain = analogRead(gainPin);
  gScale = analogRead(scalePin);  
  offSet = analogRead(offsetPin) - 512;


  unsigned long STR_VAL = pulseRead(0); // Read pulse width of

  //When the RC controller is off it goes to a max value.
  //What should the safety value be when controller is off?

  int str_clip=STR_VAL;
  int curPotOffSet = curPot + offSet;  

  // Don't narrow the rc signal but narrow the interpretation of where to go.
  int safeLOW = 1300;
  int safeHIGH = 1700;

  if (STR_VAL > rcMax || STR_VAL < rcMin) {
    str_clip=1500;
  }
  cmdPosPot = map(str_clip, rcMin, rcMax, 0, 1023);
  if (cmdPosPot < 0) {
    cmdPosPot = 0;
  }
  if (cmdPosPot > 1023) {
    cmdPosPot = 1023;
  }

  int error = cmdPosPot - curPotOffSet;

  if (abs(error ) > deadband ) {
    move(cmdPosPot, curPotOffSet);
  }
  Serial.print(" gain: ");
  Serial.print(gGain);

  Serial.print(" scale: ");
  Serial.print(gScale);

  Serial.print(" offset: ");
  Serial.print(offSet);

  Serial.print(" STR_VAL: ");
  Serial.println(STR_VAL);
  Serial.print("cmdPosPot: ");
  Serial.print(cmdPosPot);
  Serial.print(" curPot: ");
  Serial.print(curPot);
  Serial.print(" error: ");
  Serial.println(error);

  delay(250);

}
