/*
   Servo Signal Generator
   Uses input capture to decode incoming signal
   Then generators and output to command the servo to move to position
   for Fubarino Mini
*/


//These lines are for the input capture for pwm read off RC
#define RC_INPUT_STR 0
#define RC_INPUT_COUNT 1
volatile uint16_t pulseHighTime[RC_INPUT_COUNT];
volatile uint16_t pulseLowTime[RC_INPUT_COUNT];


//const int cmdPosPin = A9;
const int posPin = A7;
const int minTurnPos = 23;
const int maxTurnPos = 1000;
const int maxVel = 200;

int cmdPosPot;
int curPot;

int pwmVal;
int dr1;
int pwm1;
int en1;
int PINdr1 = 2;
int PINpwm1 = 4;
int PINen1 = 3;



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
    if (digitalRead(RC_INPUT_STR) == HIGH)
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
  int speed;
  int diff = cmdPos - servoPos; // if 0 we're on target
  bool dir = (diff < 0) ? 1 : 0;

  diff = abs(diff);
  if (diff > maxVel ) {
    diff = maxVel;
  }
  speed = diff;

  // if in a max bad place make speed zero
  // Allow movement in direction less than the max bad place
  if (servoPos > maxTurnPos && dir == 0)
  {
    stop();
  } else  if (servoPos < minTurnPos && dir == 1) {
    stop();
  } else {
    Serial.println("GO");
    //move normally
    digitalWrite(PIN_LED1, LOW);
    digitalWrite(PINen1, 1);
    digitalWrite(PINdr1, dir);
    //  analogWrite(PINpwm1, speed);

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
  IC1CONbits.ICTMR = 1; // Set to user Timer2
  IC1CONbits.ON = 1;    // Turn IC1 on


  /*We're using timer2 for the input capture. This shouldn't interfere with pwm
    output, which uses timers 3-5.
  */
  PR2 = 0xFFFF;         // This tells timer 2 to count up to 0xFFFF, after which it will restart at 0
  T2CONbits.TCKPS = 6;  // 1:64 prescale, which means 48MHz/64 or 0.75MHz clock rate
  T2CONbits.TON = 1;    // Turn on Timer2

  pinMode(RC_INPUT_STR, INPUT);
  //pinMode(PIN_LED1, OUTPUT);
  //digitalWrite(PIN_LED1, LOW);

  //these lines set up the interrupt functions to trigger
  setIntVector(_INPUT_CAPTURE_1_VECTOR, InputCaptureSTR_ISR);
  setIntPriority(_INPUT_CAPTURE_1_VECTOR, 4, 0);
  clearIntFlag(_INPUT_CAPTURE_1_IRQ);
  setIntEnable(_INPUT_CAPTURE_1_IRQ);

}

void loop() {
  //curPot = analogRead(posPin); //where i am
  unsigned long STR_VAL = pulseRead(0); // Read pulse width of

  cmdPosPot = map(STR_VAL, 1000, 2000, 0, 1023);
  if (cmdPosPot < 0) {
    cmdPosPot = 0;
  }
  if (cmdPosPot > 1023) {
    cmdPosPot = 1023;
  }

  int error = cmdPosPot - curPot;

  if (abs(error ) > 1 ) {
    move(cmdPosPot, curPot);
  }
  Serial.print("STR_VAL: ");
  Serial.println(STR_VAL);
  Serial.print("cmdPosPot: ");
  Serial.print(cmdPosPot);
  Serial.print(" curPot: ");
  Serial.print(curPot);
  Serial.print(" error: ");
  Serial.println(error);



  delay(250);

}
