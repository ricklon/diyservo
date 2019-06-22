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

const int midPotVal=480;
const int leftPotLimit=365;
const int rightPotLimit=595;

const int timeStep=250;

const int Kd_scaler=1000/timeStep;

const int minTurnPos = 23;
const int maxTurnPos = 1000;
const int maxVel = 125;
const int deadband=30;
const int maxError=500;

const int GainDivisor=205;

//servo control range. outside of this is considered bad data
int rcMax = 2100;
int rcMin =  900;

struct {
  int potValue;
  int rcInput;
  int current_error;
  int previous_error;
  int previous_potValue;
  int command;
  int output_value;
  int output_unclipped;
  bool output_dir;
  bool error_deadband;
  bool out_of_bounds;
  bool stop;
} state;

struct {
  int K;
  int Kd;
} PD_vars;

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


void move() {
  state.previous_error=state.current_error;
  state.current_error = state.command-state.potValue; // if 0 we're on target
  
  //state.current_error = abs(current_error);
  
  if(abs(state.current_error)<deadband){
    state.error_deadband=1;
    return;
  }
  state.output_value= (PD_vars.K*state.current_error)>>3-((PD_vars.Kd*(state.potValue-state.previous_potValue))>>7);//dividing the kd value by 128
	//so we avoid fp 

  state.output_unclipped=state.output_value;
  state.output_value=(state.output_value>maxVel)?maxVel:state.output_value;
  state.output_value=(state.output_value<-maxVel)?-maxVel:state.output_value;
  state.output_dir = (state.output_value< 0) ? 1 : 0;


  // if in a max bad place make speed zero
  // Allow movement in direction less than the max bad place
  if ((state.potValue>rightPotLimit) && (state.command>state.potValue))
  {
    state.out_of_bounds=1;
    stop();
  } else  if ((state.potValue<leftPotLimit) && (state.command<state.potValue)) {
    stop();
    state.out_of_bounds=1;
  } else {
    digitalWrite(PIN_LED1, LOW);
    digitalWrite(PINen1, 1);
    digitalWrite(PINdr1, 1-state.output_dir);
    analogWrite(PINpwm1, abs(state.output_value));

  }
}

void stop() {
  digitalWrite(PINen1, 0);
  digitalWrite(PINdr1, 1);
  // analogWrite(PINpwm1, 0);
}

void print_status(){

  Serial.println("KVariables:");
  Serial.print("\tK: ");
  Serial.println(PD_vars.K);
  Serial.print("\tKd: ");
  Serial.println(PD_vars.Kd);
  Serial.println();

  Serial.print(" potvalue: ");
  Serial.print(state.potValue);

  Serial.print(" RC Input: ");
  Serial.print(state.rcInput);

  Serial.print(" command: ");
  Serial.println(state.command);

  Serial.print(" error: ");
  Serial.print(state.current_error);

  Serial.print(" previous error: ");
  Serial.print(state.previous_error);

  Serial.print(" input_diff: ");
  Serial.println(state.potValue-state.previous_potValue);

  Serial.print(" output unclipped: ");
  Serial.print(state.output_unclipped);

  Serial.print(" output value: ");
  Serial.print(state.output_value);

  Serial.print(" output direction: ");
  Serial.println(state.output_dir);
  
  if(state.error_deadband){
  	Serial.println("Error within deadband");
  }

  if(state.out_of_bounds){
  	Serial.println("out of bounds");
  }

  if(state.stop){
  	Serial.println("stopped");
  }
  Serial.println("--------------");
  Serial.println();
  
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


}

void loop() {
  state.previous_potValue =state.potValue;
  state.potValue = analogRead(posPin); 
  PD_vars.K = analogRead(gainPin);
  PD_vars.Kd = analogRead(scalePin)*Kd_scaler;

  state.rcInput= pulseRead(0); // Read pulse width of

  //When the RC controller is off it goes to a max value.
  //What should the safety value be when controller is off?

  // Don't narrow the rc signal but narrow the interpretation of where to go.

  if (state.rcInput> rcMax || state.rcInput< rcMin) {
    state.rcInput=1500;
  }
  state.rcInput=(state.rcInput>2000)?2000:state.rcInput;
  state.rcInput=(state.rcInput<1000)?1000:state.rcInput;
	
  state.command = map(state.rcInput, 1000, 2000, leftPotLimit, rightPotLimit);
 
  state.error_deadband=0;
  state.out_of_bounds=0;
  state.stop=0;

  move();
  print_status();
  delay(timeStep);

}
