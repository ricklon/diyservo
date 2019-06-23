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
const int posPin = A7; //pin for servo position potentiometer input
const int gainPin = A8; //potentiometer input pin for K term
const int scalePin = A9;//potentiometer input pin for Kd term
const int offsetPin = A10;//not used

const int midPotVal=480;
const int leftPotLimit=345;
const int rightPotLimit=615;

const int timeStep=100; //milliseconds

const int Kd_scaler=1000/timeStep; //derivative term is evaluated as kd*(error-previous_error)/time, with time in seconds. 
				   //This is equivalent to multiplying by Kd_scaler parameter, which is equal to 1/(timestep/1000)
const int maxVel = 125; //this value is a guess. We probably don't want to go all the way to 255
const int deadband=20; //if abs(error) is less than deadband, we don't do anything. This probably creates wierd behavior with the control loop

//servo control range. outside of this is considered bad data
int rcMax = 2100;
int rcMin =  900;

struct {//state variables-- populated every iteration by main loop polling of RC and servo pot, and then by calculations in the move() function
  int potValue; //potentiometer value
  int rcInput; //input RC pwm value
  int current_error; //error between mapped RC input and pot value
  int previous_error; //error during last calculation -- we're not currently using this for the D-term
  int previous_potValue; //previous potentiometer value --this is what we're using to compute D-term, to avoid impulses due to setpoint change.
			 //See blog posts "beyond beginner's PID" by brett beauregard for explanation. Although, I suspect that since our setpoint 
			 //is hypothetically relatively continuous (though maybe not if our framerate is too low) we might want to be computing 
			 //the D-term in the traditional manner
  int command; //RC input mapped to range of the allowable potentiometer values
  int output_value; //computed output to the motor controller
  int output_unclipped; //output after it's been clipped to allowable range
  bool output_dir; // output to motor controller for direction of motor
  bool error_deadband; //set to true within move() if the error was within deadband
  bool out_of_bounds; //set to true within move() if position pot is out of bounds and we're trying to go more out of bounds
  bool stop; //set to true within move() if stop is called (I think)
} state;

struct {//gain variables for control loop-- these get modified in the main loop at every iteration
  int K;
  int Kd;
} PD_vars; 

const int PINdr1 = 2;  //output pin for motor controller direction
const int PINpwm1 = 4; //output pin for motor controller value (range 0-255) 
const int PINen1 = 3;  //output pin for motor controller enable



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
  
  if(abs(state.current_error)<deadband){ //this probably creates problems, but we need to leave it in or we might burn something out
    stop();
    state.error_deadband=1;
    return;
  }
  long d_term=(PD_vars.Kd*(state.potValue-state.previous_potValue)); //this is the derivative term
  //state.output_value= (PD_vars.K*state.current_error)>>7+(d_term>>10);//dividing the kd value by 128
	//so we avoid fp 
  state.output_value= (PD_vars.K*state.current_error)>>7;//+(d_term>>10);//dividing the kd value by 128
  state.output_unclipped=state.output_value;
  state.output_value=(state.output_value>maxVel)?maxVel:state.output_value;
  state.output_value=(state.output_value<-maxVel)?-maxVel:state.output_value;
  state.output_dir = (state.output_value< 0) ? 1 : 0;


  // if in a max bad place make speed zero
  // Allow movement in direction less than the max bad place
  //NOTE: since we're mapping RC signals only to allowable range, this should never happen
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
  state.potValue = analogRead(posPin);  //get value of position potentiometer
  PD_vars.K = analogRead(gainPin); //get values from K, Kd
  PD_vars.Kd = analogRead(scalePin)*Kd_scaler; //scale Kd value according to timestep

  state.rcInput= pulseRead(0); // Read pulse width of RC signal

  //if RC signal is not in range of accepted values, assume it is centered
  if (state.rcInput> rcMax || state.rcInput< rcMin) {
    state.rcInput=1500;
  }
  //clip RC signal so it's symmetrical
  state.rcInput=(state.rcInput>1980)?1980:state.rcInput;
  state.rcInput=(state.rcInput<1020)?1020:state.rcInput;
	
  //map RC signal to same range as potentiometer inputs
  state.command = map(state.rcInput, 1000, 1980, leftPotLimit, rightPotLimit);
 
  //clear these parameters in case they get populated in the move() function
  state.error_deadband=0;
  state.out_of_bounds=0;
  state.stop=0;

  move();
  print_status();
  delay(timeStep); //this is not correct-- we should be looping continuously, and checking if current time is at least {timestep} since last calculation. If yes, then compute move.

}
