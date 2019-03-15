
const int cmdPosPin = A9;
const int posPin = A7;
const int minTurnPos = 23;
const int maxTurnPos = 1000;

int cmdPosPot;
int curPot;

int pwmVal;
int dr1;
int pwm1;
int en1;
int PINdr1 = 2;
int PINpwm1 = 4;
int PINen1 = 3;


void setup() {
  Serial.begin(9600);
  pinMode(PIN_LED1, OUTPUT);
  digitalWrite(PIN_LED1, LOW);
  pinMode(PINdr1, OUTPUT);
  pinMode(PINen1, OUTPUT);
  stop();
}

void move(int cmdPos, int servoPos) {
  int speed;
  int diff = cmdPos - servoPos; // if 0 we're on target
  bool dir = (diff < 0) ? 0 : 1;
  Serial.println(dir);

  diff = abs(diff);
  int maxVel = 200;

  if (diff > maxVel ) {
    diff = maxVel;
  }
  speed = diff;
  Serial.println(speed);
  // if in a max bad place make speed zero 
  // Allow movement in direction less than the max bad place
  if (servoPos > maxTurnPos)
  {
    // only command to turn away
    if (cmdPos < maxTurnPos) {
      digitalWrite(PIN_LED1, LOW);
      digitalWrite(PINen1, 1);
      digitalWrite(PINdr1, dir);
      analogWrite(PINpwm1, speed);
    }
    return;
  }
  
  // if in a min bad place make speed zero 
  // Allow movement in direction greater than the min bad place
  if (servoPos < minTurnPos) {
    if ( cmdPos > minTurnPos) {
      digitalWrite(PIN_LED1, LOW);
      digitalWrite(PINen1, 1);
      digitalWrite(PINdr1, dir);
      analogWrite(PINpwm1, speed);
    }
    return;
  }
  //move normally
  digitalWrite(PIN_LED1, LOW);
  digitalWrite(PINen1, 1);
  digitalWrite(PINdr1, dir);
  analogWrite(PINpwm1, speed);
}

void stop() {
  digitalWrite(PINen1, 0);
  digitalWrite(PINdr1, 1);
  analogWrite(PINpwm1, 0);
}

void loop() {
  cmdPosPot = analogRead(cmdPosPin); //where i want to be
  curPot = analogRead(posPin); //where i am
  int error = cmdPosPot - curPot;
  if (abs(error ) > 1 ) {
    move(cmdPosPot, curPot);
  }

  Serial.print("cmdPosPot: ");
  Serial.print(cmdPosPot);
  Serial.print(" curPot: ");
  Serial.print(curPot);
  Serial.print(" error: ");
  Serial.println(error);
  delay(10);
}
