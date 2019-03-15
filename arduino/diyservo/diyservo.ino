
const int goalPin = A9;
const int posPin = A7;
const int minTurnPos = 23;
const int maxTurnPos = 1000;

int goalPot;
int posPot;

int pwmVal;
int dr1;
int pwm1;
int en1;
int PINdr1 = 2;
int PINpwm1 = 4;
int PINen1 = 3;
//FUbarino Mini PIN 1 is LED_PIN1

void setup() {
  Serial.begin(9600);
  pinMode(PIN_LED1, OUTPUT);
  digitalWrite(PIN_LED1, LOW);
  pinMode(PINdr1, OUTPUT);
  pinMode(PINen1, OUTPUT);
  stop();
}

void forward(int speed, int curPos) {
  if (curPos < maxTurnPos) {
    digitalWrite(PIN_LED1, LOW);
    digitalWrite(PINen1, 1);
    digitalWrite(PINdr1, 0);
    analogWrite(PINpwm1, speed);
  }
  else {
    digitalWrite(PIN_LED1, HIGH);
    stop();
  }
}

void reverse( int speed, int curPos) {
  if (curPos > minTurnPos) {
    digitalWrite(PIN_LED1, LOW);
    digitalWrite(PINen1, 1);
    digitalWrite(PINdr1, 1);
    analogWrite(PINpwm1, speed);
  }
  else {
    digitalWrite(PIN_LED1, HIGH);
    stop();
  }
}

void stop() {
  digitalWrite(PINen1, 0);
  digitalWrite(PINdr1, 1);
  analogWrite(PINpwm1, 0);

}
void loop() {
  goalPot = analogRead(goalPin); //where i want to be
  posPot = analogRead(posPin); //where i am
  int speed = 0;

  Serial.print("goalPot: ");
  Serial.println(goalPot);
  Serial.println(goalPot);

  if (goalPot > 516 ) {
    speed = map(goalPot, 517, 1023, 0, 255);
    forward(speed, posPot);
  }
  if (goalPot >= 508 && goalPot <= 516)
  {
    digitalWrite(PIN_LED1, HIGH);
    stop();
    delay(50);
    digitalWrite(PIN_LED1, LOW);
  }
  if (goalPot < 508)
  {
    speed = map(goalPot, 507, 0, 0, 255);
    reverse(speed, posPot);
  }


  Serial.print("goalPot: ");
  Serial.print(goalPot);
  Serial.print(" posPot: ");
  Serial.print(posPot);
  Serial.print(" speed: ");
  Serial.println(speed);

  delay(100);

}
