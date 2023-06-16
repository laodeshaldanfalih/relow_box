#include <NewPing.h>

#define TRIGGER_PIN 14   // Arduino pin tied to trigger pin on the ultrasonic sensor.
#define ECHO_PIN 12      // Arduino pin tied to echo pin on the ultrasonic sensor.
#define MAX_DISTANCE 200 // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.

NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE); // NewPing setup of pins and maximum distance.

// LCD
#include <LiquidCrystal_I2C.h>

// set the LCD number of columns and rows
int lcdColumns = 20;
int lcdRows = 4;

// set LCD address, number of columns and rows
// if you don't know your display address, run an I2C scanner sketch
LiquidCrystal_I2C lcd(0x27, lcdColumns, lcdRows);

// Motor
#define ENCA 2 // YELLOW
#define ENCB 4 // WHITE
#define PWM 15
#define IN2 18
#define IN1 19

volatile int posi = 0; // specify posi as volatile: https://www.arduino.cc/reference/en/language/variables/variable-scope-qualifiers/volatile/
long prevT = 0;
float eprev = 0;
float eintegral = 0;
int target = 0;
int counter;
void setup()
{
  Serial.begin(9600);
  pinMode(ENCA, INPUT);
  pinMode(ENCB, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCA), readEncoder, RISING);

  pinMode(PWM, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);

  Serial.println("target pos");

  // initialize LCD
  lcd.init();
  // turn on LCD backlight
  lcd.backlight();
}

void loop()
{
  int jarak = sonar.ping_cm();

  // set target position
  if (jarak < 20 && jarak != 0)
  {
    counter++;
  }

  if (counter == 1)
  {
    motorTarget(5000);
  }
  else if (counter == 2)
  {
    motorTarget(0);
  }
  else if (counter == 3)
  {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    counter = 0;
  }

  Serial.println(counter);
  lcd.setCursor(0, 1);
  lcd.print("Welcome to Relow Box");
  lcd.clear();
  lcd.setCursor(9, 2);
  lcd.print("HCSR: ");
  lcd.clear();
  lcd.setCursor(16, 2);
  lcd.print(jarak);
  lcd.clear();
}

void motorTarget(int targets)
{
  int target = targets;
  // PID constants
  float kp = 0.6;
  float kd = 0.012;
  float ki = 0.0;

  // time difference
  long currT = micros();
  float deltaT = ((float)(currT - prevT)) / (1.0e6);
  prevT = currT;

  // Read the position
  int pos = 0;
  noInterrupts(); // disable interrupts temporarily while reading
  pos = posi;
  interrupts(); // turn interrupts back on

  // error
  int e = pos - target;

  // derivative
  float dedt = (e - eprev) / (deltaT);

  // integral
  eintegral = eintegral + e * deltaT;

  // control signal
  float u = kp * e + kd * dedt + ki * eintegral;

  // motor power
  float pwr = fabs(u);
  if (pwr > 100)
  {
    pwr = 100;
  }

  // motor direction
  int dir = 1;
  if (u < 0)
  {
    dir = -1;
  }

  // signal the motor
  setMotor(dir, pwr, PWM, IN1, IN2);

  // store previous error
  eprev = e;
}

void setMotor(int dir, int pwmVal, int pwm, int in1, int in2)
{
  analogWrite(pwm, pwmVal);
  if (dir == 1)
  {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  }
  else if (dir == -1)
  {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  }
  else
  {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
  }
}

void readEncoder()
{
  int b = digitalRead(ENCB);
  if (b > 0)
  {
    posi++;
  }
  else
  {
    posi--;
  }
}
