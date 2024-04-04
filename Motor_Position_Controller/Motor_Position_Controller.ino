// PID motor position control.
// Thanks to Brett Beauregard for his nice PID library http://brettbeauregard.com/blog/2011/04/improving-the-beginners-pid-introduction/

//#include "C:\Git Repositories\PID-Motor-Controller\PinChangeInt\PinChangeInt.h"
//#include "C:\Git Repositories\PID-Motor-Controller\Arduino-PID-Library\PID_v1.h"

#include <PID_v1.h>

// Define the connections to the motor and the encoder
const int motorPinA = 8;  // Motor driver input A, for direction
const int motorPinB = 9;  // Motor driver input B, for PWM signal
const int encoderPinA = 2; // Encoder input A
const int encoderPinB = 3; // Encoder input B (if needed for your encoder)

double Setpoint, Input, Output;
double Kp = 2, Ki = 0.001, Kd = 1; // These will need to be tuned for your system


// PID controller
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

// Encoder position, should be changed by the encoder interrupt
volatile long encoderPosition = 0;

// Interrupt service routine for the encoder
void encoderInterrupt() {
  // Implementation of this function will depend on your specific encoder
  if(Output > 0){
    encoderPosition = encoderPosition + 1;
  }
  else{
    encoderPosition = encoderPosition - 1;
  }
  //encoderPosition++;
}

void setup() {
  // Start serial communication for debugging
  Serial.begin(9600);

  // Set up motor driver pins
  pinMode(motorPinA, OUTPUT);
  pinMode(motorPinB, OUTPUT);

  // Set up the encoder pin
  pinMode(encoderPinA, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(encoderPinA), encoderInterrupt, RISING);

  // Initialize the PID controller
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(-40, 40); // Depending on motor controller and motor

  // Set an initial setpoint
  Setpoint = 2000;
}

void loop() {
  Input = encoderPosition;
  myPID.Compute();

  // Apply PID output to motor
  if (Output > 0) {
    digitalWrite(motorPinA, LOW);
    analogWrite(motorPinB, Output);
  } else {
    digitalWrite(motorPinA, HIGH);
    analogWrite(motorPinB, -Output); // Invert Output for reverse direction
  }

  // Check if the current position is within a reasonable range of the setpoint
  if((Setpoint - encoderPosition <= 100) && (Output == 40)){
    Output = -40;
    Setpoint = 0;
  }
  if((encoderPosition - Setpoint <= 100) && (Output == -40)){
    Output = 40;
    Setpoint = 12239;
  }
  /*
  if (abs(Setpoint - encoderPosition) <= 100) { // 10 can be changed to your preferred tolerance
    // Randomize a new setpoint within a range
    //Setpoint = random(100, 2000); // For example, randomize between 100 and 2000
    if(encoderPosition > Setpoint){
      Setpoint = 0;
      Output = -40;
    }
    else{
      Setpoint = 2000;
      Output = 40;
    }

  }
  */
  // Debugging Output
  Serial.print("Position: ");
  Serial.print(encoderPosition);
  Serial.print(" | Setpoint: ");
  Serial.print(Setpoint);
  Serial.print(" | Output: ");
  Serial.println(Output);

  //delay(100); // Just to make the Serial output less frequent
}

// Remember to seed the random number generator in setup() if not already done
/*
// Define PID variables
double Setpoint, Input, Output;
double Kp = 2.0, Ki = 0.5, Kd = 1.0; // These values will need to be tuned for your system

// Specify the links and initial tuning parameters
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

// This variable will be changed by the encoder interrupt
volatile long encoderPosition = 0;

// Interrupt service routine for the encoder
void encoderInterrupt() {
  // This will need to be adjusted based on your specific encoder
  if (digitalRead(encoderPinA) == digitalRead(encoderPinB)) {
    encoderPosition++;
  } else {
    encoderPosition--;
  }
}

void setup() {
  // Initialize the Serial port
  Serial.begin(9600);

  // Set up the motor driver pins
  pinMode(motorPinA, OUTPUT);
  pinMode(motorPinB, OUTPUT);

  // Set up the encoder pins
  pinMode(encoderPinA, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(encoderPinA), encoderInterrupt, CHANGE);
  // Use the line below if you need to handle both edges of the encoder A and B signals
  // attachInterrupt(digitalPinToInterrupt(encoderPinB), encoderInterrupt, CHANGE);

  // Initialize the PID controller
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(-255, 255); // Assuming the motor driver accepts values from -255 to 255

  // Define the Setpoint (desired position)
  Setpoint = 1000; // Example setpoint, you'll set this as needed
}

void loop() {
  Input = encoderPosition;
  myPID.Compute();

  // Apply the PID output to the motor
  if (Output > 0) {
    digitalWrite(motorPinA, HIGH);
    analogWrite(motorPinB, Output);
  } else {
    digitalWrite(motorPinA, LOW);
    analogWrite(motorPinB, -Output); // Invert the output if it's negative
  }

  // For debugging, you can print the encoder position and PID output to the Serial Monitor
  Serial.print("Position: ");
  Serial.print(encoderPosition);
  Serial.print(" Output: ");
  Serial.println(Output);
  delay(10); // Just to make the printed data readable
}
*/
/*
#include "PID_v1.h"
#include "PinChangeInt.h"
#define encodPinA1      3                       // Quadrature encoder A pin
#define encodPinB1      2                       // Quadrature encoder B pin
#define M1              9                       // PWM outputs to L298N H-Bridge motor driver module
#define M2              8

double kp = 5 , ki = 1 , kd = 0.01;             // modify for optimal performance
double input = 0, output = 0, setpoint = 0;
long temp;
volatile long encoderPos = 0;
PID myPID(&input, &output, &setpoint, kp, ki, kd, DIRECT);  // if motor will only run at full speed try 'REVERSE' instead of 'DIRECT'

void setup() {
  pinMode(encodPinA1, INPUT_PULLUP);                  // quadrature encoder input A
  pinMode(encodPinB1, INPUT_PULLUP);                  // quadrature encoder input B
  attachInterrupt(0, encoder, FALLING);               // update encoder position
  TCCR1B = TCCR1B & 0b11111000 | 1;                   // set 31KHz PWM to prevent motor noise
  myPID.SetMode(AUTOMATIC);
  myPID.SetSampleTime(1);
  myPID.SetOutputLimits(0, 25555);
  randomSeed(analogRead(0));
  Serial.begin (9600);                              // for debugging
}

void loop() {
  setpoint = analogRead(0) * 5;                       // modify to fit motor and encoder characteristics, potmeter connected to A0
  Serial.println(setpoint);
  //setpoint = 1200;
  input = 0 ;                                // data from encoder
  // Serial.println(encoderPos);                      // monitor motor position
  myPID.Compute();                                    // calculate new output
  pwmOut(output);                                     // drive L298N H-Bridge module
}

void pwmOut(int out) {                                // to H-Bridge board
  if (out > 0) {
    analogWrite(M1, out);                             // drive motor CW
    analogWrite(M2, 0);
  }
  else {
    analogWrite(M1, 0);
    analogWrite(M2, abs(out));                        // drive motor CCW
  }
}

void encoder()  {                                     // pulse and direction, direct port reading to save cycles
  if (PINB & 0b00000001)    encoderPos++;             // if(digitalRead(encodPinB1)==HIGH)   count ++;
  else                      encoderPos--;             // if(digitalRead(encodPinB1)==LOW)   count --;
}
*/
