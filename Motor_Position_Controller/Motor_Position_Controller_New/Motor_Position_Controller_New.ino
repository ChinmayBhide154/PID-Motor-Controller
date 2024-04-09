/*
#include <util/atomic.h> // For the ATOMIC_BLOCK macro

#define ENCA 2 // YELLOW
#define ENCB 3 // WHITE
#define PWM 5
#define IN1 4

volatile int posi = 0; // specify posi as volatile: https://www.arduino.cc/reference/en/language/variables/variable-scope-qualifiers/volatile/
long prevT = 0;
float eprev = 0;
float eintegral = 0;

int setpoints[] = {1, 100, 500, 1000, 1300, 1500, 1700, 2000};

void setup() {
  Serial.begin(9600);
  pinMode(ENCA,INPUT);
  pinMode(ENCB,INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCA),readEncoder,RISING);
  
  pinMode(PWM,OUTPUT);
  pinMode(IN1,OUTPUT);
  
  Serial.println("target pos");
}

void loop() {
  int counter = 0;
  if(abs(err) < 100){
    counter++;
  }
  Serial.println(counter);
  // Changing setpoint if error in range
  int target = setpoint[counter]
  // set target position
  //int target = 6120;
//int target = 250*sin(prevT/1e6);
  // PID constants
  float kp = 0.31;
  float kd = 0.2;
  float ki = 0.0;

  // time difference
  long currT = micros();
  float deltaT = ((float) (currT - prevT))/( 1.0e6 );
  prevT = currT;

  // Read the position in an atomic block to avoid a potential
  // misread if the interrupt coincides with this code running
  // see: https://www.arduino.cc/reference/en/language/variables/variable-scope-qualifiers/volatile/


  // Read the position in an atomic block to avoid a potential
  // misread if the interrupt coincides with this code running
  // see: https://www.arduino.cc/reference/en/language/variables/variable-scope-qualifiers/volatile/
  int pos = 0; 
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    pos = posi;
  }

  // error
  int e = target - pos;
  
  // derivative
  float dedt = (e-eprev)/(deltaT);

  // integral
  eintegral = eintegral + e*deltaT;

  // control signal
  float u = kp*e + kd*dedt + ki*eintegral;

  // motor power
  float pwr = fabs(u);
  if( pwr > 255 ){
    pwr = 255;
  }

  // motor direction
  int dir = 1;
  if(u<0){
    dir = -1;
  }

  // signal the motor
  setMotor(dir,pwr,PWM,IN1);


  // store previous error
  eprev = e;

  Serial.print(target);
  Serial.print(" ");
  Serial.print(pos);
  Serial.println();
  
  //Serial.println(pos);
}

void setMotor(int dir, int pwmVal, int pwm, int in1){
  if(dir == 1){ //fprward
    analogWrite(pwm,pwmVal);
    digitalWrite(in1,LOW);
  }
  else if(dir == -1){
    analogWrite(pwm,pwmVal);
    digitalWrite(in1,HIGH);

  }
  else{
      digitalWrite(pwm,LOW);

  }
}

void readEncoder(){
  int b = digitalRead(ENCB);
  if(b > 0){
    posi++;
  }
  else{
    posi--;
  }
}
*/
/*
#include <util/atomic.h> // For the ATOMIC_BLOCK macro

#define ENCA 2 // YELLOW
#define ENCB 3 // WHITE
#define PWM 5
#define IN1 6

volatile int posi = 0; // specify posi as volatile: https://www.arduino.cc/reference/en/language/variables/variable-scope-qualifiers/volatile/

void setup() {
  Serial.begin(9600);
  pinMode(ENCA,INPUT);
  pinMode(ENCB,INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCA),readEncoder,RISING);
  
  pinMode(PWM,OUTPUT);
  pinMode(IN1,OUTPUT);
}

void loop() {
  
  // Read the position in an atomic block to avoid a potential
  // misread if the interrupt coincides with this code running
  // see: https://www.arduino.cc/reference/en/language/variables/variable-scope-qualifiers/volatile/
  int pos = 0; 
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    pos = posi;
  }

  setMotor(1, 25, PWM, IN1);
  delay(200);
  Serial.println(pos);
  setMotor(-1, 25, PWM, IN1);
  delay(200);
  Serial.println(pos);
  setMotor(0, 25, PWM, IN1);
  delay(20);
  Serial.println(pos);
}

void setMotor(int dir, int pwmVal, int pwm, int in1){
  if(dir == 1){ //fprward
    analogWrite(pwm,pwmVal);
    digitalWrite(in1,LOW);
  }
  else if(dir == -1){
      analogWrite(pwm,pwmVal);
    digitalWrite(in1,HIGH);

  }
  else{
      digitalWrite(pwm,LOW);


  }
}

void readEncoder(){
  int b = digitalRead(ENCB);
  if(b > 0){
    posi++;
  }
  else{
    posi--;
  }
}
*/







//DEANS CODE
/*
#include <util/atomic.h> // For the ATOMIC_BLOCK macro

#define ENCA 2 // YELLOW
#define ENCB 3 // WHITE
#define PWM 5
#define IN1 6

volatile int posi = 0; // specify posi as volatile: https://www.arduino.cc/reference/en/language/variables/variable-scope-qualifiers/volatile/
long prevT = 0;
float eprev = 0;
float eintegral = 0;
volatile float e_D_Buffer[10] = {0.0};
const int e_D_Sample = 10;


void setup() {
  Serial.begin(9600);
  pinMode(ENCA,INPUT);
  pinMode(ENCB,INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCA),readEncoder,RISING);
  
  pinMode(PWM,OUTPUT);
  pinMode(IN1,OUTPUT);
  
  Serial.println("target pos");
}

void loop() {

  // set target position
 int target = 12000;
//int target = 250*sin(prevT/1e6);
  // PID constants
  float kp = 0.31;
  float kd = 0.2;
  float ki = 0.0;

  // time difference
  long currT = micros();
  float deltaT = ((float) (currT - prevT))/( 1.0e6 );
  prevT = currT;

  // Read the position in an atomic block to avoid a potential
  // misread if the interrupt coincides with this code running
  // see: https://www.arduino.cc/reference/en/language/variables/variable-scope-qualifiers/volatile/
  int pos = 0; 
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    pos = posi;
  }
  
  // error
  int e = target - pos;
  float dedt = (e-eprev)/(deltaT);
  // filter
  for (int i=0;i<e_D_Sample-1;i++){
    e_D_Buffer[i]=e_D_Buffer[i+1];
  }
  e_D_Buffer[e_D_Sample-1]=(e-eprev)/(deltaT);
  float weight=1.0;
  float sum=0.0;
  for(int i=0;i<e_D_Sample;i++){
    dedt+=e_D_Buffer[i]*weight;
    weight=weight*1.2;
  }
  dedt/=weight;
  // derivative
  //float dedt = (e-eprev)/(deltaT);

  // integral
  eintegral = eintegral + e*deltaT;

  // control signal
  float u = kp*e + kd*dedt + ki*eintegral;

  // motor power
  float pwr = fabs(u);
  if( pwr > 255 ){
    pwr = 255;
  }

  // motor direction
  int dir = 1;
  if(u<0){
    dir = -1;
  }

  // signal the motor
  setMotor(dir,pwr,PWM,IN1);


  // store previous error
  eprev = e;

  Serial.print(target);
  Serial.print(" ");
  Serial.print(pos);
  Serial.println();
}

void setMotor(int dir, int pwmVal, int pwm, int in1){
  if(dir == 1){ //fprward
    analogWrite(pwm,pwmVal);
    digitalWrite(in1,LOW);
  }
  else if(dir == -1){
    analogWrite(pwm,pwmVal);
    digitalWrite(in1,HIGH);

  }
  else{
      digitalWrite(pwm,LOW);

  }
}

void readEncoder(){
  int b = digitalRead(ENCB);
  if(b > 0){
    posi++;
  }
  else{
    posi--;
  }
}
*/

#include <util/atomic.h> // For the ATOMIC_BLOCK macro

#define ENCA 2 // YELLOW
#define ENCB 3 // WHITE
#define PWM 5
#define IN1 4

volatile int posi = 0; // specify posi as volatile: https://www.arduino.cc/reference/en/language/variables/variable-scope-qualifiers/volatile/
long prevT = 0;
float eprev = 0;
float eintegral = 0;
int counter = 0;

int setpoints[] = {100, 200, 300, 400, 500, 400, 300, 200, 100};
int setpointReached = 0;

void setup() {
  Serial.begin(9600);
  pinMode(ENCA,INPUT);
  pinMode(ENCB,INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCA),readEncoder,RISING);
  
  pinMode(PWM,OUTPUT);
  pinMode(IN1,OUTPUT);
  
  Serial.println("target pos");
}

void loop() {

  // set target position
  //int target = 12000;
  
  int target = setpoints[counter];

  //int target = 250*sin(prevT/1e6);

  // PID constants
  float kp = 0.31;
  float kd = 0.2;
  float ki = 0.0;
  int e = setpoints[0];

  // time difference
  long currT = micros();
  float deltaT = ((float) (currT - prevT))/( 1.0e6 );
  prevT = currT;

  // Read the position in an atomic block to avoid a potential
  // misread if the interrupt coincides with this code running
  // see: https://www.arduino.cc/reference/en/language/variables/variable-scope-qualifiers/volatile/
  int pos = 0; 
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    pos = posi;
  }
  
  // error
  if(counter < 4){
    e = target - pos;
  }
  else{
    e = pos - target;
  }
  //int e = target - pos;


  // derivative
  float dedt = (e-eprev)/(deltaT);

  // integral
  eintegral = eintegral + e*deltaT;

  // control signal
  float u = kp*e + kd*dedt + ki*eintegral;

  // motor power
  float pwr = fabs(u);
  if( pwr > 255 ){
    pwr = 255;
  }

  // motor direction
  int dir = 1;
  if(u<0){
    dir = -1;
  }

  // signal the motor
  setMotor(dir,pwr,PWM,IN1);


  // store previous error
  eprev = e;

  Serial.print(target);
  Serial.print(" ");
  Serial.print(pos);
  Serial.println();
  Serial.println(target);
  Serial.println("direction: " + String(dir));

  if(abs(e) < 100){
    counter = counter + 1;
    delay(100);
  }

  if(counter == 8){
    delay(10000000);
  }
}

void setMotor(int dir, int pwmVal, int pwm, int in1){
  if(dir == 1){ //fprward
    analogWrite(pwm,pwmVal);
    //digitalWrite(in1,LOW);
    digitalWrite(PWM,HIGH);
    Serial.println(pwmVal);
    digitalWrite(in1,LOW);
  }
  else if(dir == -1){
   
    digitalWrite(in1,HIGH);
    


  }
  else{
      digitalWrite(pwm,LOW);
      

  }
}

void readEncoder(){
  int b = digitalRead(ENCB);
  if(b > 0){
    posi++;
  }
  else{
    posi--;
  }
}
