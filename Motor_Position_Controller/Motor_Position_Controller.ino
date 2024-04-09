#include "Arduino.h"

#define PWM_M1 7  //M1 PWM pin, GPIO5
#define CW_M1  2 //M1 Clockwise rotation pin, GPIO18
#define CCW_M1 3  //M1 Counterclockwise rotation pin, GPIO4
#define encoderPinA 0 // YELLOW
#define encoderPinB 1 // WHITE
//volatile long end_time = 0;

int target_M1;


volatile int pos_M1 = 0;


volatile double ePrev_M1 = 0;

volatile double integralPrev_M1 = 0.0;

volatile double derivativePrev_M1 = 0.0;

double controlSignal_M1 = 0.0;


hw_timer_t *Timer0_Cfg = NULL;
volatile long encoderPosition = 0;
volatile bool encoderAPrev = false;

void IRAM_ATTR Timer0_ISR(){
 

  //update motor position in degrees 1:134
  pos_M1 = encoderPosition;


  //update control signal
  pidController(pos_M1, &controlSignal_M1);

  //send control signal to motors
  moveMotor(CW_M1, CCW_M1, PWM_M1, controlSignal_M1);
   
  //end_time = micros() - start;
}

void setup() {
  Serial.begin(115200);

  
  pinMode(PWM_M1,OUTPUT);
  pinMode(CW_M1,OUTPUT);
  pinMode(CCW_M1,OUTPUT);

  attachInterrupt(digitalPinToInterrupt(encoderPinA), readEncoder, RISING);
  

  //initialize timer, 130us period (80us ISR runtime): 61.5% duty cycle
  Timer0_Cfg = timerBegin(0, 80, true);
  timerAttachInterrupt(Timer0_Cfg, &Timer0_ISR, true);
  timerAlarmWrite(Timer0_Cfg, 1100, true);
  timerAlarmEnable(Timer0_Cfg);
  Serial.println("target pos");

}



void pidController(int pos_M1, volatile double *u_M1){
  //sampling time of discrete controller and time constant of derivative filter
  double T = 0.0011;
  double tau = 0.02; //0.001, 0.05

  //PID gain values
  double kp_M1 = 2.5;   //2.5
  double ki_M1 = 4.0;   //2.0
  double kd_M1 = 1.0;   //0.5


  //Compute error for proportional control
  double e_M1 = (pos_M1 - target_M1);

  //Compute proportional term
  double proportional_M1 = e_M1*kp_M1;
 
  //Compute integral term
  double integral_M1 = (ki_M1*T*0.5)*(e_M1+ePrev_M1)+integralPrev_M1;


  //Compute derivative term with low-pass derivative filter
  double derivative_M1 = (2.0*kd_M1*(e_M1-ePrev_M1)+(2.0*tau-T)*(derivativePrev_M1))/(2.0*tau+T);
 

  //update variables for next iteration
  ePrev_M1 = e_M1;

  integralPrev_M1 = integral_M1;

  derivativePrev_M1 = derivative_M1;


  *u_M1 = (proportional_M1) + (integral_M1) + (derivative_M1);

}

void moveMotor(int in1, int in2, int pwm_pin, double u){
  double speed = fabs(u);  //motor power                              
  if(speed > 255){
    speed = 255;
  }

  int direction = 1;  //motor direction     
  if(u<0){            //u<0
    direction = -1;
  }

  analogWrite(pwm_pin, speed); //implement speed control using driver     

  if(direction == 1){
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  }
  else if(direction == -1){
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  }
  else{
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
  }
}

void loop() {
 int target_M1 = 6120;



}

void readEncoder(){
  int b = digitalRead(encoderPinB);
  if(b > 0){
    encoderPosition++;
  }
  else{
    encoderPosition--;
  }
}