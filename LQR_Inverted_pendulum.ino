// Magnetic encoder library requirements
#include "AS5600.h"
#include "Wire.h"
AS5600 as5600;   //  use default Wire

// Ultrasonic sensor pins
const int trigPin = 7; // trigger pin for ultrasonic sensor
const int echoPin = 6; //echo pin for ultrasonic sensor
// DC motor driver pins
const int in1 = 9; //motor direction 1 - negative direction of x 
const int in2 = 8; //motor direction 2 - positive direction of x
const int ConA = 10; //PWM speed control pin
// System input (desired positions)
const float x_desired = 0.15;
const float v_desired = 0;
const float t_desired = 3.14; // 0 is vertical down position
const float w_desired = 0;
//Error values
float x_error, v_error, t_error, w_error ;
// LQR controller parameters
const float Kx = 51.8;
const float Kv = 40.2;
const float Kth = 159.2;
const float Kw  = 40;

// Physical position limit
const float POSITION_LIMIT = 0.4;
 
// Discrete control system implementation
unsigned long time_now = 0L;
unsigned long lastTimeMicros = 0L;
// Variable declarations
float x, last_x, v, dt;
float duration, distance;
float angle, theta, last_theta, w;
float u;
float theta_init;
bool control_status;

void setup() {
  pinMode(trigPin, OUTPUT); //setting pin modes for ultrasonic distance sensor
  pinMode(echoPin, INPUT);
  as5600.begin(4);  //  set direction pin.

  pinMode(in1, OUTPUT); //setting pin modes for motor driver
  pinMode(in2, OUTPUT);
  pinMode(ConA, OUTPUT);

  Serial.begin (9600);
  Serial.println(__FILE__);
  Serial.print("AS5600_LIB_VERSION: ");
  Serial.println(AS5600_LIB_VERSION);

  Wire.begin();

  as5600.setDirection(AS5600_COUNTERCLOCK_WISE);  // NOT default
  int b = as5600.isConnected();
  Serial.print("Connect: ");  // CHECKING STATUS OF AS5600 COMMUNICATION
  Serial.println(b);

  theta_init = as5600.rawAngle() *  AS5600_RAW_TO_RADIANS;
} 

float getCartdistance(){  
  digitalWrite(trigPin, LOW);  // Added this line
  delayMicroseconds(2); // Added this line
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(12); // Added this line
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH);
  distance =  duration / 2000000.0 * 345.5;
  return distance;
}

float getAngle(){
  angle = as5600.rawAngle() *  AS5600_RAW_TO_RADIANS;
  return angle;
}

boolean isControllable(float theta_c) {
  if ((theta_c >= 2.826) && (theta_c <= (3.454))){ //Implementation of deadzone when  pendulum angle is aproaching desired (to deal with noise from sensors) experimentally determined
    if ((theta_c <= 3.10) || (theta_c >= (3.18))){
      control_status = true;
    }else{
      control_status = false;
    }
  }else{
    control_status = false;
  }
  
  return control_status;
}

void driveMotor(float u) {
  if (u > 0.0){
    digitalWrite(in1, LOW); 
    digitalWrite(in2, HIGH);
    analogWrite(ConA,fabs(u));
  }
  if (u == 0.0)
  {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    analogWrite(ConA,fabs(u));
  }
  if (u < 0.0){
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    analogWrite(ConA,fabs(u));
  }
}

float saturate(float value, float maxValue) {
  if (fabs(value) > maxValue) {
    return (value > 0) ? maxValue : -maxValue;
  } else {
    return value;
  }
}
void loop() {
   
  time_now = micros();
  dt = 1.0 * (time_now - lastTimeMicros) / 1000000;
  
  x =  0.45 - getCartdistance();
  v = (x - last_x)/dt;

  theta = getAngle() - theta_init;

  if (theta < 0){
    theta = 2*3.14 + theta;
  }

  w = (theta - last_theta)/dt;

  if (isControllable(theta)){
    //Error values calculation
    x_error = x_desired - x;
    v_error = v_desired - v;
    t_error = t_desired - theta;
    w_error = w_desired - w;
 
    u =x_error * Kx + v_error * Kv + t_error * Kth + w_error * Kw; // calculation of control value (0-12V range)
    u = 225.0 * u / 12.0; // range conversion 0-12V to 0-200 PWM (255 is the upper limit, too fast for the hardware used)
    u = saturate(u, 225);
    driveMotor(u);
  } else {
    u = 0;
    driveMotor(u);
  }

  last_x = x;
  last_theta = theta;
  lastTimeMicros = time_now; //discrete control system implementation

/** Serial print of key parameters to monitor (slows down operation)
  Serial.print(theta);
  Serial.print(" rad");
  Serial.print("\t");
  Serial.print(w);
  Serial.print(" rad/s");
  Serial.print("\t");
  Serial.print(x);
  Serial.print(" m");
  Serial.print("\t");
  Serial.print(v);
  Serial.print(" m/s");
  Serial.print("\t");
  Serial.println(u);
**/

  delay(5);
}
