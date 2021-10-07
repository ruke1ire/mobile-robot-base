#define COUNT_REV 3000

#include "SerialCommunicator.h"

const int ENC_LEFT_A = 18;
const int ENC_LEFT_B = 19;
const int ENC_RIGHT_A = 20;
const int ENC_RIGHT_B = 21;

const int MOTOR_LEFT_SPEED = 4;
const int MOTOR_LEFT_A = 6;
const int MOTOR_LEFT_B = 5;
const int MOTOR_RIGHT_SPEED = 7;
const int MOTOR_RIGHT_A = 8;
const int MOTOR_RIGHT_B = 9;

double PI_value[2] = {-10.0, -100.0};
double P_value_compensation = 1.0;
const int control_period = 20; 

const int baudrate = 19200;

volatile long int count_left = 0;
volatile long int count_right = 0;

double desired_vel_left = 0.0;
double desired_vel_right = 0.0;
double actual_vel_left = 0.0;
double actual_vel_right = 0.0;
double actual_pos_left = 0.0;
double actual_pos_right = 0.0;
double actual_vel_ratio = 1.0;

void update_left_a();
void update_left_b();
void update_right_a();
void update_right_b();

void velocity_compensation();
double control_l(double desired, double actual, double pi[2]);
double control_r(double desired, double actual, double pi[2]);
double convert_to_radians(int count);
void compute_vel();
void drive_motor(char wheel, double control_out);
void control_loop();

SerialCommunicator serial_com(baudrate);

void setup() {
  Serial.begin(baudrate);
  attachInterrupt(digitalPinToInterrupt(ENC_LEFT_A), update_left_a, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_LEFT_B), update_left_b, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_RIGHT_A), update_right_a, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_RIGHT_B), update_right_b, CHANGE);

  pinMode(MOTOR_LEFT_SPEED, OUTPUT);
  pinMode(MOTOR_LEFT_A, OUTPUT);
  pinMode(MOTOR_LEFT_B, OUTPUT);
  pinMode(MOTOR_RIGHT_SPEED, OUTPUT);
  pinMode(MOTOR_RIGHT_A, OUTPUT);
  pinMode(MOTOR_RIGHT_B, OUTPUT);
}

void loop() {
  static unsigned long int prev_control = millis();
  static unsigned long int prev_serial = millis();
  unsigned long int now = millis();

  if((now-prev_control)>control_period){
    prev_control = now;
    compute_state();
    velocity_compensation();
    double control_out_l = control_l(desired_vel_left, actual_vel_left, PI_value);
    double control_out_r = control_r(desired_vel_right, actual_vel_right, PI_value);
    
    drive_motor('l', control_out_l);
    drive_motor('r', control_out_r);
  }
  else if((now-prev_serial)>1){
    serial_com.communicator_loop();
    prev_serial = now;
  }
}

void update_left_a() {
  int a = digitalRead(ENC_LEFT_A);
  int b = digitalRead(ENC_LEFT_B);

  if(a == HIGH){
    if(b == LOW){
      count_left++;
    }
    else{
      count_left--;
    }
  }
  else{
    if(b == LOW){
      count_left--;
    }
    else{
      count_left++;
    }
  }
}

void update_left_b() {
  int a = digitalRead(ENC_LEFT_A);
  int b = digitalRead(ENC_LEFT_B);

  if(a == HIGH){
    if(b == HIGH){
      count_left++;
    }
    else{
      count_left--;
    }
  }
  else{
    if(b == HIGH){
      count_left--;
    }
    else{
      count_left++;
    }
  }
}

void update_right_a() {
  int a = digitalRead(ENC_RIGHT_A);
  int b = digitalRead(ENC_RIGHT_B);

  if(a == HIGH){
    if(b == LOW){
      count_right++;
    }
    else{
      count_right--;
    }
  }
  else{
    if(b == LOW){
      count_right--;
    }
    else{
      count_right++;
    }
  }
}

void update_right_b() {
  int a = digitalRead(ENC_RIGHT_A);
  int b = digitalRead(ENC_RIGHT_B);

  if(a == HIGH){
    if(b == HIGH){
      count_right++;
    }
    else{
      count_right--;
    }
  }
  else{
    if(b == HIGH){
      count_right--;
    }
    else{
      count_right++;
    }
  }
}

void velocity_compensation(){
  double desired_diff = desired_vel_right - desired_vel_left;
  double actual_diff = actual_vel_right - actual_vel_left;

  double compensation = (P_value_compensation*(desired_diff-actual_diff));
  desired_vel_right += compensation;
  desired_vel_left -= compensation;
}

double control_l(double desired, double actual, double pi[2]){
  static double e_sum = 0;
  static unsigned long int previous_time = micros();
  double current_time = micros();
  double e = actual-desired;
  e_sum += e*(double)(current_time - previous_time)/1000000.0;
  previous_time = current_time;
  if(e_sum > 0){
    e_sum = min(e_sum, 255.0/(-pi[1]));
  }
  else if(e_sum < 0){
    e_sum = max(e_sum, 255.0/(pi[1]));
  }
  return pi[0]*e + pi[1]*e_sum;
}

double control_r(double desired, double actual, double pi[2]){
  static double e_sum = 0;
  static unsigned long int previous_time = micros();
  double current_time = micros();
  double e = actual-desired;
  e_sum += e*(double)(current_time - previous_time)/1000000.0;
  previous_time = current_time;
  if(e_sum > 0){
    e_sum = min(e_sum, 255.0/(-pi[1]));
  }
  else if(e_sum < 0){
    e_sum = max(e_sum, 255.0/(pi[1]));
  }
  return pi[0]*e + pi[1]*e_sum;
}

double convert_to_radians(int count){
  return (2*PI*(double)count)/double(COUNT_REV);
}

void compute_state(){
  static double previous_angle_left = convert_to_radians(count_left);
  static double previous_angle_right = convert_to_radians(count_right);
  static unsigned long int previous_time = micros();
  double current_angle_left = convert_to_radians(count_left);
  double current_angle_right = convert_to_radians(count_right);
  unsigned long int current_time = micros();

  double actual_vel_left_tmp = (current_angle_left-previous_angle_left)/((double)(current_time-previous_time+1)/1000000.0);
  double actual_vel_right_tmp = (current_angle_right-previous_angle_right)/((double)(current_time-previous_time+1)/1000000.0);
  if(abs(actual_vel_left_tmp)>100.0){
    
  }
  else{
    actual_vel_left = actual_vel_left_tmp;
  }
  
  if(abs(actual_vel_right_tmp)>100.0){
    
  }
  else{
    actual_vel_right = actual_vel_right_tmp;
  }

  actual_pos_left += actual_vel_left*((double)(current_time-previous_time+1)/1000000.0);
  actual_pos_right += actual_vel_right*((double)(current_time-previous_time+1)/1000000.0);
  
  previous_angle_left = current_angle_left;
  previous_angle_right = current_angle_right;
  previous_time = current_time;
}

void drive_motor(char wheel, double control_out){
  int a;
  int b;
  int pwm;
  if(wheel == 'l'){
    a = MOTOR_LEFT_A;
    b = MOTOR_LEFT_B;
    pwm = MOTOR_LEFT_SPEED;
  }
  else if(wheel == 'r'){
    a = MOTOR_RIGHT_A;
    b = MOTOR_RIGHT_B;
    pwm = MOTOR_RIGHT_SPEED;
  }
  else{
    return;
  }
  if(control_out > 0.0){
    digitalWrite(a, HIGH);
    digitalWrite(b, LOW);
  }
  else if(control_out < 0.0){
    digitalWrite(a, LOW);
    digitalWrite(b, HIGH);
  }
  analogWrite(pwm, min(int(abs(control_out)),255));
}
