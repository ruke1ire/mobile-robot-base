#define COUNT_REV 3000

const int ENC_LEFT_A = 18;
const int ENC_LEFT_B = 19;
const int ENC_RIGHT_A = 20;
const int ENC_RIGHT_B = 21;

const int MOTOR_LEFT_SPEED = 4;
const int MOTOR_LEFT_A = 5;
const int MOTOR_LEFT_B = 6;
const int MOTOR_RIGHT_SPEED = 7;
const int MOTOR_RIGHT_A = 8;
const int MOTOR_RIGHT_B = 9;

const double PID[3] = {0.01, 0.0, 0.0};

volatile long int count_left = 0;
volatile long int count_right = 0;

double desired_vel_left = 0.0;
double desired_vel_right = 0.0;
double actual_vel_left = 0.0;
double actual_vel_right = 0.0;

void update_left_a();
void update_left_b();
void update_right_a();
void update_right_b();

double control(double desired, double actual, double pid[3]);
double convert_to_radians(int count);
void compute_vel();

void setup() {
  Serial.begin(250000);
  
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
  compute_vel();
  Serial.print("Angle Left: ");
  Serial.print(convert_to_radians(count_left));
  Serial.print("\tAngle Right: ");
  Serial.print(convert_to_radians(count_right));
  Serial.print("\tVel Left: ");
  Serial.print(actual_vel_left, 6);
  Serial.print("\tVel Right: ");
  Serial.println(actual_vel_right,6);
  
  delay(10);

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

double control(double desired, double actual, double pid[3]){
  
  
}

double convert_to_radians(int count){
  return (2*PI*(double)count)/double(COUNT_REV);
}

void compute_vel(){
  static double previous_angle_left = convert_to_radians(count_left);
  static double previous_angle_right = convert_to_radians(count_right);
  static unsigned long int previous_time = micros();
  double current_angle_left = convert_to_radians(count_left);
  double current_angle_right = convert_to_radians(count_right);
  double current_time = micros();

  actual_vel_left = (current_angle_left-previous_angle_left)/((double)(current_time-previous_time)/1000000.0);
  actual_vel_right = (current_angle_right-previous_angle_right)/((double)(current_time-previous_time)/1000000.0);

  previous_angle_left = current_angle_left;
  previous_angle_right = current_angle_right;
  previous_time = current_time;
}
