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

volatile long int count_left = 0;
volatile long int count_right = 0;

float desired_vel_left = 0.0;
float desired_vel_right = 0.0;

void update_left_a();
void update_left_b();
void update_right_a();
void update_right_b();

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
  Serial.print("Count Left: ");
  Serial.print(count_left);
  Serial.print("\tCount Right: ");
  Serial.println(count_right);
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
