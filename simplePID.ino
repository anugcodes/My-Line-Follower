

float Kp = 25, Ki=0.5, Kd=10; //change the value of kp ,ki and kd factors randomly and find a set of these value wich works good for your robot
float error = 0, P = 0, I = 0, D = 0, PID_value = 0; //defining the intial value 0
float previous_error = 0, previous_I = 0;


// sensors and sensor data
int irSensors[] = {A4, A3, A2, A1, A0}; //IR sensor pins
int irReadings[5];
int irAnalogData[5];

//left motor
int motorLForward = 4;
int motorLBackward = 5;
int motorLpwmPin = 10;

//right motor
int motorRForward = 6;
int motorRBackward = 7;
int motorRpwmPin = 9;


int initial_motor_speed = 200;
int turndelay = 200;
int turnspeed = 230;



void setup() {
  // put your setup code here, to run once:
  for (int pin = 0; pin < 3; pin++) {
    int pinNum = irSensors[pin];
    pinMode(pinNum, INPUT);
  }
  pinMode(motorLForward, OUTPUT);
  pinMode(motorLBackward, OUTPUT);
  pinMode(motorLpwmPin, OUTPUT);
  pinMode(motorRForward, OUTPUT);
  pinMode(motorRBackward, OUTPUT);
  pinMode(motorRpwmPin, OUTPUT);

  Serial.begin(9600);
}



void loop() {

  // after calibration code ---
  readIRSensors();
  // printing sensor data
  Serial.print(int(irReadings[0]));
  Serial.print(" ");
  Serial.print(int(irReadings[1]));
  Serial.print(" ");
  Serial.print(int(irReadings[2]));
  Serial.print(" ");
  Serial.print(int(irReadings[3]));
  Serial.print(" ");
  Serial.print(int(irReadings[4]));
  Serial.print(" ");
  Serial.print(int(irAnalogData[0]));
  Serial.print(" ");
  Serial.print(int(irAnalogData[1]));
  Serial.print(" ");
  Serial.print(int(irAnalogData[2]));
  Serial.print(" ");
  Serial.print(int(irAnalogData[3]));
  Serial.print(" ");
  Serial.print(int(irAnalogData[4]));
  Serial.print(" - ");



  if ((irReadings[0] == 1) && (irReadings[4] == 0))
  {
    RightTurn(turnspeed);
//    delay(turndelay);
  }
  else if ((irReadings[0] == 0) && (irReadings[4] == 1))
  {
    LeftTurn(turnspeed);
//    delay(turndelay);
  }
  else if((irReadings[0] == 0) && (irReadings[1] == 0) && (irReadings[2] == 0) && (irReadings[3] == 0) && (irReadings[4] == 0))  
  {
    motorStop();
  }
  else if(irReadings[1]==0 || irReadings[2] == 0 || irReadings[3] ==0  ) {  
    calculate_error();
    calculate_pid();
    motor_control();
  }
}



void readIRSensors() {
  //Read the IR sensors and put the readings in irReadings array
  for (int i = 0; i < 5; i++) {
    irAnalogData[i] = analogRead(irSensors[i]);
  }
  for (int i = 0; i < 5; i++) {
    irReadings[i] = digitalRead(irSensors[i]);
  }
}

void calculate_error() {
//  error = analogRead(irSensors[1]) - analogRead(irSensors[3]);  
  // error calculation
  if ((irReadings[1] == 0) && (irReadings[2] == 1) && (irReadings[3] == 1))
    error = -2;
  else if ((irReadings[1] == 1) && (irReadings[2] == 1) && (irReadings[4] == 0))
    error = 2;
  else if ((irReadings[1] == 0) && (irReadings[2] == 0) && (irReadings[3] == 1))
    error = -1;
  else if ((irReadings[1] == 1) && (irReadings[2] == 0) && (irReadings[4] == 0))
    error = 1;
  else if ((irReadings[1] == 1) && (irReadings[2] == 0) && (irReadings[4] == 1))
    error = 0;
  else if((irReadings[0] == 1) && (irReadings[1] == 1) && (irReadings[2] == 1) && (irReadings[3] == 1) && (irReadings[4] == 1)){
    if(previous_error == -2)
    error = -3;
    else if(previous_error == 2)
    error = 3;
  }
}

void calculate_pid()//calculating pid
{
  P = error;
  I = I + error;
  D = error - previous_error;
  PID_value = (Kp * P) + (Ki * I) + (Kd * D);
  previous_error = error;
}

void motor_control()//motor control
{
  // Calculating the effective motor speed:
  int left_motor_speed = initial_motor_speed - PID_value;
  int right_motor_speed = initial_motor_speed + PID_value;

  // The motor speed should not exceed the max PWM value
  constrain(left_motor_speed, 0, 255);
  constrain(right_motor_speed, 0, 255);

  analogWrite(motorLpwmPin, left_motor_speed);  //Left Motor Speed
  analogWrite(motorRpwmPin, right_motor_speed); //Right Motor Speed
  /*The pin numbers and high, low values might be different
    depending on your connections */
  digitalWrite(motorLForward, HIGH);
  digitalWrite(motorLBackward, LOW);
  digitalWrite(motorRBackward, LOW);
  digitalWrite(motorRForward, HIGH);
  Serial.println("F -");
}




void RightTurn(int speed) {
  analogWrite(motorRpwmPin, 0);
  analogWrite(motorLpwmPin, speed);
//  digitalWrite(motorLForward, HIGH);
//  digitalWrite(motorLBackward, LOW);
//  digitalWrite(motorRBackward, HIGH);
//  digitalWrite(motorRForward, LOW);
  Serial.println("RT");
}



void LeftTurn(int speed) {
  analogWrite(motorRpwmPin, speed);
  analogWrite(motorLpwmPin, 0);
//  digitalWrite(motorLForward, LOW);
//  digitalWrite(motorLBackward, HIGH);
//  digitalWrite(motorRBackward, LOW);
//  digitalWrite(motorRForward, HIGH);
  Serial.println("LT");
}

void motorStop() {
  analogWrite(motorRpwmPin, 0);
  analogWrite(motorLpwmPin, 0);
  Serial.println("S");
}
