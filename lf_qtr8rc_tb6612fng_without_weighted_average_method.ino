#include <SparkFun_TB6612.h>
#include <QTRSensors.h>

/*
  It uses qtr-8rc and tb6612fng 
  this is not using weighted average method to calculate error values.
  assuming motor1 as left motor and motor2 as right.
  implemented sharp_left , sharp_right for sharp 90' turn while left and right can be used for relaxed turns.
*/

// MOTOR DRIVER PINS - change pins according to your design.
#define AIN1 5
#define BIN1 7
#define AIN2 4
#define BIN2 8
#define PWMA 3
#define PWMB 9
#define STBY 6

const int offsetA = 1;
const int offsetB = 1;
// motor initialization
Motor motor1 = Motor(AIN1, AIN2, PWMA, offsetA, STBY);
Motor motor2 = Motor(BIN1, BIN2, PWMB, offsetB, STBY);


QTRSensors qtr;
const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];
uint16_t line_data[SensorCount];
uint16_t position;


float error = 0, P = 0, I = 0, D = 0, PID_value = 0, previous_error = 0, previous_I = 0;
// PID constants for PID tuning
float Kp = 25;
float Ki=0.5;
float Kd=10;


int L = 0,R = 0;
// motor speed config
int max_speed = 255;   // max speed of the bot
int turn = 120;        // max turning speed of the bot


void setup() {
  Serial.begin(9600);
  brake(motor1, motor2);

  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){ A0, A1, A2, A3, A4, A5, A6, A7 }, SensorCount);
  // qtr.setEmitterPin(2);
  for (uint16_t i = 0; i < 200; i++) {
    qtr.calibrate();
  }
  Serial.println("Sensor Calibration Done");  
}



void loop() {
  readLine();

  if(line_data[0] == 0 && line_data[7] == 1) {
    sharp_left();
  } else if(line_data[0] == 1 && line_data[7] == 0) {
    sharp_right();
  } else 
//if(line_data[1] == 0 || line_data[2] == 0 || line_data[3] == 0 || line_data[4] == 0 || line_data[5] == 0 || line_data[6] == 0) 
  {
    calculateError();
    pidControl();
  } 
  // else if((line_data[0] == 0) && (line_data[1] == 0) && (line_data[2] == 0) && (line_data[3] == 0) && (line_data[4] == 0) && (line_data[5] == 0) && (line_data[6] == 0) && (line_data[7] == 0)) {
  //   motor_Stop();
  // }
}


void calculateError() {
/*
  calculating error based on weights
          * *
    0 1 2 3 4 5 6 7
   -8-4-2 1 1 2 4 8
*/
  if ((line_data[3] == 0) && (line_data[4] == 0))
    error = 0;
  else if ((line_data[2] == 0) && (line_data[3] == 0) && (line_data[4] == 0))
    error = -1;
  else if (line_data[3] == 0 && line_data[4] == 0 && line_data[5] == 0) 
    error = 1;
  else if ((line_data[2] == 0) && (line_data[3] == 0))
    error = -2;
  else if((line_data[4] == 0) && (line_data[5] == 0) )
    error = 2;
  else if ((line_data[1] == 1) && (line_data[2] == 0) && (line_data[3] == 0))
    error = -3;
  else if ((line_data[4] == 0) && (line_data[5] == 0) && (line_data[6] == 0))
    error = 3;
  else if ((line_data[1] == 0) && (line_data[2] == 0))
    error = -4;
  else if ((line_data[5] == 1) && (line_data[6] == 0))
    error = 4;
  else if ((line_data[0] == 0) && (line_data[1] == 0) && (line_data[2] == 0))
    error = -5;
  else if ((line_data[6] == 0) && (line_data[5] == 0) && (line_data[7] == 0))
    error = 5;
  else if ((line_data[0] == 0) && (line_data[1] == 0))
    error = -8;
  else if ((line_data[6] == 0) && (line_data[7] == 0))
    error = 8;
  // else if((line_data[0] == 1) && (line_data[1] == 1) && (line_data[2] == 1) && (line_data[3] == 1) && (line_data[4] == 1)){
  //   if(previous_error == -2)
  //   error = -3;
  // else if(previous_error == 2)
  //    error = 3;
  // }
}


void pidControl() {
  // pid calculation
  P = error;
  I = I + error;
  D = error - previous_error;
  PID_value = (Kp * P) + (Ki * I) + (Kd * D);
  previous_error = error;

  L = max_speed + PID_value;
  R = max_speed - PID_value;

  if (L > max_speed) {
    L = max_speed;
  }
  if (R > max_speed) {
    R = max_speed;
  }
  if (L < 0) {
   L = 0;
  }
  if (R < 0) {
    R = 0;
  }
  forward(L, R);

}




void readLine(){

  position = qtr.readLineWhite(sensorValues);
  Serial.print("QTR::");
  Serial.println(position);

  // getting 1 or 0 based on value sensorValues.
  for(int i=0;i<8;i++){
    if(sensorValues[i] > 600)
    {
      line_data[i] = 1;
    }else 
    {
      line_data[i] = 0;
    }
  }
  Serial.println();
}


void forward(int L, int R) {
  motor1.drive(L);
  motor2.drive(R);
}
void sharp_right() {
  motor1.drive(max_speed);
  motor2.drive(-max_speed);
}
void sharp_left() {
  motor1.drive(-max_speed);
  motor2.drive(max_speed);
}
void right() {
  motor1.drive(turn);
  motor2.drive(0);
}
void left() {
  motor1.drive(0);
  motor2.drive(turn);
}



