#include <PID_v1.h>
#include <ros.h>

/*CONSTANTS AND GLOBAL VARIABLE DECLARATION*/
const int ENCODER_A1_PINA = 46;//pin for encoder's channel A.
const int ENCODER_A1_PINB = 48;//pin for encoder's channel B.

const int ENCODER_A2_PINA = 50;//pin for encoder's channel A.
const int ENCODER_A2_PINB = 52;//pin for encoder's channel B.

const int ENCODER_A3_PINA = 44;//pin for encoder's channel A.
const int ENCODER_A3_PINB = 42;//pin for encoder's channel B.

const int A1_DIR1 = 26;//direction 1, DIR1=0,DIR=1, CW rotation.
const int A1_DIR2 = 28;//direction 2, DIR1=0,DIR=1, CW rotation.
const int A1_ENABLE = 3;//PWM output pin connecting to the driver.

const int A2_DIR1 = 32;//direction 1, DIR1=0,DIR=1, CW rotation.
const int A2_DIR2 = 30;//direction 2, DIR1=0,DIR=1, CW rotation.
const int A2_ENABLE = 4;//PWM output pin connecting to the driver.

const int A3_DIR1 = 22;//direction 1, DIR1=0,DIR=1, CW rotation.
const int A3_DIR2 = 24;//direction 2, DIR1=0,DIR=1, CW rotation.
const int A3_ENABLE = 5;//PWM output pin connecting to the driver.

double A1_Setspeed, A2_Setspeed, A3_Setspeed = 0.0;
double A1_PWMOutput, A2_PWMOutput, A3_PWMOutput = 0.0;

volatile int A1encoder0Count = 0;
volatile int A2encoder0Count = 0;
volatile int A3encoder0Count = 0;

//Define Variables we'll be connecting to
double Setpoint1, Input1, Output1 = 0;
double Setpoint2, Input2, Output2 = 0;
double Setpoint3, Input3, Output3 = 0;

//Define the aggressive and conservative Tuning Parameters
double A1Kp = 1.0, A1Ki = 6.0, A1Kd = 0.0;
double A2Kp = 1.5, A2Ki = 6.0, A2Kd = 0.0;
double A3Kp = 1.0, A3Ki = 6.0, A3Kd = 0.0;

//Specify the links and initial tuning parameters
PID myPID1(&Input1, &Output1, &Setpoint1, A1Kp, A1Ki, A1Kd, P_ON_M, DIRECT);
PID myPID2(&Input2, &Output2, &Setpoint2, A2Kp, A2Ki, A2Kd, P_ON_M, DIRECT);
PID myPID3(&Input3, &Output3, &Setpoint3, A3Kp, A3Ki, A3Kd, P_ON_M, DIRECT);

float pv1_speed, tError1, nowTime1, tempCount1, NowCount1;
float pv2_speed, tError2, nowTime2, tempCount2, NowCount2;
float pv3_speed, tError3, nowTime3, tempCount3, NowCount3;

float wheelVelocityArray1[20] = {};
float wheelVelocityArray2[20] = {};
float wheelVelocityArray3[20] = {};

float wheelVelOutput1, wheelVelOutput2, wheelVelOutput3  = 0.0;
float wheelVelInput1, wheelVelInput2, wheelVelInput3 = 0.0;
float WD1_dot, WD2_dot, WD3_dot, theta, r_dot, WD1, WD2, WD3, r = 0.0;
float pi = 3.1415;

char A1array[5] = {};
char A2array[5] = {};

float X_pos, Y_pos, delta_t, last_time = 0.0;
bool flag = false;

void setup() {

  pinMode(ENCODER_A1_PINA, INPUT);//set pin ENCODER_A2_PINA as input.
  pinMode(ENCODER_A1_PINB, INPUT);//set pin ENCODER_A2_PINB as input.
  pinMode(A1_DIR1, OUTPUT);//set pin DIR1 as output.
  pinMode(A1_DIR2, OUTPUT);//set pin DIR2 as output.
  pinMode(A1_ENABLE, OUTPUT);//set pint ENA as output
  attachInterrupt(ENCODER_A1_PINA, A1doEncoderA, CHANGE);//set up  interrupt
  attachInterrupt(ENCODER_A1_PINB, A1doEncoderB, CHANGE);//set up  interrupt

  pinMode(ENCODER_A2_PINA, INPUT);//set pin ENCODER_A2_PINA as input.
  pinMode(ENCODER_A2_PINB, INPUT);//set pin ENCODER_A2_PINB as input.
  pinMode(A2_DIR1, OUTPUT);//set pin DIR1 as output.
  pinMode(A2_DIR2, OUTPUT);//set pin DIR2 as output.
  pinMode(A2_ENABLE, OUTPUT);//set pint ENA as output
  attachInterrupt(50, A2doEncoderA, CHANGE);//set up  interrupt
  attachInterrupt(52, A2doEncoderB, CHANGE);//set up  interrupt

  pinMode(ENCODER_A3_PINA, INPUT);//set pin ENCODER_A2_PINA as input.
  pinMode(ENCODER_A3_PINB, INPUT);//set pin ENCODER_A2_PINB as input.
  pinMode(A3_DIR1, OUTPUT);//set pin DIR1 as output.
  pinMode(A3_DIR2, OUTPUT);//set pin DIR2 as output.
  pinMode(A3_ENABLE, OUTPUT);//set pint ENA as output
  attachInterrupt(44, A3doEncoderA, CHANGE);//set up  interrupt
  attachInterrupt(42, A3doEncoderB, CHANGE);//set up  interrupt

  Serial.begin (57600); //9600

  //turn the PID on
  myPID1.SetMode(AUTOMATIC);
  myPID1.SetOutputLimits(-255, 255); //set output limits of PID controller
  myPID1.SetSampleTime(3); //set the sample time

  myPID2.SetMode(AUTOMATIC);
  myPID2.SetOutputLimits(-255, 255); //set output limits of PID controller
  myPID2.SetSampleTime(3); //set the sample time

  myPID3.SetMode(AUTOMATIC);
  myPID3.SetOutputLimits(-255, 255); //set output limits of PID controller
  myPID3.SetSampleTime(3); //set the sample time



}

void loop()
{
  Serial.print(X_pos);
  Serial.print(",");
  Serial.print(Y_pos);
  Serial.print(",");
  Serial.println(delta_t);

  //  int A2_sensorValue  = analogRead(A1);
  //  Setpoint3 = A2_sensorValue * (100 / 1023.0);

  myPID1.SetTunings(A1Kp, A1Ki, A1Kd, P_ON_M);
  myPID2.SetTunings(A2Kp, A2Ki, A2Kd, P_ON_M);
  myPID3.SetTunings(A3Kp, A3Ki, A3Kd, P_ON_M);

  myPID1.Compute();
  myPID2.Compute();
  myPID3.Compute();

  motor1_dir();
  motor2_dir();
  motor3_dir();

  velocity_A1();
  velocity_A2();
  velocity_A3();

  if (Serial.available()) {

    //manual_navigate();
    auto_navigate();

  }

  speed_Model();
  position_Model();

  Setpoint1 = WD1_dot * 9.5492;
  Setpoint2 = WD2_dot * 9.5492;
  Setpoint3 = WD3_dot * 9.5492;


}
