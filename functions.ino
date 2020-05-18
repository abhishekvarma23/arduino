void speed_Model() {

  WD1_dot = - 8.33 * ( cos(theta) * r_dot );
  WD2_dot = 11.785 * ( sin(theta) * r_dot );
  WD3_dot = 11.785 * ( sin(theta) * r_dot );

}

void position_Model() {

  delta_t = ( micros() - last_time ) / 1000000;
  last_time = micros();

  float delta_X = ( cos(theta) * r_dot ) * delta_t;
  float delta_Y = ( sin(theta) * r_dot ) * delta_t;

  X_pos += delta_X;
  Y_pos += delta_Y;


}



float velocity_A1() {

  tError1 = micros() - float(nowTime1);
  nowTime1 = micros();
  tempCount1 = NowCount1;
  NowCount1 = A1encoder0Count;

  pv1_speed = float((NowCount1 - tempCount1 ) * 60.0 * 1000000.0) / (500.0 * float(tError1) * 4.0);

  Input1 = wheelVelocityFilter1(pv1_speed);
  return Input1;

}

float velocity_A2() {

  tError2 = micros() - float(nowTime2);
  nowTime2 = micros();
  tempCount2 = NowCount2;
  NowCount2 = A2encoder0Count;

  pv2_speed = float((NowCount2 - tempCount2) * 60.0 * 1000000.0) / (500.0 * float(tError2) * 4.0);

  Input2 = wheelVelocityFilter2(pv2_speed);
  return Input2;

}

float velocity_A3() {

  tError3 = micros() - float(nowTime3);
  nowTime3 = micros();
  tempCount3 = NowCount3;
  NowCount3 = A3encoder0Count;

  pv3_speed = float((NowCount3 - tempCount3 ) * 60.0 * 1000000.0) / (500.0 * float(tError3) * 4.0);

  Input3 = wheelVelocityFilter3(pv3_speed);
  return Input3;

}

float wheelVelocityFilter1(float wheelVelInput1) {

  for (int i = 19; i >= 1; i--) {

    wheelVelocityArray1[i] = wheelVelocityArray1[i - 1];
    wheelVelOutput1 += wheelVelocityArray1[i];
  }

  wheelVelocityArray1[0] = wheelVelInput1;
  wheelVelOutput1 += wheelVelocityArray1[0];
  wheelVelOutput1 /= 20.0;
  return wheelVelOutput1;
}

float wheelVelocityFilter2(float wheelVelInput2) {

  for (int i = 19; i >= 1; i--) {

    wheelVelocityArray2[i] = wheelVelocityArray2[i - 1];
    wheelVelOutput2 += wheelVelocityArray2[i];
  }

  wheelVelocityArray2[0] = wheelVelInput2;
  wheelVelOutput2 += wheelVelocityArray2[0];
  wheelVelOutput2 /= 20.0;
  return wheelVelOutput2;
}

float wheelVelocityFilter3(float wheelVelInput3) {

  for (int i = 19; i >= 1; i--) {

    wheelVelocityArray3[i] = wheelVelocityArray3[i - 1];
    wheelVelOutput3 += wheelVelocityArray3[i];
  }

  wheelVelocityArray3[0] = wheelVelInput3;
  wheelVelOutput3 += wheelVelocityArray3[0];
  wheelVelOutput3 /= 20.0;
  return wheelVelOutput3;
}

void goForward() {
  A1_Setspeed = 0.0;
  A2_Setspeed = 15.0;
  A3_Setspeed = 15.0;
}

void goBackward() {
  A1_Setspeed = 0.0;
  A2_Setspeed = -15.0;
  A3_Setspeed = -15.0;
}

void Stop() {
  A1_Setspeed = 0.0;
  A2_Setspeed = 0.0;
  A3_Setspeed = 0.0;

}

void RotateCW() {
  A1_Setspeed = -10.0;
  A2_Setspeed = -10.0;
  A3_Setspeed = 10.0;
}

void RotateCCW() {
  A1_Setspeed = 10.0;
  A2_Setspeed = 10.0;
  A3_Setspeed = -10.0;
}

void NE() {
  A1_Setspeed = -10.0;
  A2_Setspeed = 10.0;
  A3_Setspeed = 10.0;
}

void NW() {
  A1_Setspeed = 10.0;
  A2_Setspeed = 10.0;
  A3_Setspeed = 10.0;
}

void SE() {
  A1_Setspeed = -10.0;
  A2_Setspeed = -10.0;
  A3_Setspeed = -10.0;
}

void SW() {
  A1_Setspeed = 10.0;
  A2_Setspeed = -10.0;
  A3_Setspeed = -10.0;
}

void goRight() {
  A1_Setspeed = -10.0;
  A2_Setspeed = 2.0; //5.0;
  A3_Setspeed = 0.0; //-5.0;
}

void goLeft() {
  A1_Setspeed = 10.0;
  A2_Setspeed = 0.0; //-5.0;
  A3_Setspeed = 2.0; //5.0;
}


/*====================================================*/
//motor A1
void A1doEncoderA() {
  if (digitalRead(ENCODER_A1_PINA) != digitalRead(ENCODER_A1_PINB)) {
    A1encoder0Count++;
  }
  else {
    A1encoder0Count--;
  }
}
/*===================================================================================================*/
/*INTERRUPT SUBROUTINE CHANNEL B
  When an interrupt on channel B occurs then: CW rotation: A = B, CCW rotation: A != B.*/
void A1doEncoderB() {
  if (digitalRead(ENCODER_A1_PINA) == digitalRead(ENCODER_A1_PINB)) {
    A1encoder0Count++;
  }
  else {
    A1encoder0Count--;
  }
}
/*====================================================*/
/*====================================================*/
//motor A2
void A2doEncoderA() {
  if (digitalRead(ENCODER_A2_PINA) != digitalRead(ENCODER_A2_PINB)) {
    A2encoder0Count++;
  }
  else {
    A2encoder0Count--;
  }
}
/*===================================================================================================*/
/*INTERRUPT SUBROUTINE CHANNEL B
  When an interrupt on channel B occurs then: CW rotation: A = B, CCW rotation: A != B.*/
void A2doEncoderB() {
  if (digitalRead(ENCODER_A2_PINA) == digitalRead(ENCODER_A2_PINB)) {
    A2encoder0Count++;
  }
  else {
    A2encoder0Count--;
  }
}
/*====================================================*/
/*====================================================*/
//motor A3
void A3doEncoderA() {
  if (digitalRead(ENCODER_A3_PINA) != digitalRead(ENCODER_A3_PINB)) {
    A3encoder0Count++;
  }
  else {
    A3encoder0Count--;
  }
}
/*===================================================================================================*/
/*INTERRUPT SUBROUTINE CHANNEL B
  When an interrupt on channel B occurs then: CW rotation: A = B, CCW rotation: A != B.*/
void A3doEncoderB() {
  if (digitalRead(ENCODER_A3_PINA) == digitalRead(ENCODER_A3_PINB)) {
    A3encoder0Count++;
  }
  else {
    A3encoder0Count--;
  }
}
