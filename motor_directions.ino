void motor1_dir() {

  if (Output1 >= 0)
  {
    analogWrite(A1_ENABLE, abs(Output1));
    digitalWrite(A1_DIR1, LOW);
    digitalWrite(A1_DIR2, HIGH);
  }

  if (Output1 < 0)
  {
    analogWrite(A1_ENABLE, abs(Output1));
    digitalWrite(A1_DIR1, HIGH);
    digitalWrite(A1_DIR2, LOW);
  }

}

void motor2_dir() {

  if (Output2 >= 0)
  {
    analogWrite(A2_ENABLE, abs(Output2));
    digitalWrite(A2_DIR1, LOW);
    digitalWrite(A2_DIR2, HIGH);
  }

  if (Output2 < 0)
  {
    analogWrite(A2_ENABLE, abs(Output2));
    digitalWrite(A2_DIR1, HIGH);
    digitalWrite(A2_DIR2, LOW);
  }

}

void motor3_dir() {

  if (Output3 >= 0)
  {
    analogWrite(A3_ENABLE, abs(Output3));
    digitalWrite(A3_DIR1, LOW);
    digitalWrite(A3_DIR2, HIGH);
  }

  if (Output3 < 0)
  {
    analogWrite(A3_ENABLE, abs(Output3));
    digitalWrite(A3_DIR1, HIGH);
    digitalWrite(A3_DIR2, LOW);
  }

}
