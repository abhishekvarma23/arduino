void manual_navigate() {

  int inByte = Serial.read();

  switch (inByte) {
    case '8':
      goForward();
      break;
    case '4':
      RotateCCW();
      break;
    case '6':
      RotateCW();
      break;
    case '2':
      goBackward();
      break;
    case '5':
      Stop();
      break;
    case '7':
      NW();
      break;
    case '9':
      NE();
      break;
    case '3':
      SE();
      break;
    case '1':
      SW();
      break;
    case 'd':
      goRight();
      break;
    case 'a':
      goLeft();
      break;
  }

}

void auto_navigate() {
  String Serial_A1  = Serial.readStringUntil('*');
  char c1array[5];
  Serial_A1.toCharArray(c1array, sizeof(c1array));
  r_dot = atof(c1array);
  


  String Serial_A2  = Serial.readStringUntil('\n');
  char c2array[5];
  Serial_A2.toCharArray(c2array, sizeof(c2array));
  float theta_temp = atof(c2array);
  theta = theta_temp * (pi / 180);
  

}
