void DirecaoServo() {
  if ((abs(Rumo) >= 0) and abs((Rumo) <= 90)) {
    writeServo(0, ( 90 - abs(Rumo)));
  }
  if (abs((Rumo) >= 270) and abs((Rumo) < 360)) {
    writeServo(0, ((- abs(Rumo) + 360) + 90));
  }
  if ((abs(Rumo) < 270) and (abs(Rumo) > 180)) {
    writeServo(0, 1);
  }
  if (abs((Rumo) > 90) and (abs(Rumo) < 180)) {
    writeServo(0, 179);
  }
}

void beginServo() {

#define Frequencia 50  // VALOR DA FREQUENCIA DO SERVO

  pwm.begin();                 // INICIA O OBJETO PWM
  pwm.setPWMFreq(Frequencia);  // DEFINE A FREQUENCIA DE TRABALHO DO SERVO
}

void writeServo(int nServo, int posicao) {
#define SERVOMIN 125  // VALOR PARA UM PULSO MAIOR QUE 1 mS
#define SERVOMAX 525  // VALOR PARA UM PULSO MENOR QUE 2 mS

  int pos = map(posicao, 0, 180, SERVOMIN, SERVOMAX);
  // Serial.print("ANGULO: ");
  // Serial.println(posicao);
  pwm.setPWM(nServo, 0, pos);
  delay(20);
}

void IniciaAngulo() {
  writeServo(0, 90);
  delay(20);
}