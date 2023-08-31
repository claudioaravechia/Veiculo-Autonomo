// DEFINIÇAO DOS PINOS DOS MOTORES
//MOTOR A (DIREITO)
#define IN1 5
#define IN2 6

//MOTOR B (ESQUERDO)
#define IN3 10
#define IN4 11


// VALOR DE AJUSTE DAS VELOCIDADES DOS MOTORES PARA O ROBO SEGUIR EM LINHA RETA
int AjusteVelD = 0;
int AjusteVelE = 0;



// VELOCIDADE DOS MOTORES AJUSTADA
// APENAS A MAXIMA
int VelDMax = VelMax - AjusteVelD;
int VelEMax = VelMax - AjusteVelE;


void SegueReto() {
  //Serial.println("<<<<<  segue reto  >>>>>");
  analogWrite(IN1, VelEMax);
  analogWrite(IN2, 0);
  analogWrite(IN3, VelDMax);
  analogWrite(IN4, 0);
}

void Para() {
  //Serial.println("Para");
  analogWrite(IN1, 0);
  analogWrite(IN2, 0);
  analogWrite(IN3, 0);
  analogWrite(IN4, 0);
}

void Esquerda() {
  //Serial.println("vira esquerda");
  analogWrite(IN1, VelEMax);
  analogWrite(IN2, 0);
  analogWrite(IN3, VelMin);
  analogWrite(IN4, 0);
}

void Direita() {
  //Serial.println("vira direita");
  analogWrite(IN1, VelMin);
  analogWrite(IN2, 0);
  analogWrite(IN3, VelDMax);
  analogWrite(IN4, 0);
}

void GiraEsquerda() {
  //Serial.println("<<<<<  Giro a esquerda");
  analogWrite(IN1, VelEMax);
  analogWrite(IN2, 0);
  analogWrite(IN3, 0);
  analogWrite(IN4, VelDMax);
}

void GiraDireita() {
  //Serial.println("Giro a direita  >>>>>");
  analogWrite(IN1, 0);  
  analogWrite(IN2, VelEMax);
  analogWrite(IN3, VelDMax);  
  analogWrite(IN4, 0);
}


void AcaoMotores() {
  if (Rumo >= 350) {
    SegueReto();  // seguir em frente
  }
  if ((Rumo >= 0) and (Rumo <= 10)) {
    SegueReto();  // seguir em frente
  }
  if ((Rumo > 300) & (Rumo <= 350)) {
    Esquerda();  //a esquerda
    delay(500);
    SegueReto();
  }
  if ((Rumo >= 225) & (Rumo <= 300)) {
    GiraEsquerda();  //Giro a esquerda
    delay(700);
    SegueReto();
  }
  if ((Rumo >= 10) & (Rumo < 60)) {
    Direita();  //a direita
    delay(500);
    SegueReto();
  }
  if ((Rumo >= 60) & (Rumo <= 135)) {
    GiraDireita();  //Giro a direita
    delay(700);
    SegueReto();
  }
  if ((Rumo < 225) & (Rumo > 135)) {
    GiraEsquerda();  //Giro a esquerda
    delay(700);
    SegueReto();
  }
}

