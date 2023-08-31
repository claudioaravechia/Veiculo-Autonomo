// http://dqsoft.blogspot.com/2019/12/modulo-bussola-com-hmc5833l-ou-qmc5833l.html?m=1
// https://www.magnetic-declination.com/

// Criacao do tipo bussula 
Bussola::Bussola(TIPO tipo) {
  this->tipo = tipo;
}

// Inicia comunicação com a bússola
bool Bussola::inicia() {
  if (tipo == INDEFINIDO) {
    // Tenta identificar o chip
    Wire.beginTransmission(ender_HMC);
    if (Wire.endTransmission() == 0) {
      tipo = HMC;
      ender = ender_HMC;
    } else {
      Wire.beginTransmission(ender_QMC);
      if (Wire.endTransmission() == 0) {
        tipo = QMC;
        ender = ender_QMC;
      }
    }
  }

  // Inicia o chip para modo contínuo
  if (tipo == HMC) {
    Wire.beginTransmission(ender);
    Wire.write(regMODE_HMC);
    Wire.write(0x00);
    Wire.endTransmission();
  } else if (tipo == QMC) {
    Wire.beginTransmission(ender);
    Wire.write(regSR_QMC);
    Wire.write(0x01);
    Wire.endTransmission();
    Wire.beginTransmission(ender);
    Wire.write(regCR1_QMC);
    Wire.write(0x0D);
    Wire.endTransmission();
  }

  return tipo != INDEFINIDO;
}

// Informa o tipo de bússola
Bussola::TIPO Bussola::getTipo(void) {
  return tipo;
}

// Le a direção da bússola em graus (0 a 360) em relação à marcação do eixo X
// Assume que a bússola esta na horizontal
float Bussola::leDirecao(void) {  //f
  int16_t x, y, z;

  if (tipo == INDEFINIDO) {
    return 0.0;
  }

  // Le a intesidade do campo magnético
  if (tipo == HMC) {
    pedeDados(regST_HMC, regXH_HMC);
    x = Wire.read() << 8;  //MSB  x
    x |= Wire.read();      //LSB  x
    z = Wire.read() << 8;  //MSB  z
    z |= Wire.read();      //LSB  z
    y = Wire.read() << 8;  //MSB  y
    y |= Wire.read();      //LSB  y
  } else if (tipo == QMC) {
    pedeDados(regST_QMC, regXL_QMC);
    x = Wire.read();        //LSB  x
    x |= Wire.read() << 8;  //MSB  x
    y = Wire.read();        //LSB y
    y |= Wire.read() << 8;  //MSB y
    z = Wire.read();        //LSB  z
    z |= Wire.read() << 8;  //MSB z
  }

  // Registra mínimo e máximo para a calibração
  if (x < xMin) {
    xMin = x;
  }
  if (xMax < x) {
    xMax = x;
  }
  if (y < yMin) {
    yMin = y;
  }
  if (yMax < y) {
    yMax = y;
  }

  // corrige e calcula o angulo em radianos
  float xC = (x - offX) * escX;                //f
  float yC = (y - offY) * escY;                //f
  float angulo = atan2(yC, xC) + declination;  //f

  // Garante que está entre 0 e 2*PI
  if (angulo < 0) {
    angulo += 2.0 * PI;
  } else if (angulo > 2 * PI) {
    angulo -= 2.0 * PI;
  }

  // Converte para graus
  // retorna (angulo*180.0)/PI;
  // em rad
  return (angulo);
}

void Bussola::pedeDados(int regStatus, int regDados) {
  // Espera ter um dado a ler
  do {
    Wire.beginTransmission(ender);
    Wire.write(regStatus);
    Wire.endTransmission();
    Wire.requestFrom(ender, 1);
  } while ((Wire.read() & 1) == 0);

  Wire.beginTransmission(ender);
  Wire.write(regDados);
  Wire.endTransmission();
  Wire.requestFrom(ender, 6);
}


// Inicia processo de calibração
void Bussola::iniciaCal() {
  xMax = yMax = -32768;
  xMin = yMin = 32767;
}

// Encerra a calibração
void Bussola::encerraCal() {

  // TIRAR OS COMENTARIOS CASO QUEIRA CALIBRAR A BUSSOLA NA INICIALIZACAO
  
  //Serial.print ("X: ");
  //Serial.print (xMin);
  //Serial.print (" - ");
  //Serial.println (xMax);
  //Serial.print ("Y: ");
  //Serial.print (yMin);
  //Serial.print (" - ");
  //Serial.println (yMax);

  // COLOCAR COMO COMENTARIO CASO QUEIRA CALIBRAR A BUSSOLA NA INICIALIZACAO
  xMin = -178; 
  xMax = 223; 
  yMin = -322; 
  yMax = 85; 


  // Offset para centralizar leituras em zero
  offX = (xMax + xMin) / 2;
  offY = (yMax + yMin) / 2;

  // Escala para ter a mesma variação nos dois eixos
  int16_t varX = xMax - xMin;
  int16_t varY = yMax - yMin;
  if (varY > varX) {
    escY = 1.0;
    escX = (float)varY / varX;  //f
  } else {
    escX = 1.0;
    escY = (float)varX / varY;  //f
  }
}



// calculo da direcao da bussola
float Indicacao_Bussola() {  //f

  heading = bussola.leDirecao();

  //Serial.print("DIRECAO BUSSOLA  ");
  //Serial.print(heading * 180 / M_PI);

  /*  if (headingDegrees < 22.5 || headingDegrees > 337.5) {
    Serial.println("  Norte  ");
  } else if (headingDegrees > 22.5 && headingDegrees < 67.5) {
    Serial.println("  Nordeste  ");
  } else if (headingDegrees > 67.5 && headingDegrees < 112.5) {
    Serial.println("  Leste  ");
  } else if (headingDegrees > 112.5 && headingDegrees < 157.5) {
    Serial.println("  Sudeste  ");
  } else if (headingDegrees > 157.5 && headingDegrees < 202.5) {
    Serial.println("  Sul  ");
  } else if (headingDegrees > 202.5 && headingDegrees < 247.5) {
    Serial.println("  Sudoeste  ");
  } else if (headingDegrees > 247.5 && headingDegrees < 292.5) {
    Serial.println("  Oeste  ");
  } else if (headingDegrees > 292.5 && headingDegrees < 337.5) {
    Serial.println("  Noroeste  ");
  } else {
    Serial.println("Erro");
  }*/

  return heading;
}