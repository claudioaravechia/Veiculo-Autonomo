void lerSensores() {
  //Lê a distância em centímetros.
  DistanciaE = SensorE.readRangeContinuousMillimeters() * 0.1;
  DistanciaF = SensorF.readRangeContinuousMillimeters() * 0.1;
  DistanciaD = SensorD.readRangeContinuousMillimeters() * 0.1;

  //ImprimeDistancias();
}

void Decisao() {
  //Verifica se a leitura está dentro dos parâmetros de detecção.
  if ((DistanciaE <= distanciaMinE) || (DistanciaF <= distanciaMinF) || (DistanciaD <= distanciaMinD)) {
    Serial.println("Detectou!");
    // se os tres sensores detectarem um obstaculo ao mesmo tempo
    if ((DistanciaE <= distanciaMinE) && (DistanciaF <= distanciaMinF) && (DistanciaD <= distanciaMinD)) {
      Para();
      delay(50);
    }

    // apos a deteccao do obstaculo caso ele esteja a frente
    // decide se vira a esquerda ou a direita
    if (DistanciaF <= distanciaMinF) {  // a distancia do sensor da frente e menor que a definida
      Para();
      delay(300);
      // escolhe para qual lado virar caso o sensor da frente estiver proximo ao obstaculo e ou o esquerdo ou direito estiver proximo ao obstaculo
      if ((DistanciaD <= distanciaMinD) || (DistanciaE <= distanciaMinE)) {
        if (DistanciaD <= distanciaMinD) {  // a distancia do sensor direito e menor que a definida
          GiraEsquerda();
          Serial.print(DistanciaF);
          Serial.print(" cm a frente  ");
          Serial.print(DistanciaD);
          Serial.print(" cm a direita  ");
          delay(1000);
          SegueReto();
        } else {  // a distancia do sensor esquerdo e menor que a definida
          GiraDireita();
          Serial.print(DistanciaF);
          Serial.print(" cm a frente  ");
          Serial.print(DistanciaE);
          Serial.print(" cm a esquerda  ");
          delay(1000);
          SegueReto();
        }
      } else {
        // escolhe para qual lado virar caso apenas o sensor da frente estiver proximo ao obstaculo
        // escolhe quel sensor tem a maior distacia de um obstaculo
        if (DistanciaD < DistanciaE) {  // a distancia do sensor direito e menor que a distancia do sensor esquerdo
          GiraEsquerda();
          Serial.print(DistanciaF);
          Serial.print(" cm a frente  ");
          Serial.print(DistanciaD);
          Serial.print(" cm a direita  ");
          delay(1000);
          SegueReto();
        } else {  // a distancia do sensor direito e maior que a distancia do sensor esquerdo
          GiraDireita();
          Serial.print(DistanciaF);
          Serial.print(" cm a frente  ");
          Serial.print(DistanciaE);
          Serial.print(" cm a esquerda  ");
          delay(1000);
          SegueReto();
        }
      }
    } else if (DistanciaF >= distanciaMinF) {
      // caso a distancia do sensor da frente nao esteja
      //proximo ao obstaculo mas o esquerdo ou direito esteja
      // se o sensor da Direita detectar um obstaculo
      if (DistanciaD <= distanciaMinD) {
        GiraEsquerda();
        Serial.print(DistanciaD);
        Serial.print(" cm a direita  ");
        delay(300);
        SegueReto();
      }

      // se o sensor da Esquerda detectar um obstaculo
      if (DistanciaE <= distanciaMinE) {
        GiraDireita();
        Serial.print(DistanciaE);
        Serial.print(" cm a esquerda  ");
        delay(300);
        SegueReto();
      }
    }
    Serial.println("  ");
    detectado = true;
  }
}

void ImprimeDistancias() {
  //Mostra o resultado no monitor serial.
  Serial.print(DistanciaE);
  Serial.print(" cm");
  Serial.print(" , ");
  Serial.print(DistanciaF);
  Serial.print(" cm");
  Serial.print(" , ");
  Serial.print(DistanciaD);
  Serial.print(" cm");
  Serial.println();
}