// https://www.movable-type.co.uk/scripts/latlong.html
float calc_distancia(double flat1, double flon1, double flat2, double flon2) { 

  double R = 6371000;            // metres
  double L1 = flat1 * PI / 180;  // φ, λ in radians
  double L2 = flat2 * PI / 180;
  double V1 = (flat2 - flat1) * PI / 180;
  double V2 = (flon2 - flon1) * PI / 180;

  double a = sin(V1 / 2) * sin(V1 / 2) + cos(L1) * cos(L2) * sin(V2 / 2) * sin(V2 / 2);
  double c = 2 * atan2(sqrt(a), sqrt(1 - a));

  double d = R * c;  // in metres

  //Serial.println("Distancia");
  //Serial.println(d);

  return d;
}


//Função que calcula a direção através das coordenadas geográficas
// Obtido de https://www.movable-type.co.uk/scripts/latlong.html
float Obter_Direcao(double lat1, double lon1, double lat2, double lon2) { 

  double L1 = lat1 * PI / 180;
  double L2 = lat2 * PI / 180;
  double V1 = lon1 * PI / 180;
  double V2 = lon2 * PI / 180;


  double y = sin(V2 - V1) * cos(L2);
  double x = cos(L1) * sin(L2) - sin(L1) * cos(L2) * cos(V2 - V1);
  double TETA = atan2(y, x);

  if (TETA < 0) {
    TETA += 2 * PI;
  }

  //Serial.print("ANGULO DE DIRECAO  ");
  //Serial.println(TETA * 180 / PI);
  return TETA;  // EM RADIANOS
}

void Posicao_atual() {
  bool newData = false;
  unsigned long chars;
  // Por x segundo, analisamos os dados do GPS e relatamos alguns valores-chave
  for (unsigned long start = millis(); millis() - start < 450;) {  // 650
    while (gpsSerial.available()) {
      char c = gpsSerial.read();
      // Serial.write(c); //apague o comentario para mostrar os dados crus
      if (gps.encode(c))  // Atribui true para newData caso novos dados sejam recebidos
        newData = true;
    }
  }
  
  if (newData) {
    //DadosGPS();
    gps.f_get_position(&flat, &flon, &age);
    delay(100);  

    flatAtual = (flat == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flat);
    flonAtual = (flon == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flon);

    lat_log_atu[0] = (flat == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flat);
    lat_log_atu[1] = (flon == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flon);

    //Serial.print("GPS  ");
    //Serial.print(flat == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flat, 6);
    //Serial.print("  ");
    //Serial.println(flon == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flon, 6);
  }
}

void DadosGPS() {
  //float flat, flon;

  gps.f_get_position(&flat, &flon, &age);
  Serial.print("LAT=");
  Serial.print(flat == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flat, 10);
  Serial.print(" LON=");
  Serial.print(flon == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flon, 10);
  Serial.print(" SAT=");
  Serial.print(gps.satellites() == TinyGPS::GPS_INVALID_SATELLITES ? 0 : gps.satellites());
  Serial.print(" PREC=");
  Serial.print(gps.hdop() == TinyGPS::GPS_INVALID_HDOP ? 0 : gps.hdop());
  Serial.println();
  Serial.println();
}