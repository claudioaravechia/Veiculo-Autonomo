#include <Wire.h>
#include <VL53L0X.h>
#include <Adafruit_Sensor.h>
#include <math.h>

// definicoes do GPS
#define GPS_RX 4
#define GPS_TX 3
#define GPS_Serial_Baud 9600
#include <SoftwareSerial.h>
#include <TinyGPS.h>

#include <Adafruit_PWMServoDriver.h>

// VELOCIDADE DOS MOTORES LIMHA RETA
int VelMax;
int VelMin;

// INSTANCIANDO OBJETOS
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// DECLARAÇÃO DE FUNÇÕES
void beginServo();
void writeServo(int nServo, int posicao);

// Classe simples para tratar a bússola
class Bussola {
public:
  typedef enum { INDEFINIDO,
                 HMC,
                 QMC } TIPO;
  Bussola(TIPO tipo);
  bool inicia(void);
  TIPO getTipo(void);
  void setDeclination(int graus, int mins, char dir);
  float leDirecao(void);
  void iniciaCal();
  void encerraCal();

private:
  static const int ender_HMC = 0x1E;  // endereço I2C do HMC5883
  static const int regMODE_HMC = 2;   // registrador de modo
  static const int regXH_HMC = 3;     // primeiro registrador de dados
  static const int regST_HMC = 9;     // registrador de status

  static const int ender_QMC = 0x0D;  // endereço I2C do QMC5883
  static const int regCR1_QMC = 9;    // registrador de configuração
  static const int regSR_QMC = 11;    // registador set/reset
  static const int regXL_QMC = 0;     // primeiro registrador de dados
  static const int regST_QMC = 6;     // registrador de status

  // fatores de correção determinados na calibração
  int16_t xMin, yMin, xMax, yMax;
  float escX = 1.0;
  float escY = 1.0;
  int16_t offX = 0;
  int16_t offY = 0;

  // Edereço e tipo do chip
  int ender;
  TIPO tipo;

  // Define a declinação (correção entre o Norte magnético e o Norte geofráfico)
  // ver http://www.magnetic-declination.com/
  // Diferença entre o Polo Magnético e o Geográfico
  float declination = -(21 + 9 / 60.0) * PI / 180.0;

  // Rotina para disparar leitura dos dados
  void pedeDados(int regStatus, int regDados);
};

Bussola bussola(Bussola::INDEFINIDO);

TinyGPS gps;

SoftwareSerial gpsSerial(GPS_RX, GPS_TX);

String Informacoes_gerais;

//Constantes
// DEfinicao das distancias minimas dos obstaculos
const int distanciaMinE = 50;  //Distância mínima do obstaculo
const int distanciaMinF = 50;  //Distância mínima do obstaculo
const int distanciaMinD = 50;  //Distância mínima do obstaculo

//Definição dos pinos aos quais o XSHUT está ligado.
#define XSHUT_E 7
#define XSHUT_F 8
#define XSHUT_D 9

//Definição dos endereços dos sensores VL53L0X. Endereço padrão: 0b0101001 ou 41.
#define Sensor_E 42
#define Sensor_F 43
#define Sensor_D 44

VL53L0X SensorE;
VL53L0X SensorF;
VL53L0X SensorD;

int DistanciaE;
int DistanciaF;
int DistanciaD;

// Variavel indicando que obstaculos nao foram encontrados
boolean detectado = false;

float flat, flon;
float flatAtual, flonAtual;

bool newData = false;

bool IniciarLatLong = true;

// Variaveis de armazenamento do GPS

float lat_log_atu[2];  // Latitude e Longitude atuais

int NumTrajetorias = 1;  // apenas uma trajetoria MUDAR A PARTIR DO MOMENTO QUE FOR NECESSARIO

float lat_array[5];  // Latitudes que se deseja chegar
float lon_array[5];  // Longitudes que se deseja chegar

//float lat_array[] = { -21.777598, -21.77775, -21.777885 };   // Latitude que se deseja chegar
//float lon_array[] = { -48.832853, -48.832789, -48.832940 };  // Longitude que se deseja chegar


float heading;
float dist;
unsigned long age;
float tempo;
float Rumo;

float calc_heading;

void setup() {

  //Inicia a comunicação serial.
  Serial.begin(9600);
  Wire.begin();

  Serial.begin(GPS_Serial_Baud);
  gpsSerial.begin(GPS_Serial_Baud);

  // INCIA O SERVO
  beginServo();
  delay(30);
  IniciaAngulo();

  // Iniciacao dos sensores de distancia
  //Desliga todos os VL53L0X.
  pinMode(XSHUT_E, OUTPUT);
  pinMode(XSHUT_F, OUTPUT);
  pinMode(XSHUT_D, OUTPUT);

  //Liga os sensores e altera seus endereços.
  pinMode(XSHUT_E, INPUT);
  delay(10);
  SensorE.setAddress(Sensor_E);
  pinMode(XSHUT_F, INPUT);
  delay(10);
  SensorF.setAddress(Sensor_F);
  pinMode(XSHUT_D, INPUT);
  delay(10);
  SensorF.setAddress(Sensor_D);

  //Inicializa os sensores.
  SensorE.init();
  SensorF.init();
  SensorD.init();

  //Define timeout para os sensores.
  SensorE.setTimeout(500);
  SensorF.setTimeout(500);
  SensorD.setTimeout(500);

  //Inicia o modo de leitura contínuo dos VL53L0X.
  SensorE.startContinuous();
  SensorF.startContinuous();
  SensorD.startContinuous();

  // Inicializando a bussola
  if (!bussola.inicia()) {
    Serial.println("Nao encontrou a bussola!");
    for (;;) {
      delay(100);
    }
  }

  Serial.print("Encontrada bussola ");
  Serial.println(bussola.getTipo() == Bussola::QMC ? "QMC5883L" : "HMC5883L");

  // Calibragem bussola
  CalibragemBussola();
  Serial.println("AGUARDANDO CONEXAO DO GPS");
  Serial.println("AGUARDANDO LATITUDE E LONGITUDE DESEJADA");
}

void loop() {
  bool newData = false;
  unsigned long chars;


  if (IniciarLatLong == true) {  // pega a latitude e longitude desejada
    Iniciar_LatLong();
  } else {

    // For one second we parse GPS data and report some key values
    for (unsigned long start = millis(); millis() - start < 1000;) {
      while (gpsSerial.available()) {
        char c = gpsSerial.read();
        if (gps.encode(c)) {  // Atribui true para newData caso novos dados sejam recebidos
          newData = true;
          Serial.println("CONEXAO DO GPS ESTABELECIDA");
        }
      }
    }
    if (newData) {
      // TESTE DA BUSSOLA
      // *******************************************
      // Indicacao_Bussola();
      // *******************************************

      // DESVIAR DE OBSTACULOS
      // *******************************************
      //DesviaObstaculo();
      // *******************************************

      // ROBO AUTONOMO
      // *******************************************
      RoboAutonomo();
      // *******************************************
    }
  }
}

void Iniciar_LatLong() {

  String latlog_rec;  // lê os dados recebidos como string
  String LatLong[8];  // para separar os numeros

  while (Serial.available()) {

    latlog_rec = Serial.readString();  // lê os dados recebidos como string

    Serial.println(latlog_rec);
    char sz[128];
    latlog_rec.toCharArray(sz, sizeof(sz));

    char *str, *p = sz;
    int i = 0;
    while (str = strtok(p, ",")) {
      LatLong[i] = str;
      p = NULL;
      ++i;
    }

    lat_array[0] = atof(LatLong[0].c_str());  // converte a string para float
    lon_array[0] = atof(LatLong[1].c_str());  // converte a string para float

    //Serial.println(lat_array[0], 6);
    //Serial.println(lon_array[0], 6);

    IniciarLatLong = false;
    Serial.println("AGUARDANDO CONEXAO DO GPS");
  }
}

void CalibragemBussola() {
  Serial.println("Calibrando bussola");

  // TIRAR COMENTARIO CASO QUEIRA CALIBRAR A BUSSOLA NA INICIALIZACAO

  /*Serial.println("Calibrando... rode o sensor em um círculo");
  bussola.iniciaCal();
  long tmpFim = millis() + 30000L;
  while (millis() < tmpFim) {
    bussola.leDirecao();
    delay(20);
  } */

  bussola.encerraCal();
  Serial.println("Bussola calibrada");
}


void DesviaObstaculo() {
  lerSensores();
  Decisao();
}

void RoboAutonomo() {
  float DistanciaAtual;
  float DirecaoEntrePontos;
  Serial.print("TRAJETORIA");
  Serial.print(";");
  Serial.print("LAT LONG DESEJADA");
  Serial.print(";");
  Serial.print("LAR LONG ATUA");
  Serial.print(";");
  Serial.print("IND BUSSOLA");
  Serial.print(";");
  Serial.print("DIR ENTRE PONTOS");
  Serial.print(";");
  Serial.print("RUMO");
  Serial.print(";");
  Serial.println("DISTANCIA");

  //for (int i = 0; i <= NumTrajetorias;) {
  for (int i = 1; i <= NumTrajetorias;) {

    Posicao_atual();                          // Latitude e Longitude atual armazenado em lat_log_atu[]
    float Ind_Bussola = Indicacao_Bussola();  // Imprime a direcao que a bussola esta apontada
    DistanciaAtual = calc_distancia(lat_log_atu[0], lat_log_atu[1], lat_array[i - 1], lon_array[i - 1]);
    DirecaoEntrePontos = Obter_Direcao(lat_log_atu[0], lat_log_atu[1], lat_array[i - 1], lon_array[i - 1]);

    delay(10);
    while (DistanciaAtual > 1) {

      Rumo = (DirecaoEntrePontos - Ind_Bussola) * 180 / M_PI;

      if (Rumo <= 0) {
        Rumo = 360 + Rumo;
      }

      // VELOCIDADE DOS MOTORES
      if (DistanciaAtual > 1) { // se adistancia do objetivo for maior que 1
        VelMax = 210;
        VelMin = 50;
      } else {
        VelMax = 150;
        VelMin = 30;
      }

      AcaoMotores();  // realiza o controle da direcao do robodiminuindo o giro dos motores de um lado

      //DirecaoServo();

      // ACOES ANTI COLISAO
      //DesviaObstaculo();

      // apenas para teste dos sensores
      lerSensores();
      if ((DistanciaE <= distanciaMinE) || (DistanciaF <= distanciaMinF) || (DistanciaD <= distanciaMinD)) {
        //Decisao();
        //Para();
        //delay(5000);
      }


      /* Serial.print(i);
      Serial.print(";");
      Serial.print(DistanciaE);  // SENSOR DISTANCIA ESQUERDO
      Serial.print(";");
      Serial.print(DistanciaF);  // SENSOR DISTANCIA FRONTAL
      Serial.print(";");
      Serial.print(DistanciaD);  // SENSOR DISTANCIA DIREITO
      Serial.print(";");
      Serial.print(lat_array[i - 1], 6);
      Serial.print(";");
      Serial.print(lon_array[i - 1], 6);
      Serial.print(";");
      Serial.print(lat_log_atu[0], 6);
      Serial.print(";");
      Serial.print(lat_log_atu[1], 6);
      Serial.print(";");
      Serial.print(Ind_Bussola * 180 / M_PI, 3);  // Imprime a direcao que a bussola esta apontada
      Serial.print(";");
      Serial.print(DirecaoEntrePontos * 180 / M_PI, 3);  // DirecaoEntrePontos = Obter_Direcao(lat_log_atu[0], lat_log_atu[1], lat_array[i], lon_array[i]);
      Serial.print(";");
      Serial.print(Rumo, 3);  // imprime o rumo a ser tomado pelo veiculo = DirecaoEntrePontos - direcao que a bussola esta apontada
      Serial.print(";");
      Serial.println(DistanciaAtual, 2);*/


      Informacoes_gerais = (String)(DistanciaE) + "," + (String)(DistanciaF) + "," + (String)(DistanciaD) + "," + (String)((lat_array[i - 1], 6)) + "," + (String)((lon_array[i - 1], 6)) + "," + (String)((lat_log_atu[0], 6)) + "," + (String)((lat_log_atu[1], 6)) + "," + (String)((Ind_Bussola * 180 / M_PI, 3)) + "," + (String)((DirecaoEntrePontos * 180 / M_PI, 3)) + "," + (String)((Rumo, 3)) + "," + (String)((DistanciaAtual, 2));

      Serial.println(Informacoes_gerais);
      delay(50);

      Posicao_atual();  // Latitude e Longitude atual armazenado em lat_log_atu[]
      DistanciaAtual = calc_distancia(lat_log_atu[0], lat_log_atu[1], lat_array[i - 1], lon_array[i - 1]);
      DirecaoEntrePontos = Obter_Direcao(lat_log_atu[0], lat_log_atu[1], lat_array[i - 1], lon_array[i - 1]);
      Ind_Bussola = Indicacao_Bussola();  // Imprime a direcao que a bussola esta apontada
    }

    Para();
    delay(10000);


    i++;
  }
  IniciarLatLong = true;  // obter nova latitude e longitude
}
