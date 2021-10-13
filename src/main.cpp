#include <Arduino.h>
#include <EEPROM.h>
#include <NTPClient.h>
#include <PubSubClient.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Ticker.h>
#include <DHtesp.h>
#include <ArduinoJson.h>
#include <U8x8lib.h>
#include <ESPmDNS.h>
#include <Update.h>
#include <SPI.h>
#include <WiFiClient.h>

#define EEPROM_SIZE 1024
#define WIFI_NOME "Metropole" //rede wifi específica
#define WIFI_SENHA "908070Radio"
#define BROKER_MQTT "10.71.0.132"
#define DEVICE_TYPE "ESP32"
#define TOKEN "ib+r)WKRvHCGjmjGQ0"
#define ORG "n5hyok"
#define PUBLISH_INTERVAL 1000*60*10 //intervalo de 10 min para publicar temperatura

uint64_t chipid = ESP.getEfuseMac(); // The chip ID is essentially its MAC address(length: 6 bytes).
uint16_t chip = (uint16_t)(chipid >> 32);
char DEVICE_ID[23];
char an = snprintf(DEVICE_ID, 23, "biit%04X%08X", chip, (uint32_t)chipid); // PEGA IP
String mac;
WiFiClient espClient;
PubSubClient client(espClient);
WebServer server(80);
WiFiUDP udp;
NTPClient ntp(udp, "a.st1.ntp.br", -3 * 3600, 60000); //Hr do Br
DHTesp dhtSensor;
DynamicJsonDocument doc (1024); //tamanho do doc do json
U8X8_SSD1306_128X64_NONAME_SW_I2C u8x8(15, 4, 16);
char topic[]= "teste";          // topico MQTT
char topic1[]= "teste1";        // topico MQTT
char topic2[]= "permissao";     // topico MQTT
char topic3[]= "testeCallback"; // topico MQTT
bool publishNewState = false; 
TaskHandle_t retornoTemp;
unsigned long tempo = 1000*60*1; // 15 min
unsigned long ultimoGatilho = millis()+tempo;
IPAddress ip=WiFi.localIP();

int tempAtual=0;
int tempAntiga=0;
bool mov=false;
bool tasksAtivo = true;
struct tm data; //armazena data 
char data_formatada[64];
char hora_formatada[64];
bool tensaoPin=false;
const int dhtPin1=32;
const int pirPin1=33; 
const int con=25;  //vermelha
const int eva=26;
const int sensorTensao=23;
bool novaTemp = false;
int tIdeal=24;
int Hdes=20; //desliga 8 da noite
int Hliga=07;//liga 7 da manha
int rede;
String comando;
unsigned long previousMillis2 = 0;
const long intervalo = 10000;
////////////////////////////////////////////////////////////////
void callback(char* topicc, byte* payload, unsigned int length){
   if(topic3){ //pega comando via MQTT
    Serial.println("ENTROU NO LOOP CALLBACK");
    for(int i=0; i<length;i++){
      comando=(char)payload[i]; //comando abre porta via mqtt
    }
  }
}
void conectaMQTT(){
  //Estabelece conexao c MQTT/WIFI
   if(!client.connected()){
    Serial.println("conectando...");
    if (client.connect("ESP322")){
      Serial.println("CONECTADO! :)");
      client.publish ("teste", "hello word");
      client.subscribe (topic);   //se inscreve no topico a ser usado  
      client.subscribe (topic1);
      client.subscribe (topic2);
      client.subscribe (topic3);
    } else {
      Serial.println("Falha na conexao");
      Serial.print(client.state());
      Serial.print("nova tentativa em 5s...");
      delay (5000);
    }
  }
}
void reconectaMQTT(){
  //reconecta MQTT caso desconecte
  if (!client.connected()){
    conectaMQTT();
  }
  client.loop();
}
void datahora(){
  time_t tt=time(NULL);
  data = *gmtime(&tt);
  strftime(data_formatada, 64, "%w - %d/%m/%y %H:%M:%S", &data);//hora completa
  strftime(hora_formatada, 64, "%H:%M", &data); //hora para o visor
  //Armazena na variável hora, o horário atual.
  int anoEsp = data.tm_year;
  if (anoEsp < 2020) {
    ntp.forceUpdate();
  }	
}
void dadosEEPROM(){
  //DEFINE OS DADOS EMERGENCIAIS DA EPROOM 
  if(EEPROM.read(0) != 24) {
    EEPROM.write(0, 24);  //escreve 24 no dress=0
  } else if(EEPROM.read(1) != 19){
    EEPROM.write(1, 19);  //escreve 19 no dress=1
  } else if(EEPROM.read(2) != 7){
    EEPROM.write(2, 7);  //escreve 7 no dress=2
  }
}
void iniciaWifi(){
  int cont=0;
  WiFi.begin(WIFI_NOME, WIFI_SENHA); 
  while (WiFi.status()!= WL_CONNECTED){//incia wifi ate q funcione
    Serial.print (".");
    delay(1000);
    cont++;
    if(cont==15){  //se n funcionar sai do loop e fica sem rede
      Serial.println("break");
      rede=0; //status da rede
      Serial.println(rede);
      break;
    }
  }  
  if(WiFi.status() == WL_CONNECTED){
    rede=1;
    if (!client.connected()){  //aqui é while, mudei pra teste
      conectaMQTT();
    }
  }
}
void redee(){
  if(rede==1){  //está conectado
    //protocolo online via MQTT
    Serial.println("rede 1");
  } else if(rede==0) { //n esta conectado a rede
    //protocolo offline
    Serial.println("rede 0");
    EEPROM.begin(EEPROM_SIZE);
    dadosEEPROM();
    // tIdeal=EEPROM.read(0);
    // Hdes=EEPROM.read(1);
    // Hliga=EEPROM.read(2);
  }
}
void tentaReconexao(){ //roda assincrona no processador 0
  unsigned long currentMillis = millis();
  if (currentMillis-previousMillis2<= intervalo) { //a cada 5min tenta reconectar
    Serial.print("*************************");
    Serial.print("TENTA RECONEXAO");
    Serial.println("***********************");
    previousMillis2 = currentMillis;
    iniciaWifi();
    ntp.forceUpdate();
  }
}
void sensorTemp(void *pvParameters){
  Serial.println ("sensorTemp inicio do LOOP");
  while (1) {//busca temp enquanto estiver ligado
    tempAtual = dhtSensor.getTemperature();
    tasksAtivo=false;
    vTaskSuspend (NULL);
  }
  vTaskDelay (pdMS_TO_TICKS(500));
}
void IRAM_ATTR mudaStatusPir(){
  mov=true;
  ultimoGatilho = millis()+tempo; //
}
void pegaTemp() {
      if (retornoTemp != NULL) {
    xTaskResumeFromISR (retornoTemp);
  }
}
void publish (){
  if (tempAntiga != tempAtual){
    // nova temperatura
    tempAntiga=tempAtual;
    if(tempAtual>50){
      novaTemp=0;
    } else {
      novaTemp=1;
    }
  } else {
    // temperatura igual
    novaTemp=0;
  }
}
void Tensao(){
  tensaoPin=true;
}
void arLiga(){
   String hora;
  hora= data.tm_hour;
  //liga ar
  digitalWrite(con, 1);
  digitalWrite(eva, 1);

  if(tempAtual>=(tIdeal+2)){ //quente
		if(digitalRead(eva)==1){
			digitalWrite(con, 1);
      Serial.println("condensadora ligada");
		} else {
      digitalWrite(con, 1);
      digitalWrite(eva, 1);
      Serial.println("condensadora ligada");
		}			
	} else if(tempAtual<=(tIdeal-2)){ //frio
    digitalWrite(con, 0);
    digitalWrite(eva, 1);
    Serial.print("condensadora desligada");	
	} 
  vTaskDelay(500);
}
void verificaDia(void *pvParameters){
  while(1){ 
    int Hora = data.tm_hour;
    int Minutos	=	data.tm_min;
    int data_semana = data.tm_wday; //devolve em numero
    if(data_semana!=6 || data_semana!=0){
      //se n for sabado nem domingo 
      if(Hora>=Hliga){
        //esta no horario de ligar
        if(ultimoGatilho>millis()){
          //tem movimento
          //LIGADO
          arLiga();
          Serial.println("estou dentro do horario");
        } else if(ultimoGatilho<millis()){
          //não tem movimento
          digitalWrite(con, 0);
          digitalWrite(eva, 0);
        }
      }
    }  
  vTaskDelay(pdMS_TO_TICKS(500));
  }
}
void perguntaMQTT(){
  int Hora = data.tm_hour;
  int Minutos	=	data.tm_min;
  int data_semana = data.tm_wday; //devolve em numero
  if(data_semana==6 || data_semana==4 || Hora<=Hliga || Hora>=Hdes){
    //se foir sabado ou domingo ou antes de 7h ou depois de 20h 
    //se tiver movimento
    if(ultimoGatilho>millis()){
      while(1){
        Serial.println("entrou para a parte que pergunta ao MQTT");
        StaticJsonDocument<256> doc;
        doc["perguntaMQTT"] = "Liga ar?";
        char buffer3[256];
        serializeJson(doc, buffer3);
        client.publish(topic3, buffer3);
        Serial.println(buffer3);
        if(comando=="1"){
          //pode ligar o ar 
          Serial.println("liga o ar pelo MQTT");
          Serial.println(comando);
          break;
        } else if(comando=="0"){
          Serial.println("nao liga o ar pelo MQTT");
          Serial.println(comando);
          break;
        }
      }
    }
  }
}
void PinConfig () {
  // config das portas i/o
  pinMode(dhtPin1, INPUT);
	pinMode(pirPin1, INPUT_PULLUP);
  pinMode(sensorTensao, INPUT_PULLUP);
  pinMode(eva, OUTPUT);
  pinMode(con, OUTPUT);
}
void payloadMQTT (){ 
  datahora();
  u8x8.clear();
  int tensao=digitalRead(sensorTensao);
  int movimento=digitalRead(pirPin1);
  time_t tt=time(NULL);
  Serial.println(tt);
  // String payload = "{\"local\":";
  // payload += "\"SalaTransmisssor\"";
  // payload += ",";
  // payload += "\"hora\":";
  // payload += tt;
  // payload += ",";
  // payload += "\"temp\":";
  // payload += tempAtual;
  // payload += ",";
  // payload += "\"movimento\":";
  // payload += movimento;
  // payload += ",";
  // payload += "\"tensao\":";
  // payload += tensao;
  // payload += ",";
  // payload += "\"ip\":";
  // payload +="\"";
  // payload += ip.toString();
  // payload +="\"";
  // payload += ",";
  // payload += "\"mac\":";
  // payload +="\"";
  // payload += DEVICE_ID;
  // payload +="\"";
  // payload += "}";
  // client.publish (topic, (char*) payload.c_str());

  StaticJsonDocument<256> doc;
  doc["local"] = "Porta-Transmissor";
  doc["ip"] = ip.toString();
  doc["mac"] = mac;
  doc["hora"]=tt;
  doc["Temperatura"]=tempAtual;
  doc["movimento"]=movimento; 
  doc["rede"]=tensao;
  char buffer[256];
  serializeJson(doc, buffer);
  client.publish(topic, buffer);
  Serial.println(buffer);

  novaTemp=false;
  tensaoPin=false; 
  mov=false;
}
Ticker tickerpin(publish, PUBLISH_INTERVAL);
Ticker tempTicker(pegaTemp, 10000);
void setup(){
  Serial.begin (115200);
  iniciaWifi();
  client.setServer (BROKER_MQTT, 1883);//define mqtt
  client.setCallback(callback); 
  server.begin();
  ntp.begin ();
  ntp.forceUpdate();
  if (!ntp.forceUpdate ()){
      Serial.println ("Erro ao carregar a hora");
      delay (1000);
  } else {
    timeval tv; //define a estrutura da hora 
    tv.tv_sec = ntp.getEpochTime(); //segundos
    settimeofday (&tv, NULL);
  }
  redee();  //define as variaveis
  PinConfig();
  dhtSensor.setup(dhtPin1, DHTesp::DHT11);
  xTaskCreatePinnedToCore (sensorTemp, "sensorTemp", 4000, NULL, 1, &retornoTemp, 0);
  vTaskDelay (pdMS_TO_TICKS(500));
  xTaskCreatePinnedToCore (verificaDia, "arliga", 10000, NULL, 1, NULL, 0);
  attachInterrupt (digitalPinToInterrupt(pirPin1), mudaStatusPir, RISING);
  attachInterrupt (digitalPinToInterrupt(sensorTensao), Tensao, CHANGE);
  tickerpin.start();
  tempTicker.start();
  u8x8.begin();
  u8x8.clear();
  datahora();
  ip=WiFi.localIP(); //pega ip
  mac=DEVICE_ID;     //pega mac
}
void loop(){
  datahora();
  server.handleClient();
  reconectaMQTT();
  if(tempAtual<50){  //prevenção de erros na leitura do sensor
    publishNewState=false;
  }
  if(rede==0){
    tentaReconexao();
  }
  tempTicker.update();
  tickerpin.update();
  payloadMQTT();
  delay(5000);
}