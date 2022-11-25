/*=============================================================*/

/*=============         Hardware Wiring            ============*/
/*=============================================================*/

/*          PN5180                            ESP32 (VSPI)              ESP32 (HSPI)
 *           
 *           +5V                               +5V                          +5V
 *          +3.3V                             +3.3V                        +3.3V
 *           GND                               GND                          GND
 *           RST                               G22                          G2
 *           NSS                               G21                          G4
 *          MOSI                               G23                          G13
 *          MISO                               G19                          G12
 *          SCK                                G18                          G14
 *          BUSY                                G5                          G15
 */
 
/*****************************************************************/
/*************** Bibliotecas e Arquivos auxiliares ***************/
/*****************************************************************/

#include "secrets.h"
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include "WiFi.h"
#include <WiFiUdp.h>;
#include <NTPClient.h>;

#include <PN5180ISO15693.h>
#include <PN5180.h>

/*****************************************************************/
/************************* Definições ****************************/
/*****************************************************************/

#define AWS_IOT_PUBLISH_TOPIC   "outTopic"
#define AWS_IOT_SUBSCRIBE_TOPIC "outTopic"
WiFiClientSecure net = WiFiClientSecure();
PubSubClient client(net);
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org");
char msg[150];



#define PN5180_NSS  21
#define PN5180_BUSY 5
#define PN5180_RST  22 
ISO15693ErrorCode erro;
PN5180ISO15693 nfc(PN5180_NSS, PN5180_BUSY, PN5180_RST);
uint8_t uid[8];

/*****************************************************************/
/***************** Funções Auxiliares ****************************/
/*****************************************************************/
 
void connectAWS()
{
  // Configura o Wi-fi.
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.println("Conectando ao Wi-Fi");
  while (WiFi.status() != WL_CONNECTED)  {    delay(500);    Serial.print(".");  }
 
  // Faz upload dos certificados para a Flash da Esp32.
  net.setCACert(AWS_CERT_CA);
  net.setCertificate(AWS_CERT_CRT);
  net.setPrivateKey(AWS_CERT_PRIVATE);
 
  // Seta o broker do MQTT para o AWS endpoint.
  client.setServer(AWS_IOT_ENDPOINT, 8883);
 
  // Create a message handler
  // client.setCallback(messageHandler);
 
  Serial.println("Conectando ao AWS IOT CORE");
 
  while (!client.connect(THINGNAME))
  {
    Serial.println(THINGNAME);
    Serial.println(".");
    delay(100);
  }
 
  if (!client.connected())
  {
    Serial.println("Timeout, não conectado");
    return;
  }
 
  // Inscreve-se no tópico do PubSub
  client.subscribe(AWS_IOT_SUBSCRIBE_TOPIC);
 
  Serial.println("Conectado ao AWS IOT CORE com sucesso!");
}

/*****************************************************************/
/********************** Void Setup *******************************/
/*****************************************************************/

void setup()
{
  Serial.begin(115200);
  connectAWS();
  timeClient.setTimeOffset(-10800);

  Serial.begin(115200);                                                     // Inicialize Serial
  Serial.println(F("=================================="));
  Serial.println(F("Uploaded: " __DATE__ " " __TIME__));                    // Prints the current date&time
  Serial.println(F("PN5180 ISO15693 Glucose Reader"));
  
  
  nfc.begin();                                                              // Inicialize NFC sensor
  delay(100);
  Serial.println(F("----------------------------------"));
  Serial.println(F("PN5180 Hard-Reset..."));
  nfc.reset();                                                              // Resets and cleans the EEPROM of the NFC.
  delay(100);
  Serial.println(F("----------------------------------"));
  
  uint8_t productVersion[2];
  nfc.readEEprom(PRODUCT_VERSION, productVersion, sizeof(productVersion));  // Capture the version of the sensor.
  Serial.print(F("Product version="));
  Serial.print(productVersion[1]);
  Serial.print(".");
  Serial.println(productVersion[0]);
  delay(100);
  if (0xff == productVersion[1]) {
    Serial.println(F("Initialization failed!?"));
    Serial.println(F("Press reset to restart..."));
    Serial.flush();
    exit(-1); // halt
  }
  delay(100);

  uint8_t firmwareVersion[2];
  nfc.readEEprom(FIRMWARE_VERSION, firmwareVersion, sizeof(firmwareVersion));
  Serial.print(F("Firmware version="));
  Serial.print(firmwareVersion[1]);
  Serial.print(".");
  Serial.println(firmwareVersion[0]);
  delay(100);

  uint8_t eepromVersion[2];
  nfc.readEEprom(EEPROM_VERSION, eepromVersion, sizeof(eepromVersion));
  Serial.print(F("EEPROM version="));
  Serial.print(eepromVersion[1]);
  Serial.print(".");
  Serial.println(eepromVersion[0]);
  delay(100);
  Serial.println(F("----------------------------------"));
  Serial.println(F("Enable RF field..."));
  nfc.setupRF();
  delay(100);
}

/*****************************************************************/
/*********************** Void Loop *******************************/
/*****************************************************************/
 
void loop()
{  
  nfc.reset();
  nfc.setupRF();
  ISO15693ErrorCode rc = nfc.getInventory(uid);
  
  /*********** Atualiza Data/Hora ***********************/
  timeClient.update();
  unsigned long epochTime = timeClient.getEpochTime();
  String formattedTime = timeClient.getFormattedTime();
  const char* formattedTime2 = formattedTime.c_str();
  struct tm *ptm = gmtime ((time_t *)&epochTime);
  int currentYear = ptm->tm_year+1900;
  int currentMonth = ptm->tm_mon+1;
  int monthDay = ptm->tm_mday;                               
  String currentDate = String(currentYear) + "-" + String(currentMonth) + "-" + String(monthDay);
  const char* currentDate2 = currentDate.c_str();


/*********** Lê a glicose ***********************/
uint8_t blockSize, numBlocks;
uint8_t readBuffer[blockSize];

  ISO15693ErrorCode rc2 = nfc.getSystemInfo(uid, &blockSize, &numBlocks);
  if (ISO15693_EC_OK != rc2) {
    Serial.print(F("Error in getSystemInfo: "));
    //Serial.println(nfc.strerror(rc2));
    delay(1000);
    return;
  }
  Serial.print(F("System Info retrieved: blockSize="));
  Serial.print(blockSize);
  Serial.print(F(", numBlocks="));
  Serial.println(numBlocks);
  Serial.println(F("----------------------------------"));

  
    for (int no=1; no<numBlocks; no++) {
      if (no==243) {
    ISO15693ErrorCode rc = nfc.readSingleBlock(uid, no, readBuffer, blockSize);
    char escrita[200];
    snprintf(escrita,200,"O Serial do chip é: ", rc);
    Serial.println(escrita);
    for(int k = 0; k<sizeof(readBuffer); k++){Serial.println(readBuffer[k],HEX);}
    if (ISO15693_EC_OK != rc) {
      Serial.print(F("Error in readSingleBlock #"));
      Serial.print(no);
      Serial.print(": ");
      //Serial.println(nfc.strerror(rc3));
      return;}}}
      
    for (int no=1; no<numBlocks; no++) { 
       if (no==1) {
       ISO15693ErrorCode rc = nfc.readSingleBlock(uid, no, readBuffer, blockSize);
       char escrita[200];
       snprintf(escrita, 200, "O valor da glicose é: ", rc);
       Serial.println(escrita);
       for(int k = 0; k<sizeof(readBuffer); k++){if(k==3) {Serial.print(readBuffer[k],DEC);Serial.println("mg/dl");}}       
       if (ISO15693_EC_OK != rc) {
      Serial.print(F("Error in readSingleBlock #"));
      Serial.print(no);
      Serial.print(": ");
      //Serial.println(nfc.strerror(rc3));
      return;}}}
      
  /*********** Publica AWS ***********************/
  if(rc == ISO15693_EC_OK){
  snprintf(msg,150,"{\"data_evento\": \"%s\", \"hora_evento\": \"%s\", \"Glicose\": \"%d mg/dl\"}",currentDate2,formattedTime2,readBuffer[3]);
  client.publish(AWS_IOT_PUBLISH_TOPIC, msg);
  Serial.println("Publicado no AWS IOT CORE: " + String(msg));
  client.loop();}
 
 delay(3000);
}
