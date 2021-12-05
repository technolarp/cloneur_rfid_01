/*
   ----------------------------------------------------------------------------
   TECHNOLARP - https://technolarp.github.io/
   CLONEUR RFID 01 - https://github.com/technolarp/cloneur_rfid_01
   version 1.0 - 12/2021
   ----------------------------------------------------------------------------
*/

/*
   ----------------------------------------------------------------------------
   Pour ce montage, vous avez besoin de 
   1 lecteur RFID i2c PN532
   1 ou + led neopixel
   1 ecran oled i2c
   1 interrupteur on/off
   1 MCP23017
   1 buzzer piezo
   ----------------------------------------------------------------------------
*/

/*
   ----------------------------------------------------------------------------
   PINOUT
   D0     NEOPIXEL
   D1     I2C SCL => SCL PN532
   D2     I2C SDA => SDA PN532
   D8     BUZZER

   SPI BUS
   D3     PN532 SS/SDA
   D5     PN532 SCLK
   D6     PN532 MISO
   D7     PN532 MOSI
   ----------------------------------------------------------------------------
*/

/* TODO
faire une animation hacker
check doublon
*/

#include <Arduino.h>

// WEB SERVER
#include <ESPAsyncTCP.h>
#include <ESPAsyncWebServer.h>

AsyncWebServer server(80);

// WEBSOCKET
AsyncWebSocket ws("/ws");

char bufferWebsocket[300];
bool flagBufferWebsocket = false;

// RFID SPI
#include <SPI.h>
#include <PN532_SPI.h>
#include <PN532.h>

#include "emulatetag.h"
#include "NdefMessage.h"

PN532_SPI pn532spi(SPI,D3);
PN532 nfcRead(pn532spi);
EmulateTag nfcEmulate(pn532spi);

uint8_t ndefBuf[120];
NdefMessage message;
int messageSize;

uint8_t uidRead[7] = { 0, 0, 0, 0, 0, 0, 0 };
uint8_t uidReadLength;

uint8_t uidEmulate[3] = {0, 0, 0};

#define DELAI_RW_RFID 25

// MCP23017
#include "technolarp_mcp23017.h"
M_mcp23017 aMcp23017;

// OLED
#include <technolarp_oled.h>
M_oled aOled;

// BUZZER
#define PIN_BUZZER D8
#include <technolarp_buzzer.h>
M_buzzer buzzer(PIN_BUZZER);

// CONFIG
#include "config.h"
M_config aConfig;

char bufferToSend[500];
char bufferUid[12];

// STATUTS DE L OBJET
enum {
  OBJET_OUVERT = 0,
  OBJET_FERME = 1,
  OBJET_BLOQUE = 2,
  OBJET_ERREUR = 3,
  OBJET_RECONFIG = 4,
  OBJET_BLINK = 5,
  OBJET_LECTURE = 6,
  OBJET_EMULATION = 7
};

// DIVERS
bool uneFois = true;

uint32_t previousMillisReading;
uint32_t intervalReading;

// HEARTBEAT
unsigned long int previousMillisHB;
unsigned long int currentMillisHB;
unsigned long int intervalHB;


/*
   ----------------------------------------------------------------------------
   SETUP
   ----------------------------------------------------------------------------
*/
void setup()
{
  Serial.begin(115200);

  // VERSION
  delay(500);
  Serial.println(F(""));
  Serial.println(F(""));
  Serial.println(F("----------------------------------------------------------------------------"));
  Serial.println(F("TECHNOLARP - https://technolarp.github.io/"));
  Serial.println(F("CLONEUR RFID 01 - https://github.com/technolarp/cloneur_rfid_01"));
  Serial.println(F("version 1.0 - 11/2021"));
  Serial.println(F("----------------------------------------------------------------------------"));

  // I2C RESET
  int rtn = I2C_ClearBus(); // clear the I2C bus first before calling Wire.begin()
  if (rtn != 0)
  {
    Serial.println(F("I2C bus error. Could not clear"));
    if (rtn == 1) 
    {
      Serial.println(F("SCL clock line held low"));
    }
    else if (rtn == 2)
    {
      Serial.println(F("SCL clock line held low by slave clock stretch"));
    }
    else if (rtn == 3) 
    {
      Serial.println(F("SDA data line held low"));
    }
  }
  else
  { 
    // bus clear
    Serial.println(F("I2C bus clear"));
  }
  Serial.println("setup finished");

  // MCP23017
  aMcp23017.beginMcp23017(0);

  // OLED
  aOled.beginOled();
  
  // CONFIG OBJET
  Serial.println(F(""));
  Serial.println(F(""));
  aConfig.mountFS();
  
  aConfig.listDir("/");
  aConfig.listDir("/config");
  aConfig.listDir("/www");
  
  aConfig.printJsonFile("/config/objectconfig.txt");
  aConfig.readObjectConfig("/config/objectconfig.txt");

  //aConfig.printJsonFile("/config/networkconfig.txt");
  aConfig.readNetworkConfig("/config/networkconfig.txt");

  aOled.animationDepart();

  // CHECK RESET OBJECT CONFIG  
  if (!aMcp23017.readPin(BOUTON_2) && !aMcp23017.readPin(BOUTON_4) )
  {
    aOled.displayText("RESET", 2, true, false, false, true);
    aOled.displayText("OBJET", 2, false, true, false, false);
    
    Serial.println(F(""));
    Serial.println(F("!!! RESET OBJECT CONFIG !!!"));
    Serial.println(F(""));
    aConfig.writeDefaultObjectConfig("/config/objectconfig.txt");
    aConfig.printJsonFile("/config/objectconfig.txt");

    delay(1000);
  }

  // CHECK RESET NETWORK CONFIG  
  if (!aMcp23017.readPin(BOUTON_3) && !aMcp23017.readPin(BOUTON_4) )
  {
    aOled.displayText("RESET", 2, true, false, false, true);
    aOled.displayText("NETWORK", 2, false, true, false, false);
    
    Serial.println(F(""));
    Serial.println(F("!!! RESET NETWORK CONFIG !!!"));
    Serial.println(F(""));
    aConfig.writeDefaultNetworkConfig("/config/networkconfig.txt");
    aConfig.printJsonFile("/config/networkconfig.txt");

    delay(1000);
  }

  // WIFI
  WiFi.disconnect(true);
  Serial.println(F(""));
  Serial.println(F("connecting WiFi"));
  
  
  // AP MODE
  WiFi.mode(WIFI_AP_STA);
  WiFi.softAPConfig(aConfig.networkConfig.apIP, aConfig.networkConfig.apIP, aConfig.networkConfig.apNetMsk);
  WiFi.softAP(aConfig.networkConfig.apName, aConfig.networkConfig.apPassword);
  
  /*
  // CLIENT MODE POUR DEBUG
  const char* ssid = "MYDEBUG";
  const char* password = "ppppppp";
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  */
  
  if (WiFi.waitForConnectResult() != WL_CONNECTED) 
  {
    Serial.printf("WiFi Failed!\n");
  }
  else
  {
    Serial.println(F("WiFi OK"));
  }
  
  // WEB SERVER
  // Print ESP Local IP Address
  Serial.print(F("localIP: "));
  Serial.println(WiFi.localIP());
  Serial.print(F("softAPIP: "));
  Serial.println(WiFi.softAPIP());

  // Route for root / web page
  server.serveStatic("/", LittleFS, "/www/").setDefaultFile("config.html");
  server.serveStatic("/config", LittleFS, "/config/");
  server.onNotFound(notFound);

  // WEBSOCKET
  ws.onEvent(onEvent);
  server.addHandler(&ws);

  // Start server
  server.begin();

  // HEARTBEAT
  currentMillisHB = millis();
  previousMillisHB = currentMillisHB;
  intervalHB = 5000;

  // RFID READING
  previousMillisReading = millis();
  intervalReading = 500;

  // BUZZER
  buzzer.doubleBeep();

  // SERIAL
  Serial.println(F(""));
  Serial.println(F(""));
  Serial.println(F("START !!!"));
}
/*
   ----------------------------------------------------------------------------
   FIN DU SETUP
   ----------------------------------------------------------------------------
*/




/*
   ----------------------------------------------------------------------------
   LOOP
   ----------------------------------------------------------------------------
*/
void loop()
{  
  // avoid watchdog reset
  yield();
  
  // WEBSOCKET
  ws.cleanupClients();

  // BUZZER
  buzzer.update();

  // OLED
  aOled.updateAnimation();
  
  // check selecteur mode
  if (aConfig.objectConfig.statutActuel != OBJET_BLINK)
  {
    if (aMcp23017.readPin(BOUTON_1))
    {
      aConfig.objectConfig.statutActuel = OBJET_LECTURE;
    }
    else
    {
      aConfig.objectConfig.statutActuel = OBJET_EMULATION;
    }
  
    // changement de mode
    if (aConfig.objectConfig.statutActuel != aConfig.objectConfig.statutPrecedent)
    {
      uneFois = true;
      aConfig.objectConfig.statutPrecedent = aConfig.objectConfig.statutActuel;
      buzzer.shortBeep();
      sendStatut();
    }
  }
  
  // gerer le statut du cloneur
  switch (aConfig.objectConfig.statutActuel)
  {
    case OBJET_LECTURE:
      // la serrure est fermee
      cloneurLecture();
      break;

    case OBJET_EMULATION:
      // la serrure est ouverte
      cloneurEmulation();
      break;

    case OBJET_BLINK:
      // la serrure est ouverte
      cloneurBlink();
      break;
      
    default:
      // nothing
      break;
  }

  // traite le buffer du websocket
  if (flagBufferWebsocket)
  {
    flagBufferWebsocket = false;
    handleWebsocketBuffer();
  }

  // HEARTBEAT
  currentMillisHB = millis();
  if(currentMillisHB - previousMillisHB > intervalHB)
  {
    previousMillisHB = currentMillisHB;
    
    // send new value to html
    sendUptime();
  }
}
/*
   ----------------------------------------------------------------------------
   FIN DU LOOP
   ----------------------------------------------------------------------------
*/





/*
   ----------------------------------------------------------------------------
   FONCTIONS ADDITIONNELLES
   ----------------------------------------------------------------------------
*/

void cloneurLecture()
{
  if (uneFois)
  {
    uneFois = false;
    aOled.displayText("SCAN TAG", 2, true, true, false, true);
    
    Serial.print(F("CLONEUR SCAN TAG"));
    Serial.println();

    // RFID
    Serial.println("NDEF Reader");
    nfcRead.begin();
  
    uint32_t versiondata = nfcRead.getFirmwareVersion();
    if (! versiondata) 
    {
      Serial.print("Didn't find PN53x board");
      while (1); // halt
    }
    // Got ok data, print it out!
    Serial.print("Found chip PN5"); Serial.println((versiondata>>24) & 0xFF, HEX); 
    Serial.print("Firmware ver. "); Serial.print((versiondata>>16) & 0xFF, DEC); 
    Serial.print('.'); Serial.println((versiondata>>8) & 0xFF, DEC);
    
    // configure board to read RFID tags
    nfcRead.SAMConfig();
  }
  
  // on check si un tag rfid est present
  readRfidTag();
}

// check rfid tag
void readRfidTag()
{
  if(millis() - previousMillisReading > intervalReading)
  {
    previousMillisReading = millis();
    uint8_t success = nfcRead.readPassiveTargetID(PN532_MIFARE_ISO14443A, uidRead, &uidReadLength, DELAI_RW_RFID);
    if (success)
    {
      // Display some basic information about the card
      nfcRead.PrintHex(uidRead, uidReadLength);
  
      aOled.displayText("SCAN TAG", 2, true, false, false, true);
      stringTagUid(bufferUid);      
      aOled.displayText(bufferUid, 1, false, true, false, false);

      sendTagUid();
      buzzer.shortBeep();
    }
  }
  
  // ajout d'un uid
  if (aMcp23017.checkButton(BOUTON_4))
  {
    if (aConfig.objectConfig.nbTagEnMemoireActuel<aConfig.objectConfig.nbTagEnMemoireMax)
    {
      for (uint8_t i=0;i<SIZE_UID;i++)
      {
        aConfig.objectConfig.tagUid[aConfig.objectConfig.nbTagEnMemoireActuel][i]=uidRead[i];
      }
      
      aConfig.objectConfig.nbTagEnMemoireActuel+=1;      
      writeObjectConfig();
      
      aOled.displayText("SCAN TAG", 2, true, true, false, true);
      stringTagUid(bufferUid);
      aOled.displayText(bufferUid, 1, false, false, false, true);
      aOled.displayText("ajout tag OK", 1, false, true, false, false);

      sendObjectConfig();
      
      buzzer.doubleBeep();
    
      Serial.println("ajout tag OK");
    }
    else
    {
      aOled.displayText("SCAN TAG", 2, true, true, false, true);
      stringTagUid(bufferUid);
      aOled.displayText(bufferUid, 1, false, false, false, true);
      aOled.displayText("trop de tag...", 1, false, true, false, false);

      buzzer.longBeep();
      Serial.println("trop de tag...");
    }
  }
}

void cloneurEmulation()
{
  if (uneFois)
  {
    uneFois = false;

    Serial.print(F("CLONEUR EMULATION TAG"));
    Serial.println();

    aOled.displayText("EMULATION", 2, true, true, false, false);

    for (uint8_t i=0;i<SIZE_UID-1;i++)
    {
      uidEmulate[i]=aConfig.objectConfig.tagUid[aConfig.objectConfig.indexTagActuel][i+1];
    }
    
    // uid must be 3 bytes!
    nfcEmulate.setUid(uidEmulate);
  
    nfcEmulate.init();
  }
  
  // emulation
  emulateRfidTag();
}

void emulateRfidTag()
{
  nfcEmulate.emulate(DELAI_RW_RFID);

  aOled.displayText("EMULATION", 2, true, false, false, true);
  
  for (uint8_t i=0;i<min<uint8_t>(aConfig.objectConfig.nbTagEnMemoireActuel,aConfig.objectConfig.nbTagEnMemoireMax);i++)
  {
    uint8_t indexTmp = i+1;
    
    snprintf(bufferToSend, 500, "%02X:%02X:%02X:%02X %i/%i", 
                  aConfig.objectConfig.tagUid[i][0], 
                  aConfig.objectConfig.tagUid[i][1], 
                  aConfig.objectConfig.tagUid[i][2], 
                  aConfig.objectConfig.tagUid[i][3], 
                  indexTmp, 
                  aConfig.objectConfig.nbTagEnMemoireActuel);
    
    if (i==aConfig.objectConfig.indexTagActuel)
    {
      aOled.displayText(bufferToSend, 1, false, false, false, true, true);
    }
    else
    {
      aOled.displayText(bufferToSend, 1, false, false, false, true, false);
    }
  }
  aOled.display();

  // choix uid bas
  if (aMcp23017.checkButton(BOUTON_2))
  {
    if (aConfig.objectConfig.indexTagActuel > 0)
    {
      aConfig.objectConfig.indexTagActuel-=1;
      writeObjectConfig();
      uneFois = true;
    }
  }

  // choix uid haut
  if (aMcp23017.checkButton(BOUTON_3))
  {
    if ( (aConfig.objectConfig.indexTagActuel < (aConfig.objectConfig.nbTagEnMemoireActuel-1)) && (aConfig.objectConfig.indexTagActuel < aConfig.objectConfig.nbTagEnMemoireMax-1) )
    {
      aConfig.objectConfig.indexTagActuel+=1;
      writeObjectConfig();
      uneFois = true;
    }
  }

  // supprime un uid
  if (aMcp23017.checkButton(BOUTON_4))
  {
    removeUid(aConfig.objectConfig.indexTagActuel);
    
    writeObjectConfig();
    sendObjectConfig();

    buzzer.doubleBeep();
  }
}

void cloneurBlink()
{
  if (uneFois)
  {
    uneFois = false;

    Serial.println(F("CLONEUR BLINK"));

    //aOled.animationBlink01Start(300, 20);
    aOled.animationBlink02Start(300, 3000);
  }

  // fin de l'animation blink
  if(!aOled.isAnimActive()) 
  {
    aConfig.objectConfig.statutActuel = aConfig.objectConfig.statutPrecedent;
    aConfig.objectConfig.statutPrecedent = OBJET_BLINK;
    writeObjectConfig();
    sendObjectConfig();
  }
}

void checkCharacter(char* toCheck, char* allowed, char replaceChar)
{
  for (int i = 0; i < strlen(toCheck); i++)
  {
    if (!strchr(allowed, toCheck[i]))
    {
      toCheck[i]=replaceChar;
    }
    Serial.print(toCheck[i]);
  }
  Serial.println("");
}

uint16_t checkValeur(uint16_t valeur, uint16_t minValeur, uint16_t maxValeur)
{
  return(min(max(valeur,minValeur), maxValeur));
}


int I2C_ClearBus()
{
  /**
 * This routine turns off the I2C bus and clears it
 * on return SCA and SCL pins are tri-state inputs.
 * You need to call Wire.begin() after this to re-enable I2C
 * This routine does NOT use the Wire library at all.
 *
 * returns 0 if bus cleared
 *         1 if SCL held low.
 *         2 if SDA held low by slave clock stretch for > 2sec
 *         3 if SDA held low after 20 clocks.
 *         
 * from: 
 * https://github.com/esp8266/Arduino/issues/1025
 * http://www.forward.com.au/pfod/ArduinoProgramming/I2C_ClearBus/index.html
 */
#if defined(TWCR) && defined(TWEN)
  TWCR &= ~(_BV(TWEN)); //Disable the Atmel 2-Wire interface so we can control the SDA and SCL pins directly
#endif
  pinMode(SDA, INPUT_PULLUP); // Make SDA (data) and SCL (clock) pins Inputs with pullup.
  pinMode(SCL, INPUT_PULLUP);

  delay(500);  // Wait 2.5 secs. This is strictly only necessary on the first power
  // up of the DS3231 module to allow it to initialize properly,
  // but is also assists in reliable programming of FioV3 boards as it gives the
  // IDE a chance to start uploaded the program
  // before existing sketch confuses the IDE by sending Serial data.

  boolean SCL_LOW = (digitalRead(SCL) == LOW); // Check is SCL is Low.
  if (SCL_LOW)
  { //If it is held low Arduno cannot become the I2C master. 
    return 1; //I2C bus error. Could not clear SCL clock line held low
  }

  boolean SDA_LOW = (digitalRead(SDA) == LOW);  // vi. Check SDA input.
  int clockCount = 20; // > 2x9 clock

  while (SDA_LOW && (clockCount > 0)) 
  { //  vii. If SDA is Low,
    clockCount--;
    // Note: I2C bus is open collector so do NOT drive SCL or SDA high.
    pinMode(SCL, INPUT); // release SCL pullup so that when made output it will be LOW
    pinMode(SCL, OUTPUT); // then clock SCL Low
    delayMicroseconds(10); //  for >5uS
    pinMode(SCL, INPUT); // release SCL LOW
    pinMode(SCL, INPUT_PULLUP); // turn on pullup resistors again
    // do not force high as slave may be holding it low for clock stretching.
    delayMicroseconds(10); //  for >5uS
    // The >5uS is so that even the slowest I2C devices are handled.
    SCL_LOW = (digitalRead(SCL) == LOW); // Check if SCL is Low.
    int counter = 20;
    while (SCL_LOW && (counter > 0)) 
    {  //  loop waiting for SCL to become High only wait 2sec.
      counter--;
      delay(100);
      SCL_LOW = (digitalRead(SCL) == LOW);
    }
    if (SCL_LOW)
    { // still low after 2 sec error
      return 2; // I2C bus error. Could not clear. SCL clock line held low by slave clock stretch for >2sec
    }
    SDA_LOW = (digitalRead(SDA) == LOW); //   and check SDA input again and loop
  }
  if (SDA_LOW)
  { // still low
    return 3; // I2C bus error. Could not clear. SDA data line held low
  }

  // else pull SDA line low for Start or Repeated Start
  pinMode(SDA, INPUT); // remove pullup.
  pinMode(SDA, OUTPUT);  // and then make it LOW i.e. send an I2C Start or Repeated start control.
  // When there is only one I2C master a Start or Repeat Start has the same function as a Stop and clears the bus.
  /// A Repeat Start is a Start occurring after a Start with no intervening Stop.
  delayMicroseconds(10); // wait >5uS
  pinMode(SDA, INPUT); // remove output low
  pinMode(SDA, INPUT_PULLUP); // and make SDA high i.e. send I2C STOP control.
  delayMicroseconds(10); // x. wait >5uS
  pinMode(SDA, INPUT); // and reset pins as tri-state inputs which is the default state on reset
  pinMode(SCL, INPUT);
  return 0; // all ok
}


void printTagUid()
{
  stringTagUid(bufferUid);
  Serial.println(bufferUid);
}

void stringTagUid(char * target)
{
  snprintf(target, 12, "%02x:%02x:%02x:%02x", uidRead[0], uidRead[1], uidRead[2], uidRead[3]);
}

void stringTagUid(char * target, uint8_t uidToStr)
{
  snprintf(target, 12, "%02x:%02x:%02x:%02x", 
            aConfig.objectConfig.tagUid[uidToStr][0], 
            aConfig.objectConfig.tagUid[uidToStr][1], 
            aConfig.objectConfig.tagUid[uidToStr][2], 
            aConfig.objectConfig.tagUid[uidToStr][3]);
}

void removeUid(uint8_t uidToRemove)
{
  uint8_t toRemove = min<uint8_t>(MAX_NB_TAG-1, uidToRemove);
  
  if (aConfig.objectConfig.nbTagEnMemoireActuel>0)
  {
    for (uint8_t i=toRemove;i<aConfig.objectConfig.nbTagEnMemoireActuel-1;i++)
    {
      for (uint8_t j=0;j<SIZE_UID;j++)
      {
        aConfig.objectConfig.tagUid[i][j]=aConfig.objectConfig.tagUid[i+1][j];
      }
    }

    for (uint8_t i=aConfig.objectConfig.nbTagEnMemoireActuel-1;i<MAX_NB_TAG;i++)
    {
      for (uint8_t j=0;j<SIZE_UID;j++)
      {
        aConfig.objectConfig.tagUid[i][j]=0;
      }
    }
    
    aConfig.objectConfig.nbTagEnMemoireActuel--;
    if (aConfig.objectConfig.indexTagActuel>0)
    {
      aConfig.objectConfig.indexTagActuel--;
    }
  
    Serial.println("remove done");
  }
}

void sendObjectConfig()
{
  aConfig.stringJsonFile("/config/objectconfig.txt", bufferToSend, 500);
  ws.textAll(bufferToSend);
}

void writeObjectConfig()
{
  aConfig.writeObjectConfig("/config/objectconfig.txt");
}

void sendNetworkConfig()
{
  aConfig.stringJsonFile("/config/networkconfig.txt", bufferToSend, 500);
  ws.textAll(bufferToSend);
}

void writeNetworkConfig()
{
  aConfig.writeNetworkConfig("/config/networkconfig.txt");
}

void sendUptime()
{
  uint32_t now = millis() / 1000;
  uint16_t days = now / 86400;
  uint16_t hours = (now%86400) / 3600;
  uint16_t minutes = (now%3600) / 60;
  uint16_t seconds = now % 60;
    
  char toSend[100];
  snprintf(toSend, 100, "{\"uptime\":\"%id %ih %im %is\"}", days, hours, minutes, seconds);

  ws.textAll(toSend);
  //Serial.println(toSend);
}

void sendStatut()
{
  char toSend[100];
  snprintf(toSend, 100, "{\"statutActuel\":%i}", aConfig.objectConfig.statutActuel); 

  ws.textAll(toSend);
}

void sendTagUid()
{
  char toSend[100];
  snprintf(toSend, 100, "{\"lastTagUid\":[\"%02X\",\"%02X\",\"%02X\",\"%02X\"]}", uidRead[0], uidRead[1], uidRead[2], uidRead[3]);

  ws.textAll(toSend);
}


void notFound(AsyncWebServerRequest *request)
{
    request->send(404, "text/plain", "Not found");
}



void onEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len) 
{
   switch (type) 
    {
      case WS_EVT_CONNECT:
        Serial.printf("WebSocket client #%u connected from %s\n", client->id(), client->remoteIP().toString().c_str());
        // send config value to html
        sendObjectConfig();
        sendNetworkConfig();

        // send volatile info
        sendUptime();
        sendStatut();
        
        break;
        
      case WS_EVT_DISCONNECT:
        Serial.printf("WebSocket client #%u disconnected\n", client->id());
        break;
        
      case WS_EVT_DATA:
        handleWebSocketMessage(arg, data, len);
        break;
        
      case WS_EVT_PONG:
      case WS_EVT_ERROR:
        break;
  }
}


void handleWebSocketMessage(void *arg, uint8_t *data, size_t len) 
{
  AwsFrameInfo *info = (AwsFrameInfo*)arg;
  if (info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT) 
  {
    data[len] = 0;
    sprintf(bufferWebsocket,"%s\n", (char*)data);
    Serial.print(len);
    Serial.print(bufferWebsocket);
    flagBufferWebsocket = true;
  }
}

void handleWebsocketBuffer()
{
  StaticJsonDocument<1024> doc;

  DeserializationError error = deserializeJson(doc, bufferWebsocket);
  if (error)
  {
    Serial.println(F("Failed to deserialize buffer in websocket"));
  }
  else
  {
    // write config or not
    bool writeObjectConfigFlag = false;
    bool sendObjectConfigFlag = false;
    bool writeNetworkConfigFlag = false;
    bool sendNetworkConfigFlag = false;
    
    // **********************************************
    // modif object config
    // **********************************************
    if (doc.containsKey("new_objectName"))
    {
      strlcpy(  aConfig.objectConfig.objectName,
                doc["new_objectName"],
                sizeof(aConfig.objectConfig.objectName));
  
      writeObjectConfigFlag = true;
      sendObjectConfigFlag = true;
    }
    
    if (doc.containsKey("new_objectId")) 
    {
      uint16_t tmpValeur = doc["new_objectId"];
      aConfig.objectConfig.objectId = checkValeur(tmpValeur,1,1000);
  
      writeObjectConfigFlag = true;
      sendObjectConfigFlag = true;
    }
  
    if (doc.containsKey("new_groupId")) 
    {
      uint16_t tmpValeur = doc["new_groupId"];
      aConfig.objectConfig.groupId = checkValeur(tmpValeur,1,1000);
      
      writeObjectConfigFlag = true;
      sendObjectConfigFlag = true;
    }
  
    if (doc.containsKey("new_nbTagEnMemoireMax")) 
    {
      uint16_t tmpValeur = doc["new_nbTagEnMemoireMax"];
      aConfig.objectConfig.nbTagEnMemoireMax = checkValeur(tmpValeur,1,5);
      aConfig.objectConfig.nbTagEnMemoireActuel=min<uint8_t>(aConfig.objectConfig.nbTagEnMemoireActuel,aConfig.objectConfig.nbTagEnMemoireMax);
      
      writeObjectConfigFlag = true;
      sendObjectConfigFlag = true;
    }
  
    if ( doc.containsKey("new_removeUid") && doc["new_removeUid"]>=0 && doc["new_removeUid"]<10 )
    {
      uint16_t uidToRemove = doc["new_removeUid"];
      Serial.print(F("Remove tag UID: "));
      Serial.println(uidToRemove);
  
      removeUid(uidToRemove);
  
      writeObjectConfigFlag = true;
      sendObjectConfigFlag = true;
    }
  
    
    if ( doc.containsKey("new_addLastUid") && doc["new_addLastUid"]==1 )
    {
      if (aConfig.objectConfig.nbTagEnMemoireActuel<MAX_NB_TAG)
      {
        for (uint8_t  i=0;i<uidReadLength;i++)
        {
          aConfig.objectConfig.tagUid[aConfig.objectConfig.nbTagEnMemoireActuel][i]=uidRead[i];
        }
        aConfig.objectConfig.nbTagEnMemoireActuel++;          
  
        writeObjectConfigFlag = true;
        sendObjectConfigFlag = true;
      }
      else
      {
        Serial.println(F("trop de tags"));
      }
    }
    
    if (doc.containsKey("new_newTagUid")) 
    {
      if (aConfig.objectConfig.nbTagEnMemoireActuel<MAX_NB_TAG)
      {
        JsonArray newUidValue = doc["new_newTagUid"];
          
        for (uint8_t  i=0;i<SIZE_UID;i++)
        {
          const char* tagUid_0 = newUidValue[i];
          uint8_t hexValue = (uint8_t) strtol( &tagUid_0[0], NULL, 16);
          
          aConfig.objectConfig.tagUid[aConfig.objectConfig.nbTagEnMemoireActuel][i]=hexValue;
          uidRead[i]=hexValue;
        }
        aConfig.objectConfig.nbTagEnMemoireActuel++;          
  
        writeObjectConfigFlag = true;
        sendObjectConfigFlag = true;
      }
      else
      {
        Serial.println(F("trop de tags"));
      }
    }

    if (doc.containsKey("new_statutActuel"))
    {
      aConfig.objectConfig.statutPrecedent = aConfig.objectConfig.statutActuel;
      aConfig.objectConfig.statutActuel = doc["new_statutActuel"];

      uneFois = true;
        
      writeObjectConfigFlag = true;
      sendObjectConfigFlag = true;
    }
      
    // **********************************************
    // modif network config
    // **********************************************
    if (doc.containsKey("new_apName")) 
    {
      strlcpy(  aConfig.networkConfig.apName,
                doc["new_apName"],
                sizeof(aConfig.networkConfig.apName));
  
      // check for unsupported char
      checkCharacter(aConfig.networkConfig.apName, "ABCDEFGHIJKLMNOPQRSTUVWYZ", 'A');
      
      writeNetworkConfigFlag = true;
      sendNetworkConfigFlag = true;
    }
  
    if (doc.containsKey("new_apPassword")) 
    {
      strlcpy(  aConfig.networkConfig.apPassword,
                doc["new_apPassword"],
                sizeof(aConfig.networkConfig.apPassword));
  
      writeNetworkConfigFlag = true;
    }
  
    if (doc.containsKey("new_apIP")) 
    {
      char newIPchar[16] = "";
  
      strlcpy(  newIPchar,
                doc["new_apIP"],
                sizeof(newIPchar));
  
      IPAddress newIP;
      if (newIP.fromString(newIPchar)) 
      {
        Serial.println("valid IP");
        aConfig.networkConfig.apIP = newIP;
  
        writeNetworkConfigFlag = true;
      }
      
      sendNetworkConfigFlag = true;
    }
  
    if (doc.containsKey("new_apNetMsk")) 
    {
      char newNMchar[16] = "";
  
      strlcpy(  newNMchar,
                doc["new_apNetMsk"],
                sizeof(newNMchar));
  
      IPAddress newNM;
      if (newNM.fromString(newNMchar)) 
      {
        Serial.println("valid netmask");
        aConfig.networkConfig.apNetMsk = newNM;
  
        writeNetworkConfigFlag = true;
      }
  
      sendNetworkConfigFlag = true;
    }
  
    if ( doc.containsKey("new_defaultObjectConfig") && doc["new_defaultObjectConfig"]==1 )
    {
      Serial.println(F("reset to default object config"));
      aConfig.writeDefaultObjectConfig("/config/objectconfig.txt");
      
      sendObjectConfigFlag = true;
      uneFois = true;
    }
  
    if ( doc.containsKey("new_defaultNetworkConfig") && doc["new_defaultNetworkConfig"]==1 )
    {
      Serial.println(F("reset to default network config"));
      aConfig.writeDefaultNetworkConfig("/config/networkconfig.txt");
      
      sendNetworkConfigFlag = true;
    }
    
    // actions sur le esp8266
    if ( doc.containsKey("new_restart") && doc["new_restart"]==1 )
    {
      Serial.println(F("RESTART RESTART RESTART"));
      ESP.restart();
    }
  
    if ( doc.containsKey("new_refresh") && doc["new_refresh"]==1 )
    {
      Serial.println(F("REFRESH"));
      sendObjectConfigFlag = true;
      sendNetworkConfigFlag = true;
    }
  
    // modif config
    // write object config
    if (writeObjectConfigFlag)
    {
      writeObjectConfig();
      aConfig.printJsonFile("/config/objectconfig.txt");
      
      // update statut
      uneFois = true;
    }
  
    // resend object config
    if (sendObjectConfigFlag)
    {
      sendObjectConfig();
    }
  
    // write network config
    if (writeNetworkConfigFlag)
    {
      writeNetworkConfig();
    }
  
    // resend network config
    if (sendNetworkConfigFlag)
    {
      sendNetworkConfig();
    }
  }
  
  // clear json buffer
  doc.clear();
}
