/*
   ----------------------------------------------------------------------------
   TECHNOLARP - https://technolarp.github.io/
   CLONEUR RFID 01 - https://github.com/technolarp/cloneur_rfid_01
   version 1.0 - 11/2021
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
   ----------------------------------------------------------------------------
*/

/*
   ----------------------------------------------------------------------------
   PINOUT
   D0     NEOPIXEL
   D1     I2C SCL => SCL PN532
   D2     I2C SDA => SDA PN532
   
   ----------------------------------------------------------------------------
*/

/*
TODO
choose delay emaulate et lecture dans un param
code cleanup
faire un display menu plus propre
faire une validation delete tag
*/

#include <Arduino.h>

// WEB SERVER
#include <ESPAsyncTCP.h>
#include <ESPAsyncWebServer.h>

AsyncWebServer server(80);

// WEBSOCKET
AsyncWebSocket ws("/ws");


// RFID
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


// MCP23017
#include "technolarp_mcp23017.h"
//M_mcp23017 aMcp23017(0);
M_mcp23017 aMcp23017;

// OLED
#include <technolarp_oled.h>
M_oled aOled;

// CONFIG
#include "config.h"
M_config aConfig;


// STATUTS DU CLONEUR
enum {
  CLONEUR_LECTURE = 0,
  CLONEUR_EMULATION = 1
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
  //void M_oled::displayText(String texteAAfficher, int taillePolice, bool videEcran, bool changementEcran, bool centered, bool crlf)
  aOled.displayText("SETUP", 3, true, true, true, false);
  
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

  // WIFI
  WiFi.disconnect(true);
  aOled.displayText("Init", 3, true, true, true, false);
  Serial.println(F("connect to WiFi"));
  
  /*
  // AP MODE
  WiFi.mode(WIFI_AP_STA);
  WiFi.softAPConfig(aConfig.networkConfig.apIP, aConfig.networkConfig.apIP, aConfig.networkConfig.apNetMsk);
  WiFi.softAP(aConfig.networkConfig.apName, aConfig.networkConfig.apPassword);
  */
  
  // CLIENT MODE POUR DEBUG
  const char* ssid = "MYDEBUG";
  const char* password = "ppppppppp";
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  
  if (WiFi.waitForConnectResult() != WL_CONNECTED) 
  {
    Serial.printf("WiFi Failed!\n");
  }
  
  // WEB SERVER
  // Print ESP Local IP Address
  Serial.print(F("localIP: "));
  Serial.println(WiFi.localIP());
  Serial.print(F("softAPIP: "));
  Serial.println(WiFi.softAPIP());

  // Route for root / web page
  server.serveStatic("/", LittleFS, "/www/").setDefaultFile("config.html").setTemplateProcessor(processor);
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

  previousMillisReading = millis();
  intervalReading = 500;

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

  // INTER
  if (aMcp23017.readPin(BOUTON_1))
  {
    aConfig.objectConfig.statutCloneurActuel = CLONEUR_LECTURE;
  }
  else
  {
    aConfig.objectConfig.statutCloneurActuel = CLONEUR_EMULATION;
  }

  if (aConfig.objectConfig.statutCloneurActuel != aConfig.objectConfig.statutCloneurPrecedent)
  {
    uneFois = true;
    aConfig.objectConfig.statutCloneurPrecedent = aConfig.objectConfig.statutCloneurActuel;
  }

  // gerer le statut du cloneur
  switch (aConfig.objectConfig.statutCloneurActuel)
  {
    case CLONEUR_LECTURE:
      // la serrure est fermee
      cloneurLecture();
      break;

    case CLONEUR_EMULATION:
      // la serrure est ouverte
      cloneurEmulation();
      break;
      
    default:
      // nothing
      break;
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

    sendStatutCloneur();
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
    uint8_t success = nfcRead.readPassiveTargetID(PN532_MIFARE_ISO14443A, uidRead, &uidReadLength, 50);
    if (success)
    {
      // Display some basic information about the card
      nfcRead.PrintHex(uidRead, uidReadLength);
  
      //void M_oled::displayText(String texteAAfficher, int taillePolice, bool videEcran, bool changementEcran, bool centered, bool crlf)
      aOled.displayText("SCAN TAG", 2, true, false, false, true);
      aOled.displayText(stringTagUid(), 1, false, true, false, false);

      sendTagUid();
    }
  }
  
  if (aMcp23017.checkButton(BOUTON_4))
  {
    if (aConfig.objectConfig.nbTag<4)
    {
      for (uint8_t i=0;i<4;i++)
      {
        aConfig.objectConfig.tagUid[aConfig.objectConfig.nbTag][i]=uidRead[i];
      }
      
      aConfig.objectConfig.nbTag+=1;
      
      aConfig.writeObjectConfig("/config/objectconfig.txt");
      ws.textAll(aConfig.stringJsonFile("/config/objectconfig.txt"));
      
      
      aOled.displayText("SCAN TAG", 2, true, true, false, true);
      aOled.displayText(stringTagUid(), 1, false, false, false, true);
      aOled.displayText("ajout tag OK", 1, false, true, false, false);
      Serial.println("ajout tag OK");
    }
    else
    {
      aOled.displayText("SCAN TAG", 2, true, true, false, true);
      aOled.displayText(stringTagUid(), 1, false, false, false, true);
      aOled.displayText("trop de tag...", 1, false, true, false, false);
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

    //void M_oled::displayText(String texteAAfficher, int taillePolice, bool videEcran, bool changementEcran, bool centered, bool crlf)
    aOled.displayText("EMULATION", 2, true, true, false, false);

    Serial.println("------- Emulate Tag --------");

    message = NdefMessage();
    message.addUriRecord("cloneur RFID");
    messageSize = message.getEncodedSize();
    if (messageSize > sizeof(ndefBuf))
    {
      Serial.println("ndefBuf is too small");
      while (1)
      {
      }
    }

    for (uint8_t i=0;i<3;i++)
    {
      uidEmulate[i]=aConfig.objectConfig.tagUid[aConfig.objectConfig.indexTag][i+1];
    }
  
    Serial.print("Ndef encoded message size: ");
    Serial.println(messageSize);
  
    message.encode(ndefBuf);
  
    // comment out this command for no ndef message
    nfcEmulate.setNdefFile(ndefBuf, messageSize);
  
    // uid must be 3 bytes!
    nfcEmulate.setUid(uidEmulate);
  
    nfcEmulate.init();

    sendStatutCloneur();
  }
  
  
  
  // emulation
  emulateRfidTag();
}



void emulateRfidTag()
{
  nfcEmulate.emulate(25);

  //void M_oled::displayText(String texteAAfficher, int taillePolice, bool videEcran, bool changementEcran, bool centered, bool crlf)
  aOled.displayText("EMULATION", 2, true, false, false, true);
  
  for (uint8_t i=0;i<min<uint8_t>(4,aConfig.objectConfig.nbTag);i++)
  {
    uint8_t indexTmp = i+1;
    String toShow = stringTagUid(i) + "  " + indexTmp + "/" + aConfig.objectConfig.nbTag;
    
    if (i==aConfig.objectConfig.indexTag)
    {
      aOled.displayText(toShow, 1, false, false, false, true, true);
    }
    else
    {
      aOled.displayText(toShow, 1, false, false, false, true, false);
    }
  }
  aOled.display();

  // choix uid
  if (aMcp23017.checkButton(BOUTON_2))
  {
    if (aConfig.objectConfig.indexTag > 0)
    {
      aConfig.objectConfig.indexTag-=1;
      uneFois = true;
    }
  }
  
  if (aMcp23017.checkButton(BOUTON_3))
  {
    if ( (aConfig.objectConfig.indexTag < 3) && (aConfig.objectConfig.indexTag < aConfig.objectConfig.nbTag-1) )
    {
      aConfig.objectConfig.indexTag+=1;
      //aConfig.objectConfig.indexTag = min<int8_t>(aConfig.objectConfig.indexTag, aConfig.objectConfig.nbTag-1);
      uneFois = true;
    }
  }

  if (aMcp23017.checkButton(BOUTON_4))
  {
    removeUid(aConfig.objectConfig.indexTag);
    
    aConfig.writeObjectConfig("/config/objectconfig.txt");
    ws.textAll(aConfig.stringJsonFile("/config/objectconfig.txt"));
  }
}

String processor(const String& var)
{  
  if (var == "APNAME")
  {
    return String(aConfig.networkConfig.apName);
  }

  if (var == "OBJECTNAME")
  {
    return String(aConfig.objectConfig.objectName);
  }
   
  return String();
}


void onEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len) 
{
   switch (type) 
    {
      case WS_EVT_CONNECT:
        Serial.printf("WebSocket client #%u connected from %s\n", client->id(), client->remoteIP().toString().c_str());
        // send config value to html
        ws.textAll(aConfig.stringJsonFile("/config/objectconfig.txt"));
        ws.textAll(aConfig.stringJsonFile("/config/networkconfig.txt"));

        // send volatile info
        sendUptime();
        sendStatutCloneur();
        
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
    char buffer[300];
    data[len] = 0;
    sprintf(buffer,"%s\n", (char*)data);
    Serial.print(buffer);
    
    //StaticJsonDocument<256> doc;
    DynamicJsonDocument doc(2048);
    DeserializationError error = deserializeJson(doc, buffer);
    if (error)
    {
      Serial.println(F("Failed to deserialize buffer in websocket"));
    }
    else
    {
      // write config or not
      bool writeObjectConfig = false;
      bool sendObjectConfig = false;
      bool writeNetworkConfig = false;
      bool sendNetworkConfig = false;
      
      // modif object config
      if (doc.containsKey("new_objectName"))
      {
        strlcpy(  aConfig.objectConfig.objectName,
                  doc["new_objectName"],
                  sizeof(aConfig.objectConfig.objectName));

        writeObjectConfig = true;
        sendObjectConfig = true;
      }
      
      if (doc.containsKey("new_objectId")) 
      {
        uint16_t tmpValeur = doc["new_objectId"];
        aConfig.objectConfig.objectId = checkValeur(tmpValeur,1,1000);

        writeObjectConfig = true;
        sendObjectConfig = true;
      }

      if (doc.containsKey("new_groupId")) 
      {
        uint16_t tmpValeur = doc["new_groupId"];
        aConfig.objectConfig.groupId = checkValeur(tmpValeur,1,1000);
        
        writeObjectConfig = true;
        sendObjectConfig = true;
      }

      if ( doc.containsKey("new_removeUid") && doc["new_removeUid"]>=0 && doc["new_removeUid"]<10 )
      {
        uint16_t uidToRemove = doc["new_removeUid"];
        Serial.print(F("Remove tag UID: "));
        Serial.println(uidToRemove);
        //Serial.println(aConfig.objectConfig.nbTag);

        removeUid(uidToRemove);

        writeObjectConfig = true;
        sendObjectConfig = true;
      }

      
      if ( doc.containsKey("new_addLastUid") && doc["new_addLastUid"]==1 )
      {
        if (aConfig.objectConfig.nbTag<10)
        {
          Serial.println(F("Add last tag UID: "));
          
          for (uint8_t  i=0;i<uidReadLength;i++)
          {
            aConfig.objectConfig.tagUid[aConfig.objectConfig.nbTag][i]=uidRead[i];
          }
          aConfig.objectConfig.nbTag++;          

          writeObjectConfig = true;
          sendObjectConfig = true;
        }
        else
        {
          Serial.println(F("too much tags"));
        }
      }
      
      if (doc.containsKey("new_newTagUid")) 
      {
        if (aConfig.objectConfig.nbTag<10)
        {
          Serial.println(F("Add new tag UID: "));

          JsonArray newUidValue = doc["new_newTagUid"];
            
          for (uint8_t  i=0;i<uidReadLength;i++)
          {
            String stringToConvert = newUidValue[i];
            uint8_t hexValue = (uint8_t) strtol( &stringToConvert[0], NULL, 16);
            
            aConfig.objectConfig.tagUid[aConfig.objectConfig.nbTag][i]=hexValue;
            uidRead[i]=hexValue;
          }
          aConfig.objectConfig.nbTag++;          

          writeObjectConfig = true;
          sendObjectConfig = true;
        }
        else
        {
          Serial.println(F("too much tags"));
        }
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
        
        writeNetworkConfig = true;
        sendNetworkConfig = true;
      }

      if (doc.containsKey("new_apPassword")) 
      {
        strlcpy(  aConfig.networkConfig.apPassword,
                  doc["new_apPassword"],
                  sizeof(aConfig.networkConfig.apPassword));

        writeNetworkConfig = true;
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

          writeNetworkConfig = true;
        }
        
        sendNetworkConfig = true;
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

          writeNetworkConfig = true;
        }

        sendNetworkConfig = true;
      }

      if ( doc.containsKey("new_defaultObjectConfig") && doc["new_defaultObjectConfig"]==1 )
      {
        Serial.println(F("reset to default object config"));
        aConfig.writeDefaultObjectConfig("/config/objectconfig.txt");
        
        sendObjectConfig = true;
        uneFois = true;
      }

      if ( doc.containsKey("new_defaultNetworkConfig") && doc["new_defaultNetworkConfig"]==1 )
      {
        Serial.println(F("reset to default network config"));
        aConfig.writeDefaultNetworkConfig("/config/networkconfig.txt");
        
        sendNetworkConfig = true;
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
        sendObjectConfig = true;
        sendNetworkConfig = true;
      }

      // modif config
      // write object config
      if (writeObjectConfig)
      {
        aConfig.writeObjectConfig("/config/objectconfig.txt");
        aConfig.printJsonFile("/config/objectconfig.txt");
        
        // update statut
        uneFois = true;
      }

      // resend object config
      if (sendObjectConfig)
      {
        ws.textAll(aConfig.stringJsonFile("/config/objectconfig.txt"));
      }

      // write network config
      if (writeNetworkConfig)
      {
        aConfig.writeNetworkConfig("/config/networkconfig.txt");
        //aConfig.printJsonFile("/config/networkconfig.txt");
      }

      // resend network config
      if (sendNetworkConfig)
      {
        ws.textAll(aConfig.stringJsonFile("/config/networkconfig.txt"));
      }
    }
 
    // clear json buffer
    doc.clear();
  }
}

void notFound(AsyncWebServerRequest *request)
{
    request->send(404, "text/plain", "Not found");
}

void checkCharacter(char* toCheck, char* allowed, char replaceChar)
{
  //char *allowed = "0123456789ABCD*";

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


void sendUptime()
{
  uint32_t now = millis() / 1000;
  uint16_t days = now / 86400;
  uint16_t hours = (now%86400) / 3600;
  uint16_t minutes = (now%3600) / 60;
  uint16_t seconds = now % 60;
    
  String toSend = "{\"uptime\":\"";
  toSend+= String(days) + String("d ") + String(hours) + String("h ") + String(minutes) + String("m ") + String(seconds) + String("s");
  toSend+= "\"}";

  ws.textAll(toSend);
  //Serial.println(toSend);
}

void sendStatutCloneur()
{
  String toSend = "{\"statutCloneurActuel\":";
  toSend+= aConfig.objectConfig.statutCloneurActuel;
  toSend+= "}";

  ws.textAll(toSend);
}

void sendTagUid()
{
  String toSend = "{\"lastTagUid\":[";
  for (uint8_t i=0;i<uidReadLength;i++)
  {
    // add leading zero
    char result[3];
    sprintf(result, "%02x", uidRead[i]);
    toSend+= "\"" + String(result) + "\"";
    if (i<uidReadLength-1)
    {
      toSend+= ",";
    }
  }
  
  toSend+= "]}";

  ws.textAll(toSend);
}

void printTagUid()
{
  Serial.println(stringTagUid());
  /*
  for (int i=0;i<uidReadLength;i++)
  {
    char result[3];
    sprintf(result, "%02x", uidRead[i]);
    Serial.print(result);
    if (i<uidReadLength-1)
    {
      Serial.print(":");
    }
  }
  Serial.println("");
  */
}

String stringTagUid()
{
  String resultStr;
  for (int i=0;i<uidReadLength;i++)
  {
    char result[3];
    sprintf(result, "%02x", uidRead[i]);
    resultStr+=String(result);
    if (i<uidReadLength-1)
    {
      resultStr+=String(":");
    }
  }
  resultStr.toUpperCase();
  return (resultStr);
}

String stringTagUid(uint8_t uidToString)
{
  String resultStr;
  for (int i=0;i<4;i++)
  {
    char result[3];
    sprintf(result, "%02x", aConfig.objectConfig.tagUid[uidToString][i]);    
    resultStr+=String(result);
    if (i<3)
    {
      resultStr+=String(":");
    }
  }
  resultStr.toUpperCase();
  return (resultStr);
}

void removeUid(uint8_t uidToRemove)
{
  if (aConfig.objectConfig.nbTag>0)
  {
    for (uint8_t i=uidToRemove;i<aConfig.objectConfig.nbTag-1;i++)
  {
    for (uint8_t j=0;j<4;j++)
    {
      aConfig.objectConfig.tagUid[i][j]=aConfig.objectConfig.tagUid[i+1][j];
    }
  }
  aConfig.objectConfig.nbTag--;
  
  Serial.println("done");
  }
}
