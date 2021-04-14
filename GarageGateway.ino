// The mother of all embedded development...
#include <Arduino.h>

#define MQTT_SOCKET_TIMEOUT 30
#define MQTT_KEEPALIVE 30
#define MQTT_MAX_PACKET_SIZE 512

#define TTGO_V2116 
#define INCLUDE_LORA

#include <LoRa.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <AESLib.h>
#include "gg_time.h"
#include "security.h"
#include "SysLogger.h"
#include <Wire.h>
#ifdef TTGO_V2116
#include "display.h"
#endif 

#define DS_50 0.5
#define DS_25 0.25
#define DS_75 0.75

// These constants are defined so we can use LORA
#ifdef TTGO_V2116

#define LORA_SS   18
#define LORA_RST  23
#define LORA_DIO0 26

#define PIN_ERROR       15 
#define PIN_ACTIVITY    12

#else // for TTGO V1.1 or Heltech

#define LORA_SS   18
#define LORA_RST  14
#define LORA_DIO0 26

#define PIN_ERROR       4 
#define PIN_ACTIVITY    23
#endif


String outgoing;              // outgoing message

byte msgCount = 0;            // count of outgoing messages
byte localAddress = 0xBB;     // address of this device
byte destination = 0xAA;      // destination to send to
int  mqttErrorCount = 0;

// WIFI
const int ssids = 2;
char *ssid[ssids]      = { "EDGECLIFF_24G_HA1", "EDGECLIFF" };             // Set your WiFi SSID
char *password[ssids]  = { "ha1Leapfr0g",       "leapfr0g"  };             // Set your WiFi password
IPAddress my_ip;
int nssid = 0;

// GARAGE DOOR STATUS -> This is what we think the garage door is doing.
typedef enum {
   DS_ERROR,
   DS_OPEN,
   DS_OPENING,
   DS_CLOSING,
   DS_CLOSED,
   DS_UNKNOWN
} DOORSTATUS;

//MQTT
IPAddress server(192, 168, 1, 107 );
#define MQTT_CLIENTID "GARAGEGATEWAY"
#define MQTT_UID "garage"
#define MQTT_PWD "leapfr0g"
#define SENDTOPIC "GARAGEGATEWAY/status"
#define COMMANDTOPIC "GARAGEGATEWAY/command"
#define SERVICETOPIC "GARAGEGATEWAY/service"
#define SERVICE_ALIVE "Alive"
#define SERVICE_DEAD "Dead"

// MQTT Call back function on subsctibed topic
void callback(char*, byte*, unsigned int);
WiFiClient wifiClient;

#define HOST_NAME "garagegateway"

PubSubClient mqtt(server, 1883, callback, wifiClient);

// Rmemote Logging Classes
#define SYSLOGFILE "garagegateway"
#define SYSLOGDHOST "192.168.1.107"
#define SYSLOGDPORT 514
SysLogger *errorLog;
SysLogger *warningLog;
SysLogger *noticeLog;
SysLogger *debugLog;
SysLogger *infoLog;

//#define USE_COLOUR 1
#ifdef USE_COLOUR
  #define ERROR_COL "\033[0;31m"
  #define WARNING_COL "\033[0;33m"
  #define NOTICE_COL "\033[0;36m"
  #define DEBUG_COL "\033[0;32m"
  #define INFO_COL "\033[0;34m"
#else
  #define ERROR_COL ""
  #define WARNING_COL ""
  #define NOTICE_COL ""
  #define DEBUG_COL ""
  #define INFO_COL ""
#endif

// Onscreen Information
String os_header = "Lora Gateway";
String os_lastAction = "";
String os_Detail1 = "";
String os_Detail2 = "";

// scan I2C bus for devices like ssd1306 oled
void scanI2Cdevice(void)
{
    byte err, addr;
    int nDevices = 0;
    for (addr = 1; addr < 127; addr++) {
        Wire.beginTransmission(addr);
        err = Wire.endTransmission();
        if (err == 0) {
            Serial.print("I2C device found at address 0x");
            if (addr < 16)
                Serial.print("0");
            Serial.print(addr, HEX);
            Serial.println(" !");
            nDevices++;

            if (addr == 0x3C) {
                //ssd1306_found = true;
                Serial.println("ssd1306 display found");
            }

        } else if (err == 4) {
            Serial.print("Unknow error at address 0x");
            if (addr < 16)
                Serial.print("0");
            Serial.println(addr, HEX);
        }
    }
    if (nDevices == 0)
        Serial.println("No I2C devices found\n");
    else
        Serial.println("done\n");
}

// ----------------------------------------------------------------  
// MQTT Call back function
//
// ----------------------------------------------------------------
void callback(char* topic, byte* payload, unsigned int length) 
{ 
  // If we get here we have received a message to open or close the garage door which is achieved by sending
  // the message over LoRa to the GARAGEDOOR_SENSOR
  
  String GGcommand = "";
  int i = 0;
  
  while (i < length) {
    GGcommand = GGcommand + (char)payload[i];
    i++;
  }  

  if ( 0==GGcommand.compareTo("PING") )
  {
    // Calling publish in a MQTT Callback invalidates payload
    mqtt.publish(SERVICETOPIC, SERVICE_ALIVE);
  } 
  else if ( 0==GGcommand.compareTo("REBOOT") )
  {
    infoLog->println("Rebootting due to command message");
    mqtt.disconnect();
    delay(1000);
    ESP.restart();
  } 
  else  
  {
    // Send command, but wrap it up in a defined message:
    // we send the time of day to the device
    char message[128];
    sprintf( message, "{frm:%d,to:%d,cmd:\"%s\",ts:\"%s\"}", localAddress, destination, GGcommand.c_str(), getClock().c_str()); 
    infoLog->printf("Sending MQTT Message via Lora: \"%s\"\n", message );   
    sendMessage(message);

    os_lastAction = "MQTT Msg Sent";
    os_Detail1 = GGcommand;
    os_Detail2 = getClock();
  }
  return;
}

// ----------------------------------------------------------------  
// MQTT Call to publish a message. e.g. lora packet received.
//
// ----------------------------------------------------------------
int publishMQTT(String topic, String message) {

  int bOk = false;
  
  if (mqtt.connected()) {
    digitalWrite( PIN_ACTIVITY, HIGH );
    infoLog->printf("Sending MQTT Message...");
    bOk = mqtt.publish(topic.c_str(), message.c_str());
    digitalWrite( PIN_ACTIVITY, LOW );
  } else {
    errorLog->println("MQTT Issue: Not connected. Unable to publish message");
    bOk = 0;
    flashError( 2000, DS_50, 3);
  }
  return bOk;
}

// ----------------------------------------------------------------
//
// Convert Door Status erum to printable value
// ----------------------------------------------------------------
String doorStatusToString( const DOORSTATUS status )
{
  String retVal;
  switch ( status )
  {
    case DS_ERROR:
       retVal = "Error";
       break;
     case DS_OPEN:
       retVal = "Open";
       break;
     case DS_OPENING:
       retVal = "Opening";
       break;
     case DS_CLOSED:
       retVal = "Closed";
       break;
     case DS_CLOSING:
       retVal = "Closing";
       break;
     default:
       retVal = "Unknown";
       break;    
  }
  return retVal;
}

// ----------------------------------------------------------------
//
// Update the screen with metrics info
// ----------------------------------------------------------------
void updateDisplay()
{
   String scndLine;
   String thrdLine;
   String frthLine;
   String fithLine;

   scndLine = "Last Action:";
   thrdLine = os_lastAction;
   frthLine = os_Detail1;
   fithLine = os_Detail2;
#ifdef USE_DISPLAY   
   display_Lines( os_header, scndLine, thrdLine, frthLine, fithLine );
#endif
}


String mqttDebugString(short status)
{
    String retVal = "";
    switch(status) {
    case MQTT_CONNECTION_TIMEOUT:
      retVal = "Connection Timeout";
      break;
    case MQTT_CONNECTION_LOST:
      retVal = "Connection Lost";
      break;
    case MQTT_CONNECT_FAILED:
      retVal = "Connection Failed";
      break;
    case MQTT_DISCONNECTED:
      retVal = "Disconected";
      break;
    case MQTT_CONNECTED:
      retVal = "Connected";
      break;           
    case MQTT_CONNECT_BAD_PROTOCOL: 
      retVal = "Bad Protocol";
      break;  
    case MQTT_CONNECT_BAD_CLIENT_ID: 
      retVal = "Bad Client Id"; 
      break;
    case MQTT_CONNECT_UNAVAILABLE:
      retVal = "Connect Unavailable";
      break;    
    case MQTT_CONNECT_BAD_CREDENTIALS: 
      retVal = "Bad Credentials";
      break;
    case MQTT_CONNECT_UNAUTHORIZED: 
      retVal = "Connect Unauthorised";
      break;  
    default:
      retVal = "Unknown status";  
      break; 
  }
  return retVal;
}

void flashError( const int duration, const float dutycycle, const int count )
{
   for( int i = 0; i< count; i++ ) {
     digitalWrite( PIN_ERROR, HIGH );
     delay(duration * dutycycle); 
     digitalWrite( PIN_ERROR, LOW );              
     delay(duration * ( 1 - dutycycle) ); 
   } // for
   return;
}

void flashActivity( const int duration, const float dutycycle, const int count )
{
   for( int i = 0; i< count; i++ ) {
     digitalWrite( PIN_ACTIVITY, HIGH );
     delay(duration * dutycycle); 
     digitalWrite( PIN_ACTIVITY, LOW );              
     delay(duration * ( 1 - dutycycle) ); 
   } // for
   return;
}

void setup() {
  //initialize Serial Monitor
  Serial.begin(115200);
  Serial.println("\nLoRa Duplex - Garage Gateway");

  pinMode( PIN_ERROR, OUTPUT );
  pinMode( PIN_ACTIVITY, OUTPUT );
  digitalWrite( PIN_ERROR, HIGH );
  digitalWrite( PIN_ACTIVITY, HIGH );  
  delay(500);
  digitalWrite( PIN_ERROR, LOW );
  digitalWrite( PIN_ACTIVITY, LOW );

  security_init();

  int wifierrorcount; 
  bool connected = false;

  for( nssid = 0; nssid<ssids && !connected; nssid++) {
    wifierrorcount = 0;
    Serial.println();
    Serial.print("Connecting to ");
    Serial.println(ssid[nssid]);
    WiFi.mode(WIFI_STA);
    WiFi.setHostname(HOST_NAME);
    WiFi.begin(ssid[nssid], password[nssid]);
    delay(1000);

    connected = !(WiFi.status() != WL_CONNECTED);
    while ( !connected && wifierrorcount < 5 ) {
      digitalWrite( PIN_ERROR, HIGH );
      delay(4000);
      digitalWrite( PIN_ERROR, LOW );
      delay(1000);      
      Serial.print(".");
      wifierrorcount++;
    } // while
  } // for

  if (!connected) {
     WiFi.disconnect();
     Serial.println("Forcing restart");
     flashError( 1000, DS_50, 5 );
     ESP.restart();
  }

// Now we are ready to set up remote logging....
  errorLog = new SysLogger(SYSLOGDHOST, SYSLOGDPORT, LogLevel::Error, ERROR_COL, WiFi.getHostname(), SYSLOGFILE);
  warningLog = new SysLogger(SYSLOGDHOST, SYSLOGDPORT, LogLevel::Warning, WARNING_COL, WiFi.getHostname(), SYSLOGFILE);
  noticeLog = new SysLogger(SYSLOGDHOST, SYSLOGDPORT, LogLevel::Notice, NOTICE_COL, WiFi.getHostname(), SYSLOGFILE);
  debugLog= new SysLogger(SYSLOGDHOST, SYSLOGDPORT, LogLevel::Debug, DEBUG_COL,WiFi.getHostname(), SYSLOGFILE);
  infoLog = new SysLogger(SYSLOGDHOST, SYSLOGDPORT, LogLevel::Info, INFO_COL, WiFi.getHostname(), SYSLOGFILE);

  noticeLog->println("** START UP Complete");
  noticeLog->printf("Hostname  : %s\n",WiFi.getHostname());

  bool mqttconnected = false;
  if (connected) {
    // MQTT
    if (mqtt.connect(MQTT_CLIENTID, MQTT_UID, MQTT_PWD, SERVICETOPIC, 1, false, SERVICE_DEAD, false)) {
      mqtt.publish(SERVICETOPIC, SERVICE_ALIVE);
      mqtt.subscribe(COMMANDTOPIC, 1);
      Serial.println("MQTT Connected");
      infoLog->printf("MQTT Connected");
      mqttErrorCount = 0; 
      mqttconnected = true;
    } else {
      errorLog->printf("MQTT Not Connected. Condition: %s\n", mqttDebugString(mqtt.state()) );
      mqttconnected = false;
    }
  } 
  
 #ifdef INCLUDE_LORA
  // override the default CS, reset, and IRQ pins (optional)
  LoRa.setPins(LORA_SS, LORA_RST, LORA_DIO0); // set CS, reset, IRQ pin

  if (!LoRa.begin(915E6)) {             // initialize ratio at 915 MHz
    Serial.println("LoRa init failed?");
    digitalWrite( PIN_ERROR, HIGH );    // both read and green LED on implies LORA falure and 30 seconds to restart.
    digitalWrite( PIN_ACTIVITY, HIGH );               
    delay(30000);
    ESP.restart();
  }
  LoRa.enableCrc();
  LoRa.setSpreadingFactor(10);
  //LoRa.setTxPower(19);
  //LoRa.setGain(6);
  LoRa.setSyncWord(0xe3);
#endif
 
  time_setup();

#ifdef USE_DISPLAY
  init_Display();
  display_Lines( "READY" );
#endif
 
  Serial.print("Ready @");
  Serial.println(getClock());

  delay(1000);
  Serial.flush();
}


void loop() {

  if (!mqtt.connected()) {
    warningLog->printf( "MQTT Disconnected? : Trying to reconnect");

    digitalWrite( PIN_ERROR, HIGH );
    delay(2000);
    digitalWrite( PIN_ERROR, LOW );

        
    if (!mqtt.connect(MQTT_CLIENTID, MQTT_UID, MQTT_PWD, SERVICETOPIC, 1, false, SERVICE_DEAD, false )) {
      mqttErrorCount++; 
      if (mqttErrorCount > 5 )
      {
         errorLog->println("Serious MQTT Issue. Restarting" );
         flashError( 1000, DS_25, 3 );
         delay(10000);
         ESP.restart();
      } else 
      if (WiFi.status() != WL_CONNECTED )
      {
        errorLog->println("Serious MQTT Issue due to no WiFi. Restarting" );
        flashError( 1000, DS_75, 5 );
        delay(10000);
        ESP.restart();
      } else {
        warningLog->printf("MQTT Issue. Condition: %s\n", mqttDebugString(mqtt.state()) );
        delay( mqttErrorCount * 1000);
      }
    } else {
      mqtt.publish(SERVICETOPIC, SERVICE_ALIVE);
      mqtt.subscribe(COMMANDTOPIC,1);
      mqttErrorCount = 0;  
    }
  } else {
    mqtt.loop();  
    mqttErrorCount = 0;
  }

 #ifdef INCLUDE_LORA
  onReceive(LoRa.parsePacket());
 #endif  

   updateDisplay();

}

void sendMessage(String outgoing) {

  digitalWrite(PIN_ACTIVITY, HIGH );
  byte enc_iv[N_BLOCK] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }; // iv_block gets written to, provide own fresh copy...
  char ciphertext[3*INPUT_BUFFER_LIMIT] = {0}; // THIS IS OUTPUT BUFFER (FOR BASE64-ENCODED ENCRYPTED DATA)
  uint16_t outgoing_len = outgoing.length();
  uint16_t encrypted_len = security_encrypt((char*)outgoing.c_str(), outgoing_len, enc_iv, ciphertext);
  LoRa.beginPacket();               // start packet
  LoRa.write(outgoing_len);        // add payload length
  LoRa.print(ciphertext);           // add payload
  LoRa.endPacket();  
  digitalWrite(PIN_ACTIVITY, LOW );

  msgCount++;                           // increment message ID
}

void onReceive(int packetSize) {

  if (packetSize == 0) return;          // if there's no packet, return

  byte messageLength = LoRa.read();    // incoming msg length
  String incoming = "";

  digitalWrite(PIN_ACTIVITY, HIGH );
  while (LoRa.available()) {
    incoming += (char)LoRa.read();
  }

  // we got a message
  char msg[255];
  sprintf(msg, "Lora Message received. RSSI(%s dB), SNR(%s dB)\n", String(LoRa.packetRssi()), String(LoRa.packetSnr()) );
  infoLog->printf(msg);
  infoLog->print(incoming);
  // and to console....
  Serial.println(msg);
  Serial.println(incoming);  
  
  // assume incomming is encrypted
  // Decrypt
  char cleartext[INPUT_BUFFER_LIMIT] = {0}; // THIS IS INPUT BUFFER (FOR TEXT)
  byte dec_iv[N_BLOCK] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }; // iv_block gets written to, provide own fresh copy...
  uint16_t decrypted_len = security_decrypt((byte*)incoming.c_str(), incoming.length(), dec_iv, cleartext);  

  if (messageLength != decrypted_len) {  // check length for error
    warningLog->printf("warning: message length does not match length: %d. Decrypted Len was %d. Truncating\n", messageLength, decrypted_len);
    //if (decrypted_len>messageLength) 
    //{
    //  cleartext[messageLength] = '\0';
    //}
    //return;       
  }

  infoLog->printf( "Decrypted Lora Message Received: \"%s\"\n", cleartext );

  DynamicJsonDocument jsonBuffer(512);
  DeserializationError error = deserializeJson(jsonBuffer, cleartext); 
  // Test if parsing succeeds.
  if (error) {
    warningLog->println("Unable to parse received data");
    digitalWrite(PIN_ACTIVITY, LOW );
    return;
  }
  digitalWrite(PIN_ACTIVITY, LOW );

  jsonBuffer["rssi"] = String(LoRa.packetRssi());
  jsonBuffer["snr"] = String(LoRa.packetSnr());
  jsonBuffer["ts"] = getClock();

  // if the recipient isn't this for this device or broadcast
  byte recipient = jsonBuffer[String("to")];
  if (recipient != localAddress ) { // && recipient != 0xe3) {
    warningLog->printf("warning: The message to %s is not for me", recipient );
    digitalWrite(PIN_ACTIVITY, LOW );
    return;                             // skip rest of function
  }
  digitalWrite(PIN_ACTIVITY, LOW );

  // if message is for this device, or broadcast, print details:
  infoLog->printf( "Lora Message Received: \"%s\"\n", cleartext );

  // Now sent message through MQTT
  char jsonChar[512];
  
  jsonBuffer["ds"] = doorStatusToString( (DOORSTATUS)jsonBuffer["ds"] );
  serializeJson( jsonBuffer, jsonChar, 512 );
  infoLog->printf( "Sending MQTT Message=\"%s\"\n", jsonChar );

  int status = publishMQTT( SENDTOPIC, jsonChar ); // this will also flash the activity light

   os_lastAction = "LoRa Received";
   os_Detail1 = "Rssi=" + String(LoRa.packetRssi()) + " Snr=" + String(LoRa.packetSnr());
   os_Detail2 = getClock();
 
}
