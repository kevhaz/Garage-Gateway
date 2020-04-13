/*
  LoRa Duplex communication

  Sends a message every half second, and polls continually
  for new incoming messages. Implements a one-byte addressing scheme,
  with 0xFF as the broadcast address.

  Uses readString() from Stream class to read payload. The Stream class'
  timeout may affect other functuons, like the radio's callback. For an

  created 28 April 2017
  by Tom Igoe
*/

#define MQTT_SOCKET_TIMEOUT 30
#define MQTT_KEEPALIVE 30
#define MQTT_MAX_PACKET_SIZE 512

#define INCLUDE_LORA

//#include <SPI.h>              // include libraries
//#include <Wire.h>
//#include <SSD1306.h>
#include <LoRa.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <AESLib.h>

const int csPin = 18;          // LoRa radio chip select
const int resetPin = 14;       // LoRa radio reset
const int irqPin = 26;         // change for your board; must be a hardware interrupt pin

String outgoing;              // outgoing message

byte msgCount = 0;            // count of outgoing messages
byte localAddress = 0xBB;     // address of this device
byte destination = 0xAA;      // destination to send to
//  long lastSendTime = 0;        // last send time
//  int interval = 2000;          // interval between sends
int  mqttErrorCount = 0;

// OLED pins to ESP32 GPIOs via this connecthin:
//#define OLED_ADDR 0x3c
//#define OLED_SDA  4 // GPIO4
//#define OLED_SCL  15 // GPIO15
//#define OLED_RST  16 // GPIO16

//SSD1306 display(OLED_ADDR, OLED_SDA, OLED_SCL);

// WIFI
const char* ssid     = "EDGECLIFF";
const char* password = "leapfr0g";

// Encryption.
// AES Encryption Key
AESLib aesLib;

// salt=25B61CA381BF15AC
byte aes_key[]        = { 0xB4, 0x18, 0x3C, 0x1F, 0x38, 0x86, 0x2A, 0x40, 0xD3, 0x88, 0xFA, 0x28, 0x6B, 0xF3, 0x8A, 0x33 };
byte aes_iv[N_BLOCK]  = { 0x79, 0x77, 0x98, 0x90, 0xB7, 0x34, 0xC3, 0xA1, 0x4D, 0x20, 0xE0, 0x10, 0x43, 0x3F, 0xC1, 0x2F };


//MQTT
IPAddress server(10, 0, 1, 12);
#define MQTT_CLIENTID "GARAGEGATEWAY"
#define MQTT_UID "garage1"
#define MQTT_PWD "leapfr0g"
#define SENDTOPIC "GARAGEGATEWAY/status"
#define COMMANDTOPIC "GARAGEGATEWAY/command"
#define SERVICETOPIC "GARAGEGATEWAY/service"
#define SERVICE_ALIVE "Alive"
#define SERVICE_DEAD "Dead"

// MQTT Call back function on subsctibed topic
void callback(char*, byte*, unsigned int);
WiFiClient wifiClient;

PubSubClient mqtt(server, 1883, callback, wifiClient);



// ----------------------------------------------------------------  
// MQTT Call back function
//
// ----------------------------------------------------------------
void callback(char* topic, byte* payload, unsigned int length) 
{ 
  // If we get here we have received a message to open or close the garage door 
  Serial.println("MQTT Command Message received");

  String GGcommand = "";
  int i = 0;
  
  while (i < length) {
    GGcommand = GGcommand + (char)payload[i];
    i++;
  }   
  
  Serial.print("Sending Lora Message : '");
  Serial.print(GGcommand); 
  Serial.println("'");
  
  sendMessage(GGcommand);
  return;
}

// ----------------------------------------------------------------  
// MQTT Call to publish a message. e.g. lora packet received.
//
// ----------------------------------------------------------------
int publishMQTT(String topic, String message) {

  int bOk = false;
  
  if (mqtt.connected()) {
    Serial.print("Sending MQTT Message...");
    bOk = mqtt.publish(topic.c_str(), message.c_str());
    (bOk)?Serial.println("Sent!"):Serial.println("Sent with error!");
  } else {
    Serial.println("MQTT Issue: Not connected. Unable to publish message");
    bOk = 0;
  }
  return bOk;
}

// ----------------------------------------------------------------  
// Initilize the OLED functions.
//
// ----------------------------------------------------------------

void init_oLED() 
{
#if defined OLED_RST
  Serial.println("OLED: Reseting");
  pinMode(OLED_RST,OUTPUT);
  digitalWrite(OLED_RST, LOW);  // low to reset OLED
  delay(50); 
  digitalWrite(OLED_RST, HIGH);   // must be high to turn on OLED
  delay(50);

  // Initialising the UI will init the display too.
  Serial.println("OLED: Init");
  display.init();
  display.flipScreenVertically();
  display.setFont(ArialMT_Plain_24);
  display.setTextAlignment(TEXT_ALIGN_LEFT);
  display.drawString(0, 24, "STARTING");
  display.display();
  Serial.println("OLED: Init Complete");
#endif
}

// ----------------------------------------------------------------
// Activate the OLED
//
// ----------------------------------------------------------------
void acti_oLED(String msg) 
{
#if defined OLED_RST  
  // Initialising the UI will init the display too.
  display.clear();
  
  display.setFont(ArialMT_Plain_16);
  display.drawString(0, 16, msg.c_str());
  display.display();
#endif

  Serial.print("OLED: Displaying: ");
  Serial.println(msg);
}


// ----------------------------------------------------------------
// Print the received message message on the OLED.
// Note: The whole message must fit in the buffer
//
// ----------------------------------------------------------------
void msg_oLED(int rssi, float snr, String packet) {
    
    char szBuf[40];
    sprintf(szBuf, "R:%i, S:%.0fdB", rssi, snr);

 #if defined OLED_RST
    display.clear();
    display.setFont(ArialMT_Plain_16);
    display.setTextAlignment(TEXT_ALIGN_LEFT);
    display.drawString(0, 0, packet );
    display.drawString(0, 16, String(szBuf));
    display.display();
#endif
    
    Serial.print("OLED: Displaying: ");
    Serial.println(szBuf);
    
}

// ----------------------------------------------------------------
// Print the OLED address in use
//
// ----------------------------------------------------------------
void addr_oLED() 
{
#if defined OLED_RST  
  Serial.print(F("OLED_ADDR=0x"));
  Serial.println(OLED_ADDR, HEX);
#endif
}

void mqttPrintDebug(short status, boolean nl = false)
{
    switch(status) {
    case MQTT_CONNECTION_TIMEOUT:
      Serial.print( "Connection Timeout");
      break;
    case MQTT_CONNECTION_LOST:
      Serial.print( "Connection Lost");
      break;
    case MQTT_CONNECT_FAILED:
      Serial.print( "Connection Failed");
      break;
    case MQTT_DISCONNECTED:
      Serial.print( "Disconected");
      break;
    case MQTT_CONNECTED:
      Serial.print( "Connected");
      break;           
    case MQTT_CONNECT_BAD_PROTOCOL: 
      Serial.print( "Bad Protocol");
      break;  
    case MQTT_CONNECT_BAD_CLIENT_ID: 
      Serial.print( "Bad Client Id"); 
      break;
    case MQTT_CONNECT_UNAVAILABLE:
      Serial.print( "Connect Unavailable");
      break;    
    case MQTT_CONNECT_BAD_CREDENTIALS: 
      Serial.print( "Bad Credentials");
      break;
    case MQTT_CONNECT_UNAUTHORIZED: 
      Serial.print( "Connect Unauthorized");
      break;  
    default:
      Serial.print( "Unknown status");  
      break; 
  }
  if (nl)
    Serial.print("\n");
    
  return;
}

// ----------------------------------------------------------------
//
// Generate IV (once)
// ----------------------------------------------------------------
void aes_init() {
  aesLib.gen_iv(aes_iv);
  // workaround for incorrect B64 functionality on first run...
  encrypt("HELLO WORLD!", aes_iv);
}

// ----------------------------------------------------------------
//
// Encrypt Message
// ----------------------------------------------------------------
String encrypt(char * msg, byte iv[]) {  
  int msgLen = strlen(msg);
  char encrypted[2 * msgLen];
  aesLib.encrypt64(msg, encrypted, aes_key, iv);  
  return String(encrypted);
}

// ----------------------------------------------------------------
//
// Decrypt Message
// ----------------------------------------------------------------
String decrypt(char * msg, byte iv[]) {
  //unsigned long ms = micros();
  int msgLen = strlen(msg);
  char decrypted[msgLen]; // half may be enough
  aesLib.decrypt64(msg, decrypted, aes_key, iv);  
  return String(decrypted);
}

void setup() {
  //initialize Serial Monitor
  Serial.begin(115200);
  aes_init();

  Serial.println("\nLoRa Duplex");
  // Set up Display
  init_oLED();

  acti_oLED( "Connecting to WiFi" );

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  WiFi.setHostname("garagegateway");

  int wifiErrorCount = 0;
  while (WiFi.status() != WL_CONNECTED) {
    if (wifiErrorCount > 20)
    {
      ESP.restart();
    }
    wifiErrorCount++;
    acti_oLED( "Wifi-Error Wait." );
    delay(1000);
    acti_oLED( "Connecting to WiFi" );
    Serial.print(".");
  } 

  Serial.print("Wifi Connected at IP: ");
  Serial.println(WiFi.localIP());

  acti_oLED( "Connect to MQTT..." );  

  // MQTT
  if (mqtt.connect(MQTT_CLIENTID, MQTT_UID, MQTT_PWD, SERVICETOPIC, 1, false, SERVICE_DEAD, false)) {
    mqtt.publish(SERVICETOPIC, SERVICE_ALIVE);
    mqtt.subscribe(COMMANDTOPIC, 1);
    Serial.println("MQTT Connected");
    mqttErrorCount = 0; 
    acti_oLED( "GG Ready" );  
  } else {
    Serial.print("MQTT Not Connected: ");  
    mqttPrintDebug( mqtt.state(), true );
    acti_oLED( "MQTT Error" );     
  }

 #ifdef INCLUDE_LORA

  // override the default CS, reset, and IRQ pins (optional)
  LoRa.setPins(csPin, resetPin, irqPin);// set CS, reset, IRQ pin

  if (!LoRa.begin(915E6)) {             // initialize ratio at 915 MHz
    Serial.println("LoRa init failed?");
    delay(30000);
    ESP.restart();
  }

  LoRa.setSyncWord(0xe3);
 #endif
 
  Serial.println("Ready.");
}


void loop() {

  if (!mqtt.connected()) {
    Serial.println( "MQTT Issue : Trying to reconnect");
    if (!mqtt.connect(MQTT_CLIENTID, MQTT_UID, MQTT_PWD, SERVICETOPIC, 1, false, SERVICE_DEAD, false )) {
      Serial.print( "MQTT Issue (");
      Serial.print(mqttErrorCount);
      Serial.print(") : Unable to reconnect: '"); 
      mqttPrintDebug( mqtt.state(), false );
      Serial.print("'"); 
      
      acti_oLED( "MQTT Recon Err" );
      mqttErrorCount++; 

      if (mqttErrorCount > 5 )
      {
         acti_oLED( "Rst due to MQTT" );
         delay(10000);
         ESP.restart();
      }

      if (WiFi.status() != WL_CONNECTED )
      {
        Serial.print( "Connectivity Error: Restarting in 10s");
        acti_oLED( "Restart in 10s." );             
        delay(10000);
        ESP.restart();
      }

      delay( mqttErrorCount * 1000);

    } else {
      mqtt.publish(SERVICETOPIC, SERVICE_ALIVE);
      mqtt.subscribe(COMMANDTOPIC,1);
      acti_oLED( "GG Ready." );
      mqttErrorCount = 0;  
    }
  } else {
    mqtt.loop();  
    mqttErrorCount = 0;
  }

 #ifdef INCLUDE_LORA
  onReceive(LoRa.parsePacket());
 #endif  
}

void sendMessage(String outgoing) {

  char message[128];
  sprintf( message, "{cmd:'%s',seq:%d,to:%d,frm:%d}", outgoing.c_str(), msgCount, destination, localAddress );
  byte enc_iv[N_BLOCK] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }; // iv_block gets written to, provide own fresh copy...
  String encrypted = encrypt(message, enc_iv);
  Serial.println( message );
  Serial.println( encrypted );  
  LoRa.beginPacket();                   // start packet
  LoRa.write(strlen(message));          // add payload length
  LoRa.print(encrypted);                // add payload
  LoRa.endPacket();  

  msgCount++;                           // increment message ID
}

void onReceive(int packetSize) {
 #ifdef INCLUDE_LORA
  if (packetSize == 0) return;          // if there's no packet, return

  byte messageLength = LoRa.read();    // incoming msg length
  String incoming = "";

  while (LoRa.available()) {
    incoming += (char)LoRa.read();
  }
  // assume incomming is encrypted
  // Decrypt
  byte dec_iv[N_BLOCK] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }; // iv_block gets written to, provide own fresh copy...
  String decrypted = decrypt((char*)incoming.c_str(), dec_iv);  
  Serial.print("Decrypted: ");
  Serial.println( decrypted );

  if (messageLength != decrypted.length()) {  // check length for error
    Serial.print("warning: message length does not match length: ");
    Serial.print( messageLength );
    Serial.print(" decrypted len was: "); 
    Serial.println( decrypted.length() );
    //return;                             // skip rest of function
  }

  // create an object
  //DynamicJsonBuffer jsonBuffer;
  StaticJsonBuffer<512> jsonBuffer;
    
  JsonObject& object1 = jsonBuffer.parseObject(decrypted);
  object1["rssi"] = String(LoRa.packetRssi());
  object1["snr"] = String(LoRa.packetSnr());


  // if the recipient isn't this for this device or broadcast
  byte recipient = object1[String("to")];
  if (recipient != localAddress ) { // && recipient != 0xe3) {
    Serial.print("warning: This message to '" );
    Serial.print( recipient );
    Serial.println("'; is not for me.");
    return;                             // skip rest of function
  }

  // if message is for this device, or broadcast, print details:
  Serial.println("Message: " + decrypted);
  Serial.println("RSSI: " + String(LoRa.packetRssi()));
  Serial.println("Snr: " + String(LoRa.packetSnr()));
  Serial.println();


  // Now sent message through MQTT
  char jsonChar[512];
  object1.printTo((char*)jsonChar, object1.measureLength() + 1);  

  Serial.print( "MQTT=" );
  Serial.println( jsonChar );
  // E.g: {"rx_from":"aa","tx_to":"bb","msg_id":"160","msg_len":"16","rssi":"-26","snr":"9.75","msg":"count=9632 GD=UP"}
  int status = publishMQTT( SENDTOPIC, jsonChar );
 
 #endif
}

