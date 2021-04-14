#pragma once

#include <SSD1306.h>

// OLED pins to ESP32 GPIOs via these pins:

#ifdef TTGO_V2116

// re-define pin definitions of pins_arduino.h
#define PIN_SPI_SS    18 // ESP32 GPIO18 (Pin18) -- HPD13A NSS/SEL (Pin4) SPI Chip Select Input
#define PIN_SPI_MOSI  27 // ESP32 GPIO27 (Pin27) -- HPD13A MOSI/DSI (Pin6) SPI Data Input
#define PIN_SPI_MISO  19 // ESP32 GPIO19 (Pin19) -- HPD13A MISO/DSO (Pin7) SPI Data Output
#define PIN_SPI_SCK   5  // ESP32 GPIO5 (Pin5)   -- HPD13A SCK (Pin5) SPI Clock Input

#define OLED_ADDR 0x3c
#define OLED_SDA  21 
#define OLED_SCL  22
//#define OLED_RST
#else // default for TTGO V1.1
#define OLED_ADDR 0x3c
#define OLED_SDA  4 //  GPIO4
#define OLED_SCL  15 // GPIO15
#define OLED_RST  16 // GPIO16
#endif 

#define USE_DISPLAY

SSD1306 display(OLED_ADDR, OLED_SDA, OLED_SCL);

// ----------------------------------------------------------------  
// Initilize the OLED functions.
//
// ----------------------------------------------------------------
void init_Display() 
{
  Serial.println("init_Display called");
#ifdef OLED_RST
  Serial.println( "Reseting OLED" );
  pinMode(OLED_RST,OUTPUT);
  digitalWrite(OLED_RST, LOW);    // low to reset OLED
  delay(50); 
  digitalWrite(OLED_RST, HIGH);   // must be high to turn on OLED
  delay(50);
#endif  
  // Initialising the UI will init the display too.
  Serial.print( "OLED_SDA=" );
  Serial.println( OLED_SDA );
  
  Serial.print( "OLED_SCL=" );
  Serial.println( OLED_SCL );  
  
  display.init();
  display.flipScreenVertically();
  display.setFont(ArialMT_Plain_24);
  display.setTextAlignment(TEXT_ALIGN_LEFT);
  display.drawString(0, 24, "STARTING");
  display.display();
  Serial.println("init_Display complete");
  return;
}
// ----------------------------------------------------------------
// Display up to 4 lines of text.
//
// ----------------------------------------------------------------
void display_Lines(String msg, String scndLine = "", String thrdLine = "" , String fourthLine = "", String fithLine = "") 
{
  // Initialising the UI will init the display too.
  //Serial.println( "display_Lines called");
  display.clear();
  display.setFont(ArialMT_Plain_10);
  display.setColor(WHITE); // the only colour us black and white
  display.setTextAlignment(TEXT_ALIGN_LEFT);
  display.drawString(0,  0, msg.c_str());
  display.setFont(ArialMT_Plain_16);  
  display.drawString(0, 12, scndLine.c_str()); 
  display.setFont(ArialMT_Plain_10);
  display.drawString(0, 34, thrdLine.c_str());
  display.drawString(0, 42, fourthLine.c_str());
  display.drawString(0, 50, fithLine.c_str());
  display.display();
  //Serial.println( "display_Lines complete");
  return;
}
