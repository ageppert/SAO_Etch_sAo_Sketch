/* 
  RP2040-Zero with SSD1327 on I2C0 or I2C1 with U8g2 Library

  DEPENDENCIES - FIRMWARE
    Install "Arduino Mbed OS RP2040 Boards" v4.2.2 with Arduino Boards Manager
      - The Wire Library is included, I think.
    U8g2lib SSD1327
      https://github.com/olikraus/u8g2/wiki/u8g2setupcpp#reference

  DEPENDENCIES - HARDWARE
    OLED with SSD1327 Driver
    RP2040-Zero

  DESCRIPTION
    1) Test the U8G2 library with the default I2C0 port. Done, works.
    2) Test with I2C1 port... Done. Works!
*/

#include <Arduino.h>
#include <Wire.h>
#include <U8g2lib.h>

// Choose one port
  // #define I2C_PORT0 true
  #define I2C_PORT1 true

#if I2C_PORT0
  #define I2C0_SDA_PIN 4
  #define I2C0_SCL_PIN 5
#endif

#if I2C_PORT1
  #define I2C1_SDA_PIN 2
  #define I2C1_SCL_PIN 3
#endif

/*
  The name of the U8G2 Arduino C++ constructor has the following parts:
  No	Description	        Example
  1	  Prefix	            U8G2
  2	  Display Controller	UC1701
  3	  Display Name	      DOGS102
  4	  Variant (optional)	ALT
  5	  Buffer Size	        1, 2 or F (full frame buffer), see details below
  6	  Communication	      4W_SW_SPI, ...
*/

#if I2C_PORT0
  U8G2_SSD1327_EA_W128128_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE, /* clock=*/ U8X8_PIN_NONE, /* data=*/ U8X8_PIN_NONE);
#endif

#if I2C_PORT1
  U8G2_SSD1327_EA_W128128_F_2ND_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);
#endif

void setup() {
  Serial.begin(115200);

  #if I2C_PORT0
    // Set the SDA and SCL pins for the Wire1 object before beginning
    Wire.setSDA(I2C0_SDA_PIN);
    Wire.setSCL(I2C0_SCL_PIN);
    Wire.begin();
  #endif

  #if I2C_PORT1
    // Set the SDA and SCL pins for the Wire1 object before beginning
    Wire1.setSDA(I2C1_SDA_PIN);
    Wire1.setSCL(I2C1_SCL_PIN);
    Wire1.begin();
  #endif

  u8g2.begin();
}

void loop() {
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_ncenB14_tr);
  #if I2C_PORT0
    u8g2.drawStr(0, 20, "Hello, I2C0!");
  #endif
  #if I2C_PORT1
    u8g2.drawStr(0, 20, "Hello, I2C1!");
  #endif
  u8g2.sendBuffer();
}
