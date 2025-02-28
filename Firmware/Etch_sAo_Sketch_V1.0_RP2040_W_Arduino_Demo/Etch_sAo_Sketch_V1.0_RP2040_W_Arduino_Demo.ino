/* Etch sAo Sketch Demo with Arduino 
  Github repo: https://github.com/ageppert/SAO_Etch_sAo_Sketch
  Project page: https://hackaday.io/project/197581-etch-sao-sketch

  DEPENDENCIES
    This demo will work with a huge range of IDEs and hardware, but it was developed and tested with the following:
    Arduino IDE 2.3.2 
      Install "Arduino Mbed OS RP2040 Boards" v4.2.2 with Arduino Boards Manager
        - The Wire Library is included.
      Adafruit SSD1327 libray with dependencies:
        - Adafruit BusIO
        - Adafruit GFX Library
      Adafruit LIS3DH libray with dependencies:
        - Adafruit BusIO
        - Adafruit Unified Sensor

  HARDWARE CONNECTIONS
    OLED 1.5" 128x128 SSD1327: https://learn.adafruit.com/adafruit-grayscale-1-5-128x128-oled-display/arduino-wiring-and-test
    Accelerometer LIS3DH: https://learn.sparkfun.com/tutorials/lis3dh-hookup-guide and https://learn.adafruit.com/adafruit-lis3dh-triple-axis-accelerometer-breakout/arduino  
    Demo board -> RP2040-Zero (miniature Pico/RP2040 dev board) https://www.waveshare.com/rp2040-zero.htm
    The bottom SAO port (X1) is intended to provide full access to the accelerometer, screen, and analog pots.
    SAO I2C pins connect to the OLED and Accelerometer (which also has the potentionmeters connected to built-in ADCs).
    SAO GPIO1 pin connected to the left Analog Potentiometer
    SAO GPIO2 pin connected to the right Analog Potentiometer
  
  MODES
  Startup with the boot screen. After timeout, automatcally go to sketching mode with default white background.
  In sketch mode, rotate pots to draw.
  Gestures active during sketch mode:
  Upsidedown, shake left/right to clear sketch zone.
  Rightsideup, shake left/right to change background color (also clears screen).
  Rightsideup, shake up/down to change pot input between Arduino Analog port (0-3.3V) or accelerometer analog ports (0.8-1.6V).
*/

/************************************ ETCH SAO SKETCH - FIRMWARE VERSION TABLE ************************************
| VERSION |  DATE      | MCU     | DESCRIPTION                                                                    |
-------------------------------------------------------------------------------------------------------------------
|  1.0.0  | 2024-11-01 | RP2040  | First draft, hurry up for Supercon!
|         |            |         |
|         |            |         |
-----------------------------------------------------------------------------------------------------------------*/

/************************************ ETCH SAO SKETCH - HARDWARE VERSION TABLE ************************************
| VERSION |  DATE      | MCU     | DESCRIPTION                                                                    |
-------------------------------------------------------------------------------------------------------------------
|  1.0    | 2024-09-27 |         | First prototype, as built, getting it to come alive. Analog input range of pots
|         |            |         |   is much wider than accelerometer analog inputs can handle. Recommend to use
|         |            |         |   only with 3.3V capable analog inputs of host MCU through SAO pport GPIO1&2.
|         |            |         |
|         |            |         |
-----------------------------------------------------------------------------------------------------------------*/

#include <Adafruit_SSD1327.h>
#include <Fonts/FreeMono9pt7b.h>  // https://learn.adafruit.com/adafruit-gfx-graphics-library/using-fonts

//#include <Wire.h>                             // RP2040 Pico-Zero default I2C port is GPIO4 (SDA0) and GPIO5 (SCL0).
                                              // RP2040 Pico W default I2C port is GPIO4 (SDA0) and GPIO5 (SCL0).
#include <Adafruit_LIS3DH.h>
#include <Adafruit_Sensor.h>

#define HARDWARE_VERSION_MAJOR  1
#define HARDWARE_VERSION_MINOR  0
#define HARDWARE_VERSION_PATCH  0
#define MCU_FAMILY              RP2040
  /*********************************** ETCH SAO SKETCH -  HARDWARE VERSION TABLE ************************************
  | VERSION |  DATE      | MCU     | DESCRIPTION                                                                    |
  -------------------------------------------------------------------------------------------------------------------
  |  1.0.0  | 2019-10-08 | RP2040  | First prototype, as built, getting it to come alive.
  |         |            |         |
  -----------------------------------------------------------------------------------------------------------------*/

#define OLED_ADDRESS                0x3C      // OLED SSD1327 is 0x3C
#define OLED_RESET                    -1
#define OLED_HEIGHT                  128
#define OLED_WIDTH                   128
#define OLED_WHITE                    15
#define OLED_GRAY                      7
#define OLED_BLACK                     0
Adafruit_SSD1327 display(OLED_HEIGHT, OLED_WIDTH, &Wire, OLED_RESET, 1000000);
uint8_t color = OLED_WHITE;                   // 0-15 shades of gray
int16_t cursorX = 0;                          // 0-127 pixels
int16_t cursorY = 0;                          // 0-127 pixels
bool    EASBackground = 0;                    // 0 = dark, 1 = white (default)
bool    EASAnalogSource = 1;                  // 0 = accelerometer ADC, 1 = Arduino Analog Ports (default)

#define ACCELEROMETER_ADDRESS       0x19      // LIS3DH Acceleromter default is 0x19
Adafruit_LIS3DH lis = Adafruit_LIS3DH();

#define PIN_SAO_GPIO_1_ANA_POT_LEFT   A0      // Configured as left analog pot
#define PIN_SAO_GPIO_2_ANA_POT_RIGHT  A1      // Configured as left analog pot
int16_t PotLeftADCCounts = 0;
int16_t PotRightADCCounts = 0;
int16_t PotLeftADCCountsOld = 0;
int16_t PotRightADCCountsOld = 0;
int16_t PotMarginAtLimit = 10;
int16_t PotHystersisLimit = 12;
int16_t PotGlitchLimit = 64;
int16_t PotFilterSampleCount = 3;
int16_t deltaAbsolute = 0;

enum TopLevelMode                             // Top Level Mode State Machine
{
  MODE_INIT,
  MODE_BOOT_SCREEN,
  MODE_SKETCH,
  MODE_AT_THE_END_OF_TIME
} ;
uint8_t  TopLevelMode = MODE_INIT;
uint8_t  TopLevelModeDefault = MODE_BOOT_SCREEN;
uint32_t ModeTimeoutDeltams = 0;
uint32_t ModeTimeoutFirstTimeRun = true;

void setup()
{
  // Nothing to see here. Everything runs in a simple state machine in the main loop function, and the bottom of this file.
}

bool SerialInit() {
  bool StatusSerial = 1;
  Serial.begin(115200);
  // TODO get rid of this delay, quickly detect presence of serial port, and send message only if it's detected.
  delay(1500);
  Serial.println("\nHello World! This is the serial port talking!");
  return StatusSerial;
}

bool OLEDInit() {
  Serial.print(">>> INIT OLED started... ");
  if ( ! display.begin(0x3C) ) {
     Serial.println("Unable to initialize OLED");
     return 1;
  }
  else {
    Serial.println("Initialized!");
    OLEDClear();
  }
  return 0;
}

bool AccelerometerInit() {
  Serial.print(">>> INIT Accelerometer LIS3DH started... ");
  if (! lis.begin(ACCELEROMETER_ADDRESS)) {   // change this to 0x19 for alternative i2c address
    Serial.println("Unable to initialize.");
    return 1;
  }
  else {
    Serial.println("Initialized!");
    AccelerometerQuerySettings();
  }
  return 0;
}

void AccelerometerQuerySettings() {
  Serial.println("    Accelerometer LIS3DH Settings:");  
  Serial.print("    Range = "); Serial.print(2 << lis.getRange());
  Serial.println("G");

  // lis.setPerformanceMode(LIS3DH_MODE_LOW_POWER);
  Serial.print("    Performance mode set to: ");
  switch (lis.getPerformanceMode()) {
    case LIS3DH_MODE_NORMAL: Serial.println("Normal 10bit"); break;
    case LIS3DH_MODE_LOW_POWER: Serial.println("Low Power 8bit"); break;
    case LIS3DH_MODE_HIGH_RESOLUTION: Serial.println("High Resolution 12bit"); break;
  }

  // lis.setDataRate(LIS3DH_DATARATE_50_HZ);
  Serial.print("    Data rate set to: ");
  switch (lis.getDataRate()) {
    case LIS3DH_DATARATE_1_HZ: Serial.println("1 Hz"); break;
    case LIS3DH_DATARATE_10_HZ: Serial.println("10 Hz"); break;
    case LIS3DH_DATARATE_25_HZ: Serial.println("25 Hz"); break;
    case LIS3DH_DATARATE_50_HZ: Serial.println("50 Hz"); break;
    case LIS3DH_DATARATE_100_HZ: Serial.println("100 Hz"); break;
    case LIS3DH_DATARATE_200_HZ: Serial.println("200 Hz"); break;
    case LIS3DH_DATARATE_400_HZ: Serial.println("400 Hz"); break;

    case LIS3DH_DATARATE_POWERDOWN: Serial.println("Powered Down"); break;
    case LIS3DH_DATARATE_LOWPOWER_5KHZ: Serial.println("5 Khz Low Power"); break;
    case LIS3DH_DATARATE_LOWPOWER_1K6HZ: Serial.println("1.6 Khz Low Power"); break;
  }  
}

void AccelerometerReadAccel() {
  lis.read();      // get X Y and Z data at once
  // Then print out the raw data
  // Serial.print("X:  "); Serial.print(lis.x);
  // Serial.print("  \tY:  "); Serial.print(lis.y);
  // Serial.print("  \tZ:  "); Serial.print(lis.z);

  /* Or....get a new sensor event, normalized */
  // sensors_event_t event;
  // lis.getEvent(&event);

  /* Display the results (acceleration is measured in m/s^2) */
  // Serial.print("\t\tX: "); Serial.print(event.acceleration.x);
  // Serial.print(" \tY: "); Serial.print(event.acceleration.y);
  // Serial.print(" \tZ: "); Serial.print(event.acceleration.z);
  // Serial.println(" m/s^2 ");

  Serial.println();
}

bool AccelerometerSenseGestureErase() {
  bool UpSideDown;
  bool ShakeLeft;
  bool ShakeRight;
  static uint16_t ShakeLeftCount;
  static uint16_t ShakeRightCount;
  static uint16_t ShakeSensitivyThreshold = 10000;
  static uint16_t ShakeCountThreshold = 30;
  AccelerometerReadAccel();
  // Is it up-side-down?
  if (lis.z > 10000) { UpSideDown = true;  }
  else               { UpSideDown = false; }
  // Check for Left shake
  if (lis.y > ShakeSensitivyThreshold) { ShakeLeft = true;  }
  else               { ShakeLeft = false; }
  // Check for Right shake
  if (lis.y < -ShakeSensitivyThreshold) { ShakeRight = true;  }
  else                { ShakeRight = false; }
  // If it is up-side-down...
  if (UpSideDown) {
    // ...Accumulate left/right shakes...
    if (ShakeLeft) { ShakeLeftCount++; }
    if (ShakeRight) { ShakeRightCount++; }
  }
  // ...otherwise reset the counters.
  else {
    ShakeLeftCount = 0;
    ShakeRightCount = 0;
  }
  // Serial.print(ShakeLeftCount);
  // Serial.print(", ");
  // Serial.println(ShakeRightCount);
  // If all conditions are met, signal screen clearing.
  if ((ShakeLeftCount > ShakeCountThreshold) && (ShakeRightCount > ShakeCountThreshold)) {
    ShakeLeftCount = 0;
    ShakeRightCount = 0;
    return 1;
    OLEDBackgroundReset();
  }
  else { return 0; }
}

bool AccelerometerSenseGestureChangeBackground() {
  bool RightSideUp;
  bool ShakeLeft;
  bool ShakeRight;
  static uint16_t ShakeLeftCount;
  static uint16_t ShakeRightCount;
  static uint16_t ShakeSensitivyThreshold = 10000;
  static uint16_t ShakeCountThreshold = 30;
  AccelerometerReadAccel();
  // Is it up-side-down?
  if (lis.z < 10000) { RightSideUp = true;  }
  else               { RightSideUp = false; }
  // Check for Left shake
  if (lis.y > ShakeSensitivyThreshold) { ShakeLeft = true;  }
  else               { ShakeLeft = false; }
  // Check for Right shake
  if (lis.y < -ShakeSensitivyThreshold) { ShakeRight = true;  }
  else                { ShakeRight = false; }
  // If it is right-side-up...
  if (RightSideUp) {
    // ...Accumulate left/right shakes...
    if (ShakeLeft) { ShakeLeftCount++; }
    if (ShakeRight) { ShakeRightCount++; }
  }
  // ...otherwise reset the counters.
  else {
    ShakeLeftCount = 0;
    ShakeRightCount = 0;
  }
  // If all conditions are met, signal background color change.
  if ((ShakeLeftCount > ShakeCountThreshold) && (ShakeRightCount > ShakeCountThreshold)) {
    ShakeLeftCount = 0;
    ShakeRightCount = 0;
    return 1;
  }
  else { return 0; }
}

bool AccelerometerSenseGestureChangeADCSource() {
  bool RightSideUp;
  bool ShakeUp;
  bool ShakeDown;
  static uint16_t ShakeUpCount;
  static uint16_t ShakeDownCount;
  static uint16_t ShakeSensitivyThreshold = 10000;
  static uint16_t ShakeCountThreshold = 30;
  AccelerometerReadAccel();
  // Is it up-side-down?
  if (lis.z < 10000) { RightSideUp = true;  }
  else               { RightSideUp = false; }
  // Check for up shake
  if (lis.x > ShakeSensitivyThreshold) { ShakeUp = true;  }
  else               { ShakeUp = false; }
  // Check for down shake
  if (lis.x < -ShakeSensitivyThreshold) { ShakeDown = true;  }
  else                { ShakeDown = false; }
  // If it is right-side-up...
  if (RightSideUp) {
    // ...Accumulate up/down shakes...
    if (ShakeUp) { ShakeUpCount++; }
    if (ShakeDown) { ShakeDownCount++; }
  }
  // ...otherwise reset the counters.
  else {
    ShakeUpCount = 0;
    ShakeDownCount = 0;
  }
  // If all conditions are met, signal ADC source change.
  if ((ShakeUpCount > ShakeCountThreshold) && (ShakeDownCount > ShakeCountThreshold)) {
    ShakeUpCount = 0;
    ShakeDownCount = 0;
    return 1;
  }
  else { return 0; }
}

void OLEDClear() {
  display.clearDisplay();
  display.display();
}

void OLEDFill() {
  display.fillRect(0,0,display.width(),display.height(),OLED_WHITE);
  display.display();
}

void OLEDBackgroundReset() {
  if (EASBackground) { OLEDFill();  }
  else               { OLEDClear(); }
}


void SAOGPIOPinInit (){
  // Nothing to do because analog input is default for Arduino.
}

void ModeTimeOutCheckReset () {
  ModeTimeoutDeltams = 0;
  ModeTimeoutFirstTimeRun = true;
}

bool ModeTimeOutCheck (uint32_t ModeTimeoutLimitms) {
  static uint32_t NowTimems;
  static uint32_t StartTimems;
  NowTimems = millis();
  if(ModeTimeoutFirstTimeRun) { StartTimems = NowTimems; ModeTimeoutFirstTimeRun = false;}
  ModeTimeoutDeltams = NowTimems-StartTimems;
  if (ModeTimeoutDeltams >= ModeTimeoutLimitms) {
    ModeTimeOutCheckReset();
    Serial.print(">>> Mode timeout after ");
    Serial.print(ModeTimeoutLimitms);
    Serial.println(" ms.");
    return (true);
  }
  else {
    return (false);
  }
}

// -------------------------------------------------------------------------------------------
// MAIN LOOP STARTS HERE
// -------------------------------------------------------------------------------------------
void loop()
{
  switch(TopLevelMode) {
    case MODE_INIT: {
      SerialInit();
      Serial.println(">>> Entered MODE_INIT.");
      OLEDInit();
      AccelerometerInit();
      SAOGPIOPinInit();
      Serial.println("");
      Serial.println("  |-------------------------------------------------------------------------------| ");
      Serial.println("  | Welcome to the Etch sAo Sketch Demo with Arduino IDE 2.3.2 using RP2040-Zero! | ");
      Serial.println("  |-------------------------------------------------------------------------------| ");
      Serial.println("");
      ModeTimeOutCheckReset(); 
      TopLevelMode = TopLevelModeDefault;
      Serial.println(">>> Leaving MODE_INIT.");
      break;
    }

    case MODE_BOOT_SCREEN: {
      static bool CursorState = 0;
      static uint32_t NowTime;
      static uint32_t LastTime;
      static uint32_t BlinkDeltaTime = 600;

      if (ModeTimeoutFirstTimeRun) { 
        Serial.println(">>> Entered MODE_BOOT_SCREEN."); 
        // Border
        uint8_t BorderThickness = 12;
        // display.fillRect(BorderThickness,BorderThickness,(display.width()-(2*BorderThickness)),(display.height()-(2*BorderThickness)),3);
        // display.display();
        // delay(1000);
        for (uint8_t i=0; i<BorderThickness; i+=1) {
          display.drawRect(i, i, display.width()-2*i, display.height()-2*i, OLED_WHITE);
        }
        display.display();
        delay(500);
        // Background
        // display.fillRect(2*BorderThickness,BorderThickness,2*BorderThickness,2*BorderThickness,OLED_GRAY);
        //for (uint8_t i=BorderThickness; i<display.height(); i+=1) {
        //  display.drawRect(i, i, display.width()-2*i-BorderThickness, display.height()-2*i-BorderThickness, OLED_GRAY);
        //}
        // display.setFont(&FreeMono9pt7b);
        display.setTextSize(1);
        // display.cp437(true);
        display.setTextColor(SSD1327_WHITE);
        display.setCursor(20,20);
        display.println("128 Bytes Free");
        display.setCursor(12,40);
        display.println("Ready.");
        display.display();
      }

      // Is it time to blink?
      NowTime = millis();
      if ( (NowTime-LastTime) > BlinkDeltaTime) {
        // Blink the cursor
        LastTime = NowTime;
        if (CursorState) { display.fillRect(12,47,8,10,SSD1327_WHITE); CursorState = 0;}
        else { display.fillRect(12,47,8,10,SSD1327_BLACK);  CursorState = 1;}
        display.display();
      }

      if (ModeTimeOutCheck(7000)){ 
        ModeTimeOutCheckReset();
        OLEDClear();
        TopLevelMode++; 
        Serial.println(">>> Leaving MODE_BOOT_SCREEN.");
      }
      break;
    }

    case MODE_SKETCH: {
      if (ModeTimeoutFirstTimeRun) { 
        Serial.println(">>> Entered MODE_SKETCH.");
        OLEDBackgroundReset();
        ModeTimeoutFirstTimeRun = false;
      }
      
      // Get analog readings of the two pots
      // Arduino Analog Reading (Arduino with RP2040 allows 0-3.3V, full-scale, reading)
      if (EASAnalogSource) {
        PotLeftADCCounts = analogRead(PIN_SAO_GPIO_1_ANA_POT_LEFT);
        PotRightADCCounts = analogRead(PIN_SAO_GPIO_2_ANA_POT_RIGHT);
      }
      // Accelerometer Analog Reading (LIS3DH Analog reading is partial scale, limited between 0.9 and 1.8V)
      // Also scale these readings up to what Arduino ADC uses, so the rest of the processing code below doesn't need to change.
      else {
        int16_t adc;
        uint16_t volt;
        adc = lis.readADC(1);
        volt = map(adc, -32512, 32512, 1800, 900);
        // Serial.print("ADC1:\t"); Serial.print(adc); 
        // Serial.print(" ("); Serial.print(volt); Serial.print(" mV)  ");
        PotLeftADCCounts = map(adc, -32512, 32512, 1023, 0);
        adc = lis.readADC(2);
        volt = map(adc, -32512, 32512, 1800, 900);
        // Serial.print("ADC2:\t"); Serial.print(adc); 
        // Serial.print(" ("); Serial.print(volt); Serial.print(" mV)  ");
        PotRightADCCounts = map(adc, -32512, 32512, 1023, 0);
      }
      // Apply glitch rejection

      // Apply some filtering

      // Apply some hysteresis
      deltaAbsolute = abs(PotLeftADCCounts-PotLeftADCCountsOld);
      if (deltaAbsolute < PotHystersisLimit) {
        PotLeftADCCounts = PotLeftADCCountsOld;
      }
      deltaAbsolute = abs(PotRightADCCounts-PotRightADCCountsOld);
      if (deltaAbsolute < PotHystersisLimit) {
        PotRightADCCounts = PotRightADCCountsOld;
      }
      
      // Scale ADC counts to the useable screen with margins at end of pot travel.
      cursorX = map(PotLeftADCCounts , (1023-PotMarginAtLimit), PotMarginAtLimit, 0, 127);
      cursorY = map(PotRightADCCounts, PotMarginAtLimit, (1023-PotMarginAtLimit), 0, 127);
      PotLeftADCCountsOld = PotLeftADCCounts;
      PotRightADCCountsOld = PotRightADCCounts;
      // Serial.print("Cursor position: ");
      // Serial.print(cursorX);
      // Serial.print(",");
      // Serial.println(cursorY);

      // Update the display
      if (EASBackground) { display.drawRect(cursorX, cursorY, 2, 2, OLED_BLACK); }
      else { display.drawRect(cursorX, cursorY, 2, 2, OLED_WHITE); }
      display.display();

      // Check for gestures
      if (AccelerometerSenseGestureErase()) { 
        OLEDBackgroundReset(); 
        }
      if (AccelerometerSenseGestureChangeBackground()) {
        if (EASBackground) { 
          EASBackground = 0;
          OLEDBackgroundReset();
          }
        else {
          EASBackground = 1;
          OLEDBackgroundReset();
          }
        }
      if (AccelerometerSenseGestureChangeADCSource()) {
        if (EASAnalogSource) { EASAnalogSource = 0;  }
        else                 { EASAnalogSource = 1; }
        }
      break;
    }

    case MODE_AT_THE_END_OF_TIME: {
      Serial.println(">>> Stuck in the MODE_AT_THE_END_OF_TIME! <<<");
      break;
    }

    default: {
      Serial.println(">>> Something is broken! <<<");
      break;
    }
  }
}
