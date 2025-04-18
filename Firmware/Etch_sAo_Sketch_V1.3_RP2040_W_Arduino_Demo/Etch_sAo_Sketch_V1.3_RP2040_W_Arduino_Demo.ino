/******************************************************************************************************************
  Etch sAo Sketch (EAS) Demo code using RP2040 and the Arduino IDE

  Github repo: https://github.com/ageppert/SAO_Etch_sAo_Sketch
  Project page: https://hackaday.io/project/197581-etch-sao-sketch

  DEPENDENCIES - FIRMWARE
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

  DEPENDENCIES - HARDWARE
    OLED with SSD1327 Driver
    Accelerometer LIS3DH
    External user provided RP2040 MCU tested so far

  CONFIGURE SAO DEMO CONTROLLER FOR ETCH SAO SKETCH HARDWARE V1.0
    If you are using SAO Demo Controller V2 with EAS V1.0 you need to close solder jumper JP5&6 pads 2-3 so the
    analog pots will be connected to GP26/ADC0 and GP27/ADC1 of the RP2040-Zero on the SAO Demo Controller. This enables
    access to the full range of the potentiometers in EAS V1.0 which are configured for 0 to 3.3V ADC sensing.
    Etch sAo Sketch V1.1+ can access full range pot control with the ADC in the accelerometer.

  CONFIGURE THIS FIRMWARE TO WORK WITH CORRECT ETCH SAO SKETCH HARDWARE
    See the ETCH SAO SKETCH - HARDWARE VERSION TABLE and hardware version setting variables immediately below it to compile
    this firmware to work correctly with your specific Etch sAo Sketch board.

  HARDWARE CONNECTIONS
    OLED 1.5" 128x128 SSD1327: https://learn.adafruit.com/adafruit-grayscale-1-5-128x128-oled-display/arduino-wiring-and-test
    Accelerometer LIS3DH: https://learn.sparkfun.com/tutorials/lis3dh-hookup-guide and https://learn.adafruit.com/adafruit-lis3dh-triple-axis-accelerometer-breakout/arduino  
    Demo board -> RP2040-Zero (miniature Pico/RP2040 dev board) https://www.waveshare.com/rp2040-zero.htm
    The SAO port (X1) is intended to provide full access to the accelerometer, screen, and analog pots.
    SAO I2C pins connect to the OLED and Accelerometer (which also has the potentionmeters connected to built-in ADCs).
    SAO GPIO1 pin connected to the left Analog Potentiometer
    SAO GPIO2 pin connected to the right Analog Potentiometer
  
  MODES OF OPERATION
    MODE_INIT
      Connect to the serial port for status and debugging  
    MODE_BOOT_SCREEN
      Startup with the boot screen. Configured analog input source per hardware version specified beloe the HARDWARE VERSION TABLE.
      After timeout, automatcally go to sketching mode with default black background.
    MODE_SKETCH
      In sketch mode, rotate pots to draw.
      Gestures active during sketch mode:
        Upsidedown, shake left/right to clear sketch zone.
        Rightsideup, shake left/right to change background color (also clears screen).
        Rightsideup, shake up/down to change pot input between Arduino Analog port (0-3.3V) or accelerometer analog ports (0.8-1.6V).
    MODE_AT_THE_END_OF_TIME
      I hope you never land in this mode!

  /************************************ ETCH SAO SKETCH - FIRMWARE VERSION TABLE ************************************
  | VERSION |  DATE      | MCU     | DESCRIPTION                                                                    |
  -------------------------------------------------------------------------------------------------------------------
  |  1.0.0  | 2024-11-01 | RP2040  | First draft, hurry up for Supercon!
  |  1.1.0  | 2024-11-23 | RP2040  | Correct pot scaling for full range hardware mods with Accel ADC.
  |  1.3.0  | 2025-04-12 | RP2040  | Clarify hardware versions and configurations. Supports all hardware versions.
  |         |            |         |
  |         |            |         |
  -----------------------------------------------------------------------------------------------------------------*/
  // TODO: Make this work with Hackaday Supercon 2024 and Berlin 2025 Badge I2C ports 4-5-6 on pins 31 CL / 32 DA GPIO 26/27. 
  //        Ports 1-2-3 on pins 1 DA and 2 CL. GPIO 0/1
    uint8_t FirmwareVersionMajor  = 1;
    uint8_t FirmwareVersionMinor  = 3;
    uint8_t FirmwareVersionPatch  = 0;

  /************************************ ETCH SAO SKETCH - HARDWARE VERSION TABLE ************************************
  | VERSION |  DATE      |         | DESCRIPTION                                                                    |
  -------------------------------------------------------------------------------------------------------------------
  |  1.0    | 2024-09-27 |         | First prototype, as built, getting it to come alive. Analog input range of pots
  |         |            |         |   is much wider than accelerometer analog inputs can handle. Recommend to use
  |         |            |         |   only with 3.3V capable analog inputs of host MCU through SAO port GPIO1&2.
  |  1.0.1  | 2025-04-14 |         | First prototype, without excess pull-up resistors R30 and R31 10K.
  |  1.1    | 2024-11-23 |         | V1.0.0 modified with resistors inline (10K highside and 4.8K lowside) with the
  |         |            |         |   pots to enable full travel to map to accelerometer ADC 0.8 to 1.6V input range.
  |  1.2    | 2024-12-10 |         | RFQ only. Added resistors to scale pot voltage to range accelerometer accepts.
  |  1.3    | 2025-01-13 |         | Batch for Hackaday Europe with supplier name Elecrow on the front. Some may be
  |         |            |         |   updated with an additional clear Etch sAo Sketch sticker on the top as well.
  |  1.3.1  | 2025-04-13 |         | Removed R1 and R2 to reduce the excessive pull-up resistance for wider MCU
  |         |            |         |   compatibility.
  |         |            |         |
  |         |            |         |
  -------------------------------------------------------------------------------------------------------------------
  ************************************* Etch sAo Sketch Hardware Version Setting ************************************
  This default mode of input from the analog pots (I2C Accelerometer ADC = 0 or Arduino/RP2040 ADCs = 1) 
  will be set during MODE_BOOT_SCREEN...                                                                           */
    bool    EASAnalogSource;
  // ...based on the hardware version set in the following three lines.
    uint8_t HardwareVersionMajor  = 1;
    uint8_t HardwareVersionMinor  = 0;
    uint8_t HardwareVersionPatch  = 1;
/*******************************************************************************************************************/

#include <Adafruit_SSD1327.h>
#include <Fonts/FreeMono9pt7b.h>  // https://learn.adafruit.com/adafruit-gfx-graphics-library/using-fonts
// #include <Wire.h>                             // RP2040 Pico-Zero default I2C port is GPIO4 (SDA0) and GPIO5 (SCL0).
                                              // RP2040 Pico W default I2C port is GPIO4 (SDA0) and GPIO5 (SCL0).
#include <Adafruit_LIS3DH.h>
#include <Adafruit_Sensor.h>

#define OLED_ADDRESS                0x3C      // OLED SSD1327 is 0x3C
#define OLED_RESET                    -1
#define OLED_HEIGHT                  128
#define OLED_WIDTH                   128
#define OLED_WHITE                    15
#define OLED_GRAY                      7
#define OLED_BLACK                     0
// MbedI2C i2c(p0,p1);
Adafruit_SSD1327 display(OLED_HEIGHT, OLED_WIDTH, &Wire, OLED_RESET, 1000000);
uint8_t  color = OLED_WHITE;                        // 0-15 shades of gray
uint16_t cursorX = 0;                               // 0-127 pixels
uint16_t cursorY = 0;                               // 0-127 pixels
uint16_t cursorXold = 0;                            // 0-127 pixels
uint16_t cursorYold = 0;                            // 0-127 pixels
bool     EASBackground = 0;                         // 0 = dark, 1 = white (default)

#define ACCELEROMETER_ADDRESS       0x19      // LIS3DH Acceleromter default is 0x19
Adafruit_LIS3DH lis = Adafruit_LIS3DH();

#define PIN_SAO_GPIO_1_ANA_POT_LEFT   A0      // Configured as left analog pot
#define PIN_SAO_GPIO_2_ANA_POT_RIGHT  A1      // Configured as right analog pot
uint16_t PotLeftADCCounts = 0;
uint16_t PotRightADCCounts = 0;
uint16_t PotLeftADCCountsOld = 0;
uint16_t PotRightADCCountsOld = 0;
uint16_t PotMarginAtLimit = 10;
// uint16_t PotHystersisLimit = 12;
// uint16_t PotGlitchLimit = 64;
// uint16_t PotFilterSampleCount = 3;
// uint16_t deltaAbsolute = 0;
// uint16_t CursorHystersisLimit = 4;
int16_t AccelADCRangeLowCounts  = -4000;      // Tested with 3.3V supply voltage, R5 10K, R6 4K7
                                              // FWV V1.3 
int16_t AccelADCRangeHighCounts = 32512;
int16_t AccelADCRangeLowmV      =   900;      // FWV V1.3 
int16_t AccelADCRangeHighmV     =  1400;

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

class EmaFilterWithPriming
// from https://blog.mbedded.ninja/programming/signal-processing/digital-filters/exponential-moving-average-ema-filter
// with minimal round-off error, noticeable lag with the double precision calcs
// Alpha (0 to 1) is cut-off frequency, lower number is more filtering
{
public:
    EmaFilterWithPriming(double alpha) :
        m_alpha(alpha), m_lastOutput(0.0), m_firstRun(true) {}

    double Run(double input)
    {
        if (m_firstRun)
        {
            m_firstRun = false;
            m_lastOutput = input; // Bypass filter and prime with input
        }
        else
        {
            m_lastOutput = m_alpha * input + (1 - m_alpha) * m_lastOutput;
        }
        return m_lastOutput;
    }
private:
    double m_alpha;
    double m_lastOutput;
    bool m_firstRun;
};
EmaFilterWithPriming emaFilterX(0.1);
EmaFilterWithPriming emaFilterY(0.1);

// TODO: IMPLEMENT THE BIT SHIFT IIR THAT WORKS TO THE POLES
class IIRFilterWithIntegers
// from https://electronics.stackexchange.com/questions/30370/fast-and-memory-efficient-moving-average-calculation
// and https://forum.arduino.cc/t/understanding-some-code/499217 
// with round-off error, but very fast with bit shifting
// where alpha = 1 / 2^BITFILTER
{
public:
    IIRFilterWithIntegers(double alpha) :
        m_alpha(alpha), m_lastOutput(0.0), m_firstRun(true) {}

    double Run(double input)
    {
        if (m_firstRun)
        {
            m_firstRun = false;
            m_lastOutput = input; // Bypass filter and prime with input
        }
        else
        {
            m_lastOutput = m_alpha * input + (1 - m_alpha) * m_lastOutput;
        }
        return m_lastOutput;
    }
private:
    double m_alpha;
    double m_lastOutput;
    bool m_firstRun;
};
IIRFilterWithIntegers IIRFilterX(0.1);
IIRFilterWithIntegers IIRFilterY(0.1);


void setup()
{
  // Wire.setSDA(26);
  // Wire.setSCL(27);
  // MbedI2C i2c(1,2);

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
  if ( ! display.begin(OLED_ADDRESS) ) {
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

  // Serial.println();
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
  static uint16_t ShakeSensitivyThreshold = 8000;
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
      Serial.println("  |------------------------------------------------------------------------------------| ");
      Serial.println("  | Welcome to the Etch sAo Sketch Demo made with Arduino IDE 2.3.2 using RP2040-Zero! | ");
      Serial.println("  |------------------------------------------------------------------------------------| ");
      Serial.println("");
      Serial.print("Hardware Version: ");
      Serial.print(HardwareVersionMajor);
      Serial.print(".");
      Serial.print(HardwareVersionMinor );
      Serial.print(".");
      Serial.println(HardwareVersionPatch);
      Serial.print("Firmware Version: ");
      Serial.print(FirmwareVersionMajor);
      Serial.print(".");
      Serial.print(FirmwareVersionMinor );
      Serial.print(".");
      Serial.println(FirmwareVersionPatch);
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
        OLEDClear();
        display.display();
        // Border
        uint8_t BorderThickness = 12;
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
        // Configure analog input source based on hardware version.
        // See comments at top of this file in ETCH SAO SKETCH - HARDWARE VERSION TABLE for expected input source
        // 0 = accelerometer ADC, 1 = Arduino Analog Ports
        if (HardwareVersionMajor == 1) {
          if      (HardwareVersionMinor == 0) { EASAnalogSource = 1; }
          else if (HardwareVersionMinor == 1) { EASAnalogSource = 0; }
          else if (HardwareVersionMinor == 2) { EASAnalogSource = 0; }
          else if (HardwareVersionMinor == 3) { EASAnalogSource = 0; }
        }
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

      if (ModeTimeOutCheck(4000)){ 
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
      if (EASAnalogSource) {
        // Arduino Analog Reading (Arduino with RP2040 allows 0-3.3V, full-scale reading with 10-bit resolution, 0-1023 )
        PotLeftADCCounts = analogRead(PIN_SAO_GPIO_1_ANA_POT_LEFT);
        PotRightADCCounts = analogRead(PIN_SAO_GPIO_2_ANA_POT_RIGHT);
        // Scale ADC counts to the useable screen with margins at end of pot travel.
        cursorX = map(PotLeftADCCounts , (1023-PotMarginAtLimit), PotMarginAtLimit, 0, 127);
        cursorY = map(PotRightADCCounts, PotMarginAtLimit, (1023-PotMarginAtLimit), 0, 127);
      }
      else {
        // Accelerometer Analog Reading (LIS3DH Analog reading is partial scale, limited between 0.9 and 1.8V)
        // Also scale these readings up to what Arduino ADC uses, so the rest of the processing code below doesn't need to change.
        int16_t adc;
        uint16_t volt;
        adc = lis.readADC(1);
        // volt = map(adc, -32512, 32512, 1400, 900);
        volt = map(adc, AccelADCRangeLowCounts, AccelADCRangeHighCounts, AccelADCRangeHighmV, AccelADCRangeLowmV);
        // Serial.print("ADC1:\t"); Serial.print(adc); 
        // Serial.print(" ("); Serial.print(volt); Serial.print(" mV)  ");
        // PotLeftADCCounts = map(adc, -32512, 32512, 1023, 0);
        cursorX = map(adc, AccelADCRangeLowCounts, AccelADCRangeHighCounts, 0, 127);
        adc = lis.readADC(2);
        // volt = map(adc, -32512, 32512, 1400, 900);
        volt = map(adc, AccelADCRangeLowCounts, AccelADCRangeHighCounts, AccelADCRangeHighmV, AccelADCRangeLowmV);
        // Serial.print("ADC2:\t"); Serial.print(adc); 
        // Serial.print(" ("); Serial.print(volt); Serial.print(" mV)  ");
        // PotRightADCCounts = map(adc, -32512, 32512, 1023, 0);
        cursorY = map(adc, AccelADCRangeLowCounts, AccelADCRangeHighCounts, 127, 0);
      }
      // Glitch rejection
      if (cursorX > 130) { cursorX = 0;}
      if (cursorY > 130) { cursorY = 0;}

      // Hysteresis
      // if (cursorX > (cursorXold + CursorHystersisLimit) ) { cursorX = cursorXold;}
      // if (cursorX < (cursorXold - CursorHystersisLimit) ) { cursorX = cursorXold;}

      // Filtering
      #define FILTER_BITS   4
      #define SHIFT_BITS    8
      #define ROUNDUP       (2^SHIFT_BITS/2)
      // K = 1 / 2 ^ FILTER_BITS
      // y(n)        = K * x(n)     -  K * y(n-1)       +  y(n-1)
      // filter_new  = frac_sample  -  frac_filter_old  +  filter_old
      // local values are left-shifted up by SHIFT_BITS to maintain precision

/*
      Serial.print("cursorX_in:");
      Serial.print(cursorX);
      Serial.print(",");

      uint32_t local_sampleX = ((uint32_t)cursorX) << SHIFT_BITS;
      Serial.print("local_sampleX:");
      Serial.print(local_sampleX);
      Serial.print(",");

      uint32_t local_sampleX_fraction = local_sampleX >> FILTER_BITS;
      Serial.print("local_sampleX_fraction:");
      Serial.print(local_sampleX_fraction);
      Serial.print(",");

      uint32_t local_cursorX_old = ((uint32_t)cursorXold) << SHIFT_BITS;
      uint32_t local_cursorX_old_fraction = local_cursorX_old >> FILTER_BITS;

      Serial.print("local_cursorX_old_fraction:-");
      Serial.print(local_cursorX_old_fraction);
      Serial.print(",");

      Serial.print("local_cursorX_old:");
      Serial.print(local_cursorX_old);
      Serial.print(",");

      uint32_t local_filterX_new = local_sampleX_fraction - local_cursorX_old_fraction + local_cursorX_old + ROUNDUP ;
      Serial.print("local_filterX_new:");
      Serial.print(local_filterX_new);
      Serial.print(",");

      Serial.print("cursorX:");
      cursorX = (uint16_t) ( (local_filterX_new ) >> SHIFT_BITS);
      if (cursorX > 127) {cursorX = 127;}
      Serial.print(cursorX);
      Serial.println();
*/
      double inputX = (double)cursorX;
      double outputX = emaFilterX.Run(inputX);
      cursorX = (uint16_t)outputX;
/*
      Serial.print("cursorX_out:");
      Serial.print(cursorX);
      Serial.println();

      // Serial.print("cursorY:");
      // Serial.print(cursorY);
      // Serial.print(",");

      uint32_t local_sampleY = ((uint32_t)cursorY) << SHIFT_BITS;
      // Serial.print("local_sampleY:");
      // Serial.print(local_sampleY);
      // Serial.print(",");

      uint32_t local_sampleY_fraction = local_sampleY >> FILTER_BITS;
      // Serial.print("local_sampleY_fraction:");
      // Serial.print(local_sampleY_fraction);
      // Serial.print(",");

      uint32_t local_cursorY_old = ((uint32_t)cursorYold) << SHIFT_BITS;
      uint32_t local_cursorY_old_fraction = local_cursorY_old >> FILTER_BITS;

      // Serial.print("local_cursorY_old_fraction:-");
      // Serial.print(local_cursorY_old_fraction);
      // Serial.print(",");

      // Serial.print("local_cursorY_old:");
      // Serial.print(local_cursorY_old);
      // Serial.print(",");

      uint32_t local_filterY_new = local_sampleY_fraction - local_cursorY_old_fraction + local_cursorY_old ;
      // Serial.print("local_filterY_new:");
      // Serial.print(local_filterY_new);
      // Serial.print(",");

      // Serial.print("cursorY:");
      cursorY = (uint16_t) (local_filterY_new >> SHIFT_BITS);
      if (cursorY > 127) {cursorY = 127;}
      // Serial.print(cursorY);
      // Serial.println();
*/
      double inputY = (double)cursorY;
      double outputY = emaFilterY.Run(inputY);
      cursorY = (uint16_t)outputY;

      PotLeftADCCountsOld = PotLeftADCCounts;
      PotRightADCCountsOld = PotRightADCCounts;
      cursorXold = cursorX;
      cursorYold = cursorY;
      Serial.print("Cursor position: ");
      Serial.print(cursorX);
      Serial.print(",");
      Serial.println(cursorY);

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
