#include <Arduino.h>
#include <Wire.h>
#include <U8g2lib.h>

// 1. Instantiate both possible display structures globally (adjust resolutions/names to match your exact hardware)
U8G2_SSD1327_EA_W128128_F_HW_I2C display_SSD1327(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);
U8G2_SH1107_SEEED_128X128_F_HW_I2C display_SH1107(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);

// 2. Define the generic base object pointer we will use throughout the code
U8G2 *u8g2 = nullptr;

#define DISPLAY_I2C_ADDRESS 0x3C

// bool isSSD1327Connected() {
//   Wire.beginTransmission(DISPLAY_I2C_ADDRESS);
//   // Send a control byte indicating a command stream, followed by an SSD1327-specific command
//   // SSD1327 uses 0x15 for Set Column Address; SH1107 does not have a 0x15 command at top level.
//   Wire.write(0x00); // Co = 0, D/C# = 0 (Command stream)
//   Wire.write(0x15); // Set Column Address command (Unique marker behavior for SSD1327 identification)
  
//   byte error = Wire.endTransmission();
//   return (error == 0); // If 0, the device successfully acknowledged the sequence
// }

bool isSSD1327Connected() {
  // Step 1: Send a status request/command stream token to the display
  Wire.beginTransmission(DISPLAY_I2C_ADDRESS);
  Wire.write(0x00); // Control byte: Co = 0, D/C# = 0 (Command register stream)
  // We send 0x00 which is a NOP (No Operation) or safe command for both chips
  Wire.write(0x00); 
  if (Wire.endTransmission() != 0) {
    return false; // Physical device missing completely
  }

  // Step 2: Request 1 byte back from the display controller's internal status register
  // SH1107 explicitly responds to I2C read requests with its Status Byte.
  // SSD1327 requires a different read routine or returns a static/different bit layout.
  uint8_t readCount = Wire.requestFrom(DISPLAY_I2C_ADDRESS, 1);
  
  if (readCount > 0) {
    uint8_t statusByte = Wire.read();
    Serial.print("Debug - Read Status Byte: 0x");
    Serial.println(statusByte, HEX);

    // On power-up (before .begin() clears or alters states):
    // SH1107 status byte naturally exposes power flags (typically 0x00 or 0x40 depending on panel configurations)
    // We can explicitly look for the SSD1327's unique signature layout.
    // If the read value matches SH1107 default states, assume SH1107.
    
    // A highly reliable fallback is checking a specific bit pattern:
    // SH1107 Bit 6 is 1 when display is ON, 0 when display is OFF. 
    // At boot, the display is OFF, so Bit 6 is 0.
    // If the returned byte is exactly 0x00 or matches SH1107 traits, it's an SH1107.
    if (statusByte == 0x00 || (statusByte & 0x40) == 0) {
      return false; // It's the SH1107
    }
  }

  return true; // Treat as SSD1327 fallback
}

void setup() {
  Serial.begin(115200);
  Wire.begin(); // Boot standard hardware I2C line first
  delay(3000);   // Give the hardware basic stabilization time

  Serial.println("Probing I2C bus for display type...");

  // Verify if a device is active on the expected address at all
  Wire.beginTransmission(DISPLAY_I2C_ADDRESS);
  if (Wire.endTransmission() != 0) {
    Serial.println("Error: No display detected at address 0x3C!");
    while (1); // Halt if completely missing
  }

  //Differentiate between SSD1327 and SH1107
  if (isSSD1327Connected()) {
    Serial.println("Detected: SSD1327 Display. Assigning driver...");
    u8g2 = &display_SSD1327;
  } else {
    Serial.println("Detected: SH1107 Display. Assigning driver...");
    u8g2 = &display_SH1107;
  }

  // 3. Initialize the dynamically assigned driver configuration 
  u8g2->begin();
}

void loop() {
  // 4. Use the generic pointer cleanly everywhere else in your program execution loop
  u8g2->clearBuffer();
  u8g2->setFont(u8g2_font_ncenB14_tr);
  u8g2->drawStr(0, 20, "System Online");
  u8g2->sendBuffer();
  Serial.println("Connected to display.");
  delay(2000);
}
