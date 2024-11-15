from machine import I2C, Pin
import time

__version__ = '0.1.0'

class SSD1327():
    WIDTH = 128
    HEIGHT = 128
    I2C_ADDRESS = 0x3C
    
    def __init__(self, i2c: I2C):
        self.i2c = i2c
        self.buffer = bytearray(self.WIDTH * self.HEIGHT // 8)  # Buffer for the display
        self.init_display()

    def command(self, cmd):
        """Send a 1-byte command to the display."""
        self.i2c.writeto(self.I2C_ADDRESS, bytearray([0x00, cmd]))

    def command2(self, cmd, cmd2):
        """Send a 2-byte command to the display."""
        self.i2c.writeto(self.I2C_ADDRESS, bytearray([0x00, cmd, cmd2]))

    def split_into_segments(self, ydata, segment_length=1024):
        """Split an array into segments, so we don't timeout"""
        return [ydata[i : i + segment_length] for i in range(0, len(ydata), segment_length)]

    def data(self, xdata):
        segments = self.split_into_segments(xdata)
        for i, segment in enumerate(segments):
            self.i2c.writeto_mem(self.I2C_ADDRESS, 0x40, segment)

    def init_display(self):
      # Init sequence for 128x128 OLED module
      self.command(0xAE) # 
      self.command2(0x81,0x80) #
      self.command2(0xA0,0x5F) #
      self.command2(0xA1,0x40) #
      self.command2(0xA2,0x00) #
      self.command(0xA6) #
      self.command2(0xB1,0x11) #
      self.command2(0xB3,0x00) #
      self.command2(0xAB,0x01) #
      self.command2(0xB6,0x04) #
      self.command2(0xBE,0x0F) #
      self.command2(0xBC,0x08) #
      self.command2(0xD5,0x62) #
      self.command2(0xFD,0x12) #
      self.command(0xA4) #
      self.command(0xAF) #

    def clear(self):
        """Clear the display buffer and update the display."""
        self.buffer = bytearray(self.WIDTH * self.HEIGHT // 8)  # Clear buffer
        self.show()

    def show(self):
        """Send the buffer to the display."""
        self.command(0x21)  # Set column address
        self.command(0x00)  # Start column
        self.command(self.WIDTH - 1)  # End column
        
        self.command(0x22)  # Set page address
        self.command(0x00)  # Start page
        self.command((self.HEIGHT // 8) - 1)  # End page

        self.data(self.buffer)  # Write the display buffer to the OLED

    def draw_pixel(self, y, x, color):
        """Set a pixel in the buffer."""
        x = max(0, min([127,x])) # x: 0..127
        y = max(0, min([127,y])) # y: 0..127
        if (y>64): # Fix display bug (probably a register setting)
           y=y-64
        else:
          y=y+64
          
        if 0 <= x < self.WIDTH and 0 <= y < self.HEIGHT:
            index = x + (y // 8) * self.WIDTH
            if color:
                self.buffer[index] |= (1 << (y % 8))  # Set the bit
            else:
                self.buffer[index] &= ~(1 << (y % 8))  # Clear the bit

    def fill_rect(self, x, y, width, height, color):
        """Draw a filled rectangle on the buffer."""
        for i in range(width):
            for j in range(height):
                self.draw_pixel(x + i, y + j, color)

# Example usage
if __name__ == "__main__":
    # Initialize I2C
    i2c = I2C(1, sda=Pin(26), scl=Pin(27), freq=400_000)
    
    oled = SSD1327(i2c)

    # Clear the display
    oled.clear()
    oled.clear()

    for k in range(256):
        x=k
        y=k
        oled.draw_pixel(x, y, 1) # on
        oled.show()

