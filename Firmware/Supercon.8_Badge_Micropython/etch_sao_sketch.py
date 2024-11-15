from machine import I2C, Pin
import time
import ssd1327
import lis3dh

__version__ = '0.1.5'

# GPIO pins to input so I2C ADC on LIS3DH on Etch-sAo-sketch works.
# We assume SAO is connected to "port 6" (pin 17/18)
gpio61 = Pin(18, Pin.IN)
gpio62 = Pin(17, Pin.IN)

class EtchSaoSketch():
    _display = None

    def __init__(self, i2c_bus):
        print ("saving i2c bus")
        self._i2c = i2c_bus
        print ("init ssd1327 display")
        # self._display = ssd1327.SSD1327(self._i2c)
        print ("init accel")
        self._lis3dh = lis3dh.Lis3dh(self._i2c)
   
    @property
    def left(self):
        return self._lis3dh.left

    @property
    def right(self):
        return self._lis3dh.right
    
    def shake(self):
        if self._display:
            self._display.clear()
        
    def draw_pixel(self, x, y, color):
        if self._display:
            self._display.draw_pixel(x, y, color)
    
    def draw_display(self):
        if self._display:
            self._display.show()
    
if __name__ == "__main__":
    
    i2c1 = I2C(1, sda=Pin(26), scl=Pin(27), freq=400_000)
    
    sao = EtchSaoSketch(i2c1)
    sao.shake() # clear display
    
    while True:
        left = sao.left
        right = 128 - sao.right
        print (left, right)
        sao.draw_pixel(left, right, 1)
        sao.draw_display()
