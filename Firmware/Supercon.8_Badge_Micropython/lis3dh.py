from machine import I2C, Pin
import time

__version__ = '0.1.0'

class Lis3dh():
    I2C_ADDRESS = 0x19
    LEFT_ADC_REG = 0x88
    RIGHT_ADC_REG = 0x8A

    def _read_register(self, addr, reg, nbytes=1):
        return self._i2c.readfrom_mem(addr, reg, nbytes)

    def _read_register_expect(self, addr, reg, expected):
        value = self._read_register(addr, reg)

    def _init_lis3dh(self):
        self._i2c.writeto_mem(self.I2C_ADDRESS, 0x20, bytearray([0x07])) #
        self._read_register_expect(self.I2C_ADDRESS, 0x20, 0x07)
        self._i2c.writeto_mem(self.I2C_ADDRESS, 0x20, bytearray([0x77])) #
        self._i2c.writeto_mem(self.I2C_ADDRESS, 0x23, bytearray([0x88])) #
        self._read_register_expect(self.I2C_ADDRESS, 0x22, 0x10)
        self._i2c.writeto_mem(self.I2C_ADDRESS, 0x22, bytearray([0x10])) #
        self._i2c.writeto_mem(self.I2C_ADDRESS, 0x1F, bytearray([0x80])) #
        self._read_register_expect(self.I2C_ADDRESS, 0x23,0x88)
        self._i2c.writeto_mem(self.I2C_ADDRESS, 0x23, bytearray([0x88])) #
        self._read_register_expect(self.I2C_ADDRESS, 0x23, 0x88)
        time.sleep_ms(100)

    def __init__(self, i2c_bus):
        self._i2c = i2c_bus
        self._init_lis3dh()

    def read_adc(self):
        adc_registers = [self.LEFT_ADC_REG, self.RIGHT_ADC_REG]
        adc_values = []
        for reg in adc_registers:
            data = self._i2c.readfrom_mem(self.I2C_ADDRESS, reg, 2)
            value = (data[0] | (data[1] << 8))
            adc_values.append(value)
        return adc_values

    def _read_left(self):
      dataL = ((self._read_register(self.I2C_ADDRESS, self.LEFT_ADC_REG, 2)))
      left = (dataL[1]<<8)|dataL[0]
      if (left > 32512):
         left = left-65536
      left = left + 32512 # value from 0..65535
      return (left // 508) # value from 0..128
      
    def _read_right(self):
      dataR = ((self._read_register(self.I2C_ADDRESS, self.RIGHT_ADC_REG, 2)))
      right = (dataR[1]<<8)|dataR[0]
      if (right > 32512):
        right = right-65535
      right = right + 32512 # value from 0..65535
      return (right // 508) # value from 0..128
   
    @property
    def left(self):
        return self._read_left()
   
    @property
    def right(self):
        return self._read_right()
    
    
if __name__ == "__main__":
    # Initialize I2C
    i2c1 = I2C(1, sda=Pin(26), scl=Pin(27), freq=400_000)
    
    lis = Lis3dh(i2c1)
    while True:
        print (lis.left, lis.right)
    

