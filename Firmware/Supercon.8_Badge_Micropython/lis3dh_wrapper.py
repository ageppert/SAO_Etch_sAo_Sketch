from tinypico_micropython import lis3dh
import time, math
from machine import Pin, I2C

class lis3dh_wrapper:
        
    last_convert_time = 0
    convert_interval = 100 #ms
    
    def __init__(self, i2c_bus):
        self._imu = lis3dh.LIS3DH_I2C(i2c_bus, address=0x19)

        # Set range of accelerometer (can be RANGE_2_G, RANGE_4_G, RANGE_8_G or RANGE_16_G).
        self._imu.range = lis3dh.RANGE_8_G
        
        self.roll = 0
        self.pitch = 0

    # Convert acceleration to Pitch and Roll
    def get_accell_rotation(self):
        vec = self._imu.acceleration
        
        x_Buff = vec[0] # x
        y_Buff = vec[1] # y
        z_Buff = vec[2] # z

        # We only want to re-process the values every 100 ms
        if self.last_convert_time < time.ticks_ms():
            self.last_convert_time = time.ticks_ms() + self.convert_interval

            self.roll = math.atan2(y_Buff , z_Buff) * 57.3
            self.pitch = math.atan2((- x_Buff) , math.sqrt(y_Buff * y_Buff + z_Buff * z_Buff)) * 57.3

        # Return the current values in roll and pitch
        return ( self.roll, self.pitch )

    def _read_left(self):
      left = self._imu.read_adc_raw(1)
      if (left > 32512):
         left = left-65536
      left = left + 32512 # value from 0..65535
      return left 
      
    def _read_right(self):
      right = self._imu.read_adc_raw(2)
      if (right > 32512):
        right = right-65535
      right = right + 32512 # value from 0..65535
      return right
    
    def set_tap(self, tap, threshold,
                time_limit=10, time_latency=20, time_window=255, click_cfg=None):
        return self._imu.set_tap(tap, threshold, time_limit=time_limit, time_latency=time_latency, time_window=time_window, click_cfg=click_cfg)
    
    @property
    def left(self):
        return self._read_left()
   
    @property
    def right(self):
        return self._read_right()
    
    @property
    def acceleration(self):
        return self._imu.acceleration
    
    @property
    def tapped(self):
        return self._imu.tapped
    
    def shake(self, shake_threshold=19.5, avg_count=100, total_delay=0.01):
        return self._imu.shake(shake_threshold, avg_count, total_delay)

# Example usage
if __name__ == "__main__":    # If we have found the LIS3DH
    i2c = I2C(1, sda=Pin(26), scl=Pin(27))
    imu_wrapper = lis3dh_wrapper(i2c)

    if imu_wrapper._imu.device_check():
        # Loop forever printing values
        while True:
            # Read accelerometer values (in m / s ^ 2).  Returns a 3-tuple of x, y,
            # z axis values.  Divide them by 9.806 to convert to Gs.
            x, y, z = [value / lis3dh.STANDARD_GRAVITY for value in imu_wrapper.acceleration]
            print("x = %0.3f G, y = %0.3f G, z = %0.3f G" % (x, y, z))

            # Convert acceleration to Pitch and Roll and print values
            p, r = imu_wrapper.get_accell_rotation()
            print("pitch = %0.2f, roll = %0.2f" % (p,r))
            
            # read two ADCs (left, right)
            print("left: %d, right: %d" % (imu_wrapper.left, imu_wrapper.right))

            # Small delay to keep things responsive but give time for interrupt processing.
            time.sleep(0.1)

