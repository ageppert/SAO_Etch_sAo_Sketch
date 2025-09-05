from machine import SoftI2C, Pin
import time
import ssd1327
import lis3dh_wrapper

__version__ = '0.1.5'

class EtchSaoSketch():
    _display = None
    
    calib_left_zero_offset = 27880
    calib_right_zero_offset = 27945
    calib_voltage_scaling = 1.750592

    def __init__(self, i2c_bus, enable_calib=True, enable_shaking=True):
        self._i2c = i2c_bus
        self._enable_calib = enable_calib
        self._enable_shaking = enable_shaking
        print ("init ssd1327 display")
        self._display = ssd1327.WS_OLED_128X128(self._i2c, max_segment_length=512)
        print ("init accelerometer (and ADC)")
        self._lis3dh = lis3dh_wrapper.lis3dh_wrapper(self._i2c)
        self._lis3dh.set_tap(tap=2, threshold=80, time_limit=1, click_cfg=0x04)
    
    @property
    def shake_detected(self):
        # tapped is faster, shake takes too much time
        return self._lis3dh.tapped
        #return self._lis3dh.shake(shake_threshold=19.5, avg_count=10)
    
    @property
    def rotation(self):
        return self._lis3dh.get_accell_rotation()
   
    @property
    def left(self):
        self.prev_raw_left = self._lis3dh.left
        calibrated_left = (self.prev_raw_left - self.calib_left_zero_offset) * self.calib_voltage_scaling
        scaled_left = calibrated_left // 508
        clamped_left = max(0, min(scaled_left, 127))
        return int(clamped_left)

    @property
    def right(self):
        self.prev_raw_right = self._lis3dh.right
        calibrated_right = (self.prev_raw_right- self.calib_right_zero_offset) * self.calib_voltage_scaling
        scaled_right = calibrated_right // 508
        clamped_right = max(0, min(scaled_right, 127))
        return int(clamped_right)
    
    def try_calibration_routine(self):
        # Tries to calibrate lowest and highest measured values on the ADC.
        # If the values don't look intentional they will be ignored.
        # Expected calibration routine:
        # Within 5 seconds, in the following order
        # 1. Turn both knobs all the way right
        # 2. Turn both knobs all the way left
        
        self.draw_text("Calibrating", 10, 20, 15)
        self.draw_text("1. Turn knobs", 10, 30, 15)
        self.draw_text("to right limit", 10, 40, 15)
        
        highest_r = 0
        highest_l = 0
        lowest_r = 65535
        lowest_l = 65535
        calib_initiated = False
        left_registered = False
        calib_successful = False
        deadline = time.ticks_add(time.ticks_ms(), 5000)
        while time.ticks_diff(deadline, time.ticks_ms()) > 0 and not left_registered:
            r = self._lis3dh.right
            l = self._lis3dh.left
            # Debug:
            # self.draw_pixel(int(r/(lowest_r/128)), int(l/(lowest_l/128)), 10)
            # self.draw_display()
            if abs(r-l) < 1000:
                # Both knobs are more or less in the same position
                if r > 60000:
                    # Knobs are assumed to be all the way to the right. Start calibration routine
                    if r > highest_r:
                        highest_r = r
                    if l > highest_l:
                        highest_l = l
                    if not calib_initiated:
                        print("Registered knobs all the way right")
                        self.draw_text("2. Turn knobs", 10, 50, 15)
                        self.draw_text("to left limit", 10, 60, 15)
                        deadline = time.ticks_add(time.ticks_ms(), 5000) # Extend deadline
                        calib_initiated = True
                elif r < highest_r - 25000 and calib_initiated:
                    # Knobs are assumed to be all the way to the left
                    if r < lowest_r:
                        lowest_r = r
                    if l < lowest_l:
                        lowest_l = l
                    if not left_registered:
                        print("Registered knobs all the way left")
                        left_registered = True
        self.clear_display()
        if lowest_r < 65535 and lowest_l < 65535 and highest_r > 0 and highest_l > 0:
            calib_successful = True
            # Since we are not averaging the low and high values over time, we are reading uncommonly low and high values.
            # Add some margin to counteract this effect, preventing the screen area from being unintentionally clipped
            lowest_r = lowest_r + 1000
            lowest_l = lowest_l + 1000
            highest_r = highest_r - 1000
            highest_r = highest_r - 1000
            print(f"l_r={lowest_r}, l_l={lowest_l}, h_r={highest_r}, h_l={highest_l}")
            self.calib_right_zero_offset = lowest_r
            self.calib_left_zero_offset = lowest_l
            self.calib_voltage_scaling = max(highest_r, highest_l) / (max(highest_r, highest_l) - min(lowest_r, lowest_l))
            
            self.draw_text("Calibration", 10, 20, 15)
            self.draw_text("successful!", 10, 30, 15)
            self.draw_text("Values:", 10, 40, 15)
        else:
            self.draw_text("Calibration", 10, 20, 15)
            self.draw_text("failed...", 10, 30, 15)
            self.draw_text("Defaults:", 10, 40, 15)
        self.draw_text(f"{self.calib_left_zero_offset},", 10, 50, 15)
        self.draw_text(f"{self.calib_right_zero_offset},", 10, 60, 15)
        self.draw_text(f"{self.calib_voltage_scaling:.6f}", 10, 70, 15)
        return calib_successful
                    
    def clear_display(self):
        if self._display:
            self._display.fill(0)
            self.draw_display()
        
    def draw_pixel(self, x, y, color):
        if self._display:
            self._display.pixel(x, y, color)
            
    def draw_line(self, x1, y1, x2, y2, color):
        if self._display:
            self._display.line(x1, y1, x2, y2, color)
    
    def draw_text(self, text, x, y, color):
        if self._display:
            self._display.text(text, x, y, color)
        self.draw_display()
    
    def draw_display(self):
        if self._display:
            self._display.show()

    # to initialize the sketch game
    def sketch_init(self):
        self.clear_display()

        if self._enable_calib:
            # Calibrate after screen has started, to account for power drop caused by the OLED current draw
            print("Starting ADC calibration routine:")
            print("Within 5 seconds, in the following order")
            print("1. Turn both knobs all the way right")
            print("2. Turn both knobs all the way left")
            success = self.try_calibration_routine()
            if success:
                print("ADC calibration succeeded.")
                print(f"Calibration values: r={self.calib_right_zero_offset}, l={self.calib_left_zero_offset}, s={self.calib_voltage_scaling}")
            else:
                print("ADC calibration failed.")
                print(f"Using default values: r={self.calib_right_zero_offset}, l={self.calib_left_zero_offset}, s={self.calib_voltage_scaling}")

        time.sleep_ms(500)
        self.clear_display()

        self._cycles = 20 # 20 seems OK with fully populated badge, 40 is OK with only Etch sAo Sketch connected, but brings little additional benefit
        self._avg_cycles = self._cycles
        self._avg_left = 0
        self._avg_right = 0
        self._prev_left = self.left
        self._prev_right = 127 - self.right
    
    # the main loop of the sketch game to be called periodically
    def sketch_loop(self):
        # Check if the badge has been flipped, and clear the screen if it has
        if self._enable_shaking and self.shake_detected:
            print("Shake detected")
            self.clear_display()

        self._avg_left += self.left
        self._avg_right += 127 - self.right

        if self._avg_cycles == 0:
            etch_left = min(max(int(self._avg_left/self._cycles), 0), 127)
            etch_right = min(max(int(self._avg_right/self._cycles), 0), 127)
        
            self.draw_line(self._prev_left, self._prev_right, etch_left, etch_right, 15)
            self.draw_display()

            self._prev_left = etch_left
            self._prev_right = etch_right

            self._avg_cycles = self._cycles
            self._avg_left = 0
            self._avg_right = 0
        else:
            self._avg_cycles -= 1


if __name__ == "__main__":
    
    i2c1 = SoftI2C(sda=Pin(26), scl=Pin(27))
    
    sao = EtchSaoSketch(i2c1)
    sao.sketch_init()
    
    while True:
       sao.sketch_loop()
