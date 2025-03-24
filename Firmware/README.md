# Etch_sAo_Sketch Firmware Examples

The Etch sAo Sketch requires a microcontroller with an I2C bus to run. This folder contains a few examples.

"Etch_sAo_Sketch_V1.0_RP2040_W_Arduino_Demo" is designed to work with the SAO Demo Controller, or any RP2040-based board. It provides the full drawing functionality for the V1 Etch sAo Sketch.

"Etch_sAo_Sketch_V1.1_RP2040_W_Arduino_Demo" is refined for the V1.1 Etch sAo Sketch which has resistors added in series with the potentiometers to allow full travel of the knobs to map to the full screen. It's a quality of life improvement.

"OLED_ssd1327_test" is Arduino sample code to activate the OLED screen.

"Supercon.8_Badge_Micropython" are the files needed to run the Etch sAo Sketch with the 2024 Supercon.8 and Hackaday Europe 2025 Badges using Micropython. Etch-sAo-Sketch must be plugged into the port marked "6" when the badge boots up. If used, the petal and touchwheel SAOs goes in "1", "2" or "3".

<img src="Simen says HELP.jpeg" height="200">

Special thanks to these folks for the Micropython work!

https://github.com/AakaFosfor
Added filtering to analog signals.

https://github.com/ORelio
Bug fix, compatibility with Bendy SAO.

https://github.com/SimenZhor
Enabled full functionality and wrestling the SSD1327 setup config, live at Hackaday Europe!
Added calibration to the ends of the analog pot ranges.

https://github.com/jamisonderek
https://youtube.com/@MrDerekJamison
Supercon.8 2024 sample code provided by Derek Jamison.
"If the drawing area is smaller than the size of the screen, change the `calib_` values in `etch_sao_sketch.py` (`calib_left_zero_offset`, `calib_right_zero_offset`, and `calib_voltage_scaling`). The default values **work for me™**. Shaking the SAO will clear the screen, to turn off this functionality set `enable_shaking = False` in `main.py`."
