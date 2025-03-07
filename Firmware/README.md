# Etch_sAo_Sketch Firmware Examples

The Etch sAo Sketch requires a microcontroller with an I2C bus to run. This folder contains a few examples.

"Etch_sAo_Sketch_V1.0_RP2040_W_Arduino_Demo" is designed to work with the SAO Demo Controller, or any RP2040-based board. It provides the full drawing functionality for the V1 Etch sAo Sketch.

"Etch_sAo_Sketch_V1.1_RP2040_W_Arduino_Demo" is refined for the V1.1 Etch sAo Sketch which has resistors added in series with the potentiometers to allow full travel of the knobs to map to the full screen. It's a quality of life improvement.

"OLED_ssd1327_test" is Arduino sample code to activate the OLED screen.

"Supercon.8_Badge_Micropython" are the files needed to run the Etch sAo Sketch with the Supercon 8 Badge using Micropython. Etch-sAo-Sketch goes in the port marked "6". The petal SAO goes in "1" and the touchwheel goes in "2". Sample code provided by Derek Jamison. https://youtube.com/@MrDerekJamison
