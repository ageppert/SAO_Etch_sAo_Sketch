# Etch sAo Sketch

The perfect pairing of an OLED Display, analog potentiometers, and a 3-axis accelerometer!

This is the primary repository for all things relevant to the Etch sAo Sketch.

Learn more about this project at: https://hackaday.io/project/197581-etch-sao-sketch

If you use the Arduino Demo code make sure it is configured for your hardware version (HWV) of the Etch sAo Sketch.

IMPORTANT ERRATA APPLIES TO ALL VERSIONS:

There is too much combined pull-up resistance with this board, the OLED board, the Hackaday Badge, and the Pico. This inhibits I2C hardware implementations from working with the OLED display.

SOLUTION A: Firmware workaround is to use software I2C.

   See: https://github.com/ageppert/SAO_Etch_sAo_Sketch/pull/1

SOLUTION B: Hardware mods to remove interfering components.

   See: https://github.com/ageppert/SAO_Etch_sAo_Sketch/issues/8


-------------

# V1.3 - OLED VERSION - Hackaday Europe 2025 Elecrow version

<img src="Images/Etch_sAo_Sketch V1.3 Elecrow.png" height="300">

This version was manufactured for Hackaday Europe 2025 by Elecrow and includes the Elecrow logo on the PCB below the screen. The files have been copied into this repository for easy reference, although it was in a more generic "SAO Sketch" repo for production. Hopefully you got a sticker from one of us to upgrade your SAO to "Etch sAo Sketch" when we were together in Berlin!

-------------

# V1.2 - OLED VERSION - NOT BUILT

This version was an un-fabricated step in the development for Hackaday Europe 2025 with a separate repository located here: https://github.com/ageppert/SAO_Sketch. This version builds on V1.1 with some clean-up in the schematic and layout.

-------------

# V1.1 - OLED VERSION - Small batch to sell on Tindie

Refinements include:
1) Enable full range of the analog pots via the accelerometer's [reduced range] ADC inputs, with the option to still use analog signals on GPIO 1 and 2.
2) Use a taller male SAO header for easier connection to the main (red) PCBA.
3) Tweak alignment hole size/position for the alignment spheres and OLED position.

V1.1.1 FIXES: Removed I2C Pull-up Resistors R1/2 from the board.

Otherwise, it looks and functions just like V1.0.

-------------

# V1.0 - OLED VERSION - Shared at Hackaday Supercon.8 in 2024 and on Tindie

<img src="Images/Etch sAo Sketch Thumbnail crop center.jpeg" height="300">

This version was shared at Hackaday Supercon.8 2024. It is available for purchase here: https://www.tindie.com/products/36383/

V1.0.1 FIXES: Removed I2C Pull-up Resistors R30/31 from the board.

V1.0 ERRATA: The full travel of the analog pots is only available through [analog] GPIO1&2 of the SAO port, and this works best when connected to analog pins of an MCU. My SAO DEMO CONTROLLER is a good host to use the V1.0 Etch sAo Sketch with. A reduced range of travel is available through the accelerometer ADC inputs via I2C. The included SAO Header pins need to be pushed back into the header body 1 mm so they extend far enough to be connected to the red PCBA after the OLED PCBA is stacked on the bottom.

The SAO DEMO CONTROLLER is available for purchase here: https://www.tindie.com/products/36033/

