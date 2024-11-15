from machine import I2C, Pin
import time

counter = 0

## do a quick spiral to test
if petal_bus:
    for j in range(8):
        which_leds = (1 << (j+1)) - 1 
        for i in range(1,9):
            petal_bus.writeto_mem(PETAL_ADDRESS, i, bytes([which_leds]))
            time.sleep_ms(30)
            petal_bus.writeto_mem(PETAL_ADDRESS, i, bytes([which_leds]))

if etch_sao_sketch_device:
    etch_sao_sketch_device.shake() # clear display

while True:

    ## display button status on RGB
    if petal_bus:
        if not buttonA.value():
            petal_bus.writeto_mem(PETAL_ADDRESS, 2, bytes([0x80]))
        else:
            petal_bus.writeto_mem(PETAL_ADDRESS, 2, bytes([0x00]))

        if not buttonB.value():
            petal_bus.writeto_mem(PETAL_ADDRESS, 3, bytes([0x80]))
        else:
            petal_bus.writeto_mem(PETAL_ADDRESS, 3, bytes([0x00]))

        if not buttonC.value():
            petal_bus.writeto_mem(PETAL_ADDRESS, 4, bytes([0x80]))
        else:
            petal_bus.writeto_mem(PETAL_ADDRESS, 4, bytes([0x00]))

    ## see what's going on with the touch wheel
    if touchwheel_bus:
        tw = touchwheel_read(touchwheel_bus)

    ## display touchwheel on petal
    if petal_bus and touchwheel_bus:
        if tw > 0:
            tw = (128 - tw) % 256 
            petal = int(tw/32) + 1
        else: 
            petal = 999
        for i in range(1,9):
            if i == petal:
                petal_bus.writeto_mem(0, i, bytes([0x7F]))
            else:
                petal_bus.writeto_mem(0, i, bytes([0x00]))
    
    if etch_sao_sketch_device:
        etch_left = etch_sao_sketch_device.left
        etch_right = etch_sao_sketch_device.right
        print (etch_left, etch_right)
        etch_sao_sketch_device.draw_pixel(etch_left, etch_right, 1)
        etch_sao_sketch_device.draw_display()
    
    time.sleep_ms(20)
    bootLED.off()
