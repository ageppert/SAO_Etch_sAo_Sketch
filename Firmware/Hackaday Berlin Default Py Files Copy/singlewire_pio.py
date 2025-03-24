import rp2

## Implement the WH32 single-wire protocol in a PIO

## Normal mode:
## 1    = low 1T to  4T, high 1T to 16T
## 0    = low 6T to 64T, high 1T to 16T
## Stop = high 18T
## If T = 125 ns
## Normal mode:
## 1    = low 125ns to  500ns, high 125ns to 2000ns
## 0    = low 750ns to 8000ns, high 125ns to 2000ns
## Stop = high 2250 ns

# ## glitch replaces
# nop() .side(0)[2] # rising

# with

# set(pins, 1).side(1)[0] ## glitch up
# set(pins, 0).side(0)[0] ## undo glitch, and go high-z
# nop().side(0)[0]

## when rising but needs to be able to be pulled down

## when normal digital logic, can use 
# set(pins, 1).side(1)[0]
# set(pins, 0).side(1)[0]


#@rp2.asm_pio(sideset_init=rp2.PIO.OUT_HIGH)  
@rp2.asm_pio(set_init=rp2.PIO.OUT_HIGH, out_init=rp2.PIO.OUT_HIGH, sideset_init=rp2.PIO.OUT_HIGH, autopush=True)
def singlewire_pio():

    wrap_target()
    label("start")

    set(pins, 1)               .side(1)[0]
    pull()                     .side(1)[1]  #  Pull the address from the fifo and let the bus pull high for 300 ns
    set(pins, 0)               .side(1)[0]
    out(y, 24)                 .side(1)[0]  #  Move high 24 bits of the address to y and send the start bit
    set(pins, 1)               .side(1)[2]
    set(y, 0)                  .side(1)[0]   ## use this to compare to x to make an "x is 1" test

    label("addr_loop")
    set(pins, 0)               .side(1)[0]
    out(x, 1)                  .side(1)[0]  # short pulse 200 ns
    jmp(x_not_y, "addr_zero")  .side(1)[0]
    nop()                      .side(1)[7]  # long pulse 800ns

    label("addr_zero")
    
    set(pins, 1)               .side(1)[0]
    jmp(not_osre, "addr_loop") .side(1)[3]  # end bit, pull up 300 ns
    jmp(x_not_y, "op_write")   .side(1)[0]  # Read/write bit
    
    label("op_read")
    set(pins, 1)               .side(1)[0]   ## pull up
    set(x, 31)                 .side(1)[0]

    label("read_loop")     # loop time ~1100 ns: 11 clocks
    set(pins,0)                .side(1)[0]  #  000 ns - Start pulse. Target will drive pin low starting immediately and continue for ~800 ns to signal 0.
    set(pins, 1)               .side(1)[0] ## glitch up
    set(pins, 0)               .side(0)[8] ## undo glitch, and go high-z
    in_(pins, 1)               .side(0)[0]  #  500 ns - Read pin and then wait for target to release it.
    set(pins, 1)               .side(1)[0] ## glitch up
    jmp(x_dec, "read_loop")    .side(0)[1]  #  800 ns - Pin should be going high by now.

    set(pins, 1)               .side(1)[6]  ## stop bit?
    jmp("start")               .side(1)[10]
    
    label("op_write")  ## state should still be driven high
    pull()                     .side(1)[1]

    label("write_loop")
    set(pins,0)                .side(1)[0] # start bit?
    out(x,1)                   .side(1)[0]
    jmp(x_not_y, "data_zero")  .side(1)[0]
    nop()                      .side(1)[7] # length of long pulse

    label("data_zero")
    set(pins,1)                .side(1)[2]
    jmp(not_osre, "write_loop").side(1)[2] # end bit and pull up for 300 ns
    set(pins, 1)               .side(1)[6]  ## stop bit
    jmp("start")               .side(1)[10]
    
    wrap()




