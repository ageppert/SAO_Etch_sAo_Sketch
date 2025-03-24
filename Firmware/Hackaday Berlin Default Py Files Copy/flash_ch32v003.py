import time
import rp2
import machine
import singlewire_pio
import gc
import sys

from machine import Pin
from constants import *

## To test
'''
from flash_ch32v003 import CH32_Flash
flasher = CH32_Flash(1)
flasher.flash_binary("blink.bin")
'''
# or
'''
from flash_ch32v003 import CH32_Flash
flasher = CH32_Flash(1)
flasher.flash_binary("cap_touch_adc.bin")
flasher.monitor()
'''



class CH32_Flash():
    def __init__(self, pin_number):

        ports = [None, 7, 5, 3, 22, 20, 18]

        self.prog_pin = Pin(ports[pin_number], mode=Pin.IN, pull=Pin.PULL_UP)
        self.swio_sm = rp2.StateMachine(1, singlewire_pio.singlewire_pio, freq=12_000_000,
                                        sideset_base=self.prog_pin, out_base=self.prog_pin, 
                                            set_base=self.prog_pin, in_base=self.prog_pin)
        ## Some unavoidable bit-twiddling
        ## Set side-set to control pindirs state machine
        machine.mem32[PIO0_BASE + SM1_EXECCTRL] |= (1 << SIDE_PINDIR)

        ## What mode of programming we are currently engaged in.
        self.progmode = 0
        self.progptr = 0
        self.inter_packet_delay = 60 

    def read_address(self, register):
        return(register << 1)
    def write_address(self, register):
        return((register << 1) + 1)

    def send_write(self, address, data):
        self.swio_sm.put(self.write_address(address))
        self.swio_sm.put(data)
        self.swio_sm.active(1)
        time.sleep_us(self.inter_packet_delay)
        self.swio_sm.active(0)

    def send_read(self, address):
        self.swio_sm.put(self.read_address(address))
        self.swio_sm.active(1)
        time.sleep_us(self.inter_packet_delay)
        self.swio_sm.active(0)
        return(self.swio_sm.get())

    def b32(num):
        aa = (num & 0xFF000000) >> 24 
        bb= (num & 0x00FF0000) >> 16
        cc= (num & 0x0000FF00) >> 8
        dd= num & 0x000000FF
        print(f'{aa:#010b} {bb:#010b} {cc:#010b} {dd:#010b}')

    def enter_debug_mode(self):
        self.send_write(WCH_DM_SHDWCFGR, OUTSTA)
        self.send_write(WCH_DM_CFGR, OUTSTA) ## OUTSTA: 0: The debug slave has output function.
        ## guarantee halt
        self.send_write(DM_CTRL, 0x80000001) ## 1: Debug module works properly 
        self.send_write(DM_CTRL, 0x80000003) ## 1: Debug module works properly 
        self.send_write(DM_CTRL, 0x80000001) ## 1: halt
        self.send_write(DM_CTRL, 0x80000001) ## 1: halt
        self.send_write(DM_CTRL, 0x80000001) ## 1: halt

        ## Setup several registers for usage later.
        self.send_write( DMDATA0, 0x40022010 )  #;   // FLASH->CTLR
        self.send_write( DMCOMMAND, 0x0023100c )  #; // Copy data to x12
        self.send_write( DMDATA0, CR_PAGE_PG | 0x00040000 )  #;   // FLASH_CTLR_BUF_LOAD = 0x00040000
        self.send_write( DMCOMMAND, 0x0023100d )  #; // Copy data to x13
        self.send_write( DMDATA0, 0xe00000f4 )  #;   // DATA0's location in memory.
        self.send_write( DMCOMMAND, 0x00231008 )  #; // Copy data to x8
        self.send_write( DMDATA0, 0xe00000f8 )  #;   // DATA1's location in memory.
        self.send_write( DMCOMMAND, 0x00231009 )  #; // Copy address to x9
        self.send_write( DMABSTRACTCS, 7<<8 )   # // Clear out any abstract command execution errors.

        self.send_write( DMABSTRACTAUTO, 0x00000000 )  #; // Disable autoexec on DATA0
        # We don't need to autoexec, if we don't want to, we can manually exec with the following:
        #send_write( DMCOMMAND, ((1 << 18) | (1<<21)) )

        self.progmode = 0
        self.progptr = 0


    def print_status_capabilities(self):
        status = self.send_read(DM_STATUS)
        print("status")
        self.b32(status)
        capabilities = self.send_read(WCH_DM_CPBR)
        print("capabilities")
        self.b32(capabilities)

    ## OK, now flash in the bootloader

    def wait_for_done(self):
        is_busy = True
        while is_busy:
            abstract_control_status = self.send_read(DMABSTRACTCS)
            if (abstract_control_status & (7<<8)):
                raise Exception(f"error, bail out: abs_ctrl {hex(abstract_control_status)}")
            if (abstract_control_status & (1 << 12)) == 0:  ## abstract command busy bit
                is_busy = False

    def write_word(self, address, data):
        if self.progmode != 1:
            self.send_write( DMPROGBUF0, 0x408c_4008 )  #   c.lw x11, 0(x9) <-  c.lw x10, 0(x8)
            self.send_write( DMPROGBUF1, 0x9002_c188 )  #   c.ebreak    <- c.sw x10, 0(x11)
            self.send_write( DMABSTRACTAUTO, 0x00000001 )  #; // Enable autoexec on DATA0
            self.progmode = 1
        self.send_write( DMDATA1, address )  #; Write address we want to write
        self.send_write( DMDATA0, data )  #; This will autoexec
        self.wait_for_done()

    def write_word_flash(self, address, data):
        if address != self.progptr or self.progmode != 3:
            self.send_write( DMDATA1, address )  #; Only need to load address in once, will autoincrement
            self.progptr = address

        if self.progmode != 3:
            # Assume x9 contains the address we want to write to.
            self.send_write( DMPROGBUF0, 0x408c_4008 )  #   c.lw x11, 0(x9) <-  c.lw x10, 0(x8)
            self.send_write( DMPROGBUF1, 0x0591_c188 )  #   c.addi x11, 4 <- c.sw x10, 0(x11)
            self.send_write( DMPROGBUF2, 0xc08c_c214 )  #   c.sw x11, 0(x9) <- c.sw x13, 0(x12)
            self.send_write( DMPROGBUF3, 0x9002_9002 )  #   c.ebreak <- c.ebreak
            self.send_write( DMABSTRACTAUTO, 0x00000001 )  #; Enable autoexec on DATA0
            self.progmode = 3

        self.send_write( DMDATA0, data ) #; this will autoexec the flash algo.
        self.progptr += 4
        self.wait_for_done()

    def read_word(self, address):
        if self.progmode != 2:
            self.send_write( DMPROGBUF0, 0x4108_4008)   # c.lw x10, 0(x10) <-  c.lw x10, 0(x8)
            self.send_write( DMPROGBUF1, 0x9002_c088)   # break     <- sw x10, 0(x9) 
            self.send_write( DMABSTRACTAUTO, 0x00000000 )  #; // Disable autoexec (Seems to cause an issue for reads)
            self.progmode = 2
        self.send_write( DMDATA0, address )  # ; Program will autoexec
        self.send_write( DMCOMMAND, ((1 << 18) | (1<<21)) )
        self.wait_for_done()
        data = self.send_read(DMDATA1)
        return(data)

    def wait_for_flash(self):
        ## add timeout
        for i in range(200):
            rw = self.read_word( 0x4002200C )  # FLASH_STATR => 0x4002200C
            if not (rw & 1):   # BSY flag 
                break
        self.write_word( 0x4002200C, 0 )  #;
        if( rw & FLASH_STATR_WRPRTERR ):
            raise Exception("Misc Flash error")

        if( rw & 1 ):
            raise Exception("Flash timed out")

    def unlock_flash(self):
        rw = self.read_word( 0x40022010 )  # FLASH->CTLR = 0x40022010
        if( rw & 0x8080 ):
            self.write_word( 0x40022004, 0x45670123 )  #; // FLASH->KEYR = 0x40022004
            self.write_word( 0x40022004, 0xCDEF89AB )  #;
            self.write_word( 0x40022008, 0x45670123 )  #; // OBKEYR = 0x40022008
            self.write_word( 0x40022008, 0xCDEF89AB )  #;
            self.write_word( 0x40022024, 0x45670123 )  #; // MODEKEYR = 0x40022024
            self.write_word( 0x40022024, 0xCDEF89AB )  #;
            rw = self.read_word( 0x40022010 )  # FLASH->CTLR = 0x40022010
            if( rw & 0x8080 ):
                raise Exception("flash could not be unlocked")

    def erase_chip(self):
        self.write_word( 0x40022010, 0 )  # FLASH->CTLR = 0x40022010
        self.write_word( 0x40022010, FLASH_CTLR_MER  )  #;
        self.write_word( 0x40022010, CR_STRT_Set|FLASH_CTLR_MER )  #;
        self.wait_for_flash()
        self.write_word( 0x40022010, 0 )  #  FLASH->CTLR = 0x40022010

    def simple_64_byte_write(self, start_address, data):
        """start_address = MUST_BE_64_BYTE_ALIGNED;"""

        self.write_word( 0x40022010, CR_PAGE_PG )  ##   // (intptr_t)&FLASH->CTLR = 0x40022010  
        self.write_word( 0x40022010, CR_BUF_RST | CR_PAGE_PG );  
        self.write_word( 0x40022014, start_address )  # ;
        self.wait_for_flash()
        for i in range(16): 
            addr = start_address+(i*4)
            value = int.from_bytes(data[(i*4):(i*4)+4], "little")
            #print(hex(addr), hex(value))
            #write_word( addr, value )
            #write_word( 0x40022010, CR_PAGE_PG | 0x00040000 ); #; // FLASH_CTLR_BUF_LOAD = 0x00040000   
            self.write_word_flash( addr, value )

        self.write_word( 0x40022010, CR_PAGE_PG|CR_STRT_Set ) #;  // R32_FLASH_CTLR
        self.wait_for_flash()

    ## reset and resume
    def reset_and_resume(self):
        self.send_write( DM_CTRL, 0x80000001)  
        self.send_write( DM_CTRL, 0x80000001) 
        self.send_write( DM_CTRL, 0x00000001)  
        self.send_write( DM_CTRL, 0x40000001)

    def displaychar(self, ascii):
        sys.stdout.write( chr(ascii) );

    def monitor(self):
        self.send_write( WCH_DM_SHDWCFGR, OUTSTA )
        self.send_write( WCH_DM_CFGR, OUTSTA ) ## OUTSTA: 0: The debug slave has output function.
        self.send_write( DM_CTRL, 0x80000001 )
        self.send_write( DM_CTRL, 0x80000003 )
        self.send_write( WCH_DM_SHDWCFGR, OUTSTA) # Try again, just in case.
        self.send_write( WCH_DM_CFGR, OUTSTA )
        self.send_write( DM_CTRL, 0x80000001 )
        self.send_write( DM_CTRL, 0x80000003 )
        self.send_write( DM_CTRL, 0x00000001 )
        self.send_write( DM_CTRL, 0x40000001 )

        while True:
            d0 = self.send_read( DMDATA0 )
            statusword = (d0 & 0xff);
            if statusword & 0x80:
                swb = (statusword&0x7f);
                if swb > 4:
                    to_read = swb - 4;
                    d1 = 0
                    if to_read > 3:
                        d1 = self.send_read( DMDATA1 )
                    if to_read > 0:
                        self.displaychar( ( ( d0 >> 8 ) & 0xff) )
                    if to_read > 1:
                        self.displaychar( ( ( d0 >> 16 ) & 0xff) );
                    if to_read > 2:
                        self.displaychar( ( ( d0 >> 24 ) & 0xff) );
                    if to_read > 3:
                        self.displaychar( ( ( d1 >> 0 ) & 0xff) );
                    if to_read > 4:
                        self.displaychar( ( ( d1 >> 8 ) & 0xff) );
                    if to_read > 5:
                        self.displaychar( ( ( d1 >> 16 ) & 0xff) );
                    if to_read > 6:
                        self.displaychar( ( ( d1 >> 24 ) & 0xff) );
                self.send_write( DMDATA0, 0 )
                gc.collect()

    def flash_binary(self, filename):
        self.enter_debug_mode()
        self.enter_debug_mode()
        flash_start_time = time.ticks_us()
        self.unlock_flash()
        self.erase_chip()

        data = open(filename, "b").read()
        
        for i in range(len(data) // 64): 
            byte_block = data[i*64:(i+1)*64]
            self.simple_64_byte_write(0x0800_0000 + i*64, byte_block)
        if len(data) % 64:
            residual = bytearray([0xff] * 64) 
            for j, this_byte in enumerate(data[(i+1)*64:]):
                residual[j] = this_byte
            self.simple_64_byte_write(0x0800_0000 + (i+1)*64, residual)

        print( "Time spent: " + str(time.ticks_us() - flash_start_time) )

        for i in range(4):
            print(hex(i) + " " + hex(self.read_word(0x0800_0000+i*4)))

        self.reset_and_resume()


## Connect the ch32v003 board to Port 1 
if __name__ == "main":
    flasher = CH32_Flash(1)
    flasher.flash_binary("blink2.bin")

