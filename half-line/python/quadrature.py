#Very simple encoder class to return counts to the line follower diagnostic program
#Uses a quadrature encoder counter implemented in PIO from:
#https://github.com/raspberrypi/pico-examples/blob/master/pio/quadrature_encoder/quadrature_encoder.pio
#Encoder pin assignments and rotataion directions are hard coded in the class definition
#
from rp2 import PIO, StateMachine, asm_pio
from machine import Pin
import array
import utime
from hardware import *

class Quadrature:
    @asm_pio(autopush=False, autopull=False)
    
    def encoder():
        jmp("update")    # 00 -> 00
        jmp("decrement") # 00 -> 01
        jmp("increment") # 00 -> 10
        jmp("update")    # 00 -> 11
        
        jmp("increment") # 01 -> 00
        jmp("update")    # 01 -> 01
        jmp("update")    # 01 -> 10
        jmp("decrement") # 01 -> 11
        
        jmp("decrement") # 10 -> 00
        jmp("update")    # 10 -> 01
        jmp("update")    # 10 -> 10
        jmp("increment") # 10 -> 11
        
        jmp("update")    # 11 -> 00
        jmp("increment") # 11 -> 01
        label("decrement")#11 -> 10
        jmp(y_dec, "update")
        label("update")  # 11 -> 11
        
        wrap_target()
        set(x, 0)
        pull(noblock) # check for request
        
        mov(x, osr)
        mov(osr, isr) # OSR = previous pins
        jmp(not_x, "sample_pins")
        
        # report data
        mov(isr, y)
        push()
        
        label("sample_pins")
        mov(isr, null)
        in_(osr, 2)
        in_(pins, 2)
        mov(pc, isr) # jump to <previous>:<current>
        
        label("increment")
        mov(x, invert(y))
        jmp(x_dec, "increment2")
        label("increment2")
        mov(y, invert(x))
        wrap()
        nop() # We have to fill up the program space or Micropython won't put it at the
        nop() # beginning of the program, and the jump table doesn't work.
        nop()

    def begin(self, sm_number, pin_a, pin_b):
        self.sm = StateMachine(sm_number, Quadrature.encoder, freq=125_000_000, in_base=Pin(pin_a), jmp_pin=Pin(pin_b))
        self.sm.active(1)
        self.buf = array.array('i', [0])
        self.reset_count()

    def get_count(self):
        # Request a reading from the state machine
        self.sm.put(1)
        # For efficiency, here we use an undocumented feature of
        # StateMachine to read a SIGNED integer value without
        # allocating a new object.
        self.sm.get(self.buf)
        return self.buf[0]
    
    def delta_count(self):
        count = self.get_count()
        delta = count - self.last_count
        self.last_count = count
        return delta
    
    def reset_count(self):
        self.sm.exec("set(y, 0)")
        self.last_count = 0
