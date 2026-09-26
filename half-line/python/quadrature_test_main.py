# SPDX-FileCopyrightText: 2022 Jamon Terrell <github@jamonterrell.com>
# SPDX-License-Identifier: MIT
import time
from rp2 import PIO, StateMachine, asm_pio
from machine import Pin
import utime
@asm_pio(autopush=True, push_thresh=32)
def encoder():
    label("start")
    wait(0, pin, 0)         # Wait for CLK to go low
    jmp(pin, "WAIT_HIGH")   # if Data is low
    mov(x, invert(x))           # Increment X
    jmp(x_dec, "nop1")
    label("nop1")
    mov(x, invert(x))
    label("WAIT_HIGH")      # else
    jmp(x_dec, "nop2")          # Decrement X
    label("nop2")
    
    wait(1, pin, 0)         # Wait for CLK to go high
    jmp(pin, "WAIT_LOW")    # if Data is low
    jmp(x_dec, "nop3")          # Decrement X
    label("nop3")
    
    label("WAIT_LOW")       # else
    mov(x, invert(x))           # Increment X
    jmp(x_dec, "nop4")
    label("nop4")
    mov(x, invert(x))
    wrap()

    
sm1 = StateMachine(1, encoder, freq=1_000_000, in_base=Pin(2), jmp_pin=Pin(3))
sm1.active(1)
sm2 = StateMachine(2, encoder, freq=1_000_000, in_base=Pin(5), jmp_pin=Pin(4))
sm2.active(2)

def to_signed32(val):
    """Convert unsigned 32-bit to signed 32-bit."""
    if val & 0x80000000:  # If sign bit is set
        return val - 0x100000000
    return val

# Clear
sm1.exec("set(x, 0)")
sm2.exec("set(x, 0)")

while(True):
    utime.sleep(0.2)
    
    tick_time = time.ticks_us()
    sm1.exec("in_(x, 32)")
    left = to_signed32(sm1.get())
    tick_time_elapsed = time.ticks_us() - tick_time
    
    sm2.exec("in_(x, 32)")
    right = to_signed32(sm2.get())

    print("@{}, {}, {}".format(tick_time_elapsed, left, right))
