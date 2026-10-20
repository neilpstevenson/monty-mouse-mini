from machine import Pin, SPI, PWM
import time
import struct
from framebuf import FrameBuffer, RGB565

def print_hex(prefix, data):
    """
    Prints bytes/bytearray/list of integers as hex values.
    Works in MicroPython and CPython.
    """
    if isinstance(data, (bytes, bytearray)):
        # Convert to hex string
        print(prefix, data.hex())
    elif isinstance(data, int):
        # Convert to hex string
        print(prefix, '{:02x}'.format(data))
    elif isinstance(data, list):
        # Ensure all elements are integers 0-255
        try:
            print(prefix, ''.join(f'{b:02X}' for b in data))
        except ValueError:
            print("List contains non-integer values.")
    else:
        print("Unsupported type:", type(data))

class LCD2IN25(FrameBuffer):
    # Panel dimensions
    WIDTH_PORTRAIT = 76
    HEIGHT_PORTRAIT = 284

    # Change these to suit your module
    X_OFFSET_PORTRAIT = 82
    Y_OFFSET_PORTRAIT = 18

    PORTRAIT = 0
    LANDSCAPE = 1
    PORTRAIT_ROT = 2
    LANDSCAPE_ROT = 3

    def __init__(self,
                 spi_id=0,
                 sck=18,
                 mosi=19,
                 miso=16,
                 cs=17,
                 dc=20,
                 rst=21,
                 baudrate=40000000):

        self.spi = SPI(
            spi_id,
            baudrate=baudrate,
            polarity=0,
            phase=0,
            sck=Pin(sck),
            mosi=Pin(mosi),
            miso=None
        )

        self.cs = Pin(cs, Pin.OUT, value=1)
        self.dc = Pin(dc, Pin.OUT, value=1)
        #self.rst = Pin(rst, Pin.OUT, value=1)

        self.width = self.WIDTH_PORTRAIT
        self.height = self.HEIGHT_PORTRAIT
        self.xoffset = self.X_OFFSET_PORTRAIT
        self.yoffset = self.Y_OFFSET_PORTRAIT

        self.buffer = bytearray(self.height * self.width * 2)
        super().__init__(self.buffer, self.width, self.height, RGB565)
    # --------------------------------------------------
    # Low-level functions
    # --------------------------------------------------

    def reset(self):
        #self.rst.value(1)
        #time.sleep_ms(100)
        #self.rst.value(0)
        #time.sleep_ms(100)
        #self.rst.value(1)
        self.command(0x01)	#SWRESET
        time.sleep_ms(100)

    def command(self, cmd):
        #print_hex('C:', cmd)
        self.dc.value(0)
        self.cs.value(0)
        self.spi.write(bytes([cmd]))
        self.cs.value(1)

    def data(self, data):
        #print_hex('D:  ', data)
        self.dc.value(1)
        self.cs.value(0)

        if isinstance(data, int):
            self.spi.write(bytes([data]))
        else:
            self.spi.write(data)

        self.cs.value(1)

    def command_data_list(self, command_data):
        for cmd in command_data:
            self.command(cmd[0])
            if cmd[1] is not None:
                self.data(cmd[1])

    def data16(self, value):
        self.data(struct.pack(">H", value))

    # --------------------------------------------------
    # Orientation
    # --------------------------------------------------

    def set_attributes(self, scan_dir):

        if scan_dir == self.LANDSCAPE:
            self.width = self.HEIGHT_PORTRAIT
            self.height = self.WIDTH_PORTRAIT
            self.xoffset = self.Y_OFFSET_PORTRAIT
            self.yoffset = self.X_OFFSET_PORTRAIT
            super().__init__(self.buffer, self.width, self.height, RGB565)
            madctl = 0x60 #0xE0 #0xA0 if USB edge is left, 0x60 for opposite. 0x08 bit swaps R & B 
        elif scan_dir == self.LANDSCAPE_ROT:
            self.width = self.HEIGHT_PORTRAIT
            self.height = self.WIDTH_PORTRAIT
            self.xoffset = self.Y_OFFSET_PORTRAIT
            self.yoffset = self.X_OFFSET_PORTRAIT
            super().__init__(self.buffer, self.width, self.height, RGB565)
            madctl = 0xA0 #0xE0 #0xA0 if USB edge is left, 0x60 for opposite. 0x08 bit swaps R & B 
        elif scan_dir == self.PORTRAIT:
            self.width = self.WIDTH_PORTRAIT
            self.height = self.HEIGHT_PORTRAIT
            self.xoffset = self.X_OFFSET_PORTRAIT
            self.yoffset = self.Y_OFFSET_PORTRAIT
            super().__init__(self.buffer, self.width, self.height, RGB565)
            madctl = 0x00 # 0xc0 for top at USB end, 0x00 for opposite
        elif scan_dir == self.PORTRAIT_ROT:
            self.width = self.WIDTH_PORTRAIT
            self.height = self.HEIGHT_PORTRAIT
            self.xoffset = self.X_OFFSET_PORTRAIT
            self.yoffset = self.Y_OFFSET_PORTRAIT
            super().__init__(self.buffer, self.width, self.height, RGB565)
            madctl = 0xc0 # 0xc0 for top at USB end, 0x00 for opposite

        self.command(0x36)	#MADCTL
        self.data(madctl)

    # --------------------------------------------------
    # ST7789 Initialisation
    # --------------------------------------------------

    def init_registers(self):
        command_data = [
                (0xB2, bytes([0x0C, 0x0C, 0x00, 0x33, 0x33])),	#PORCTRL
                (0xB0, bytes([0x00, 0xE8])),	#RAMCTRL : xx x8 = Little Endian used in FrameBuffer
                (0x3A, 0x05),	#COLMOD - RGB565
                (0xB7, 0x45),	#GCTRL
                (0xBB, 0x1D),	#GTADJ
                (0xC0, 0x2C),	#LCMCTRL
                (0xC2, 0x01),	#VDVVRHEN
                (0xC3, 0x19),	#VRH
                (0xC4, 0x20),	#VDV
                (0xC6, 0x0F),	#FRCTRL2
                (0xD0, bytes([0xA4, 0xA1])),	#PWCTRL1
                (0xD6, 0xA1),	# ?
                (0xE0, bytes([
                    0xD0,0x10,0x21,0x14,
                    0x15,0x2D,0x41,0x44,
                    0x4F,0x28,0x0E,0x0C,
                    0x1D,0x1F])),	#PVGAMCTRL
                (0xE1, bytes([
                    0xD0,0x0F,0x1B,0x0D,
                    0x0D,0x26,0x42,0x54,
                    0x50,0x3E,0x1A,0x18,
                    0x22,0x25])),	#NVGAMCTRL
                ]
        
        self.command(0x11)	#SLPOUT
        time.sleep_ms(120)
        
        self.command_data_list(command_data)

    def init(self, scan_dir=PORTRAIT):
        self.backlight(0)	# Stops seeing white screen at power-up/reset
        self.reset()
        self.init_registers()
        self.set_attributes(scan_dir)
        self.display_on()
        self.clear(0)
        time.sleep_ms(100)	# Allow screen to be blanked

    def display_on(self):
        self.command(0x29)	#DISPON

    # --------------------------------------------------
    # Windowing
    # --------------------------------------------------

    def set_window(self, xs, ys, xe, ye):

        xs += self.xoffset
        xe += self.xoffset - 1

        ys += self.yoffset
        ye += self.yoffset - 1

        window_ctrl = [
                    (0x2A, bytes([
                        xs >> 8, xs & 0xFF, xe >> 8, xe & 0xFF])),	#CASET
                    (0x2B, bytes([
                        ys >> 8, ys & 0xFF, ye >> 8, ye & 0xFF])),	#RASET
                    (0x2C, None)									#RAMWR
                    ]
        self.command_data_list(window_ctrl)

    # --------------------------------------------------
    # Graphics
    # --------------------------------------------------

    def clear(self, color):
        self.fill(color)
        self.display()
        '''
        color_hi = (color >> 8) & 0xFF
        color_lo = color & 0xFF

        line = bytearray(self.width * 2)

        for i in range(self.width):
            idx = i * 2
            line[idx] = color_hi
            line[idx + 1] = color_lo

        self.set_window(
            0,
            0,
            self.width,
            self.height
        )

        self.dc.value(1)
        self.cs.value(0)

        for _ in range(self.height):
            #print_hex('X:', line)
            self.spi.write(line)

        self.cs.value(1)
        '''
        
    def pixel(self, x, y, color):

        self.set_window(
            x,
            y,
            x + 1,
            y + 1
        )

        self.data16(color)

    def display(self):
        self.set_window(0, 0, self.width, self.height)
        self.dc.value(1)
        self.cs.value(0)
        self.spi.write(self.buffer)
        self.cs.value(1)
        
    def backlight(self, value):#value:  min:0  max:1000
        pwm = PWM(Pin(23))#BL
        pwm.freq(1000)
        if value>=1000:
            value=1000
        data=int (value*65536/1000)       
        pwm.duty_u16(data)
        
    def RBG(r,g,b):
        # Actually byte-swapped RGB565 encoded
        rgb565 = (r << 11) + (g << 5) + b
        return rgb565
        #return ((rgb565 & 0xff) << 8) + (rgb565 >> 8)	# If BigEndian
        
lcd = LCD2IN25(
    spi_id=1,
    sck=30,#18,
    mosi=31,#19,
    cs=29,#17,
    dc=28,#20,
    rst=None, #10,#21
    baudrate=10_000_000
    )

# Clear framebuffer (black)
#fb.fill(0x0000)

#backlight = Pin(23, Pin.OUT, value=1)
#time.sleep(1)
#print('init')

#lcd.init(LCD2IN25.PORTRAIT)
#lcd.init(LCD2IN25.PORTRAIT_ROT)
lcd.init(LCD2IN25.LANDSCAPE)
#lcd.init(LCD2IN25.LANDSCAPE_ROT)

# Clear GRAM first
#lcd.clear(0x0000)
#time.sleep(5)
# Send to LCD
#lcd.display()

#time.sleep(1)
#print('on')

# Then turn panel output on
#lcd.display_on()

lcd.backlight(1000)

# Red pixel
#lcd.pixel(10, 10, 0xF800)

#time.sleep(0.2)

RED = LCD2IN25.RBG(31,0,0)
GREEN = LCD2IN25.RBG(0,63,0)
BLUE = LCD2IN25.RBG(0,0,31)
WHITE = LCD2IN25.RBG(31,63,31)

# Fill screen
#lcd.clear(RED)	#Red
#time.sleep(0.5)
#lcd.clear(GREEN)	#Green
#time.sleep(0.5)
#lcd.clear(BLUE)	#Blue
#time.sleep(0.5)

# Draw some text

start = time.ticks_ms()
last = start-1
while True:
    lcd.fill(0)
    # Draw text
    lcd.text("Hello!", 40, 12, WHITE)
    lcd.text("RP2350", 40, 28, RED)
    lcd.text("ST7789", 40, 44, GREEN)
    lcd.text(str(time.ticks_ms() - start), 200, 28, BLUE)
    now = time.ticks_ms()
    fps = 1000//(now - last)
    lcd.text(str(fps) + 'fps', 200, 44, BLUE)

    lcd.hline(5,5,265,GREEN)
    lcd.hline(5,67,265,GREEN)
    lcd.vline(5,5,62,GREEN)
    lcd.vline(270,5,62,GREEN)
  
    lcd.hline(0,0,275,BLUE)
    lcd.hline(0,72,275,BLUE)
    lcd.vline(0,0,72,BLUE)
    lcd.vline(275,0,72,BLUE) 

    # Send to LCD
    lcd.display()

    last = now
    #time.sleep(0.01)
    
time.sleep(1)
