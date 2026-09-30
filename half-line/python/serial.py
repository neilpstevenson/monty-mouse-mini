# cli.py
from machine import UART
import time
from config import *

INPUT_BUFFER_SIZE = 32

# ------------------------------------------------------------------
# Serial handling class
# ------------------------------------------------------------------
class Serial:
    BACKSPACE = 0x08

    def __init__(self):
        self.uart = UART(0, baudrate=115200)
        self.m_buffer = ""
        self.m_index = 0
        
    def print(self, text="", end=""):
        self.uart.write(str(text))
        self.uart.write(end)

    def println(self, text=""):
        self.uart.write(str(text) + "\r\n")

    def read_line(self):
        while self.uart.any():

            c = self.uart.read(1)

            if not c:
                break

            c = c.decode("utf-8", "ignore")

            if c == '\r' or c == '\n':
                self.println()
                return True

            elif ord(c) == self.BACKSPACE:

                if self.m_index > 0:

                    self.m_index -= 1
                    self.m_buffer = self.m_buffer[:-1]

                    self.print(c)
                    self.print(' ')
                    self.print(c)

            elif 32 <= ord(c) <= 126:

                c = c.upper()

                self.print(c)

                if self.m_index < INPUT_BUFFER_SIZE - 1:
                    self.m_buffer += c
                    self.m_index += 1

        return False
    
    def get_read_line(self):
        line = self.m_buffer
        self.m_buffer = ""
        self.m_index = 0
        return line


# Single instance
serial = Serial()