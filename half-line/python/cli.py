import time
from config import *
from serial import serial
from sensors import sensors
from dragster import dragster_run, dragster_track_calibrate

# ------------------------------------------------------------------
# CLI Configuration
# ------------------------------------------------------------------

MAX_ARGC = 16
MAX_DIGITS = 8

# ------------------------------------------------------------------
# Utility functions
# ------------------------------------------------------------------

def read_integer(text):
    """
    Returns (success, value)
    """
    if text is None:
        return False, 0

    try:
        return True, int(text)
    except:
        return False, 0


def read_float(text):
    """
    Returns (success, value)
    """
    if text is None:
        return False, 0.0

    try:
        return True, float(text)
    except:
        return False, 0.0


# ------------------------------------------------------------------
# Args
# ------------------------------------------------------------------

class Args:

    def __init__(self):
        self.argv = []
        self.argc = 0

    def print(self):
        serial.println(" ".join(self.argv))


# ------------------------------------------------------------------
# Command Line Interface
# ------------------------------------------------------------------

class CommandLineInterface:

    # --------------------------------------------------------------
    # Parse and run command
    # --------------------------------------------------------------

    def interpret_line(self, line):

        args = self.get_tokens(line)

        if args.argc > 0:

            if len(args.argv[0]) == 1:
                self.run_short_cmd(args)
            else:
                self.run_long_cmd(args)

        self.prompt()

    # --------------------------------------------------------------
    # Tokeniser
    # --------------------------------------------------------------

    def get_tokens(self, line):

        args = Args()

        line = line.replace(',', ' ')
        line = line.replace('=', ' ')

        tokens = [t for t in line.split() if t]

        args.argv = tokens[:MAX_ARGC]
        args.argc = len(args.argv)

        return args

    # --------------------------------------------------------------
    # Long commands
    # --------------------------------------------------------------

    def run_long_cmd(self, args):

        cmd = args.argv[0]

        if cmd == "HELP":
            self.help()

        elif cmd == "SEARCH":

            x = 7
            y = 7

            if args.argc > 1:
                ok, value = read_integer(args.argv[1])
                if ok:
                    x = value

            if args.argc > 2:
                ok, value = read_integer(args.argv[2])
                if ok:
                    y = value

            serial.println(
                "SEARCH target ({},{})".format(x, y)
            )

            # mouse.search_to(Location(x, y))

    # --------------------------------------------------------------
    # Single character commands
    # --------------------------------------------------------------

    def run_short_cmd(self, args):

        cmd = args.argv[0]
        
        # Run the letter command
        if cmd in CLI_SHORT_COMMANDS:
            serial.println(CLI_SHORT_COMMANDS[cmd][0])
            # Special cases
            if cmd == 'F':
                if args.argc > 1:
                    ok, function = read_integer(args.argv[1])
                    if ok:
                        CLI_SHORT_COMMANDS[cmd][1](function)
            else:
                CLI_SHORT_COMMANDS[cmd][1]()
        else:
            serial.println("Unknown command")
    '''        
        if c == '?':
            self.help()

        elif c == 'X':
            serial.println("Reset Maze")
            # maze.initialise()

        elif c == 'W':
            # reporter.print_maze(PLAIN)
            serial.println("Display maze walls")

        elif c == 'C':
            # reporter.print_maze(COSTS)
            serial.println("Display maze costs")

        elif c == 'D':
            # reporter.print_maze(DIRS)
            serial.println("Display maze directions")

        elif c == 'B':
            # voltage = battery.voltage()
            voltage = 4.20
            serial.println("Battery: {:.2f} Volts".format(voltage))

        elif c == 'S':
            # sensors.enable()
            # reporter.print_wall_sensors()
            # sensors.disable()
            serial.println("Sensor readings")
            
        elif c == 'F':

            if args.argc > 1:

                ok, function = read_integer(args.argv[1])

                if ok:
                    self.run_function(function)
    '''
    # --------------------------------------------------------------
    # User functions
    # --------------------------------------------------------------

    def run_function(self, func):
        
        if func == 0:
            return

        # Run the numbered funtion
        if func in CLI_FUNCTIONS:
            serial.println(CLI_FUNCTIONS[func][0])
            CLI_FUNCTIONS[func][1]()
        else:
            serial.println("Unknown function")

    # --------------------------------------------------------------
    # Console helpers
    # --------------------------------------------------------------

    def prompt(self):

        serial.println()
        serial.print("> ")

    def help(self):

        serial.println()
        serial.println(MOUSE_NAME)
        serial.println(MOUSE_DESC)
        serial.println()
        for f in sorted(CLI_SHORT_COMMANDS):
            serial.println(f + "   : " + CLI_SHORT_COMMANDS[f][0])
        '''        serial.println("?   : this text")
        serial.println("X   : reset maze")
        serial.println("W   : display maze walls")
        serial.println("C   : display maze costs")
        serial.println("D   : display maze directions")
        serial.println("B   : show battery voltage")
        serial.println("S   : show sensor readings")
        serial.println("F n : Run user function n")
        '''
        serial.println("Functions:")
        for f in sorted(CLI_FUNCTIONS):
            serial.println("F " + str(f) + " : " + CLI_FUNCTIONS[f][0])
            '''        serial.println("SEARCH x y : search to location")
        serial.println("HELP       : this text")
        '''


# ------------------------------------------------------------------
# Global instance
# ------------------------------------------------------------------

cli = CommandLineInterface()

# ------------------------------------------------------------------
# CLI function definitions
# ------------------------------------------------------------------
CLI_SHORT_COMMANDS = {
        "?": ("This help text", cli.help),
        "S": ("Test sensors", sensors.test_sensors),
        "F": ("Run user function n", cli.run_function)
        }
CLI_FUNCTIONS = {
        1: ("Dragster Run", dragster_run),
        2: ("Calibrate drag track", dragster_track_calibrate)
        }
