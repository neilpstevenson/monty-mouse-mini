from time import ticks_ms, ticks_diff
from config import config
from encoders import encoders
from profile import forward_profile, rotation_profile
from motors import motors
from sensors import sensors, STEER_NORMAL, STEERING_OFF
from serial import serial
from cli import *

CLI_OK = 0

class MotorLab:

    def __init__(self):
        self.last_result = 0.0
        #self.init()

    def init(self):
        self.reset_drive()

    def reset_drive(self):
        motors.stop()
        encoders.reset()
        forward_profile.reset()
        rotation_profile.reset()
        motors.reset_controllers()

    def enable_drive(self):
        self.reset_drive()
        motors.enable_controllers()

    def disable_drive(self):
        self.reset_drive()
        motors.disable_controllers()

    def show_encoders(self, args):
        encoders.reset()
        serial.print("Encoders:\r\n")
        # TODO encoder count calibration

    def do_open_loop_trial(self, args):

        end_time = 2000
        volts = 3.0

        if args.argc > 1:
            volts = float(args.argv[1])

        if args.argc > 2:
            end_time = int(args.argv[2])

        serial.print(
            "#Open Loop Identification - {:.1f} Volts\r\n".format(volts)
        )
        serial.print("$time(ms) Volts(V) Speed(deg/s)\r\n")

        encoders.reset()

        motors.disable_controllers()
        motors.set_closed_loop(False)

        motors.set_left_motor_volts(volts)

        start_time = ticks_ms()
        sample_time = start_time

        while ticks_diff(ticks_ms(), start_time) <= end_time:

            now = ticks_ms()

            if ticks_diff(now, sample_time) < 5:
                continue

            sample_time += 5

            elapsed = ticks_diff(now, start_time)

            serial.print(
                "{} {:.1f} {:.1f}\r\n".format(
                    elapsed,
                    motors.get_left_motor_volts(),
                    encoders.motor_lab_speed()
                )
            )

        motors.set_left_motor_volts(0)

        end_time += 200

        while ticks_diff(ticks_ms(), start_time) <= end_time:

            now = ticks_ms()

            if ticks_diff(now, sample_time) < 5:
                continue

            sample_time += 5

            elapsed = ticks_diff(now, start_time)

            serial.print(
                "{} {:.1f} {:.1f}\r\n".format(
                    elapsed,
                    motors.get_left_motor_volts(),
                    encoders.motor_lab_speed()
                )
            )

        motors.set_closed_loop(True)

    def report_controller_header(self):
        serial.println("$time set_pos robot_pos set_speed robot_speed ctrl_volts ff_volts motor_volts")
        self.start_time = ticks_ms()
        
    def report_controller(self):
        setPos = forward_profile.position();
        robot_pos = encoders.robot_distance()
        setSpeed = forward_profile.speed()
        robot_speed = encoders.robot_speed()
        ctrl_volts = motors.get_position_control_output()
        ff_volts = motors.get_left_feed_forward()
        motor_volts = motors.get_left_motor_volts() #; //motors.get_rotation_control_output(); 
        elapsed = ticks_diff(ticks_ms(), self.start_time)
        serial.println("{} {} {} {} {} {} {} {} ".format(
            elapsed, setPos, robot_pos, setSpeed, robot_speed, ctrl_volts, ff_volts, motor_volts))

    def do_move_trial(self, args):

        mode = int(args.argv[1]) if args.argc > 1 else 0
        dist = float(args.argv[2]) if args.argc > 2 else 0
        top_speed = float(args.argv[3]) if args.argc > 3 else 0
        end_speed = float(args.argv[4]) if args.argc > 4 else 0
        accel = float(args.argv[5]) if args.argc > 5 else 0

        if dist == 0:
            dist = 1440

        if top_speed == 0:
            top_speed = 2000 #SEARCH_SPEED_DEFAULT

        if end_speed == 0:
            end_speed = 0

        if accel == 0:
            accel = 4000 # SEARCH_ACCEL_HALF_CELL(SEARCH_SPEED_DEFAULT)

        self.enable_drive()

        serial.print(
            "# {} {} {} {} {} {}\r\n".format(
                args.argv[0],
                mode,
                dist,
                top_speed,
                end_speed,
                accel
            )
        )

        if mode == 0:
            serial.print("# Full Control\r\n")
            motors.enable_feed_forward()
            motors.enable_controllers()

        elif mode == 1:
            serial.print("# No Feedforward\r\n")
            motors.disable_feed_forward()
            motors.enable_controllers()

        elif mode == 2:
            serial.print("# Only Feedforward\r\n")
            motors.enable_feed_forward()
            motors.disable_controllers()
            motors.enable_feed_forward_to_motors()

        self.report_controller_header()

        forward_profile.start(
            dist,
            top_speed,
            end_speed,
            accel
        )

        while not forward_profile.is_finished():
            self.report_controller()

        motors.stop()
        motors.disable_controllers()

        end_time = ticks_ms()

        while ticks_diff(ticks_ms(), end_time) < 200:
            self.report_controller()

        self.disable_drive()

        serial.print("#\r\n")

    def do_turn_trial(self, args):

        mode = int(args.argv[1])

        ang = float(args.argv[2])
        rot_speed = float(args.argv[3])
        rot_accel = float(args.argv[4])
        fwd_speed = float(args.argv[5])
        fwd_accel = float(args.argv[6])
        lead_in = float(args.argv[7])
        lead_out = float(args.argv[8])

        if ang == 0:
            ang = 90

        if rot_speed == 0:
            rot_speed = OMEGA_SPIN_TURN_DEFAULT

        if rot_accel == 0:
            rot_accel = ALPHA_SPIN_TURN_DEFAULT

        if fwd_speed == 0:
            fwd_speed = SEARCH_SPEED_DEFAULT

        if fwd_accel == 0:
            fwd_accel = SEARCH_ACCEL_HALF_CELL(fwd_speed)

        if lead_in == 0:
            lead_in = 100

        if lead_out == 0:
            lead_out = 100

        self.enable_drive()

        if mode == 0:
            serial.print("# Full Control\r\n")
            motors.enable_feed_forward()
            motors.enable_controllers()

        elif mode == 1:
            serial.print("# No Feedforward\r\n")
            motors.disable_feed_forward()
            motors.enable_controllers()

        elif mode == 2:
            serial.print("# Only Feedforward\r\n")
            motors.enable_feed_forward()
            motors.disable_controllers()

        reporter.report_rotation_controller_header()

        forward_profile.start(
            lead_in,
            fwd_speed,
            fwd_speed,
            fwd_accel
        )

        while not forward_profile.is_finished():
            reporter.report_rotation_controller(
                forward,
                rotation
            )

        rotation_profile.start(
            ang,
            rot_speed,
            0,
            rot_accel
        )

        while not rotation_profile.is_finished():
            reporter.report_rotation_controller(
                forward,
                rotation
            )

        forward_profile.start(
            lead_out,
            fwd_speed,
            0,
            fwd_accel
        )

        while not forward_profile.is_finished():
            reporter.report_rotation_controller(
                forward,
                rotation
            )

        motors.stop()
        motors.disable_controllers()

        end_time = ticks_ms()

        while ticks_diff(ticks_ms(), end_time) < 200:
            reporter.report_rotation_controller(
                forward,
                rotation
            )

        self.disable_drive()

        serial.print("#\r\n")

    def do_step_trial(self, args):

        dist = float(args.argv[1])

        if dist == 0:
            dist = 30

        serial.print("# Controller Only\r\n")

        self.enable_drive()

        motors.disable_feed_forward()
        motors.enable_controllers()

        start_time = ticks_ms()

        reporter.self.report_controller_header()

        while ticks_diff(ticks_ms(), start_time) < 100:
            self.report_controller()

        forward_profile.set_position(dist)

        while ticks_diff(ticks_ms(), start_time) < 600:
            self.report_controller()

        motors.set_left_motor_volts(0)

        end_time = ticks_ms()

        while ticks_diff(ticks_ms(), end_time) < 100:
            self.report_controller()

        self.disable_drive()

        serial.print("#\r\n")


motor_lab = MotorLab()

def cmd_set_get(name, min, max, args, dp = 2):
    if args.argc > 1:
        ok, var = read_float(args.argv[1])
        if var > max:
            var = max
        if var < min:
            var = min
        config.set_by_name(name, var)
    else:
        config.get_by_name(name)
    serial.print( "{} = {:f}".format(args.argv[0], round(config.get_by_name(name), dp)))

def send_id(args):
    serial.print("MOTORLAB V1.0\r\n")
    return CLI_OK


def init_settings(args):
    serial.print("NOT IMPLEMENTED!!\r\n")
    #settings.init(defaults)
    return CLI_OK


def print_settings(args):
    serial.println("    degPerCount = {:f}".format(round(config.get_by_name("degPerCount"), 5)))
    serial.println("             Km = {:f}".format(round(config.get_by_name("FWD_KM"), 2)))
    serial.println("             Tm = {:f}".format(round(config.get_by_name("FWD_TM"), 5)))
    serial.println("            rKm = {:f}".format(round(config.get_by_name("ROT_KM"), 2)))
    serial.println("            rTm = {:f}".format(round(config.get_by_name("ROT_TM"), 5)))
    serial.println("         biasFF = {:f}".format(round(config.get_by_name("BIAS_FF"), 5)))
    serial.println("        speedFF = {:f}".format(round(config.get_by_name("SPEED_FF"), 5)))
    serial.println("          accFF = {:f}".format(round(config.get_by_name("ACC_FF"), 5)))
    serial.println("           zeta = {:f}".format(round(config.get_by_name("FWD_ZETA"), 5)))
    serial.println("             Td = {:f}".format(round(config.get_by_name("FWD_TD"), 5)))
    serial.println("             KP = {:f}".format(round(config.get_by_name("FWD_KP"), 5)))
    serial.println("             KD = {:f}".format(round(config.get_by_name("FWD_KD"), 5)))
    serial.println("          rZeta = {:f}".format(round(config.get_by_name("ROT_ZETA"), 5)))
    serial.println("            rTd = {:f}".format(round(config.get_by_name("ROT_TD"), 5)))
    serial.println("            rKP = {:f}".format(round(config.get_by_name("ROT_KP"), 5)))
    serial.println("            rKD = {:f}".format(round(config.get_by_name("ROT_KD"), 5)))
    return CLI_OK


def set_get_km(args):
    cmd_set_get("FWD_KM", 0.0, 10000.0, args, 1)
    return CLI_OK

def set_get_tm(args):
    cmd_set_get("FWD_TM", 0.0, 10.0, args, 6)
    return CLI_OK


def set_get_rkm(args):
    cmd_set_get("ROT_KM", 0.0, 10000.0, args, 1)
    return CLI_OK


def set_get_rtm(args):
    cmd_set_get("ROT_TM", 0.0, 10.0, args, 6)
    return CLI_OK


def set_get_kp(args):
    cmd_set_get("FWD_KP", 0.0, 10.0, args, 6)
    return CLI_OK


def set_get_kd(args):
    cmd_set_get("FWD_KD", 0.0, 10.0, args, 6)
    return CLI_OK


def set_get_zeta(args):
    cmd_set_get("FWD_ZETA", 0.0, 10.0, args, 6)
    return CLI_OK


def set_get_td(args):
    cmd_set_get("FWD_TD", 0.0, 1.0, args, 6)
    return CLI_OK


def set_get_rkp(args):
    cmd_set_get("ROT_KP", 0.0, 10.0, args, 6)
    return CLI_OK


def set_get_rkd(args):
    cmd_set_get("ROT_KD", 0.0, 10.0, args, 6)
    return CLI_OK


def set_get_rzeta(args):
    cmd_set_get("ROT_ZETA", 0.0, 10.0, args, 6)
    return CLI_OK


def set_get_rtd(args):
    cmd_set_get("ROT_RD", 0.0, 1.0, args, 6)
    return CLI_OK


def set_get_bias_ff(args):
    cmd_set_get("BIAS_FF", 0.0, 10.0, args, 3)
    return CLI_OK


def set_get_speed_ff(args):
    cmd_set_get("SPEED_FF", 0.0, 10.0, args, 5)
    return CLI_OK


def set_get_acc_ff(args):
    cmd_set_get("ACC_FF", 0.0, 10.0, args, 6)
    return CLI_OK


def write_settings(args):
    settings.write()
    return CLI_OK


def read_settings(args):
    settings.read()
    return CLI_OK


def action(args):
    serial.print("{}:{}\r\n".format(args.argc, args.argv[0]))
    return CLI_OK


def get_battery_volts(args):
    serial.print("{:.2f} Volts\r\n".format(battery.voltage()))
    return CLI_OK


def do_move(args):
    motor_lab.do_move_trial(args)
    return CLI_OK


def do_turn(args):
    motor_lab.do_turn_trial(args)
    return CLI_OK


def do_step(args):
    motor_lab.do_step_trial(args)
    return CLI_OK


def do_encoders(args):
    # TODO: encoder/gearbox calibration
    return CLI_OK


def do_open_loop(args):
    motor_lab.do_open_loop_trial(args)
    return CLI_OK

def add_motor_lab_cli_menus():
    cli.add_menu_item("*IDN?", "Request robot ID", send_id)
    cli.add_menu_item("$",  "Display all Setting", print_settings)
    cli.add_menu_item("!",  "Write settings to EEPROM", write_settings)
    cli.add_menu_item("@",  "Read settings from EEPROM", read_settings)
    cli.add_menu_item("#",  "Initialise settings to defaults", init_settings)
    cli.add_menu_item("KM",  "Set/Get Km", set_get_km)
    cli.add_menu_item("TM",  "Set/Get Tm", set_get_tm)
    cli.add_menu_item("RKM",  "Set/Get Km", set_get_rkm)
    cli.add_menu_item("RTM",  "Set/Get Tm", set_get_rtm)
    cli.add_menu_item("KP",  "Set/Get Kp", set_get_kp)
    cli.add_menu_item("KD",  "Set/Get Kd", set_get_kd)
    cli.add_menu_item("ZETA",  "Set/Get Damping Ratio, zeta", set_get_zeta)
    cli.add_menu_item("RTD",  "Set/Get Rot settling time, rTd", set_get_rtd)
    cli.add_menu_item("RKP",  "Set/Get Rot rKp", set_get_rkp)
    cli.add_menu_item("RKD",  "Set/Get Rot rKd", set_get_rkd)
    cli.add_menu_item("RZETA",  "Set/Get Rot Damping Ratio, rzeta", set_get_rzeta)
    cli.add_menu_item("TD",  "Set/Get settling time, Td", set_get_td)
    cli.add_menu_item("BIASFF",  "Set/Get bias feed forward", set_get_bias_ff)
    cli.add_menu_item("SPEEDFF",  "Set/Get speed feedforward", set_get_speed_ff)
    cli.add_menu_item("ACCFF",  "Set/Get accel feedforward", set_get_acc_ff)
    cli.add_menu_item("BATT",  "Get battery Voltage", get_battery_volts)
    cli.add_menu_item("MOVE",  "Execute move profile", do_move)
    cli.add_menu_item("TURN",  "Execute turn profile", do_turn)
    cli.add_menu_item("STEP",  "Execute single step", do_step)
    cli.add_menu_item("ENC",  "Set/Get KM",do_encoders)
    cli.add_menu_item("VOLTS",  "Execute open loop", do_open_loop)
