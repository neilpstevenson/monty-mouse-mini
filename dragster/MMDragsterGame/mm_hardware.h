#define MOTORCTRL_DRV8833
#define STEERINGCTRL_PCB
#define HAS_ENCODERS
//#define MOTORCTRL_TB6612
//#define STEERINGCTRL_PROTO

// the number of the motor pins
#ifdef MOTORCTRL_TB6612
const int gpioMotorA1 = 17;
const int gpioMotorA2 = 2;
const int gpioMotorAPwm = 15;
const int gpioMotorB1 = 12;
const int gpioMotorB2 = 13;
const int gpioMotorBPwm = 27;
#endif //MOTORCTRL_TB6612
#ifdef MOTORCTRL_DRV8833
const int gpioMotorA1 = 17;
const int gpioMotorA2 = 2;
const int gpioMotorB1 = 13;
const int gpioMotorB2 = 12;
#endif //MOTORCTRL_DRV8833

#ifdef HAS_ENCODERS
// Encoders
const int gpioMotorEncoderRA = 26;
const int gpioMotorEncoderRB = 27;
const int gpioMotorEncoderLA = 38;
const int gpioMotorEncoderLB = 37;

const int encodeCountsPerRev = 12; // 12 in full quad, 6 in half-quad mode
const float wheelDiameter = 37.0;
const float encoderDistanceCalibration = wheelDiameter * 3.14159 / encodeCountsPerRev; // approx 9.7mm/count;
#endif // HAS_ENCODERS

// setting PWM properties
#ifdef MOTORCTRL_TB6612
const int motorRPwmChannel = 0;
const int motorLPwmChannel = 1;
#endif //MOTORCTRL_TB6612
#ifdef MOTORCTRL_DRV8833
const int motorA1PwmChannel = 0;
const int motorA2PwmChannel = 1;
const int motorB1PwmChannel = 2;
const int motorB2PwmChannel = 3;
#endif //MOTORCTRL_DRV8833

const int pwmFreq = 20000;
const int pwmResolution = 8; //bits

// TTGO Display buttons
const int gpioSelectButton = 35;
const int gpioEnterButton = 0;

// Connected Servos
#ifdef STEERINGCTRL_PROTO
const int gpioSteeringServo = 26;
#endif //STEERINGCTRL_PROTO
#ifdef STEERINGCTRL_PCB
const int gpioSteeringServo = 15;
#endif //STEERINGCTRL_PCB

const int default_steeringServoCentre = 1500; // Nominally 1500 = mid range of movement (bigger = steer more left)
const int steeringServoFreq = 333;  // Servo refresh rate (50Hz to about 300Hz for most digial servos)

// Sensors
const int gpioSensorStartFinish = A5; // Rightmost sensor
const int gpioSensorRightLine = A4;
const int gpioSensorLeftLine = A3;
const int gpioSensorRadius = A0;  // Leftmost sensor

// LEDs
const int gpioIlluminationLED = 25;

// PID values
const float LOOP_INTERVAL = 0.003;  // 1mS
const float LOOP_FREQUENCY = 1/LOOP_INTERVAL;  // Hz
const float MAX_MOTOR_VOLTS = 300.0;
// Defaults (actual from NVRam
const float PID_Kp = 0.03; //0.005; //0.01;// 0.007;
const float PID_Ki = 0.0;
const float PID_Kd = 0.005; //0.002;
