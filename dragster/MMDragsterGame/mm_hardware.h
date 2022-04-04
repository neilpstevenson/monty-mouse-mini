// the number of the motor pins
const int gpioMotorA1 = 17;
const int gpioMotorA2 = 2;
const int gpioMotorAPwm = 15;
const int gpioMotorB1 = 12;
const int gpioMotorB2 = 13;
const int gpioMotorBPwm = 27;

// setting PWM properties
const int pwmFreq = 20000;
const int motorRPwmChannel = 0;
const int motorLPwmChannel = 1;
const int pwmResolution = 8; //bits

// TTGO Display buttons
const int gpioSelectButton = 35;
const int gpioEnterButton = 0;

// Connected Servos
const int gpioSteeringServo = 26;
const int steeringServoCentre = 105; // Nominally 100 = mid range of movement (bigger = steer more left)
const int steeringServoFreq = 300;  // Servo refresh rate (50Hz to about 300Hz for most digial servos)

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
const float MAX_MOTOR_VOLTS = 100.0;
const float PID_Kp = 0.005;// 0.007;
const float PID_Ki = 0.0;
const float PID_Kd = 0.001;
