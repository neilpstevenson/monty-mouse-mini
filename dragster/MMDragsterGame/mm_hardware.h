#include <Arduino.h>

#define MOTORCTRL_DRV8833
#define STEERINGCTRL_PCB
#define HAS_ENCODERS
#define INVERTED_SENSORS
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
const int gpioMotorA1 = 2;   // Left connector = Right Motor
const int gpioMotorA2 = 17;
const int gpioMotorB1 = 13;   // Right connector = Left Motor
const int gpioMotorB2 = 12;
#endif //MOTORCTRL_DRV8833

#ifdef HAS_ENCODERS
// Encoders
const int gpioMotorEncoderRA = 38;
const int gpioMotorEncoderRB = 37;
//const int gpioMotorEncoderLA = 38;
//const int gpioMotorEncoderLB = 37;

const int encodeCountsPerRev = 12; // 12 in full quad, 6 in half-quad mode
const float wheelDiameter = 26.5; //-32.09; //37.0;
const float encoderDistanceCalibration = wheelDiameter * 3.14159 / encodeCountsPerRev; // approx 8.4mm (32mm wheel) or 19.7mm (37mm)/count;
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

const int pwmFreq = 5000; // 5000; 20000;
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
const int gpioSensorStartFinish = 27; // Rightmost sensor
const int gpioSensorRightLine = 25;
const int gpioSensorLeftLine = 33;
const int gpioSensorRadius = 39;  // Leftmost sensor

const int phototransistorsResponseTimeMicroS = 1000;
const int adcConversionTimeMicroS = 87;   // Typical ADC conversion time

// LEDs
const int gpioIlluminationLED = 32;

// Aux I2C port
const int gpioI2cSda = 21;
const int gpioI2cScl = 22;

