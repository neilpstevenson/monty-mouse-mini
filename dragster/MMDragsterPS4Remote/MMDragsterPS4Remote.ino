#include <PS4Controller.h>
#include <ESP32Servo.h>
#include "mm_hardware.h"
#include "pid.h"
#include "debounce.h"

#include <TFT_eSPI.h> // Hardware-specific library

extern TFT_eSPI tft;
extern int value[6];
extern void plotPointers(void);
extern void plotLinear(const char *label, int x, int y);


Servo steeringServo;
float pidInput = 0.0;
float pidSetpoint = 0.0;
PID pid(PID_Kp, PID_Ki, PID_Kd, &pidInput, &pidSetpoint);
Debounce startFinish(markerLowThreshold, markerHighThreshold, false);
int startFinishCount = 0;

void setup() 
{
  Serial.begin(115200);

  // Display
  tft.init();
  tft.setRotation(3);
  tft.fillScreen(TFT_BLACK);

  // Set up analog inputs
  pinMode(gpioSensorStartFinish, INPUT);
  pinMode(gpioSensorRightLine, INPUT);
  pinMode(gpioSensorLeftLine, INPUT);
  pinMode(gpioSensorRadius, INPUT);

  // Draw 4 linear meters
  byte d = 60;
  plotLinear("S-F", 0, 0);
  plotLinear(" R", 1 * d, 0);
  plotLinear(" L", 2 * d, 0);
  plotLinear("Rad", 3 * d, 0);

  // LEDs
  pinMode(gpioIlluminationLED, OUTPUT);
  digitalWrite(gpioIlluminationLED, LOW);
  
  // configure motor PWM output
  ledcSetup(motorRPwmChannel, pwmFreq, pwmResolution);
  ledcAttachPin(gpioMotorAPwm, motorRPwmChannel);
  pinMode(gpioMotorA1, OUTPUT);
  digitalWrite(gpioMotorA1, LOW);
  pinMode(gpioMotorA2, OUTPUT);
  digitalWrite(gpioMotorA2, LOW);
  
  ledcSetup(motorLPwmChannel, pwmFreq, pwmResolution);
  ledcAttachPin(gpioMotorBPwm, motorLPwmChannel);
  pinMode(gpioMotorB1, OUTPUT);
  digitalWrite(gpioMotorB1, LOW);
  pinMode(gpioMotorB2, OUTPUT);
  digitalWrite(gpioMotorB2, LOW);

  // Steering 
  ESP32PWM::allocateTimer(2);
  steeringServo.setPeriodHertz(steeringServoFreq);
  steeringServo.attach(gpioSteeringServo, 1000, 2000);

  // PS4 controller
  //PS4.begin("44:17:93:89:84:6a"); // this esp32
  PS4.begin();  // default MAC

  Serial.println("Waiting for PS4 controller...");
  while(!PS4.isConnected()) 
  {
    delay(100);
  }
  Serial.println("Connected.");
}

void loop() 
{
  if (!PS4.isConnected()) 
  {
     // Stop motors
    digitalWrite(gpioMotorA1, LOW);
    digitalWrite(gpioMotorA2, LOW);
    digitalWrite(gpioMotorB1, LOW);
    digitalWrite(gpioMotorB2, LOW);
    ledcWrite(motorRPwmChannel, 0);
    ledcWrite(motorLPwmChannel, 0);
    //digitalWrite(gpioIlluminationLED, LOW);
    
    Serial.println("Waiting for PS4 controller...");
    delay(500);
  }
  else
  {
    digitalWrite(gpioIlluminationLED, HIGH);
    
    int forward = PS4.LStickY();
    if(forward > 2)
    {
      // Forward
      int dutyCycle = forward*2;
      digitalWrite(gpioMotorA1, HIGH);
      digitalWrite(gpioMotorA2, LOW);
      digitalWrite(gpioMotorB1, HIGH);
      digitalWrite(gpioMotorB2, LOW);
      ledcWrite(motorRPwmChannel, dutyCycle);
      ledcWrite(motorLPwmChannel, dutyCycle);
    }
    else if(forward < -2)
    {
      // Reverse
      int dutyCycle = (-forward-1)*2;
      digitalWrite(gpioMotorA1, LOW);
      digitalWrite(gpioMotorA2, HIGH);
      digitalWrite(gpioMotorB1, LOW);
      digitalWrite(gpioMotorB2, HIGH);
      ledcWrite(motorRPwmChannel, dutyCycle);
      ledcWrite(motorLPwmChannel, dutyCycle);
    }
    else
    {
      // Stop
      ledcWrite(motorRPwmChannel, 0);
      ledcWrite(motorLPwmChannel, 0);
    }

    // Steering
//    int steer = PS4.RStickX();
//    int steerServoPos =  steeringServoCentre - steer;
//    steeringServo.write(steerServoPos);
    
  }

  // Get the current sensor data
  int sensorStartFinish = analogRead(gpioSensorStartFinish);
  int sensorRightLine = analogRead(gpioSensorRightLine);
  int sensorLeftLine = analogRead(gpioSensorLeftLine);
  int sensorRadius = analogRead(gpioSensorRadius);

  // Calculate the steering error
  int steeringError = sensorLeftLine - sensorRightLine;
  pidInput = steeringError;
  float pidOutput = pid.compute();
  //Serial.printf("PID in = %f, out = %f\n", pidInput, pidOutput);

  int steer = (int)pidOutput;
  int steerServoPos =  steeringServoCentre + steer;
  steeringServo.write(steerServoPos);

  // Display the sensors
  value[0] = map(sensorStartFinish, 0, 4095, 0, 100);
  value[1] = map(sensorRightLine, 0, 4095, 0, 100);
  value[2] = map(sensorLeftLine, 0, 4095, 0, 100);
  value[3] = map(sensorRadius, 0, 4095, 0, 100);
  plotPointers();

  // Check start/finish sensor
  if(startFinish.isTriggered(sensorStartFinish))
  {
    ++startFinishCount;
    Serial.printf("Start/Finish triggered %d times\n", startFinishCount);
  }
  
  delay(10);
}
