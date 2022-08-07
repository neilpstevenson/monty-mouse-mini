//#define DEBUG_MOTORS   

class Motors
{
  public:
    void initHardware();
    void setSpeed(int forwardSpeed);
    void stopAll();
    void breakStop();  
};

void Motors::initHardware()
{
#ifdef MOTORCTRL_TB6612
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
#endif //MOTORCTRL_TB6612

#ifdef MOTORCTRL_DRV8833
  ledcSetup(motorA1PwmChannel, pwmFreq, pwmResolution);
  ledcAttachPin(gpioMotorA1, motorA1PwmChannel);
  ledcSetup(motorA2PwmChannel, pwmFreq, pwmResolution);
  ledcAttachPin(gpioMotorA2, motorA2PwmChannel);

  ledcSetup(motorB1PwmChannel, pwmFreq, pwmResolution);
  ledcAttachPin(gpioMotorB1, motorB1PwmChannel);
  ledcSetup(motorB2PwmChannel, pwmFreq, pwmResolution);
  ledcAttachPin(gpioMotorB2, motorB2PwmChannel);
#endif //MOTORCTRL_DRV8833
}

// Stop the motors to idle (non-breaking)
void Motors::stopAll()
{
    // Stop motors
#ifdef MOTORCTRL_TB6612
    digitalWrite(gpioMotorA1, LOW);
    digitalWrite(gpioMotorA2, LOW);
    digitalWrite(gpioMotorB1, LOW);
    digitalWrite(gpioMotorB2, LOW);
    ledcWrite(motorRPwmChannel, 0);
    ledcWrite(motorLPwmChannel, 0);
#endif //MOTORCTRL_TB6612
#ifdef MOTORCTRL_DRV8833
    ledcWrite(motorA1PwmChannel, 0);
    ledcWrite(motorA2PwmChannel, 0);
    ledcWrite(motorB1PwmChannel, 0);
    ledcWrite(motorB2PwmChannel, 0);
#endif //MOTORCTRL_DRV8833
    digitalWrite(gpioIlluminationLED, LOW);
}

// Set both motors to speed
void Motors::setSpeed(int forwardSpeed)
{
#ifdef DEBUG_MOTORS
   Serial.printf("Speed: %d\n", forwardSpeed);
#else //DEBUG_MOTORS
   if(forwardSpeed >= 0)
    {
      // Forward
      int dutyCycle = forwardSpeed*2;
#ifdef MOTORCTRL_TB6612
      digitalWrite(gpioMotorA1, HIGH);
      digitalWrite(gpioMotorA2, LOW);
      digitalWrite(gpioMotorB1, HIGH);
      digitalWrite(gpioMotorB2, LOW);
      ledcWrite(motorRPwmChannel, dutyCycle);
      ledcWrite(motorLPwmChannel, dutyCycle);
#endif //MOTORCTRL_TB6612
#ifdef MOTORCTRL_DRV8833
#ifdef MOTORCTRL_WITHBREAK
      ledcWrite(motorA1PwmChannel, 256);
      ledcWrite(motorA2PwmChannel, 255-dutyCycle);
      ledcWrite(motorB1PwmChannel, 256);
      ledcWrite(motorB2PwmChannel, 255-dutyCycle);
#else //MOTORCTRL_WITHBREAK
      ledcWrite(motorA1PwmChannel, dutyCycle);
      ledcWrite(motorA2PwmChannel, 0);
      ledcWrite(motorB1PwmChannel, dutyCycle);
      ledcWrite(motorB2PwmChannel, 0);
#endif //MOTORCTRL_WITHBREAK
#endif //MOTORCTRL_DRV8833
    }
    else
    {
      // Reverse
      int dutyCycle = (-forwardSpeed-1)*2;
#ifdef MOTORCTRL_TB6612
      digitalWrite(gpioMotorA1, LOW);
      digitalWrite(gpioMotorA2, HIGH);
      digitalWrite(gpioMotorB1, LOW);
      digitalWrite(gpioMotorB2, HIGH);
      ledcWrite(motorRPwmChannel, dutyCycle);
      ledcWrite(motorLPwmChannel, dutyCycle);
#endif //MOTORCTRL_TB6612
#ifdef MOTORCTRL_DRV8833
#ifdef MOTORCTRL_WITHBREAK
      ledcWrite(motorA1PwmChannel, 255-dutyCycle);
      ledcWrite(motorA2PwmChannel, 256);
      ledcWrite(motorB1PwmChannel, 255-dutyCycle);
      ledcWrite(motorB2PwmChannel, 256);
#else //MOTORCTRL_WITHBREAK
      ledcWrite(motorA1PwmChannel, 0);
      ledcWrite(motorA2PwmChannel, dutyCycle);
      ledcWrite(motorB1PwmChannel, 0);
      ledcWrite(motorB2PwmChannel, dutyCycle);
#endif //MOTORCTRL_WITHBREAK
#endif //MOTORCTRL_DRV8833
    }
#endif // DEBUG_MOTORS
}

// Stop using electronic braking
void Motors::breakStop()
{
#ifdef MOTORCTRL_TB6612
    digitalWrite(gpioMotorA1, HIGH);
    digitalWrite(gpioMotorA2, HIGH);
    digitalWrite(gpioMotorB1, HIGH);
    digitalWrite(gpioMotorB2, HIGH);
    ledcWrite(motorRPwmChannel, 0);
    ledcWrite(motorLPwmChannel, 0);
#endif //MOTORCTRL_TB6612
#ifdef MOTORCTRL_DRV8833
    ledcWrite(motorA1PwmChannel, 256);
    ledcWrite(motorA2PwmChannel, 256);
    ledcWrite(motorB1PwmChannel, 256);
    ledcWrite(motorB2PwmChannel, 256);
#endif //MOTORCTRL_DRV8833
}
