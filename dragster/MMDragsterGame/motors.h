//#define DEBUG_MOTORS   

// Stop the motors to idle (non-breaking)
void stopAll()
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
void setSpeed(int forwardSpeed)
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
      ledcWrite(motorA1PwmChannel, dutyCycle);
      ledcWrite(motorA2PwmChannel, 0);
      ledcWrite(motorB1PwmChannel, dutyCycle);
      ledcWrite(motorB2PwmChannel, 0);
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
      ledcWrite(motorA1PwmChannel, 0);
      ledcWrite(motorA2PwmChannel, dutyCycle);
      ledcWrite(motorB1PwmChannel, 0);
      ledcWrite(motorB2PwmChannel, dutyCycle);
#endif //MOTORCTRL_DRV8833
    }
#endif // DEBUG_MOTORS
}

// Stop using electronic braking
void breakStop()
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
