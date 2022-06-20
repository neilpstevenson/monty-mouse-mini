//#define DEBUG_MOTORS   

// Stop the motors to idle (non-breaking)
void stopAll()
{
    // Stop motors
    digitalWrite(gpioMotorA1, LOW);
    digitalWrite(gpioMotorA2, LOW);
    digitalWrite(gpioMotorB1, LOW);
    digitalWrite(gpioMotorB2, LOW);
    ledcWrite(motorRPwmChannel, 0);
    ledcWrite(motorLPwmChannel, 0);
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
      digitalWrite(gpioMotorA1, HIGH);
      digitalWrite(gpioMotorA2, LOW);
      digitalWrite(gpioMotorB1, HIGH);
      digitalWrite(gpioMotorB2, LOW);
      ledcWrite(motorRPwmChannel, dutyCycle);
      ledcWrite(motorLPwmChannel, dutyCycle);
    }
    else
    {
      // Reverse
      int dutyCycle = (-forwardSpeed-1)*2;
      digitalWrite(gpioMotorA1, LOW);
      digitalWrite(gpioMotorA2, HIGH);
      digitalWrite(gpioMotorB1, LOW);
      digitalWrite(gpioMotorB2, HIGH);
      ledcWrite(motorRPwmChannel, dutyCycle);
      ledcWrite(motorLPwmChannel, dutyCycle);
    }
#endif // DEBUG_MOTORS
}

// Stop using electronic braking
void breakStop()
{
    digitalWrite(gpioMotorA1, HIGH);
    digitalWrite(gpioMotorA2, HIGH);
    digitalWrite(gpioMotorB1, HIGH);
    digitalWrite(gpioMotorB2, HIGH);
    ledcWrite(motorRPwmChannel, 0);
    ledcWrite(motorLPwmChannel, 0);
}
