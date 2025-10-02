#include "hardware.h"
#include "config.h"

class Motor
{
  private:
    // we use the mbed call rather than the Arduino interfaces to access the PWM channels as it gives us more 
    // control of PWM frequency etc. and also a much faster interface
    mbed::PwmOut motorA;
    mbed::PwmOut motorB;
    int currentPower;
    int maxAcceleration;

    const float MOTOR_PWM_PERIOD = 1.0 / 20000; // 20kHz give a reasonable response
   
  public:
    Motor(int pinA, int pinB, int maxAcceleration = max_acceleration_per_update): 
      motorA(PinName(pinA)),
      motorB(PinName(pinB)),
      currentPower(0),
      maxAcceleration(maxAcceleration)
    {
      // Set PWM frequency
      motorA.period(MOTOR_PWM_PERIOD);
      motorB.period(MOTOR_PWM_PERIOD);
      // Default to "brake" mode - both inputs high
      motorA.write(1);
      motorB.write(1);
    }

    // Set power, -255 to +255
    void setPower(int power, bool limit_accel = true)    
    {
      if(limit_accel)
      {
        // Limit acceleration
        if(power > currentPower + maxAcceleration)
          power = currentPower + maxAcceleration;
        else if(power < currentPower - maxAcceleration)
          power = currentPower - maxAcceleration;
      }
      
#ifdef MOTORS_USE_BREAK_MODE
      // Break mode
      if(power >= 0)
      {
        if(power > 255)
          power = 255;
        motorA.write(1);  // 1 = break mode for better control
        motorB.write((255-power) / 255.0);
      }
      else
      {
        if(power < -255)
          power = -255;
        motorB.write(1);  // 1 = break mode for better control
        motorA.write((255+power) / 255.0);
      }
      currentPower = power;
    } 
#else //MOTORS_USE_BREAK_MODE
      // Freewheel mode
      if(power >= 0)
      {
        if(power > 255)
          power = 255;
        motorB.write(0);  // 0 = coast mode for better power & speed
        motorA.write(power / 255.0);
      }
      else
      {
        if(power < -255)
          power = -255;
        motorA.write(0);  // 0 = coast mode for better power & speed
        motorB.write(-power / 255.0);
      }
      currentPower = power;
    } 
#endif //MOTORS_USE_BREAK_MODE

    int getPower()
    {
      return currentPower;
    }
    
    void stop(bool breakMode = false)
    {
      if(breakMode)
      {
        motorA.write(1);  // Breaking to a halt
        motorB.write(1);
      }
      else
      {
        motorA.write(0);  // Coast to a halt
        motorB.write(0);
      }
      currentPower = 0;
    }      
};

class Motors
{
  public:
    Motor left;
    Motor right;
    
  public:
    Motors(int leftA1 = motorLeftA1, int leftA2 = motorLeftA2, int rightB1 = motorRightB1, int rightB2 = motorRightB2)
      : left(leftA1, leftA2), right(rightB1, rightB2)
    {
    }

    // Move forward/reverse -255 to 255
    void forwardPower(int power)
    {
      if(power >= 0)
      {
        left.setPower((int)(power * motor_compensation_left));
        right.setPower(power);
      }
      else
      {
        left.setPower(power);
        right.setPower((int)(power * motor_compensation_right));
      }
    }

    // Turn on an arc forward/reverse power+/-turn
    void turn(int power, int turn)
    {
      if(power >= 0)
      {
        // clamp to max power
        if(turn > power - 2)
          turn = power - 2;
        else if(turn < -(power - 2))
          turn = -(power - 2);
        if(turn >= 0)
        {
          // Turn left
          left.setPower((power - turn) * motor_compensation_left);
          //right.setPower(power + turn);
          right.setPower(power);
        }
        else
        {
          // Turn right
          left.setPower(power * motor_compensation_left);
          //right.setPower(power + turn);
          right.setPower(power + turn);
        }
      }
      else
      {
        // clamp to max power
        if(turn > -power)
          turn = -power;
        else if(turn < power)
          turn = power;
        left.setPower(power - turn);
        //right.setPower((power + turn) * motor_compensation_right);
        right.setPower(power * motor_compensation_right);
      }
    }

    // Rotate on spot -255 to 255
    void turn(int turn)
    {
      if(turn >= 0)
      {
        left.setPower(turn * motor_compensation_left, false);
        right.setPower(-turn, false);
      }
      else
      {
        left.setPower(turn, false);
        right.setPower(-turn * motor_compensation_right, false);
      }
    }

    void stop(bool breakMode = false)
    {
      left.stop(breakMode);
      right.stop(breakMode);
    }
};
