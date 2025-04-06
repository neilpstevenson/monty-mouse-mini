#include "hardware.h"
#include "config.h"

#define MOTORS_USE_BREAK_MODE

class Motor
{
  private:
    int pinA;
    int pinB;
    int currentPower;
    
  public:
    Motor(int pinA, int pinB)
      : pinA(pinA), pinB(pinB), currentPower(0)
    {
      pinMode(pinA, OUTPUT);
      pinMode(pinB, OUTPUT);
      // Set initial state
      analogWrite(pinA, 0);  // 0 = coast mode, 255 = break-mode
      analogWrite(pinB, 0);
    }

#ifdef MOTORS_USE_BREAK_MODE
    // Set power, -255 to +255
    void setPower(int power)
    {
      // Break mode
      if(power >= 0)
      {
        if(power > 255)
          power = 255;
        analogWrite(pinA, 255);  // 0 = coast mode, 255 = break-mode
        analogWrite(pinB, 255-power);
      }
      else
      {
        if(power < -255)
          power = -255;
        analogWrite(pinA, 255+power);
        analogWrite(pinB, 255);
      }
      currentPower = power;
    } 
#else //MOTORS_USE_BREAK_MODE
    // Set power, -255 to +255
    void setPower(int power)
    {
      // Break mode
      if(power >= 0)
      {
        if(power > 255)
          power = 255;
        analogWrite(pinA, power);  // 0 = coast mode, 255 = break-mode
        analogWrite(pinB, 0);
      }
      else
      {
        if(power < -255)
          power = -255;
        analogWrite(pinA, 0);
        analogWrite(pinB, -power);
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
        analogWrite(pinA, 255);  // 0 = coast mode, 255 = break-mode
        analogWrite(pinB, 255);
      }
      else
      {
        analogWrite(pinA, 0);
        analogWrite(pinB, 0);
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
        left.setPower(turn * motor_compensation_left);
        right.setPower(-turn);
      }
      else
      {
        left.setPower(turn);
        right.setPower(-turn * motor_compensation_right);
      }
    }

    void stop(bool breakMode = false)
    {
      left.stop(breakMode);
      right.stop(breakMode);
    }
};
