#include "hardware.h"
#include "config.h"

#define MOTORS_USE_BREAK_MODE

class Motor
{
  private:
    int pinA;
    int pinB;
    
  public:
    Motor(int pinA, int pinB)
      : pinA(pinA), pinB(pinB)
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
    } 
#endif //MOTORS_USE_BREAK_MODE

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
    }      
};

class Motors
{
  private:
    Motor left;
    Motor right;
    
  public:
    Motors(int leftA1 = motorA2, int leftA2 = motorA1, int rightB1 = motorB1, int rightB2 = motorB2)
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

    // Move forward/reverse -255 to 255
    void turn(int power, int turn)
    {
      left.setPower(power - turn);
      right.setPower(power + turn);
    }

    void stop(bool breakMode = false)
    {
      left.stop(breakMode);
      right.stop(breakMode);
    }
};
