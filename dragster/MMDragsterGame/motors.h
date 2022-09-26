#pragma once
//#define DEBUG_MOTORS   

class Motors
{
  public:
    void initHardware();
    void setSpeed(int forwardSpeed);
    void stopAll();
    void breakStop();  
};
