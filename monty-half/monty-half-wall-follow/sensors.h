#include <mbed.h>
#include "config.h"
//#include <pico/multicore.h>

//using namespace mbed;
//using namespace rtos;

class WallSensor
{
  private:
    unsigned int ambientRaw;
    unsigned int litRaw;

    unsigned int litMinimum;
    unsigned int litMaximum;

    analogin_t halObject;

  public:
    WallSensor(PinName pin, unsigned int litMinimum = 100, unsigned int litMaximum = 5000)
      : litMinimum(litMinimum), litMaximum(litMaximum)
    {
       analogin_init(&halObject, pin);
       // Get an initial value
       sampleAmbient();
       litRaw = ambientRaw;
    }
    
    // Get the ambient light level (unlit by our LEDs)
    void sampleAmbient()
    {
      ambientRaw = analogin_read_u16(&halObject);
    }
    
    // Get the light level once lit by our LEDs
    void sampleLit()
    {
      litRaw = analogin_read_u16(&halObject);
    }

    // Get the raw value above ambient light level
    unsigned int getRaw()
    {
      return litRaw > ambientRaw ? litRaw - ambientRaw : 0;
    }

    // Get the calibrated value where 0 = black, 100 = max
    unsigned int getCalibrated()
    {
      unsigned int raw = getRaw();
      if(raw <= litMinimum)
        return 0;
      else if(raw >= litMaximum)
        return 100;
      else
        return (unsigned int)((raw - litMinimum) * 100L / (litMaximum - litMinimum));
    }

    void setCalibration(unsigned int litMinimum, unsigned int litMaximum)
    {
      this->litMinimum = litMinimum;
      this->litMaximum = litMaximum;
    }
};


class WallSensors
{
  public:
    static const int adcPollIntervalMs = 2;
    static const int adcSettlingDelayNs = 100000;

  private:
    mbed::DigitalOut frontLeds;
    WallSensor sensorFL;
    WallSensor sensorFR;

    mbed::DigitalOut sideLeds;
    WallSensor sensorL;
    WallSensor sensorR;

    // Thread to poll the sensors
    rtos::Thread sensorThread;

    // Event flags to signal new sample available
    rtos::EventFlags adcReadyEvents;

  public:
    WallSensors()
    : frontLeds(p14), // TODO - Use hardware constants
      sensorFL(p28, sensor_frontleft_min_raw, sensor_frontleft_max_raw),
      sensorFR(p27, sensor_frontright_min_raw, sensor_frontright_max_raw),
      sideLeds(p15),
      sensorL(p26, sensor_left_min_raw, sensor_left_max_raw),
      sensorR(p29, sensor_right_min_raw, sensor_right_max_raw),
      sensorThread(osPriorityRealtime)
    {
      // Ensure LEDs off
      frontLeds = 0;
      sideLeds = 0;
    }

    void startSensors() 
    {
        sensorThread.start({this, &WallSensors::readAdcThreadLoop});
        //multicore_launch_core1({this, &WallSensors::readAdcThreadLoop});
    }
  
    void waitForSample()
    {
      adcReadyEvents.wait_any(1);
    }

    // Accessors
    WallSensor &frontLeft()
    {
      return sensorFL;
    }
    WallSensor &frontRight()
    {
      return sensorFR;
    }
    WallSensor &left()
    {
      return sensorL;
    }
    WallSensor &right()
    {
      return sensorR;
    }

    // Calibrate the left and right sensor, assuing we're midway between two walls
    void calibrateLRSensors()
    {
      unsigned int whiteRawL = sensorL.getRaw();
      unsigned int whiteRawR = sensorR.getRaw();
      // Convert this to 85% by assuming black is 50% of the current reading
      unsigned int blackRawL = whiteRawL / 2;
      sensorL.setCalibration(blackRawL, (whiteRawL - blackRawL) * 20 / 17 + blackRawL);
      unsigned int blackRawR = whiteRawR / 2;
      sensorR.setCalibration(blackRawR, (whiteRawR - blackRawR) * 20 / 17 + blackRawR);
    }
    
  private:
    void readAdcThreadLoop() 
    {
      while(1)
      {
          // Sample ambient first
          sensorFL.sampleAmbient();
          sensorFR.sampleAmbient();
          sensorL.sampleAmbient();
          sensorR.sampleAmbient();
          
          // Sample front sensors lit values
          frontLeds = 1;
          wait_ns(adcSettlingDelayNs);
          sensorFL.sampleLit();
          sensorFR.sampleLit();
          frontLeds = 0;
          // Then the side sensors lit values
          sideLeds = 1;
          wait_ns(adcSettlingDelayNs);
          sensorL.sampleLit();
          sensorR.sampleLit();
          sideLeds = 0;

          // Signal to listeners
          adcReadyEvents.set(1);

          // Loop delay
          rtos::ThisThread::sleep_for(adcPollIntervalMs);
       }
    }
};
