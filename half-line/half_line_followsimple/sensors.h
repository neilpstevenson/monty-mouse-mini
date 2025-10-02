#include <mbed.h>
#include "config.h"
//#include <pico/multicore.h>

//using namespace mbed;
//using namespace rtos;

class Sensor
{
  private:
    unsigned int ambientRaw;
    unsigned int litRaw;

    unsigned int litMinimum;
    unsigned int litMaximum;

    analogin_t halObject;

  public:
    Sensor(PinName pin, unsigned int litMinimum = 100, unsigned int litMaximum = 5000)
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
    float getCalibrated()
    {
      float raw = getRaw();
      if(raw <= litMinimum)
        return 0;
      else if(raw >= litMaximum)
        return 100;
      else
        return (raw - litMinimum) * 100.0 / (litMaximum - litMinimum);
    }

    void setCalibration(unsigned int litMinimum, unsigned int litMaximum)
    {
      this->litMinimum = litMinimum;
      this->litMaximum = litMaximum;
    }

    // Adjust base on last reading
    void adjustCalibration()
    {
      litMinimum = std::min(litMinimum, getRaw()); 
      litMaximum = std::max(litMaximum, getRaw()); 
    }

    unsigned int calibrationMin()
    {
      return litMinimum;
    }
    
    unsigned int calibrationMax()
    {
      return litMaximum;
    }

};


class LineSensors
{
  public:
    static const int adcPollIntervalMs = 2;
    static const int adcSettlingDelayNs = 40000;

  private:
    mbed::DigitalOut leftLeds;
    Sensor sensorLeft;
    Sensor sensorMidLeft;
    Sensor sensorCentreLeft;

    mbed::DigitalOut rightLeds;
    Sensor sensorRight;
    Sensor sensorMidRight;
    Sensor sensorCentreRight;

    // Thread to poll the sensors
    rtos::Thread sensorThread;

    // Event flags to signal new sample available
    rtos::EventFlags adcReadyEvents;

  public:
    LineSensors()
    : leftLeds(p14), // TODO - Use hardware constants
      sensorLeft(p26, sensor_left_min_raw, sensor_left_max_raw),
      sensorMidLeft(p27, sensor_left_mid_min_raw, sensor_left_mid_max_raw),
      sensorCentreLeft(p28, sensor_left_centre_min_raw, sensor_left_centre_max_raw),
      rightLeds(p15),
      sensorRight(p26, sensor_right_min_raw, sensor_right_max_raw),
      sensorMidRight(p27, sensor_right_mid_min_raw, sensor_right_mid_max_raw),
      sensorCentreRight(p28, sensor_right_centre_min_raw, sensor_right_centre_max_raw),
      sensorThread(osPriorityRealtime)
    {
      // Ensure LEDs off
      leftLeds = 0;
      rightLeds = 0;
    }

    void startSensors() 
    {
        sensorThread.start({this, &LineSensors::readAdcThreadLoop});
    }
  
    void waitForSample()
    {
      adcReadyEvents.wait_any(1);
    }

    // Accessors
    Sensor &left()
    {
      return sensorLeft;
    }
    Sensor &midLeft()
    {
      return sensorMidLeft;
    }
    Sensor &centreLeft()
    {
      return sensorCentreLeft;
    }
    Sensor &right()
    {
      return sensorRight;
    }
    Sensor &midRight()
    {
      return sensorMidRight;
    }
    Sensor &centreRight()
    {
      return sensorCentreRight;
    }

    // Get a simple weighted error from the 4 centre sensors, weighted at 3:1 to git an approx linear error value
    float getWeightedCentreOffset()
    {
      return ((sensorMidLeft.getCalibrated()*3.0 + sensorCentreLeft.getCalibrated()) - (sensorMidRight.getCalibrated()*3.0 + sensorCentreRight.getCalibrated())) / 3.0;
    }


    // Calibrate the sensors by adjusting the max/min value of each depending on current reading
    void calibrateSensors(int timeMs)
    {
      // Clear current calibration
      sensorLeft.setCalibration(sensorLeft.getRaw(), sensorLeft.getRaw() + 1);
      sensorMidLeft.setCalibration(sensorMidLeft.getRaw(), sensorMidLeft.getRaw() + 1);
      sensorCentreLeft.setCalibration(sensorCentreLeft.getRaw(), sensorCentreLeft.getRaw() + 1);
      sensorRight.setCalibration(sensorRight.getRaw(), sensorRight.getRaw() + 1);
      sensorMidRight.setCalibration(sensorMidRight.getRaw(), sensorMidRight.getRaw() + 1);
      sensorCentreRight.setCalibration(sensorCentreRight.getRaw(), sensorCentreRight.getRaw() + 1);

      // Wait for specified time
      for(int i=0; i < timeMs / adcPollIntervalMs; i++)
      {
        waitForSample();
        sensorLeft.adjustCalibration();
        sensorMidLeft.adjustCalibration();
        sensorCentreLeft.adjustCalibration();
        sensorRight.adjustCalibration();
        sensorMidRight.adjustCalibration();
        sensorCentreRight.adjustCalibration();
      }
    }


  private:
    void readAdcThreadLoop() 
    {
      while(1)
      {
          // Sample ambient first
          sensorLeft.sampleAmbient();
          sensorMidLeft.sampleAmbient();
          sensorCentreLeft.sampleAmbient();
          
          {
            mbed::CriticalSectionLock criticalSectionLock; // To prevent encoder interrupts
            // Sample left sensors lit values
            leftLeds = 1;
            wait_ns(adcSettlingDelayNs);
            sensorLeft.sampleLit();
            sensorMidLeft.sampleLit();
            sensorCentreLeft.sampleLit();
            leftLeds = 0;
          }

          rtos::ThisThread::sleep_for(adcPollIntervalMs/2);

          // ToDo - Optimimise combined readings
          sensorRight.sampleAmbient();
          sensorMidRight.sampleAmbient();
          sensorCentreRight.sampleAmbient();
          {
            mbed::CriticalSectionLock criticalSectionLock; // To prevent encoder interrupts
            // Then the side sensors lit values
            rightLeds = 1;
            wait_ns(adcSettlingDelayNs);
            sensorRight.sampleLit();
            sensorMidRight.sampleLit();
            sensorCentreRight.sampleLit();
            rightLeds = 0;
          }

          // Signal to listeners
          adcReadyEvents.set(1);

          // Loop delay
          rtos::ThisThread::sleep_for(adcPollIntervalMs/2);
       }
    }
};
