//#include "mbed.h"
//using namespace mbed;
//using namespace rtos;

#include "sensors.h"
#include "motors.h"
#include "quadrature.h"

WallSensors sensors;
Motors motors;

/*
DigitalOut frontleds(p14);
DigitalOut sideleds(p15);
//AnalogIn sensorFL(p27);
//AnalogIn sensorFR(p28);
AnalogIn sensorL(p29);
AnalogIn sensorR(p26);
Thread poller(osPriorityRealtime);
EventFlags adcReadyEvent;

volatile unsigned short darkAdcFL;
volatile unsigned short lightAdcFL;
volatile unsigned short darkAdcFR;
volatile unsigned short lightAdcFR;
volatile unsigned short darkAdcL;
volatile unsigned short lightAdcL;
volatile unsigned short darkAdcR;
volatile unsigned short lightAdcR;

static const int adcPollIntervalMs = 2;
static const int adcSettlingDelayNs = 100000;
*/
/*
void readAdcLoop() 
{
  //analogin_t obj;
  //analogin_init(&obj, p27);
  while(1)
  {
    //darkAdcFL = sensorFL.read_u16(); 
    //darkAdcFR = sensorFR.read_u16(); 
    sensorFL.sampleAmbient();
    darkAdcL = sensorL.read_u16();
    darkAdcR = sensorR.read_u16();
    frontleds = 1;
    wait_ns(adcSettlingDelayNs);
//    lightAdcFL = sensorFL.read_u16();
//    lightAdcFR = sensorFR.read_u16();
     //   lightAdcL = analogin_read_u16(&obj);
         sensorFL.sampleLit();
        //lightAdcFL = analogin_read_u16(&obj);
        //lightAdcFR = analogin_read_u16(&obj);
     //   lightAdcR = analogin_read_u16(&obj);
    frontleds = 0;
    sideleds = 1;
    wait_ns(adcSettlingDelayNs);
    lightAdcL = sensorL.read_u16();
    lightAdcR = sensorR.read_u16();
    sideleds = 0;
    adcReadyEvent.set(1);
    ThisThread::sleep_for(adcPollIntervalMs);
  }
}
*/

void setup() 
{
    Serial.begin(115200);
    //Scheduler.startLoop(readAdcLoop);
    //poller.start(readAdcLoop);
    sensors.startSensors();
}

void loop() 
{

    Quadrature_encoder<2, 3> encoder1;
    Quadrature_encoder<8, 9> encoder2;
    encoder1.begin(pull_direction::up, resolution::full);
    encoder2.begin(pull_direction::up, resolution::full);
  
    while(true) {
        //delay(20);
        //adcReadyEvent.wait_any(1);
        sensors.waitForSample();
        Serial.print(sensors.left().getRaw());              Serial.print(" ");
        Serial.print(sensors.left().getCalibrated());       Serial.print(" ");
        Serial.print(sensors.frontLeft().getRaw());         Serial.print(" ");
        Serial.print(sensors.frontLeft().getCalibrated());  Serial.print(" ");
        Serial.print(sensors.frontRight().getRaw());        Serial.print(" ");
        Serial.print(sensors.frontRight().getCalibrated()); Serial.print(" ");
        Serial.print(sensors.right().getRaw());             Serial.print(" ");
        Serial.print(sensors.right().getCalibrated());      Serial.print(" ");

        // Encoders
        Serial.print(encoder1.count());                     Serial.print(" ");
        Serial.print(encoder1.interval());                  Serial.print(" ");
        Serial.print(encoder1.speed());                     Serial.print(" ");
        Serial.print(encoder2.count());                     Serial.print(" ");
        Serial.print(encoder2.interval());                  Serial.print(" ");
        Serial.print(encoder2.speed());                     Serial.print(" ");
        
        /*
        Serial.print(darkAdcL);        Serial.print(" ");
        Serial.print(lightAdcL);        Serial.print(" ");
        Serial.print(sensorFL.getRaw());        Serial.print(" ");
        Serial.print(sensorFL.getCalibrated());        Serial.print(" ");
        Serial.print(darkAdcFR);        Serial.print(" ");
        Serial.print(lightAdcFR);        Serial.print(" ");
        Serial.print(darkAdcR);        Serial.print(" ");
        Serial.print(lightAdcR);
        */
        //Serial.println();

        /*
         * Simple move to a set distance
        if(sensors.frontLeft().getCalibrated() < 30)
        {
          motors.forwardPower(128);
          Serial.println(" FORWARD");
        }
        else if(sensors.frontLeft().getCalibrated() > 60)
        {
          motors.forwardPower(-128);
          Serial.println(" BACKUP");
        }
        else
        {
          motors.stop();
          Serial.println(" STOP");
        }
        */

        // Very simple PI-controller
        // Line follower
        static const float kp = 2.0;
        static const float kd = 0.02;
        static float lastDist = 0.0;
        static const float interval = 1000.0 / WallSensors::adcPollIntervalMs;
        static const float targetDist = 50.0;

        float newDist = sensors.right().getCalibrated();
        float error = (newDist - targetDist) * kp + (newDist - lastDist) * kd * interval;
        lastDist = newDist;
        
        Serial.println(error);
        
        if(sensors.frontLeft().getCalibrated() > 30)
        {
          motors.stop();
          Serial.println(" STOP");
        }
        else
        {
          motors.turn(128, error);
        }
        
        //yield();
    }
}
