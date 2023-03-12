//#include "mbed.h"
//using namespace mbed;
//using namespace rtos;
#include <Adafruit_NeoPixel.h>
#include "sensors.h"
#include "motors.h"
#include "quadrature.h"
#include "config.h"

WallSensors sensors;
Motors motors;

Quadrature_encoder<8, 9> encoder_l;
Quadrature_encoder<2, 3> encoder_r;

Adafruit_NeoPixel pixels(1, neoPixel, NEO_GRB + NEO_KHZ800);

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

void forward(int distance, int speed)
{
  // Move forward until distance reached or too near the forward wall
  float start_pos_r = encoder_r.count() * encode_calibrate_r;
  motors.forwardPower(speed);
  while(encoder_r.count() * encode_calibrate_r - start_pos_r < distance &&
        sensors.frontLeft().getCalibrated() < wall_follow_forward_min_distance)
  {
    delay(10);
    //Serial.print("FORWARD ");
    //Serial.print(encoder_r.count() * encode_calibrate_l);  Serial.print("mm ");
    //Serial.println();
    logSensors("FORWARD");
  }
}

void turn_left_90(int speed)
{
  // Rotate 90 degrees
  float start_pos_l = encoder_l.count() * encode_calibrate_l;
  float start_pos_r = encoder_r.count() * encode_calibrate_r;
  motors.turn(speed, speed);

  int required_dist_r = (int)(3.14159/2.0 * turning_diameter_mm);
  
  while(encoder_r.count() * encode_calibrate_r - start_pos_r <
        (encoder_l.count() * encode_calibrate_l - start_pos_l + required_dist_r))
  {
    delay(10);
    //Serial.print("LEFT ");
    //Serial.print(encoder_r.count() * encode_calibrate_l);  Serial.print("mm ");
    //Serial.print(encoder_l.count() * encode_calibrate_r);  Serial.print("mm ");
    //Serial.println();
    logSensors("LEFT90");
  }
}

void turn_right_90(int speed)
{
  // Rotate 90 degrees
  float start_pos_l = encoder_l.count() * encode_calibrate_l;
  float start_pos_r = encoder_r.count() * encode_calibrate_r;
  motors.turn(speed, -speed);

  int required_dist_r = (int)(3.14159/2.0 * turning_diameter_mm);
  
  while(encoder_l.count() * encode_calibrate_l - start_pos_l <
        (encoder_r.count() * encode_calibrate_r - start_pos_r + required_dist_r))
  {
    delay(10);
    //Serial.print("RIGHT ");
    //Serial.print(encoder_r.count() * encode_calibrate_l);  Serial.print("mm ");
    //Serial.print(encoder_l.count() * encode_calibrate_r);  Serial.print("mm ");
    //Serial.println();
    logSensors("RIGHT90");
  }
}

void turn_right_180(int speed)
{
  // Rotate 90 degrees
  float start_pos_l = encoder_l.count() * encode_calibrate_l;
  float start_pos_r = encoder_r.count() * encode_calibrate_r;
  motors.turn(0, -speed);

  int required_dist_r = (int)(3.14159 * turning_diameter_mm);
  
  while(encoder_l.count() * encode_calibrate_l - start_pos_l <
        (encoder_r.count() * encode_calibrate_r - start_pos_r + required_dist_r))
  {
    delay(10);
    logSensors("RIGHT180");
  }
}


void logSensors(const char *mode)
{
  Serial.print(mode); Serial.print(" ");
  
  // Sensors
  Serial.print(sensors.left().getCalibrated());       Serial.print(" ");
  Serial.print(sensors.frontLeft().getCalibrated());  Serial.print(" ");
  Serial.print(sensors.frontRight().getCalibrated()); Serial.print(" ");
  Serial.print(sensors.right().getCalibrated());      Serial.print(" ");
  
  // Encoders
  Serial.print(encoder_r.count() * encode_calibrate_r);Serial.print("mm ");
  Serial.print(encoder_l.count() * encode_calibrate_l);Serial.print("mm ");

  Serial.println();
}

void setup() 
{
    Serial.begin(115200);
    //Scheduler.startLoop(readAdcLoop);
    //poller.start(readAdcLoop);

    pixels.clear(); // Set all neopixels to 'off'

    pinMode(buttonA, INPUT_PULLUP);
    pinMode(buttonB, INPUT_PULLUP);
}

void loop() 
{
    encoder_r.begin(pull_direction::up, resolution::full);
    encoder_l.begin(pull_direction::up, resolution::full);

    // Display starting flashes
    bool triggered = false;
    Serial.println("waiting");
    for(int count = 0; !triggered; count++)
    {
      if(count % 30 == 0)
      {
        // On
        pixels.setPixelColor(0, pixels.Color(8, 0, 0));
        pixels.show();
      }
      else if(count % 30 == 15)
      {
        // Off
        pixels.setPixelColor(0, pixels.Color(0, 0, 0));
        pixels.show();
      }

      if(digitalRead(buttonB) == 0)
      {
        triggered = true;
        Serial.println("triggered");
      }
      
      delay(20);
    }
    Serial.println("running");

    // Show starting
    pixels.setPixelColor(0, pixels.Color(0, 16, 0));
    pixels.show();
    
    sensors.startSensors();

    delay(2000);

    // Go
    bool justTurned = false;
    
    while(true) {
        //delay(20);
        //adcReadyEvent.wait_any(1);
        sensors.waitForSample();
        
        /*
        Serial.print(sensors.left().getRaw());              Serial.print(" ");
        Serial.print(sensors.left().getCalibrated());       Serial.print(" ");
        Serial.print(sensors.frontLeft().getRaw());         Serial.print(" ");
        Serial.print(sensors.frontLeft().getCalibrated());  Serial.print(" ");
        Serial.print(sensors.frontRight().getRaw());        Serial.print(" ");
        Serial.print(sensors.frontRight().getCalibrated()); Serial.print(" ");
        Serial.print(sensors.right().getRaw());             Serial.print(" ");
        Serial.print(sensors.right().getCalibrated());      Serial.print(" ");

        // Encoders
        Serial.print(encoder_r.count());                     Serial.print(" ");
        Serial.print(encoder_r.count() * encode_calibrate_r);Serial.print("mm ");
        Serial.print(encoder_r.interval());                  Serial.print(" ");
        Serial.print(encoder_r.speed());                     Serial.print(" ");
        Serial.print(encoder_l.count());                     Serial.print(" ");
        Serial.print(encoder_l.count() * encode_calibrate_l);Serial.print("mm ");
        Serial.print(encoder_l.interval());                  Serial.print(" ");
        Serial.print(encoder_l.speed());                     Serial.print(" ");
        */
        
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
/*
        motors.stop();

        // Foward 100mm
        float start_pos_r = encoder_r.count() * encode_calibrate_r;
//        while(1)
//        {
        motors.forwardPower(127);
//        delay(2000);
//        motors.forwardPower(-127);
//        delay(2000);
//        }
        while(encoder_r.count() * encode_calibrate_r - start_pos_r < 100)
        {
          delay(10);
          Serial.print(encoder_r.count() * encode_calibrate_r);Serial.print("mm ");
          Serial.print(encoder_l.count() * encode_calibrate_l);Serial.print("mm ");
          Serial.println();
        }
       
        // Stop
        motors.stop();

        // Allow to stop
        delay(1000);
        Serial.print(encoder_r.count() * encode_calibrate_r);Serial.print("mm ");
        Serial.print(encoder_l.count() * encode_calibrate_l);Serial.print("mm ");
        Serial.println();
        
        // Wait 10s to retry
        delay(10000);
*/

        // Test encoder-based moves
        /*
        forward(100);
        motors.stop();
        delay(2000);
        forward(100);
        motors.stop();
        delay(2000);
        turn_left_90();
        motors.stop();
        delay(2000);
        */

        // Test simple moves
        /*
        while(1)
        {
          motors.forwardPower(128);
          delay(1000);
          motors.stop();
          delay(1000);
          motors.forwardPower(-128);
          delay(1000);
          motors.stop();
          delay(1000);
        }
        */

        // Very simple PI-controller
        // Line follower
        static float lastDist = 0.0;
        static const float interval = 1000.0 / WallSensors::adcPollIntervalMs;
        //static const float targetDist = 80.0;

        float newDist = sensors.left().getCalibrated();
        float error = (newDist - wall_follow_left_distance) * kp + (newDist - lastDist) * kd * interval;
        lastDist = newDist;
        
        //Serial.println(error);

        // Gap on left?
        if(newDist < wall_follow_left_gap_threshold)
        {
          logSensors("GAP LEFT");
          digitalWrite(ledGreen, 1);

          // Move past the gap, so can turn
          if(!justTurned)
          {
            // Need to move wheels into the gap
            forward(60, turn_leadin_speed);
          }
          else
          {
            // Wheels already in gap, just need a bit of clearance
            forward(20, turn_leadin_speed);
          }          

          // Turn
          turn_left_90(turn_speed);

          digitalWrite(ledGreen, 0);

          // Straighten up
          forward(10, turn_leadout_speed);
          
          // Reset PID
          lastDist = sensors.left().getCalibrated();
          justTurned = true;
        }
        else 
        // Blocked ahead
        if(sensors.frontLeft().getCalibrated() > wall_follow_ahead_blocked_threshold)
        {
          //Serial.println(" BLOCKED AHEAD");
          digitalWrite(ledRed, 1);

          // Do we need to do 90 degree or 180 degree?
          if(sensors.right().getCalibrated() < wall_follow_right_gap_threshold)
          {
            // Ok to do 90 degree
            logSensors("BLOCKED");
            forward(30, turn_leadin_speed);
            turn_right_90(turn_speed);
            
            digitalWrite(ledRed, 0);
            
            forward(10, turn_leadout_speed);
          }
          else
          {
            // Need to do 180 degree
            logSensors("CUL-DE-SAC");
            turn_right_180(turn_180_speed);

            digitalWrite(ledRed, 0);
            
            forward(10, turn_leadout_speed);
          }

          // Reset PID
          lastDist = sensors.left().getCalibrated();
          justTurned = true;
        }
        else
        
        {
          // Continue ahead
          logSensors("AHEAD");
          motors.turn(forward_speed, -error);
          justTurned = false;
        }
      
        // Wall follower approach
        // 1) Forward until gap seen on left or blocked ahead
        // 2) Continue to centre of cell (10cm)
        // 3) If gap to left, turn 90deg left then forward 10cm
        // 4) If blocked ahead and open to right, turn 90deg right then foward 10cm
        // 5) else about turn

        //yield();
    }
}
