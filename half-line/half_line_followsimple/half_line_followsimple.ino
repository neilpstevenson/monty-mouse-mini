//#include "mbed.h"
//using namespace mbed;
//using namespace rtos;
#include <Adafruit_NeoPixel.h>
#include "sensors.h"
#include "motors.h"
#include "quadrature.h"
#include "config.h"

LineSensors sensors;
Motors motors;

Quadrature_encoder<leftEncoderA, leftEncoderB> encoder_l;
Quadrature_encoder<rightEncoderA, rightEncoderB> encoder_r;

Adafruit_NeoPixel pixels(1, neoPixel, NEO_GRB + NEO_KHZ800);

float lastDist = 0.0;

void logSensors(const char *mode, float error, bool logNew = false)
{
  static bool firstLog = true;
  static unsigned long startTime = 0;
  static unsigned int log_count = 0;

  if(firstLog || logNew)
  {
    firstLog = false;
    log_count = 0;
    startTime = millis();

    // Print header
    DebugPort.print("time,Weighted,Err,Left,LMid,LCent,RCent,RMid,Right,MotorL,MotorR,EncL,EncR");
#ifdef LOG_RAW_SENSORS  
    DebugPort.println(",rawL,rawLMid,rawLCent,rawRCent,rawRMid,rawR,rawEncL,rawEncL");
#else
    DebugPort.println();
#endif
  }

  // Limit log frequency
  if(++log_count % log_frequency == 0)
  {
    DebugPort.print(millis() - startTime);                DebugPort.print(",");

    //DebugPort.print(mode);        DebugPort.print(",");
    
    // Sensors
    DebugPort.print(sensors.getWeightedCentreOffset());    DebugPort.print(",");
    DebugPort.print(error);                                DebugPort.print(", ");
    
    DebugPort.print(sensors.left().getCalibrated());       DebugPort.print(",");
    DebugPort.print(sensors.midLeft().getCalibrated());    DebugPort.print(",");
    DebugPort.print(sensors.centreLeft().getCalibrated()); DebugPort.print(",");
    DebugPort.print(sensors.centreRight().getCalibrated());DebugPort.print(",");
    DebugPort.print(sensors.midRight().getCalibrated());   DebugPort.print(",");
    DebugPort.print(sensors.right().getCalibrated());      DebugPort.print(", ");

    DebugPort.print(motors.left.getPower());               DebugPort.print(",");
    DebugPort.print(motors.right.getPower());              DebugPort.print(", ");
    
    // Encoders
    DebugPort.print(encoder_l.count() * encode_calibrate_l);  DebugPort.print("mm,");
    DebugPort.print(encoder_r.count() * encode_calibrate_r);  DebugPort.print("mm");

  #ifdef LOG_RAW_SENSORS  
    DebugPort.print(", ");
    DebugPort.print(sensors.left().getRaw());       DebugPort.print(",");
    DebugPort.print(sensors.midLeft().getRaw());    DebugPort.print(",");
    DebugPort.print(sensors.centreLeft().getRaw()); DebugPort.print(",");
    DebugPort.print(sensors.centreRight().getRaw());DebugPort.print(",");
    DebugPort.print(sensors.midRight().getRaw());   DebugPort.print(",");
    DebugPort.print(sensors.right().getRaw());      DebugPort.print(", ");
    
    // Encoders
    DebugPort.print(encoder_l.count());             DebugPort.print(",");
    DebugPort.print(encoder_r.count());
  #endif

    DebugPort.println();
  }
}

float get_line_follow_error()
{
  static const float interval = 1000.0 / LineSensors::adcPollIntervalMs;

  // Simple P(I)D controller using 4 centre sensors, weighted at 3:1
  float lineDist = sensors.getWeightedCentreOffset();
  float error = lineDist * kp + (lineDist - lastDist) * kd * interval;
/*
  DebugPort.print(lineDist);        DebugPort.print(",");
  DebugPort.print(kp);              DebugPort.print(",");
  DebugPort.print(kd * interval);   DebugPort.print(",");
  DebugPort.print(error);           
  DebugPort.println();
*/
  lastDist = lineDist;

  //DebugPort.print("PID in=");
  //DebugPort.print(lineDist);
  //DebugPort.print(", err=");
  //DebugPort.println(error);

  return error * forward_speed;
}

void reset_PID()
{
  get_line_follow_error();  // Since we're not accumulating anything apart from last reading
}

void simple_line_follow(int distance, int speed)
{
  unsigned long startTime = millis();

  sensors.waitForSample();
  reset_PID();
  logSensors("START", 0, true);

  // Move forward until distance reached or too near the forward wall
  float start_pos_r = encoder_r.count() * encode_calibrate_r;
  while(encoder_r.count() * encode_calibrate_r - start_pos_r < distance)
  {
      sensors.waitForSample();
      float error = get_line_follow_error();
      motors.turn(speed, error);
      logSensors("FOLLOW", error);
  }

  unsigned long endTime = millis();

  motors.stop();

  DebugPort.print("Total time taken: ");
  DebugPort.print((endTime - startTime)/1000.0);
  DebugPort.println("secs");
}




void setup() 
{
    Serial.begin(115200);   // USB
    Serial1.begin(115200);  // Logger/bluetooth
//#ifdef SERIAL_DEBUG_PORT 
//    DebugPort.begin(115200);
//#endif

    //Scheduler.startLoop(readAdcLoop);
    //poller.start(readAdcLoop);

    pixels.clear(); // Set all neopixels to 'off'

    pinMode(buttonA, INPUT_PULLUP);
    pinMode(buttonB, INPUT_PULLUP);
}


int getMode()
{
    // Display starting flashes
    bool triggered = false;
    static int mode = 0;
    DebugPort.println("ready for start");

    for(int count = 0; !triggered; count++)
    {
      if(count % 30 == 0)
      {
        // On - Red=0 (1/2 size), Green=1 (full), Blue=2 (Full faster), 3 = cyan (Test) 
        pixels.setPixelColor(0, pixels.Color(mode == 0 ? 8 : 0, mode == 1 || mode == 3 ? 8 : 0, mode == 2 || mode == 3 ? 8 : 0));
        pixels.show();
      }
      else if(count % 30 == 20)
      {
        // Off
        pixels.setPixelColor(0, pixels.Color(0, 0, 0));
        pixels.show();
      }

      if(digitalRead(buttonA) == 0)
      {
        if(++mode > 4)
          mode = 0;
          
        DebugPort.print("mode ");
        DebugPort.println(mode);
        
        // On - Red=0 (1/2 size), Green=1 (full), Blue=2 (Full faster), 3 = cyan (Test) 
        pixels.setPixelColor(0, pixels.Color(mode == 0 ? 8 : 0, mode == 1 || mode == 3 ? 8 : 0, mode == 2 || mode == 3 ? 8 : 0));
        pixels.show();

        delay(100);
        while(digitalRead(buttonA) == 0)
          delay(100);
      }

      if(digitalRead(buttonB) == 0)
      {
        triggered = true;
        DebugPort.println("go");

        // Show starting - Solid white
        pixels.setPixelColor(0, pixels.Color(8, 8, 8));
        pixels.show();
      
        delay(100);
        while(digitalRead(buttonB) == 0)
          delay(100);
      }
      
      delay(20);
    }

    return mode;
}

void calibrateSensors()
{
  // Rotate for 1 second, which should travel over a while line completely
  motors.turn(-64); // Rotate anti-clockwise at exacly this speed
  sensors.calibrateSensors(1000);
  motors.stop();

  // Print results
  DebugPort.println("Calibration min/max:");
  DebugPort.print(sensors.left().calibrationMin());       DebugPort.print(","); DebugPort.print(sensors.left().calibrationMax());  DebugPort.print(", ");
  DebugPort.print(sensors.midLeft().calibrationMin());    DebugPort.print(","); DebugPort.print(sensors.midLeft().calibrationMax());  DebugPort.print(", ");
  DebugPort.print(sensors.centreLeft().calibrationMin()); DebugPort.print(","); DebugPort.print(sensors.centreLeft().calibrationMax());  DebugPort.print(", ");
  DebugPort.print(sensors.centreRight().calibrationMin());DebugPort.print(","); DebugPort.print(sensors.centreRight().calibrationMax());  DebugPort.print(", ");
  DebugPort.print(sensors.midRight().calibrationMin());   DebugPort.print(","); DebugPort.print(sensors.midRight().calibrationMax());  DebugPort.print(", ");
  DebugPort.print(sensors.right().calibrationMin());      DebugPort.print(","); DebugPort.print(sensors.right().calibrationMax());
  DebugPort.println();
}

void testSensors()
{
  while(1)
  {
    sensors.waitForSample();
    logSensors("TEST SENSORS", 0);
  }
 }

void loop() 
{
    encoder_r.begin(pull_direction::up, resolution::full);
    encoder_l.begin(pull_direction::up, resolution::full);

    int mode = getMode();

    DebugPort.print("running mode: ");
    DebugPort.println(mode);

    sensors.startSensors();

/*
    // Take a sample and use the left/right readings as midpoint between the two initial walls
    // We will use this as a centre line when following subsequent walls
    sensors.calibrateLRSensors();

    // Confirm the calibrated readings
    DebugPort.println("Calibration:");
    DebugPort.println("LeftCal,LeftRaw, FrontLCal,FrontLRaw, FrontRCal,FrontRRaw, RightCal,RightRaw");
    DebugPort.print(sensors.left().getCalibrated());      DebugPort.print(",");
    DebugPort.print(sensors.left().getRaw());             DebugPort.print(", ");
    DebugPort.print(sensors.frontLeft().getCalibrated()); DebugPort.print(",");
    DebugPort.print(sensors.frontLeft().getRaw());        DebugPort.print(", ");
    DebugPort.print(sensors.frontRight().getCalibrated());DebugPort.print(",");
    DebugPort.print(sensors.frontRight().getRaw());       DebugPort.print(", ");
    DebugPort.print(sensors.right().getCalibrated());     DebugPort.print(",");
    DebugPort.print(sensors.right().getRaw());
    DebugPort.println();

    delay(500);
*/

    // Test mode
    if(mode == 0) 
    {
      delay(500);
      simple_line_follow(1000,forward_speed);
    }
    else if(mode == 1)
    {
      delay(200);
      simple_line_follow(500000,forward_speed);
    }
    else if(mode == 2)
    {
      delay(500);
      calibrateSensors();
    }
    else if(mode == 3)
    {
      testSensors();
    }
    else if(mode == 4)
    {
      motors.left.setPower(-64);
      delay(500);
      motors.left.setPower(0);
      delay(500);
      motors.left.setPower(64);
      delay(500);
      motors.left.setPower(0);
      delay(500);
      motors.right.setPower(-64);
      delay(500);
      motors.right.setPower(0);
      delay(500);
      motors.right.setPower(64);
      delay(500);
      motors.right.setPower(0);
      delay(500);
    }
  
}
