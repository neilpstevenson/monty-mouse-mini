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

Quadrature_encoder<leftEncoderA, leftEncoderB> encoder_l;
Quadrature_encoder<rightEncoderA, rightEncoderB> encoder_r;

Adafruit_NeoPixel pixels(1, neoPixel, NEO_GRB + NEO_KHZ800);

typedef struct  
{
  unsigned int ahead_max_speed;
  unsigned int left_leadin_distance;
  unsigned int left_leadin_distance_short;
  unsigned int left_leadin_speed;
  unsigned int left_turn_speed;
  unsigned int left_leadout_distance;
  unsigned int left_leadout_speed;
  unsigned int right_leadin_distance;
  unsigned int right_leadin_speed;
  unsigned int right_turn_speed;
  unsigned int right_leadout_distance;
  unsigned int right_leadout_speed;
  unsigned int about_leadin_distance;
  unsigned int about_leadin_speed;
  unsigned int about_turn_speed;
  unsigned int about_leadout_distance;
  unsigned int about_leadout_speed;
} MODE_PROFILE_TABLE;

static MODE_PROFILE_TABLE mode_profiles[] =
{
  // Half-size maze - Red
  {
    // Ahead 
    forward_speed,
    // Left
    80, 40, turn_leadin_speed,
    turn_speed,
    10, turn_leadout_speed,
    // Right
    80, turn_leadin_speed,
    turn_speed,
    5, turn_leadout_speed,
    // About turn
    100, turn_leadin_speed,
    turn_180_speed,
    5, turn_leadout_speed
  },
  // Classic maze - Green
  {
    // Ahead 
    forward_speed,
    // Left
    160, 50, turn_leadin_speed,
    turn_speed,
    100, turn_leadout_speed,
    // Right
    160, turn_leadin_speed,
    turn_speed,
    100, turn_leadout_speed,
    // About turn
    40, turn_leadin_speed,
    turn_180_speed,
    20, turn_leadout_speed
  },
  // Classic maze faster - Blue
  {
    // Ahead 
    forward_speed,
    // Left
    160, 50, turn_leadin_speed * 3 / 2,
    turn_speed * 3 / 2,
    30, turn_leadout_speed * 3 / 2,
    // Right
    30, turn_leadin_speed * 3 / 2,
    turn_speed * 3 / 2,
    90, turn_leadout_speed * 3 / 2,
    // About turn
    40, turn_leadin_speed * 3 / 2,
    turn_180_speed * 3 / 2,
    5, turn_leadout_speed * 3 / 2
  }
};

float wall_follow_error_filtered = 0.0;
float lastDistL = 0.0;
float lastDistR = 0.0;

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
  if(distance > 0)
  {
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
  else
  {
    motors.forwardPower(-speed);
    while(encoder_r.count() * encode_calibrate_r - start_pos_r > distance)
    {
      delay(10);
      //Serial.print("FORWARD ");
      //Serial.print(encoder_r.count() * encode_calibrate_l);  Serial.print("mm ");
      //Serial.println();
      logSensors("REVERSE");
    }
  }
}

float get_wall_follow_error()
{
  static const float interval = 1000.0 / WallSensors::adcPollIntervalMs;

  // Simple P(I)D controller using the nearest wall
  float newDistL = sensors.left().getCalibrated();
  float newDistR = sensors.right().getCalibrated();
  float error;

  // Prefer left wall, if we have one, or the right if this is seeable
  if(newDistL > wall_follow_left_distance_min)
    error = (newDistL - wall_follow_left_distance) * kp + (newDistL - lastDistL) * kd * interval;
  else if(newDistR > wall_follow_left_distance_min)
    error = -((newDistR - wall_follow_left_distance) * kp + (newDistR - lastDistR) * kd * interval);
  else
    // Clamp error if reached a potential gap
    error = 0.0;

  // Simple filter
  wall_follow_error_filtered = wall_follow_error_filtered * (1.0-wall_sensor_filter_ratio) + error * wall_sensor_filter_ratio;

  lastDistL = newDistL;
  lastDistR = newDistR;

  return wall_follow_error_filtered;
}

void reset_PID()
{
  lastDistL = sensors.left().getCalibrated();
  lastDistR = sensors.right().getCalibrated();
  wall_follow_error_filtered = 0.0;
}

void forward_with_wall_follow(int distance, int speed)
{
  // Move forward until distance reached or too near the forward wall
  float start_pos_r = encoder_r.count() * encode_calibrate_r;
  while(encoder_r.count() * encode_calibrate_r - start_pos_r < distance &&
        sensors.frontLeft().getCalibrated() < wall_follow_forward_min_distance)
  {
      logSensors("FORWARD-FOLLOW");
      motors.turn(speed, -get_wall_follow_error());
      sensors.waitForSample();
  }
}

void turn_left_90(int speed)
{
//  motors.stop();
//  delay(500);
 
  // Rotate 90 degrees
  float start_pos_l = encoder_l.count() * encode_calibrate_l;
  float start_pos_r = encoder_r.count() * encode_calibrate_r;
  motors.turn(speed, speed);

  int required_dist_r = (int)(3.14159/2.0 * turning_diameter_mm) - turn_angle_inertia_compensation;
  
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
//  motors.stop();
//  delay(500);
}

void turn_right_90(int speed)
{
//  motors.stop();
//  delay(500);
  // Rotate 90 degrees
  float start_pos_l = encoder_l.count() * encode_calibrate_l;
  float start_pos_r = encoder_r.count() * encode_calibrate_r;
  motors.turn(speed, -speed);

  int required_dist_l = (int)(3.14159/2.0 * turning_diameter_mm) - turn_angle_inertia_compensation;
  
  while(encoder_l.count() * encode_calibrate_l - start_pos_l < 
        (encoder_r.count() * encode_calibrate_r - start_pos_r + required_dist_l))
  {
    delay(10);
    //Serial.print("RIGHT ");
    //Serial.print(encoder_r.count() * encode_calibrate_l);  Serial.print("mm ");
    //Serial.print(encoder_l.count() * encode_calibrate_r);  Serial.print("mm ");
    //Serial.println();
    logSensors("RIGHT90");
  }
//  motors.stop();
//  delay(500);
}

void turn_right_180(int speed)
{
//  motors.stop();
//  delay(500);
  // Rotate 90 degrees
  float start_pos_l = encoder_l.count() * encode_calibrate_l;
  float start_pos_r = encoder_r.count() * encode_calibrate_r;
  motors.turn(0, -speed);

  int required_dist_l = (int)(3.14159 * turning_diameter_mm) - turn_angle_inertia_compensation;
  
  while(encoder_l.count() * encode_calibrate_l - start_pos_l <
        (encoder_r.count() * encode_calibrate_r - start_pos_r + required_dist_l))
  {
    delay(10);
    logSensors("RIGHT180");
  }
//  motors.stop();
//  delay(500);
}


void logSensors(const char *mode)
{
  static bool firstLog = true;
  if(firstLog)
  {
    firstLog = false;
    // Print header
    DebugPort.print("Mode,LeftCal,FrontLCal,FrontRCal,RightCal,EncLCal,EncRCal");
#ifdef LOG_RAW_SENSORS  
    DebugPort.println(",LeftRaw,FrontLRaw,FrontRRaw,RightRaw,EncLRaw,EncRRaw");
#else
    DebugPort.println();
#endif
  }

  DebugPort.print(mode);        DebugPort.print(",");
  
  // Sensors
  DebugPort.print(sensors.left().getCalibrated());       DebugPort.print(",");
  DebugPort.print(sensors.frontLeft().getCalibrated());  DebugPort.print(",");
  DebugPort.print(sensors.frontRight().getCalibrated()); DebugPort.print(",");
  DebugPort.print(sensors.right().getCalibrated());      DebugPort.print(",");
  
  // Encoders
  DebugPort.print(encoder_l.count() * encode_calibrate_l);  DebugPort.print("mm,");
  DebugPort.print(encoder_r.count() * encode_calibrate_r);  DebugPort.print("mm,");

#ifdef LOG_RAW_SENSORS  
  DebugPort.print(sensors.left().getRaw());       DebugPort.print(",");
  DebugPort.print(sensors.frontLeft().getRaw());  DebugPort.print(",");
  DebugPort.print(sensors.frontRight().getRaw()); DebugPort.print(",");
  DebugPort.print(sensors.right().getRaw());      DebugPort.print(",");
  
  // Encoders
  DebugPort.print(encoder_l.count());             DebugPort.print(",");
  DebugPort.print(encoder_r.count());
#endif

  DebugPort.println();
}

void setup() 
{
    Serial.begin(115200);
#ifdef SERIAL_DEBUG_PORT 
    DebugPort.begin(115200);
#endif

    //Scheduler.startLoop(readAdcLoop);
    //poller.start(readAdcLoop);

    pixels.clear(); // Set all neopixels to 'off'

    pinMode(buttonA, INPUT_PULLUP);
    pinMode(buttonB, INPUT_PULLUP);
}

void print_sensors()
{
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
  Serial.print(encoder_l.speed());  //                   Serial.print(" ");

  Serial.println();
}

void testMode()
{
    while(1)
    {
      sensors.waitForSample();
      forward(70, turn_leadin_speed);
      motors.stop(true);
      delay(500);
      forward(-70, turn_leadin_speed);
      motors.stop(true);
      delay(500);
    }
}

void fullWallFollow(int mode)
{
    // Go
    bool justTurned = false;

    // Reset PID
    reset_PID();

    while(true) 
    {
        sensors.waitForSample();
        
        // Gap on left?
        if(sensors.left().getCalibrated() < wall_follow_left_gap_threshold)
        {
          logSensors("GAP LEFT");
          digitalWrite(ledGreen, 1);
  
          // Move past the gap, so can turn
          if(!justTurned)
          {
            // Need to move wheels into the gap
            forward(mode_profiles[mode].left_leadin_distance, mode_profiles[mode].left_leadin_speed);
          }
          else
          {
            // Wheels already in gap, just need a bit of clearance
            forward(mode_profiles[mode].left_leadin_distance_short, mode_profiles[mode].left_leadin_speed);
          }          

          // Turn
          turn_left_90(mode_profiles[mode].left_leadin_speed);

          // Straighten up
          reset_PID();
          forward(mode_profiles[mode].left_leadout_distance, mode_profiles[mode].left_leadout_speed);
          
          digitalWrite(ledGreen, 0);

          // Reset PID
//          reset_PID();
          justTurned = true;
        }
        else 
        // Blocked ahead - need to turn right or u-turn?
        if(sensors.frontLeft().getCalibrated() > wall_follow_ahead_blocked_threshold)
        {
          //Serial.println(" BLOCKED AHEAD");
          // Do we need to do 90 degree or 180 degree?
          if(sensors.right().getCalibrated() < wall_follow_right_gap_threshold)
          {
            // Ok to do 90 degree
            logSensors("GAP RIGHT");
            digitalWrite(ledRed, 1);

  //motors.stop();
  //delay(500);
            forward(mode_profiles[mode].right_leadin_distance, mode_profiles[mode].right_leadin_speed); // Will stop if gets too close
            turn_right_90(mode_profiles[mode].right_turn_speed);

            // We will have a left wall, follow it
            reset_PID();
            forward_with_wall_follow(mode_profiles[mode].right_leadout_distance, mode_profiles[mode].right_leadout_speed);

            digitalWrite(ledRed, 0);
          }
          else
          {
            // Need to do 180 degree
            logSensors("CUL-DE-SAC");
            digitalWrite(ledRed, 1);
            digitalWrite(ledGreen, 1);

  //motors.stop();
  //delay(500);

            forward(mode_profiles[mode].about_leadin_distance, mode_profiles[mode].about_leadin_speed); // Will stop if gets too close
            turn_right_180(mode_profiles[mode].about_turn_speed);

            // We will have a left wall, follow it
            reset_PID();
            forward_with_wall_follow(mode_profiles[mode].about_leadout_distance, mode_profiles[mode].about_leadout_speed);

            digitalWrite(ledGreen, 0);
            digitalWrite(ledRed, 0);
          }

          // Reset PID
//          reset_PID();
          justTurned = true;
        }
        else
        {
          // Continue ahead
          logSensors("AHEAD");
          forward_with_wall_follow(5, mode_profiles[mode].ahead_max_speed);
          //motors.turn(mode_profiles[mode].ahead_max_speed, -get_wall_follow_error());
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

// Very simplistic mode
void simpleWallFollow(int mode)
{
    // Reset PID
    reset_PID();

    int leftTurnCount = 0;
    int basespeed = mode_profiles[mode].ahead_max_speed;
    
    // How many loops for delay?
    int wallFollowerLeftTurnDelay = wall_follower_simple_left_turn_delay;
 
    while(true) 
    {
      sensors.waitForSample();
      
      // Gap on left?
      bool gapLeft = sensors.left().getCalibrated() < wall_follow_simple_left_gap_threshold;
      bool blockedAhead = sensors.frontLeft().getCalibrated() > wall_follow_simple_ahead_blocked_threshold ||
                          sensors.frontRight().getCalibrated() > wall_follow_simple_ahead_blocked_threshold;
      bool blockedRight = sensors.right().getCalibrated() > wall_follow_simple_right_gap_threshold;
    
      float followerError = get_wall_follow_error();

      if(!gapLeft)
      {
        if(!blockedAhead)
        {
          // Keep on following left wall
          motors.turn(basespeed, -followerError);

          digitalWrite(ledGreen, 0); // Left indicator
          digitalWrite(ledRed, 0); // Right indicator

          // We've seen a wall, reset the coast counter
          leftTurnCount = 0;
        }
        else
        {
          // Blocked ahead - turn right
          motors.turn(int(basespeed * 0.5), -int(basespeed * 0.8));

          digitalWrite(ledGreen, 0); // Left indicator
          digitalWrite(ledRed, 1); // Right indicator

          // May need a very short turn
          leftTurnCount = wallFollowerLeftTurnDelay;
        }
      }
      else if(++leftTurnCount <= wallFollowerLeftTurnDelay)
      {
        if(blockedAhead)
        {
          // Blocked ahead - turn right a bit
          motors.turn(int(basespeed * 0.5), -int(basespeed * 0.8));

          digitalWrite(ledGreen, 0); // Left indicator
          digitalWrite(ledRed, 1); // Right indicator
        }
        else
        {
          // Gap on left, but keep going ahead a small amount first
          motors.turn(basespeed, 0); //int(basespeed * 0.1));

          digitalWrite(ledGreen, 1); // Left indicator
          digitalWrite(ledRed, 0); // Right indicator
        }
      }
      else
      {
          // Gap on left, turn into it now
          motors.turn(basespeed, int(basespeed * 0.5));

          digitalWrite(ledGreen, 1); // Left indicator
          digitalWrite(ledRed, 0); // Right indicator
      }
    }
}


int getMode()
{
    // Display starting flashes
    bool triggered = false;
    int mode = 0;
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
        if(++mode > 3)
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

void loop() 
{
    encoder_r.begin(pull_direction::up, resolution::full);
    encoder_l.begin(pull_direction::up, resolution::full);

    int mode = getMode();

    DebugPort.print("running mode: ");
    DebugPort.println(mode);

    sensors.startSensors();

    delay(1500);

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

    // Test mode
    if(mode == 0) 
    {
      simpleWallFollow(mode);
    }
    else if(mode == 3) 
    {
      testMode();
    }
    else
    {
      fullWallFollow(mode-1);
    }
   
}
