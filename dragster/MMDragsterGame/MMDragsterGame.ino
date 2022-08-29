#include <PS4Controller.h>
#include <ESP32Servo.h>
#include <ESP32Encoder.h>
#include <TFT_eSPI.h> // TTGO T-Display library

#include "mm_hardware.h"
#include "pid.h"
#include "debounce.h"
#include "dragster_run.h"
#include "nvram.h"
#include "motors.h"

extern TFT_eSPI tft;
extern int value[6];
extern void plotPointers(void);
extern void plotLinear(const char *label, int x, int y);
extern void pairBluetooth(void);

int previousPosition; // mm
unsigned long previousPositionTime; // mS
int actualSpeed;  // mm/S

unsigned long accelTimeout;
unsigned long startTime;
unsigned long segmentStartTime;
unsigned long lastRunTime;
#ifdef HAS_ENCODERS
int lastRunDistanceMm;
#endif //HAS_ENCODERS
unsigned long segmentLoopCount;
Motors motors;
#ifdef HAS_ENCODERS
ESP32Encoder positionEncoder;
#endif //HAS_ENCODERS

Servo steeringServo;
float pidInput = 0.0;
float pidSetpoint = 0.0;
PID pid(PID_Kp, PID_Ki, PID_Kd, &pidInput, &pidSetpoint);
Debounce startFinish(markerLowThreshold, markerHighThreshold, false);
int startFinishCount = 0;
bool manualAutoSteer = false;
DRAGSTER_NVRAM_T hwconfig;

/////////////////////////////////////
// Common profiles
RunProfile_t allRunProfileStopDone = {
    "Done",
    FLAG_DISPLAY_LAST_RUN | FLAG_IGNORE_MARKERS | FLAG_BREAK_STOP,
    10000,
    0,
    0,
    0,
    0,
    0,
    0
};

/////////////////////////////////////
// Crazy Max speed
RunProfile_t crazyRunProfileStop = {
    "Stop",
    FLAG_IGNORE_MARKERS | FLAG_END_ON_MIN_SPEED,
    2000,
    -70,
    -70,
    0,
    0,
    &allRunProfileStopDone,
    0
};
RunProfile_t crazyRunProfileCoast = {
    "Coast S",
    FLAG_COAST_ON_MAX_SPEED,
    1000,
    20,
    20,
    0,
    500,
    0,
    &crazyRunProfileStop
};
RunProfile_t crazyRunProfileDecel = {
    "Decel",
    FLAG_END_ON_MIN_SPEED,
    1000,
    -60,
    -60,
    (courseTimedDistance + courseTargetStoppingDistance),
    1000,
    &crazyRunProfileCoast,
    &crazyRunProfileStop
};
RunProfile_t crazyRunProfileMidCruise = {
    "Cruise",
    0,
    3000,
    128,
    128,
    (courseTimedDistance + courseTargetStoppingDistance)*1/2, // Max 1/2 of track
    0,  // Unlimited speed
    &crazyRunProfileDecel,
    &crazyRunProfileStop
};
RunProfile_t crazyRunProfile = {
    "Accel",
    FLAG_IGNORE_MARKERS,
    200,
    50,
    128,
    (courseTimedDistance + courseTargetStoppingDistance)/3, // Max 1/3rd of track
    0,  // Unlimited
    &crazyRunProfileMidCruise,
    &crazyRunProfileStop
};
/*
RunProfile_t crazyRunProfileStop = {
    "",
    FLAG_IGNORE_MARKERS,
    800,
    -80,
    -30,
    0,
    0,
    &allRunProfileStopDone,
    0
};
RunProfile_t crazyRunProfileCoast = {
    "",
    0,
    2000,
    30,
    30,
    0,
    0,
    0,
    &crazyRunProfileStop
};
RunProfile_t crazyRunProfileDecel = {
    "",
    0,
    400,
    -50,
    30,
    0,
    0,
    &crazyRunProfileCoast,
    &crazyRunProfileStop
};
RunProfile_t crazyRunProfileSteady = {
    "",
    0,
    950, //650,
    128,
    128,
    0,
    0,
    &crazyRunProfileDecel,
    &crazyRunProfileStop
};
RunProfile_t crazyRunProfile = {
    "",
    FLAG_IGNORE_MARKERS,
    200,
    50,
    128,
    0,
    0,
    &crazyRunProfileSteady,
    &crazyRunProfileStop
};
*/

/////////////////////////////////////
// Very fast speed
RunProfile_t veryFastRunProfileStop = {
    "Stop",
    FLAG_IGNORE_MARKERS | FLAG_END_ON_MIN_SPEED,
    2000,
    -70,
    -70,
    0,
    0,
    &allRunProfileStopDone,
    0
};
RunProfile_t veryFastRunProfileCoast = {
    "Coast S",
    FLAG_COAST_ON_MAX_SPEED,
    1000,
    20,
    20,
    0,
    500,
    0,
    &veryFastRunProfileStop
};
RunProfile_t veryFastRunProfileDecel = {
    "Decel",
    FLAG_END_ON_MIN_SPEED,
    1000,
    -70,
    -70,
    (courseTimedDistance + courseTargetStoppingDistance),
    500,
    &veryFastRunProfileCoast,
    &veryFastRunProfileStop
};
RunProfile_t veryFastRunProfileMidCruise = {
    "Cruise",
    FLAG_COAST_ON_MAX_SPEED,
    3000,
    100,
    100,
    (courseTimedDistance + courseTargetStoppingDistance)*3/5, // Max 3/5th of track
    4000,
    &veryFastRunProfileDecel,
    &veryFastRunProfileStop
};
RunProfile_t veryFastRunProfile = {
    "Accel",
    FLAG_IGNORE_MARKERS | FLAG_END_ON_MAX_SPEED,
    200,
    40,
    100,
    (courseTimedDistance + courseTargetStoppingDistance)/3, // Max 1/3rd of track
    4000,
    &veryFastRunProfileMidCruise,
    &veryFastRunProfileStop
};
/*
RunProfile_t veryFastRunProfileStop = {
    "",
    FLAG_IGNORE_MARKERS,
    800,
    -80,
    -40,
    0,
    0,
    &allRunProfileStopDone,
    0
};
RunProfile_t veryFastRunProfileCoast = {
    "",
    0,
    2000,
    30,
    30,
    0,
    0,
    0,
    &veryFastRunProfileStop
};
RunProfile_t veryFastRunProfileDecel = {
    "",
    0,
    500, //350,
    -50,
    30,
    0,
    0,
    &veryFastRunProfileCoast,
    &veryFastRunProfileStop
};
RunProfile_t veryFastRunProfileSteady = {
    "",
    0,
    1150, //900,
    100,
    100,
    0,
    0,
    &veryFastRunProfileDecel,
    &veryFastRunProfileStop
};
RunProfile_t veryFastRunProfile = {
    "",
    FLAG_IGNORE_MARKERS,
    200,
    30,
    100,
    0,
    0,
    &veryFastRunProfileSteady,
    &veryFastRunProfileStop
};
*/

/////////////////////////////////////
// Fast speed
RunProfile_t fastRunProfileStop = {
    "Stop",
    FLAG_IGNORE_MARKERS | FLAG_END_ON_MIN_SPEED,
    2000,
    -70,
    -70,
    0,
    0,
    &allRunProfileStopDone,
    0
};
RunProfile_t fastRunProfileCoast = {
    "Coast S",
    FLAG_COAST_ON_MAX_SPEED,
    1000,
    20,
    20,
    0,
    500,
    0,
    &fastRunProfileStop
};
RunProfile_t fastRunProfileDecel = {
    "Decel",
    FLAG_END_ON_MIN_SPEED,
    1000,
    -70,
    -70,
    (courseTimedDistance + courseTargetStoppingDistance),
    500,
    &fastRunProfileCoast,
    &fastRunProfileStop
};
RunProfile_t fastRunProfileMidCruise = {
    "Cruise",
    FLAG_COAST_ON_MAX_SPEED,
    3000,
    70,
    70,
    (courseTimedDistance + courseTargetStoppingDistance)*3/4, // Max 3/4 of track
    3000,
    &fastRunProfileDecel,
    &fastRunProfileStop
};
RunProfile_t fastRunProfile = {
    "Accel",
    FLAG_IGNORE_MARKERS | FLAG_END_ON_MAX_SPEED,
    200,
    40,
    70,
    (courseTimedDistance + courseTargetStoppingDistance)/4, // Max 1/4 of track
    3000,
    &fastRunProfileMidCruise,
    &fastRunProfileStop
};
/*
RunProfile_t fastRunProfileStop = {
    "",
    FLAG_IGNORE_MARKERS,
    800,
    -40,
    -70,
    0,
    0,
    &allRunProfileStopDone,
    0
};
RunProfile_t fastRunProfileCoast = {
    "",
    0,
    2000,
    15,
    15,
    0,
    0,
    0,
    &fastRunProfileStop
};
RunProfile_t fastRunProfileDecel = {
    "",
    0,
    400,
    -40,
    15,
    0,
    0,
    &fastRunProfileCoast,
    &fastRunProfileStop
};
RunProfile_t fastRunProfileSteady = {
    "",
    0,
    1500, //1450,
    70,
    70,
    0,
    0,
    &fastRunProfileDecel,
    &fastRunProfileStop
};
RunProfile_t fastRunProfile = {
    "",
    FLAG_IGNORE_MARKERS,
    200,
    30,
    70,
    0,
    0,
    &fastRunProfileSteady,
    &fastRunProfileStop
};
*/

/////////////////////////////////////
// Medium speed
RunProfile_t mediumRunProfileStop = {
    "Stop",
    FLAG_IGNORE_MARKERS | FLAG_END_ON_MIN_SPEED,
    2000,
    -50,
    -50,
    0,
    0,
    &allRunProfileStopDone,
    0
};
RunProfile_t mediumRunProfileCoast = {
    "Coast S",
    FLAG_COAST_ON_MAX_SPEED,
    1000,
    20,
    20,
    0,
    500,
    0,
    &mediumRunProfileStop
};
RunProfile_t mediumRunProfileDecel = {
    "Decel",
    FLAG_END_ON_MIN_SPEED,
    1000,
    -50,
    -50,
    (courseTimedDistance + courseTargetStoppingDistance),
    500,
    &mediumRunProfileCoast,
    &mediumRunProfileStop
};
RunProfile_t mediumRunProfileMidCruise = {
    "Cruise",
    FLAG_COAST_ON_MAX_SPEED,
    3000,
    52,
    52,
    (courseTimedDistance + courseTargetStoppingDistance)*3/4, // Max 3/4 of track
    3000,
    &mediumRunProfileDecel,
    &mediumRunProfileStop
};
RunProfile_t mediumRunProfile = {
    "Accel",
    FLAG_IGNORE_MARKERS | FLAG_END_ON_MAX_SPEED,
    200,
    40,
    52,
    (courseTimedDistance + courseTargetStoppingDistance)/4, // Max 1/4 of track
    3000,
    &mediumRunProfileMidCruise,
    &mediumRunProfileStop
};
/*
RunProfile_t mediumRunProfileStop = {
    "",
    FLAG_IGNORE_MARKERS,
    700,
    -15,
    -53,
    0,
    0,
    &allRunProfileStopDone,
    0
};
RunProfile_t mediumRunProfileCoast = {
    "",
    0,
    2000,
    13,
    13,
    0,
    0,
    0,
    &mediumRunProfileStop
};
RunProfile_t mediumRunProfileDecel = {
    "",
    0,
    150,
    -15,
    13,
    0,
    0,
    &mediumRunProfileCoast,
    &mediumRunProfileStop
};
RunProfile_t mediumRunProfileSteady = {
    "",
    0,
    2500, //1600,
    52,
    52,
    0,
    0,
    &mediumRunProfileDecel,
    &mediumRunProfileStop
};
RunProfile_t mediumRunProfile = {
    "",
    FLAG_IGNORE_MARKERS,
    150,
    13,
    53,
    0,
    0,
    &mediumRunProfileSteady,
    &mediumRunProfileStop
};
*/

/////////////////////////////////////
// Slow speed
RunProfile_t slowRunProfileStop = {
    "Stop",
    FLAG_IGNORE_MARKERS | FLAG_END_ON_MIN_SPEED,
    1000,
    -40,
    -40,
    0,
    0,
    &allRunProfileStopDone,
    0
};
RunProfile_t slowRunProfileCoast = {
    "Coast S",
    FLAG_COAST_ON_MAX_SPEED,
    2000,
    20,
    20,
    0,
    500,
    0,
    &slowRunProfileStop
};
RunProfile_t slowRunProfileDecel = {
    "Decel",
    FLAG_END_ON_MIN_SPEED,
    4000,
    -30,
    -30,
    (courseTimedDistance + courseTargetStoppingDistance),
    500,
    &slowRunProfileCoast,
    &slowRunProfileStop
};
RunProfile_t slowRunProfileMidCruise = {
    "Cruise",
    FLAG_COAST_ON_MAX_SPEED,
    4000,
    40,
    40,
    (courseTimedDistance + courseTargetStoppingDistance)*4/5, // Max 4/5th of track
    2000,
    &slowRunProfileDecel,
    &slowRunProfileStop
};
RunProfile_t slowRunProfile = {
    "Accel",
    FLAG_IGNORE_MARKERS | FLAG_END_ON_MAX_SPEED,
    1000,
    40,
    40,
    (courseTimedDistance + courseTargetStoppingDistance)/5, // Max 1/5th of track
    2000,
    &slowRunProfileMidCruise,
    &slowRunProfileStop
};

/*
RunProfile_t slowRunProfileStop = {
    "",
    FLAG_IGNORE_MARKERS,
    1000,
    -20,
    -5,
    0,
    0,
    &allRunProfileStopDone,
    0
};
RunProfile_t slowRunProfileCoast = {
    "",
    0,
    2000,
    10,
    10,
    0,
    0,
    0,
    &slowRunProfileStop
};
RunProfile_t slowRunProfileDecel = {
    "",
    0,
    200,
    -5,
    10,
    0,
    0,
    &slowRunProfileCoast,
    &slowRunProfileStop
};
RunProfile_t slowRunProfileSteady = {
    "",
    0,
    1800,
    40,
    40,
    0,
    0,
    &slowRunProfileDecel,
    &slowRunProfileStop
};
RunProfile_t slowRunProfile = {
    "",
    FLAG_IGNORE_MARKERS,
    200,
    10,
    40,
    0,
    0,
    &slowRunProfileSteady,
    &slowRunProfileStop
};
*/

/////////////////////////////////////
// Test profile
RunProfile_t testRunProfileStop = {
    "",
    FLAG_IGNORE_MARKERS,
    800,
    -80,
    -30,
    0,
    0,
    &allRunProfileStopDone,
    0
};
RunProfile_t testRunProfileCoast = {
    "",
    0,
    2000,
    0,
    0,
    0,
    0,
    0,
    &testRunProfileStop
};
RunProfile_t testRunProfileDecel = {
    "",
    0,
    350,
    -50,
    10,
    courseTimedDistance,
    0,
    &testRunProfileCoast,
    &testRunProfileStop
};
RunProfile_t testRunProfileSteady = {
    "",
    0,
    2000,
    100,
    100,
    courseTimedDistance/2,
    0,
    &testRunProfileDecel,
    &testRunProfileStop
};
RunProfile_t testRunProfile = {
    "",
    FLAG_IGNORE_MARKERS,
    200,
    30,
    100,
    0,
    0,
    &testRunProfileSteady,
    &testRunProfileStop
};

/////////////////////////////////////

RunProfile_t *pCurrentRunProfile;

void readConfig()
{
  EEPROM.get(0, hwconfig);
  if(hwconfig.magic != DRAGSTER_NVRAM_T::nvramMagic)
  {
    // Initialise the data
    hwconfig.steer_centre = default_steeringServoCentre;
    hwconfig.magic = DRAGSTER_NVRAM_T::nvramMagic;
    hwconfig.Kp = PID_Kp;
    hwconfig.Ki = PID_Ki;
    hwconfig.Kd = PID_Kd;
    writeConfig();
    Serial.println("Initialised NVRAM");
  }
  else
  {
    Serial.println("Reading config from NVRAM:");
    Serial.print("  Steering centre = ");
    Serial.println(hwconfig.steer_centre);
  }
  pid.set_tunings(hwconfig.Kp, hwconfig.Ki, hwconfig.Kd);
}

void writeConfig()
{
  // Update config in NVRAM
  EEPROM.put(0, hwconfig);
  EEPROM.commit();
}

void setup() 
{
  Serial.begin(115200);

  // Display
  tft.init();
  tft.setRotation(1);
  tft.fillScreen(TFT_BLACK);

  // Set up analog inputs
  pinMode(gpioSensorStartFinish, INPUT);
  pinMode(gpioSensorRightLine, INPUT);
  pinMode(gpioSensorLeftLine, INPUT);
  pinMode(gpioSensorRadius, INPUT);

  // Buttons
  pinMode(gpioSelectButton, INPUT);
  pinMode(gpioEnterButton, INPUT_PULLUP);

  // LEDs
  pinMode(gpioIlluminationLED, OUTPUT);
  digitalWrite(gpioIlluminationLED, LOW);
  
  // configure motor PWM output
  motors.initHardware();

  // Steering 
  ESP32PWM::allocateTimer(2);
  steeringServo.setPeriodHertz(steeringServoFreq);
  steeringServo.attach(gpioSteeringServo, 500, 2500);

  // Encoder
#ifdef HAS_ENCODERS
  positionEncoder.attachFullQuad(gpioMotorEncoderRA, gpioMotorEncoderRB);
#endif //HAS_ENCODERS

  // PS4 controller
  //PS4.begin("44:17:93:89:84:6a"); // this esp32
  PS4.begin();  // default MAC

  // Get the peristent config data
  EEPROM.begin(sizeof(hwconfig));
  readConfig();
}

void initialMenu()
{
  tft.fillScreen(TFT_BLACK);
  tft.setCursor(0,0);
  tft.setTextSize(4);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.print("Mode?");

  static int menuSelected = 0;

  // Ensure repeat last menu
  if(menuSelected)
    menuSelected--;
  
  Debounce selectSwitch(LOW, HIGH, false);
  Debounce enterSwitch(LOW, HIGH, false);
  while(!state)
  {
    if(selectSwitch.isTriggered(digitalRead(gpioSelectButton) && !PS4.Down()))
    {
      tft.fillScreen(TFT_BLACK);
      tft.setCursor(0,0);
      switch(++menuSelected)
      {
      case 1:
        tft.print("Manual\nControl");
        break;
      case 2:
        tft.print("Manual\nAuto steer");
        break;
      case 3:
        tft.print("Race\nSlow");
        pCurrentRunProfile = &slowRunProfile;
        break;
      case 4:
        tft.print("Race\nMedium");
        pCurrentRunProfile = &mediumRunProfile;
        break;
      case 5:
        tft.print("Race\nFast");
        pCurrentRunProfile = &fastRunProfile;
        break;
      case 6:
        tft.print("Race\nVery Fast");
        pCurrentRunProfile = &veryFastRunProfile;
        break;
      case 7:
        tft.print("Race\nCrazy Fast");
        pCurrentRunProfile = &crazyRunProfile;
        break;
      case 8:
        tft.print("Test Run");
        pCurrentRunProfile = &testRunProfile;
        break;
      case 9:
        tft.print("Steering\ncalibrate");
        break;
      case 10:
        tft.print("PID\ncalibrate");
        break;
      case 11:
        tft.print("Distance\ncalibrate");
        break;
      case 12:
        tft.print("Pair\nBluetooth");
        break;
      default:
        menuSelected = 0;
        tft.print("Mode?");
        break;
      }
    }
    
    if(menuSelected && enterSwitch.isTriggered(digitalRead(gpioEnterButton) && !PS4.Right()))
    {
      // Execute this state
      switch(menuSelected)
      {
      case 1:
        manualAutoSteer = false;
        startManualControl();
        break;
      case 2:
        manualAutoSteer = true;
        startManualControl();
        break;
      case 3:
      case 4:
      case 5:
      case 6:
      case 7:
      case 8:
        startProfileDisarmed();
        break;
      case 9:
        startSteeringCalibrate();
        break;
      case 10:
        startCalibratePID();
        break;
      case 11:
        startDistanceCalibrate();
        break;
      case 12:
        pairBluetooth();
        break;
      }
    }
    
    delay(20);
  }
}

void waitForPS4Controller()
{
    if(!PS4.isConnected()) 
    {
      tft.fillScreen(TFT_BLACK);
      tft.setCursor(0,0);
      tft.setTextSize(4);
      tft.print("Connect\nController");
      
      Serial.println("Waiting for PS4 controller...");
      while(!PS4.isConnected()) 
      {
        delay(100);
      }
      // Flash to acknowledge
      tft.fillScreen(TFT_GREEN);
      delay(250);
      //tft.fillScreen(TFT_BLACK);
    }
    Serial.println("PS4 controller connected.");
}

void startManualControl()
{
  waitForPS4Controller();
 
  // Draw 4 linear meters
  tft.fillScreen(TFT_BLACK);
  byte d = 60;
  plotLinear("Rad", 0, 0);
  plotLinear(" L", 1 * d, 0);
  plotLinear(" R", 2 * d, 0);
  plotLinear("StFi", 3 * d, 0);
  // Ensure we reset pointer positions shown
  value[0] = 0;
  value[1] = 0;
  value[2] = 0;
  value[3] = 0;
  plotPointers();

  state = STATE_MANUAL;
  digitalWrite(gpioIlluminationLED, HIGH);
}

void manualControl()
{
  if (!PS4.isConnected()) 
  {
     // Stop motors
    motors.stopAll();
    Serial.println("Waiting for PS4 controller...");
    delay(500);
  }
  else if(PS4.Cross())
  {
    // Switch to Race mode
    motors.stopAll();
    state = STATE_INITIAL;
    return;
  }
  else
  {
    int forward = PS4.LStickY();
    if(forward > 2 || forward < -2)
    {
      motors.setSpeed(forward);
    }
    else
    {
      // Stop
      motors.breakStop();
    }
    
    // Steering
    if(!manualAutoSteer)
    {
      int steer = steeringGainManual * PS4.RStickX();
      int steerServoPos =  hwconfig.steer_centre - steer;
      steeringServo.write(steerServoPos > 500 ? steerServoPos : 500);
      Serial.printf("Steer: %d\n", steerServoPos);
    }
  
    // Get the current sensor data
    int sensorStartFinish = analogRead(gpioSensorStartFinish);
    int sensorRightLine = analogRead(gpioSensorRightLine);
    int sensorLeftLine = analogRead(gpioSensorLeftLine);
    int sensorRadius = analogRead(gpioSensorRadius);
    
    if(manualAutoSteer)
    {
      // Calculate the steering error
      int steeringError = sensorLeftLine - sensorRightLine;
      pidInput = steeringError;
      float pidOutput = pid.compute();
      //Serial.printf("PID in = %f, out = %f\n", pidInput, pidOutput);
      
      int steer = (int)pidOutput;
      int steerServoPos = (forward >= -2 ? hwconfig.steer_centre + steer : hwconfig.steer_centre - steer);
      steeringServo.write(steerServoPos > 500 ? steerServoPos : 500);
    }
    
    // Display the sensors
    value[0] = map(sensorRadius, 0, 4095, 0, 100);
    value[1] = map(sensorLeftLine, 0, 4095, 0, 100);
    value[2] = map(sensorRightLine, 0, 4095, 0, 100);
    value[3] = map(sensorStartFinish, 0, 4095, 0, 100);
    plotPointers();
    
    // Check start/finish sensor
    if(startFinish.isTriggered(sensorStartFinish))
    {
      ++startFinishCount;
      Serial.printf("Start/Finish triggered %d times\n", startFinishCount);
    }

    // Display position
#ifdef HAS_ENCODERS
    int64_t positionRaw = positionEncoder.getCount();
    Serial.printf("Position: %lld (%dmm)\n", positionRaw, (int)(positionRaw*encoderDistanceCalibration));
#endif //HAS_ENCODERS
  }
}

void startSteeringCalibrate()
{
  waitForPS4Controller();
  tft.fillScreen(TFT_BLACK);
  tft.setCursor(0,0);
  tft.setTextSize(4);
  tft.print("Centre:\n");
  tft.print(hwconfig.steer_centre);
  manualAutoSteer  = false;
  state = STATE_STEER_CALIBRATE;
  // Wait for menu button to be released
  while(PS4.Right())
    delay(10);
  digitalWrite(gpioIlluminationLED, HIGH);
}

void steeringCalibrate()
{
  if (!PS4.isConnected()) 
  {
    motors.stopAll();
    state = STATE_INITIAL;
    return;
  }
  else if(PS4.Left())
  {
    // Move steering centre to right
    hwconfig.steer_centre++;
    tft.fillScreen(TFT_BLACK);
    tft.setCursor(0,0);
    tft.setTextSize(4);
    tft.print("Centre:\n");
    tft.print(hwconfig.steer_centre);
    tft.print("\nA to save");
    delay(100);
  }
  else if(PS4.Right())
  {
    // Move steering centre to left
    hwconfig.steer_centre--;
    tft.fillScreen(TFT_BLACK);
    tft.setCursor(0,0);
    tft.setTextSize(4);
    tft.print("Centre:\n");
    tft.print(hwconfig.steer_centre);
    tft.print("\nA to save");
    delay(100);
  }
  else if(PS4.Triangle())
  {
    // Commit any changes
    writeConfig();
    
    // Switch to Race mode
    motors.stopAll();
    state = STATE_INITIAL;
    return;
  }
  else
  {
    int forward = PS4.LStickY();
    if(forward > 2 || forward < -2)
    {
      motors.setSpeed(forward);
    }
    else
    {
      // Stop
      motors.breakStop();
    }

    // Steering
    if(!manualAutoSteer)
    {
      int steer = PS4.RStickX();
      int steerServoPos =  hwconfig.steer_centre - steer;
      steeringServo.write(steerServoPos > 500 ? steerServoPos : 500);
    }
  
    // Get the current sensor data
    int sensorStartFinish = analogRead(gpioSensorStartFinish);
    int sensorRightLine = analogRead(gpioSensorRightLine);
    int sensorLeftLine = analogRead(gpioSensorLeftLine);
    int sensorRadius = analogRead(gpioSensorRadius);
    
    if(manualAutoSteer)
    {
      // Calculate the steering error
      int steeringError = sensorLeftLine - sensorRightLine;
      pidInput = steeringError;
      float pidOutput = pid.compute();
      //Serial.printf("PID in = %f, out = %f\n", pidInput, pidOutput);
      
      int steer = (int)pidOutput;
      int steerServoPos = (forward >= -2 ? hwconfig.steer_centre + steer : hwconfig.steer_centre - steer);
      steeringServo.write(steerServoPos > 500 ? steerServoPos : 500);
    }
    
    // Print the sensors
    Serial.printf("%d %d %d %d\n", sensorRadius, sensorLeftLine, sensorRightLine, sensorStartFinish);
  }
}


void startDistanceCalibrate()
{
  waitForPS4Controller();
  tft.fillScreen(TFT_BLACK);
  tft.setCursor(0,0);
  tft.setTextSize(4);
  tft.print("Position:\n");
  manualAutoSteer  = false;
  state = STATE_POSITION_CALIBRATE;
  digitalWrite(gpioIlluminationLED, HIGH);

  // Zero the counter
#ifdef HAS_ENCODERS
  positionEncoder.clearCount();
#endif //HAS_ENCODERS

}

void positionCalibrate()
{
  if (!PS4.isConnected()) 
  {
    motors.stopAll();
    state = STATE_INITIAL;
    return;
  }
  else if(PS4.Triangle())
  {
    // Commit any changes
    //writeConfig();
    
    // Switch to Race mode
    motors.stopAll();
    state = STATE_INITIAL;
    return;
  }
  else
  {
    
    int forward = PS4.LStickY();
    if(forward > 2 || forward < -2)
    {
      motors.setSpeed(forward);
    }
    else
    {
      // Stop
      motors.breakStop();
    }

    // Steering
    if(!manualAutoSteer)
    {
      int steer = steeringGainManual * PS4.RStickX();
      int steerServoPos =  hwconfig.steer_centre - steer;
      steeringServo.write(steerServoPos > 500 ? steerServoPos : 500);
    }
  
    // Get the current sensor data
    int sensorStartFinish = analogRead(gpioSensorStartFinish);
    int sensorRightLine = analogRead(gpioSensorRightLine);
    int sensorLeftLine = analogRead(gpioSensorLeftLine);
    int sensorRadius = analogRead(gpioSensorRadius);
    
    if(manualAutoSteer)
    {
      // Calculate the steering error
      int steeringError = sensorLeftLine - sensorRightLine;
      pidInput = steeringError;
      float pidOutput = pid.compute();
      //Serial.printf("PID in = %f, out = %f\n", pidInput, pidOutput);
      
      int steer = (int)pidOutput;
      int steerServoPos = (forward >= -2 ? hwconfig.steer_centre + steer : hwconfig.steer_centre - steer);
      steeringServo.write(steerServoPos > 500 ? steerServoPos : 500);
    }
    
    // Print the sensors
   // Serial.printf("%d %d %d %d\n", sensorRadius, sensorLeftLine, sensorRightLine, sensorStartFinish);

    // Show the distance
#ifdef HAS_ENCODERS
    int64_t positionRaw = positionEncoder.getCount();
    int calibratedPosnMm = (int)(positionRaw*encoderDistanceCalibration);
    Serial.printf("Position: %lld (%dmm)\n", positionRaw, calibratedPosnMm);
    tft.setCursor(20,64);
    tft.print(calibratedPosnMm);
    tft.print("mm     ");
#endif //HAS_ENCODERS

  }
}

void startCalibratePID()
{
  waitForPS4Controller();
  manualAutoSteer = true;
  displayPID();
  state = STATE_PID_CALIBRATE;
  // Wait for menu button to be released
  while(PS4.Right())
    delay(10);
  digitalWrite(gpioIlluminationLED, HIGH);
}

void displayPID()
{
  // Display current
  tft.fillScreen(TFT_BLACK);
  tft.setCursor(0,0);
  tft.setTextSize(4);
  tft.printf("P: %6.4f\n", pid.mKP);
  tft.printf("I: %6.4f\n", pid.mKI);
  tft.printf("D: %6.4f\n", pid.mKD);
}

void calibratePID()
{
  if (!PS4.isConnected()) 
  {
    motors.stopAll();
    state = STATE_INITIAL;
    return;
  }
  else if(PS4.Right())
  {
    pid.mKP += 0.001;
    displayPID();
    delay(100);
  }
  else if(PS4.Left() && pid.mKP > 0.001)
  {
    pid.mKP -= 0.001;
    displayPID();
    delay(100);
  }
  else if(PS4.Up())
  {
    pid.mKD += 0.0002;
    displayPID();
    delay(100);
  }
  else if(PS4.Down() && pid.mKD > 0.0002)
  {
    pid.mKD -= 0.0002;
    displayPID();
    delay(100);
  }
  else if(PS4.Triangle())
  {
    // Commit any changes
    hwconfig.Kp = pid.mKP;
    hwconfig.Ki = pid.mKI;
    hwconfig.Kd = pid.mKD;
    writeConfig();
   
    motors.stopAll();
    state = STATE_INITIAL;
    return;
  }
  else
  {
    int forward = PS4.LStickY();
    if(forward > 2 || forward < -2)
    {
      motors.setSpeed(forward);
    }
    else
    {
      // Stop
      motors.breakStop();
    }

    // Steering
    if(!manualAutoSteer)
    {
      int steer = PS4.RStickX();
      int steerServoPos =  hwconfig.steer_centre - steer;
      steeringServo.write(steerServoPos > 500 ? steerServoPos : 500);
    }
  
    // Get the current sensor data
    int sensorStartFinish = analogRead(gpioSensorStartFinish);
    int sensorRightLine = analogRead(gpioSensorRightLine);
    int sensorLeftLine = analogRead(gpioSensorLeftLine);
    int sensorRadius = analogRead(gpioSensorRadius);
    
    if(manualAutoSteer)
    {
      // Calculate the steering error
      int steeringError = sensorLeftLine - sensorRightLine;
      pidInput = steeringError;
      float pidOutput = pid.compute();
      //Serial.printf("PID in = %f, out = %f\n", pidInput, pidOutput);
      
      int steer = (int)pidOutput;
      int steerServoPos = (forward >= -2 ? hwconfig.steer_centre + steer : hwconfig.steer_centre - steer);
      steeringServo.write(steerServoPos > 500 ? steerServoPos : 500);
    }
    
    // Print the sensors
    //Serial.printf("%d %d %d %d\n", sensorRadius, sensorLeftLine, sensorRightLine, sensorStartFinish);
  }
}



void startProfileDisarmed()
{
  waitForPS4Controller();
  tft.fillScreen(TFT_BLACK);
  tft.setCursor(0,0);
  tft.setTextSize(4);
  tft.print("Press []\nto arm");

  state = STATE_PROFILE_DISARMED;
}

void startProfileArmed()
{
  waitForPS4Controller();
  tft.fillScreen(TFT_ORANGE);
  tft.setCursor(0,0);
  tft.setTextSize(4);
  tft.setTextColor(TFT_BLACK, TFT_ORANGE);
  tft.print("Press O\nto start");
  tft.setTextColor(TFT_GREEN, TFT_BLACK);

  state = STATE_PROFILE_ARMED;
}

void startProfileRun()
{
  startTime = millis();
  startFinishCount = 0;
  // Zero the counter
#ifdef HAS_ENCODERS
  positionEncoder.clearCount();
#endif //HAS_ENCODERS
  state = STATE_PROFILE_RUN;
  startProfileRunSegment();
}

void startProfileRunSegment()
{
  segmentStartTime = millis();
  segmentLoopCount = 0;
  motors.setSpeed(pCurrentRunProfile->startPower);
  if(lastRunTime && (pCurrentRunProfile->flags & FLAG_DISPLAY_LAST_RUN))
  {
    displayLastRunTime();
  }
  else
  {
    tft.setCursor(0,0);
    tft.setTextSize(6);
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.printf("%-20s", pCurrentRunProfile->name);
  }
}

void startProfileStopping()
{
  state = STATE_PROFILE_STOPPING;
  startProfileRunSegment();
}

void waitForProfileArmed()
{
    // Wait for Square button
    if(PS4.Square())
    {
      // Light on
      digitalWrite(gpioIlluminationLED, HIGH);
      startProfileArmed();
    }
    else if(PS4.Triangle())
    {
      // Switch to manual mode
      startManualControl();
    }
}

void waitForProfileGo()
{
    startFinishCount = 0;
    
    // Wait for Circle button
    if(PS4.Circle())
    {
      startProfileRun();
      //tft.fillScreen(TFT_GREEN);
    }
    else
    {
      // Get initial steer
      profileSensorActions();

      static int loopCount;
      if(loopCount++ % 20 == 0)
      {
        // Display steering error
        float pidOutput = pid.output();
        tft.setCursor(0,80);
        tft.setTextSize(4);
        if(pidOutput < 10.0 && pidOutput > -10.0)
          tft.setTextColor(TFT_GREEN, TFT_ORANGE);
        else
          tft.setTextColor(TFT_RED, TFT_ORANGE);
        tft.printf("%8.2f",pidOutput);
        tft.setTextColor(TFT_GREEN, TFT_BLACK);

        // Recompute PID to avoid display code disrupting the I & D parts on next cycle
        pid.compute();
      }
    }
}

// Accelerate up to speed
void profileRun()
{
    segmentLoopCount++;
    
    // Check if we've gone far enough
    int32_t timeNow = millis();
    int32_t elapsed = timeNow - segmentStartTime;

#ifdef HAS_ENCODERS
    int64_t positionRaw = positionEncoder.getCount();
    int calibratedPosnMm = (int)(positionRaw*encoderDistanceCalibration);

    // Assess speed
    int32_t elapsedSpeed = timeNow - previousPositionTime;
    if(elapsedSpeed >= speedAssessmentTime)
    {
        actualSpeed = (calibratedPosnMm - previousPosition) * 1000 / elapsedSpeed;
        
        previousPositionTime = timeNow;
        previousPosition = calibratedPosnMm;

        printf("%d %d\n", calibratedPosnMm, actualSpeed);
    }
#endif //HAS_ENCODERS
    
    if(elapsed >= pCurrentRunProfile->runTimeMs 
#ifdef HAS_ENCODERS
      || (pCurrentRunProfile->endDistance && calibratedPosnMm >= pCurrentRunProfile->endDistance)
      || ((pCurrentRunProfile->flags & FLAG_END_ON_MAX_SPEED) && actualSpeed >= pCurrentRunProfile->targetSpeed)
      || ((pCurrentRunProfile->flags & FLAG_END_ON_MIN_SPEED) && actualSpeed <= pCurrentRunProfile->targetSpeed)
#endif //HAS_ENCODERS
      )
    {
      printf("Loop: %ul in %ulmS => %fmS/loop\n", segmentLoopCount, elapsed, ((float)elapsed)/segmentLoopCount);
#ifdef HAS_ENCODERS
      printf("Ended %s at %dmm, speed %dmm/S\n", pCurrentRunProfile->name, calibratedPosnMm, actualSpeed);
#endif //HAS_ENCODERS
      
      // Next stage
      if(pCurrentRunProfile->pNext)
      {
        pCurrentRunProfile = pCurrentRunProfile->pNext;
        startProfileRunSegment();
      }
      else
      {
        // Taken too long, abort
        motors.breakStop();
        // Light off
        digitalWrite(gpioIlluminationLED, LOW);
        // Restart
        state = STATE_INITIAL;
      }
    }
#ifdef HAS_ENCODERS
    else if ((pCurrentRunProfile->flags & FLAG_COAST_ON_MAX_SPEED) && actualSpeed >= pCurrentRunProfile->targetSpeed)
    {
      motors.setSpeed(0);
      profileSensorActions();
    }
#endif //HAS_ENCODERS
    else if (pCurrentRunProfile->flags & FLAG_BREAK_STOP)
    {
      motors.breakStop();
      profileSensorActions();
    }
    else
    {
      // Accelerate to required new speed
      int32_t newSpeed = (pCurrentRunProfile->startPower + ((int32_t)pCurrentRunProfile->endPower - (int32_t)pCurrentRunProfile->startPower) * elapsed / pCurrentRunProfile->runTimeMs);
      motors.setSpeed(newSpeed);
      profileSensorActions();
    }
}

void profileSensorActions()
{
   // Get the current sensor data
  int sensorStartFinish = analogRead(gpioSensorStartFinish);
  int sensorRightLine = analogRead(gpioSensorRightLine);
  int sensorLeftLine = analogRead(gpioSensorLeftLine);
  int sensorRadius = analogRead(gpioSensorRadius);

  // Calculate the steering error
  int steeringError = sensorLeftLine - sensorRightLine;
  pidInput = steeringError;
  float pidOutput = pid.compute();
  //Serial.printf("PID in = %f (%d-%d), out = %f\n", pidInput, sensorLeftLine, sensorRightLine, pidOutput);

  int steer = (int)pidOutput;
  int steerServoPos =  hwconfig.steer_centre + steer;
  steeringServo.write(steerServoPos);
  steeringServo.write(steerServoPos > 500 ? steerServoPos : 500);

  // End or abort run?
  if( !(pCurrentRunProfile->flags & FLAG_IGNORE_MARKERS) && state != STATE_PROFILE_ARMED)
  {
    // Check to abort run if off line
    if( sensorRightLine >= maxLineDetectorThreshold && sensorLeftLine >= maxLineDetectorThreshold)
    {
      // We've lost all contact wtih the line, just abort stop
      if(pCurrentRunProfile->pStop)
      {
        // Run the stop action
        pCurrentRunProfile = pCurrentRunProfile->pStop;
        startProfileStopping();
      }
      else
      {
        // Default stop action
        motors.breakStop();
        // Restart
        state = STATE_INITIAL;
      }
      tft.fillScreen(TFT_RED);
      lastRunTime = 0;
#ifdef HAS_ENCODERS
      lastRunDistanceMm = 0;
#endif //HAS_ENCODERS
    }
    else
    {
      // Check start/finish sensor
      if(startFinish.isTriggered(sensorStartFinish))
      {
        ++startFinishCount;
        //Serial.printf("Start/Finish triggered %d times\n", startFinishCount);
      }
    
      // Reached end maker?
      if(startFinishCount >= startFinishCountLimit)
      {
        lastRunTime = millis() - startTime;
#ifdef HAS_ENCODERS
        int64_t positionRaw = positionEncoder.getCount();
        lastRunDistanceMm = (int)(positionRaw*encoderDistanceCalibration);
#endif //HAS_ENCODERS
        
        if(pCurrentRunProfile->pStop)
        {
          // Run the stop action
          pCurrentRunProfile = pCurrentRunProfile->pStop;
          startProfileStopping();
        }
        else
        {
          // Default stop action
          motors.breakStop();
          // Restart
          state = STATE_INITIAL;
        }
      }
    }
  }
}

void displayLastRunTime()
{
    // Display end result
    tft.fillScreen(TFT_BLUE);
    // Show elapsed time
    tft.setCursor(10,8);
    tft.setTextSize(6);
    tft.setTextColor(TFT_BLACK, TFT_BLUE);
    tft.print(lastRunTime);
    tft.print("mS");
    // Show the distance
#ifdef HAS_ENCODERS
    tft.setCursor(10,60);
    tft.print(lastRunDistanceMm);
    tft.print("mm");
#endif //HAS_ENCODERS
    tft.setTextColor(TFT_GREEN, TFT_BLACK);
    
}

void loop() 
{
  switch(state)
  {
  case STATE_INITIAL:
    initialMenu();
    break;
  case STATE_MANUAL:
    manualControl();
    break;
  case STATE_PROFILE_DISARMED:
    waitForProfileArmed();
    break;
  case STATE_PROFILE_ARMED:
    waitForProfileGo();
    break;
  case STATE_PROFILE_RUN:
  case STATE_PROFILE_STOPPING:
    profileRun();
    break;
    
  case STATE_STEER_CALIBRATE:
    steeringCalibrate();
    break;
  case STATE_PID_CALIBRATE:
    calibratePID();
    break;
  case STATE_POSITION_CALIBRATE:
    positionCalibrate();
    break;
    
  default:
    state = STATE_INITIAL;
    break;
  }
  
  delay(3);
}
