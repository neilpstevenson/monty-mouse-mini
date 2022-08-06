#include <PS4Controller.h>
#include <ESP32Servo.h>
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
extern "C" void pairBluetooth(void);

unsigned long accelTimeout;
unsigned long startTime;
unsigned long segmentStartTime;
unsigned long lastRunTime;
unsigned long segmentLoopCount;

Servo steeringServo;
float pidInput = 0.0;
float pidSetpoint = 0.0;
PID pid(PID_Kp, PID_Ki, PID_Kd, &pidInput, &pidSetpoint);
Debounce startFinish(markerLowThreshold, markerHighThreshold, false);
int startFinishCount = 0;
bool manualAutoSteer = false;
DRAGSTER_NVRAM_T hwconfig;

// Common profiles
RunProfile_t allRunProfileStopDone = {
    FLAG_DISPLAY_LAST_RUN | FLAG_IGNORE_MARKERS,
    10000,
    -5,
    0,
    0,
    0
};

// Crazy Max speed
RunProfile_t crazyRunProfileStop = {
    FLAG_IGNORE_MARKERS,
    800,
    -80,
    -30,
    &allRunProfileStopDone,
    0
};
RunProfile_t crazyRunProfileCoast = {
    0,
    2000,
    30,
    30,
    0,
    &crazyRunProfileStop
};
RunProfile_t crazyRunProfileDecel = {
    0,
    400,
    -50,
    30,
    &crazyRunProfileCoast,
    &crazyRunProfileStop
};
RunProfile_t crazyRunProfileSteady = {
    0,
    950, //650,
    128,
    128,
    &crazyRunProfileDecel,
    &crazyRunProfileStop
};
RunProfile_t crazyRunProfile = {
    FLAG_IGNORE_MARKERS,
    200,
    50,
    128,
    &crazyRunProfileSteady,
    &crazyRunProfileStop
};

// Very fast speed
RunProfile_t veryFastRunProfileStop = {
    FLAG_IGNORE_MARKERS,
    800,
    -80,
    -40,
    &allRunProfileStopDone,
    0
};
RunProfile_t veryFastRunProfileCoast = {
    0,
    2000,
    30,
    30,
    0,
    &veryFastRunProfileStop
};
RunProfile_t veryFastRunProfileDecel = {
    0,
    500, //350,
    -50,
    30,
    &veryFastRunProfileCoast,
    &veryFastRunProfileStop
};
RunProfile_t veryFastRunProfileSteady = {
    0,
    1150, //900,
    100,
    100,
    &veryFastRunProfileDecel,
    &veryFastRunProfileStop
};
RunProfile_t veryFastRunProfile = {
    FLAG_IGNORE_MARKERS,
    200,
    30,
    100,
    &veryFastRunProfileSteady,
    &veryFastRunProfileStop
};

// Very fast speed - short course
RunProfile_t veryFastShortRunProfileStop = {
    FLAG_IGNORE_MARKERS,
    800,
    -80,
    -30,
    &allRunProfileStopDone,
    0
};
RunProfile_t veryFastShortRunProfileCoast = {
    0,
    2000,
    30,
    30,
    0,
    &veryFastShortRunProfileStop
};
RunProfile_t veryFastShortRunProfileDecel = {
    0,
    350,
    -50,
    30,
    &veryFastShortRunProfileCoast,
    &veryFastShortRunProfileStop
};
RunProfile_t veryFastShortRunProfileSteady = {
    0,
    900,
    100,
    100,
    &veryFastShortRunProfileDecel,
    &veryFastShortRunProfileStop
};
RunProfile_t veryFastShortRunProfile = {
    FLAG_IGNORE_MARKERS,
    200,
    30,
    100,
    &veryFastShortRunProfileSteady,
    &veryFastShortRunProfileStop
};

// Fast speed
RunProfile_t fastRunProfileStop = {
    FLAG_IGNORE_MARKERS,
    800,
    -40,
    -70,
    &allRunProfileStopDone,
    0
};
RunProfile_t fastRunProfileCoast = {
    0,
    2000,
    15,
    15,
    0,
    &fastRunProfileStop
};
RunProfile_t fastRunProfileDecel = {
    0,
    400,
    -40,
    15,
    &fastRunProfileCoast,
    &fastRunProfileStop
};
RunProfile_t fastRunProfileSteady = {
    0,
    1500, //1450,
    70,
    70,
    &fastRunProfileDecel,
    &fastRunProfileStop
};
RunProfile_t fastRunProfile = {
    FLAG_IGNORE_MARKERS,
    200,
    30,
    70,
    &fastRunProfileSteady,
    &fastRunProfileStop
};

// Medium speed
RunProfile_t mediumRunProfileStop = {
    FLAG_IGNORE_MARKERS,
    700,
    -15,
    -53,
    &allRunProfileStopDone,
    0
};
RunProfile_t mediumRunProfileCoast = {
    0,
    2000,
    13,
    13,
    0,
    &mediumRunProfileStop
};
RunProfile_t mediumRunProfileDecel = {
    0,
    150,
    -15,
    13,
    &mediumRunProfileCoast,
    &mediumRunProfileStop
};
RunProfile_t mediumRunProfileSteady = {
    0,
    2500, //1600,
    52,
    52,
    &mediumRunProfileDecel,
    &mediumRunProfileStop
};
RunProfile_t mediumRunProfile = {
    FLAG_IGNORE_MARKERS,
    150,
    13,
    53,
    &mediumRunProfileSteady,
    &mediumRunProfileStop
};

// Slow speed
RunProfile_t slowRunProfileStop = {
    FLAG_IGNORE_MARKERS,
    1000,
    -20,
    -5,
    &allRunProfileStopDone,
    0
};
RunProfile_t slowRunProfileCoast = {
    0,
    2000,
    10,
    10,
    0,
    &slowRunProfileStop
};
RunProfile_t slowRunProfileDecel = {
    0,
    200,
    -5,
    10,
    &slowRunProfileCoast,
    &slowRunProfileStop
};
RunProfile_t slowRunProfileSteady = {
    0,
    1800,
    40,
    40,
    &slowRunProfileDecel,
    &slowRunProfileStop
};
RunProfile_t slowRunProfile = {
    FLAG_IGNORE_MARKERS,
    200,
    10,
    40,
    &slowRunProfileSteady,
    &slowRunProfileStop
};

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
  tft.setRotation(3);
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
#ifdef MOTORCTRL_TB6612

  ledcSetup(motorRPwmChannel, pwmFreq, pwmResolution);
  ledcAttachPin(gpioMotorAPwm, motorRPwmChannel);
  pinMode(gpioMotorA1, OUTPUT);
  digitalWrite(gpioMotorA1, LOW);
  pinMode(gpioMotorA2, OUTPUT);
  digitalWrite(gpioMotorA2, LOW);

  ledcSetup(motorLPwmChannel, pwmFreq, pwmResolution);
  ledcAttachPin(gpioMotorBPwm, motorLPwmChannel);
  pinMode(gpioMotorB1, OUTPUT);
  digitalWrite(gpioMotorB1, LOW);
  pinMode(gpioMotorB2, OUTPUT);
  digitalWrite(gpioMotorB2, LOW);
#endif //MOTORCTRL_TB6612

#ifdef MOTORCTRL_DRV8833
  ledcSetup(motorA1PwmChannel, pwmFreq, pwmResolution);
  ledcAttachPin(gpioMotorA1, motorA1PwmChannel);
  ledcSetup(motorA2PwmChannel, pwmFreq, pwmResolution);
  ledcAttachPin(gpioMotorA2, motorA2PwmChannel);

  ledcSetup(motorB1PwmChannel, pwmFreq, pwmResolution);
  ledcAttachPin(gpioMotorB1, motorB1PwmChannel);
  ledcSetup(motorB2PwmChannel, pwmFreq, pwmResolution);
  ledcAttachPin(gpioMotorB2, motorB2PwmChannel);
#endif //MOTORCTRL_DRV8833

  // Steering 
  ESP32PWM::allocateTimer(2);
  steeringServo.setPeriodHertz(steeringServoFreq);
  steeringServo.attach(gpioSteeringServo, 500, 2500);

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
        tft.print("Race\nVery Fast\nShort");
        pCurrentRunProfile = &veryFastShortRunProfile;
        break;
      case 9:
        tft.print("Steering\ncalibrate");
        break;
      case 10:
        tft.print("PID\ncalibrate");
        break;
      case 11:
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
        manualAutoSteer = false;
        startSteeringCalibrate();
        break;
      case 10:
        startCalibratePID();
        break;
      case 11:
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
}

void manualControl()
{
  if (!PS4.isConnected()) 
  {
     // Stop motors
    stopAll();
    Serial.println("Waiting for PS4 controller...");
    delay(500);
  }
  else if(PS4.Cross())
  {
    // Switch to Race mode
    stopAll();
    state = STATE_INITIAL;
    return;
  }
  else
  {
    digitalWrite(gpioIlluminationLED, HIGH);
    
    int forward = PS4.LStickY();
    if(forward > 2 || forward < -2)
    {
      setSpeed(forward);
    }
    else
    {
      // Stop
      breakStop();
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
}

void steeringCalibrate()
{
  if (!PS4.isConnected()) 
  {
    stopAll();
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
    stopAll();
    state = STATE_INITIAL;
    return;
  }
  else
  {
    digitalWrite(gpioIlluminationLED, HIGH);
    
    int forward = PS4.LStickY();
    if(forward > 2 || forward < -2)
    {
      setSpeed(forward);
    }
    else
    {
      // Stop
      breakStop();
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

void startCalibratePID()
{
  waitForPS4Controller();
  manualAutoSteer = true;
  displayPID();
  state = STATE_PID_CALIBRATE;
  // Wait for menu button to be released
  while(PS4.Right())
    delay(10);
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
    stopAll();
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
   
    stopAll();
    state = STATE_INITIAL;
    return;
  }
  else
  {
    digitalWrite(gpioIlluminationLED, HIGH);
    
    int forward = PS4.LStickY();
    if(forward > 2 || forward < -2)
    {
      setSpeed(forward);
    }
    else
    {
      // Stop
      breakStop();
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
  state = STATE_PROFILE_RUN;
  startProfileRunSegment();
}

void startProfileRunSegment()
{
  segmentStartTime = millis();
  segmentLoopCount = 0;
  setSpeed(pCurrentRunProfile->startSpeed);
  if(lastRunTime && (pCurrentRunProfile->flags & FLAG_DISPLAY_LAST_RUN))
    displayLastRunTime();
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
    int32_t elapsed = millis() - segmentStartTime;
    if(elapsed >= pCurrentRunProfile->runTimeMs)
    {
      printf("Loop: %ul in %ulmS => %fmS/loop\n", segmentLoopCount, elapsed, ((float)elapsed)/segmentLoopCount);
      
      // Next stage
      if(pCurrentRunProfile->pNext)
      {
        pCurrentRunProfile = pCurrentRunProfile->pNext;
        startProfileRunSegment();
      }
      else
      {
        // Taken too long, abort
        breakStop();
        // Restart
        state = STATE_INITIAL;
      }
    }
    else
    {
      // Accelerate to required new speed
      int32_t newSpeed = (pCurrentRunProfile->startSpeed + ((int32_t)pCurrentRunProfile->endSpeed - (int32_t)pCurrentRunProfile->startSpeed) * elapsed / pCurrentRunProfile->runTimeMs);
      setSpeed(newSpeed);
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
        breakStop();
        // Restart
        state = STATE_INITIAL;
      }
      tft.fillScreen(TFT_RED);
      lastRunTime = 0;
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
        
        if(pCurrentRunProfile->pStop)
        {
          // Run the stop action
          pCurrentRunProfile = pCurrentRunProfile->pStop;
          startProfileStopping();
        }
        else
        {
          // Default stop action
          breakStop();
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
    tft.setCursor(0,0);
    tft.setTextSize(4);
    tft.setTextColor(TFT_BLACK, TFT_BLUE);
    tft.print(lastRunTime);
    tft.print("mS");
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

  default:
    state = STATE_INITIAL;
    break;
  }
  
  delay(3);
}
