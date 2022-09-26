#include "dragster_control.h"

void DragsterControl::readConfig()
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

void DragsterControl::writeConfig()
{
  // Update config in NVRAM
  EEPROM.put(0, hwconfig);
  EEPROM.commit();
}

void DragsterControl::setup() 
{
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

void DragsterControl::initialMenu()
{
  tft.fillScreen(TFT_BLACK);
  tft.setCursor(0,0);
  tft.setTextSize(4);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.print("Mode?");

  static int menuSelected = 0;
  bool forceDisplay = false;

  // Ensure repeat last menu
  if(menuSelected)
  {
    forceDisplay = true;
  }
  
  while(!state)
  {
    // Allow quick peak at stats
    if(PS4.Triangle())
    {
      displayStats();
      while(PS4.Triangle())
        delay(50);
      // Ensure repeat last menu
      forceDisplay = true;
    }
    
    if(selectSwitch.isTriggered(digitalRead(gpioSelectButton) && !PS4.Down()) || forceDisplay)
    {
      if(!forceDisplay)
        ++menuSelected;
      else
        forceDisplay = false;
      tft.fillScreen(TFT_BLACK);
      tft.setCursor(0,0);
      tft.setTextSize(4);
      tft.setTextColor(TFT_WHITE, TFT_BLACK);
      switch(menuSelected)
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
        tft.print("Race\nVV Fast");
        pCurrentRunProfile = &vVFastRunProfile;
        break;
      case 8:
        tft.print("Race\nCrazy Fast");
        pCurrentRunProfile = &crazyRunProfile;
        break;
      case 9:
        tft.print("Test Run");
        pCurrentRunProfile = &testRunProfile;
        break;
      case 10:
        tft.print("Steering\ncalibrate");
        break;
      case 11:
        tft.print("PID\ncalibrate");
        break;
      case 12:
        tft.print("Distance\ncalibrate");
        break;
      case 13:
        tft.print("Pair\nBluetooth");
        break;
      case 14:
        tft.print("Show\nStats");
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
      case 9:
        startProfileDisarmed();
        break;
      case 10:
        startSteeringCalibrate();
        break;
      case 11:
        startCalibratePID();
        break;
      case 12:
        startDistanceCalibrate();
        break;
      case 13:
        pairBluetooth();
        break;
      case 14:
        displayStats();
        // Ensure repeat last menu
        forceDisplay = true;
        break;
      }
    }
    
    delay(20);
  }
}

void DragsterControl::waitForPS4Controller()
{
#ifdef NO_PS4_REQUIRED  
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
#endif //NO_PS4_REQUIRED  
}

void DragsterControl::displayStats()
{
    tft.fillScreen(TFT_BLACK);
    tft.setCursor(0,0);
    tft.setTextSize(2);
    tft.print("  Time   Dist  Speed\n");
    for(int i = 0; i < stats.numStats; i++)
    {
      tft.printf("%6d %6d %6d\n", stats.stats[i].timeMs, stats.stats[i].distanceMm, stats.stats[i].speedMmS );
    }
    while(!PS4.Triangle())
      delay(50);
}

#ifdef HAS_ENCODERS
int DragsterControl::getCalibratedPosnMm()
{
    int64_t positionRaw = positionEncoder.getCount();
    return (int)(positionRaw*encoderDistanceCalibration);
}
#endif

void DragsterControl::startManualControl()
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

void DragsterControl::manualControl()
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

void DragsterControl::startSteeringCalibrate()
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

void DragsterControl::steeringCalibrate()
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


void DragsterControl::startDistanceCalibrate()
{
  waitForPS4Controller();
  tft.fillScreen(TFT_BLACK);
  tft.setCursor(0,0);
  tft.setTextSize(4);
  tft.print("Position:");
  tft.setCursor(0,64);
  tft.print("Speed:");
  manualAutoSteer  = false;
  state = STATE_POSITION_CALIBRATE;
  digitalWrite(gpioIlluminationLED, HIGH);

  // Zero the counter
#ifdef HAS_ENCODERS
  positionEncoder.clearCount();
#endif //HAS_ENCODERS

}

void DragsterControl::positionCalibrate()
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
    // Assess speed and position
    int calibratedPosnMm = getCalibratedPosnMm();
    speed.logDistance(calibratedPosnMm);
    int16_t actualSpeed = speed.getSpeed();
    
    //Serial.printf("Position: %lld (%dmm), Speed=%dmm/S\n", positionRaw, calibratedPosnMm, actualSpeed);
    Serial.printf("%d %d\n", calibratedPosnMm, actualSpeed);
    tft.setCursor(10,32);
    tft.printf("%6dmm", calibratedPosnMm);
    tft.setCursor(10,96);
    tft.printf("%5dmm/S", actualSpeed);
#endif //HAS_ENCODERS

  }
}

void DragsterControl::startCalibratePID()
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

void DragsterControl::displayPID()
{
  // Display current
  tft.fillScreen(TFT_BLACK);
  tft.setCursor(0,0);
  tft.setTextSize(4);
  tft.printf("P: %6.4f\n", pid.mKP);
  tft.printf("I: %6.4f\n", pid.mKI);
  tft.printf("D: %6.4f\n", pid.mKD);
}

void DragsterControl::calibratePID()
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



void DragsterControl::startProfileDisarmed()
{
  waitForPS4Controller();
  tft.fillScreen(TFT_BLACK);
  tft.setCursor(0,0);
  tft.setTextSize(4);
  tft.print("Press []\nto arm");

  if(!PS4.isConnected())
  {
      // Light on - we're using this to trigger the start
      digitalWrite(gpioIlluminationLED, HIGH);
  }

  state = STATE_PROFILE_DISARMED;
}

void DragsterControl::startProfileArmed()
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

void DragsterControl::startProfileRun()
{
  startTime = millis();
  startFinishCount = 0;
  // Zero the counter
#ifdef HAS_ENCODERS
  positionEncoder.clearCount();
#endif //HAS_ENCODERS
  stats.clear();
  state = STATE_PROFILE_RUN;
  startProfileRunSegment();
}

void DragsterControl::startProfileRunSegment()
{
  static char colour = 0;
  segmentStartTime = millis();
  segmentLoopCount = 0;
  motors.setSpeed(pCurrentRunProfile->startPower);

  if(lastRunTime && (pCurrentRunProfile->flags & FLAG_DISPLAY_LAST_RUN))
  {
    displayLastRunTime();
  }
  else
  {
#ifdef DEBUG_ON_DISPLAY    
    tft.setCursor(0,0);
    tft.setTextSize(6);
    // Cycle colours
    if(colour++ & 1)
      tft.setTextColor(TFT_WHITE, TFT_BLACK);
    else
      tft.setTextColor(TFT_WHITE, TFT_BLUE);
    tft.printf("%-12s", pCurrentRunProfile->name);
#endif //DEBUG_ON_DISPLAY    
  }
}

void DragsterControl::startProfileStopping()
{
  state = STATE_PROFILE_STOPPING;
  startProfileRunSegment();
}

void DragsterControl::waitForProfileArmed()
{
    // Wait for Square button 
    // or button with radius sensor over white
    int sensorRadius = analogRead(gpioSensorRadius);
    //Serial.println(sensorRadius);
    if(PS4.Square() || 
       (selectSwitch.isTriggered(digitalRead(gpioSelectButton)) && sensorRadius < markerLowThreshold))
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

void DragsterControl::waitForProfileGo()
{
    startFinishCount = 0;
    
    // Wait for Circle button
    // or manual trigger if no PS4 controller
    int sensorRadius = analogRead(gpioSensorRadius);
    if(PS4.Circle() ||
       (!PS4.isConnected() && sensorRadius > markerHighThreshold))
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
void DragsterControl::profileRun()
{
    segmentLoopCount++;
    
    // Check if we've gone far enough
    int32_t timeNow = millis();
    int32_t elapsed = timeNow - segmentStartTime;

#ifdef HAS_ENCODERS
    // Assess speed and position
    int calibratedPosnMm = getCalibratedPosnMm();
    speed.logDistance(calibratedPosnMm);
    int16_t actualSpeed = speed.getSpeed();
    
    //printf("%d %d\n", calibratedPosnMm, actualSpeed);
#endif //HAS_ENCODERS
    
    if(elapsed >= pCurrentRunProfile->runTimeMs 
#ifdef HAS_ENCODERS
      || (pCurrentRunProfile->endDistance && calibratedPosnMm >= pCurrentRunProfile->endDistance)
      || ((pCurrentRunProfile->flags & FLAG_END_ON_MAX_SPEED) && actualSpeed >= pCurrentRunProfile->targetSpeed)
      || ((pCurrentRunProfile->flags & FLAG_END_ON_MIN_SPEED) && actualSpeed <= pCurrentRunProfile->targetSpeed)
#endif //HAS_ENCODERS
      )
    {

#ifdef HAS_ENCODERS
#ifdef DEBUG_LOOP_SERIAL_INFO    
      printf("Loop: %ul in %ulmS => %fmS/loop\n", segmentLoopCount, elapsed, ((float)elapsed)/segmentLoopCount);
      printf("Ended %s at %dmm, speed %dmm/S\n", pCurrentRunProfile->name, calibratedPosnMm, actualSpeed);
#endif //DEBUG_LOOP_SERIAL_INFO    
      
      // Log the latest stats
      stats.log(timeNow - startTime, calibratedPosnMm, actualSpeed);
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

void DragsterControl::profileSensorActions()
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
      // Log the latest stats
#ifdef HAS_ENCODERS
      lastRunTime = millis() - startTime;
      lastRunDistanceMm = getCalibratedPosnMm();
      stats.log(lastRunTime, lastRunDistanceMm, speed.getSpeed());
#endif //HAS_ENCODERS
      
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
        lastRunDistanceMm = getCalibratedPosnMm();
        // Log the latest stats
        stats.log(lastRunTime, lastRunDistanceMm, speed.getSpeed());
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

void DragsterControl::displayLastRunTime()
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

void DragsterControl::stateMachineRun() 
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
