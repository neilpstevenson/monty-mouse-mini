#include "dragster_control.h"

void DragsterControl::readConfig()
{
  EEPROM.get(NVRAM_DRAGSTER_NVRAM, hwconfig);
  if(hwconfig.magic != DRAGSTER_NVRAM_T::nvramMagic)
  {
    // Initialise the data
    hwconfig.magic = DRAGSTER_NVRAM_T::nvramMagic;
    
    hwconfig.steer_centre = default_steeringServoCentre;
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
  EEPROM.put(NVRAM_DRAGSTER_NVRAM, hwconfig);
  EEPROM.commit();
}

void DragsterControl::readRunParameters(DRAGSTER_NVRAM_RACE_CONFIG_T &runParams)
{
  EEPROM.get(NVRAM_DRAGSTER_NVRAM_RACE_CONFIG, runParams);
  if(runParams.magic != DRAGSTER_NVRAM_RACE_CONFIG_T::nvramMagic)
  {
    // Initialise the data
    runParams.magic = DRAGSTER_NVRAM_RACE_CONFIG_T::nvramMagic;

    // Course dimensions
    runParams.timed_distance = courseTimedDistance;
    runParams.target_stopping_distance = courseTargetStoppingDistance;

    // Initial acceleration phase
    runParams.initial_power = 40;
    runParams.acceleration_distance = 500;
    runParams.max_power = 105;
    // Cruise
    runParams.target_top_speed = 4300;
    // Slowdown
    runParams.slowdown_distance_from_end = 1400;
    runParams.initial_deceleration_power = -20;
    runParams.max_deceleration_power = -90;
    runParams.target_end_speed = 3000;
    // Stop
    runParams.break_deceleration_power = -55;
    
    writeRunParameters(runParams);
    Serial.println("Initialised NVRAM Run Params");
  }
  else
  {
    Serial.println("Reading Run Params from NVRAM");
  }
}

void DragsterControl::writeRunParameters(DRAGSTER_NVRAM_RACE_CONFIG_T &runParams)
{
  // Update config in NVRAM
  EEPROM.put(NVRAM_DRAGSTER_NVRAM_RACE_CONFIG, runParams);
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
  EEPROM.begin(NVRAM_MAX_SIZE);
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
        tft.print("Race\nUse Config");
        pCurrentRunProfile = &vConfigFastRunProfile;
        // Load up config params
        {
          DRAGSTER_NVRAM_RACE_CONFIG_T runParams;
          readRunParameters(runParams);

          // Configure the run
          // Start
          vConfigFastRunProfile.startPower = runParams.initial_power;
          vConfigFastRunProfile.endPower = runParams.max_power;
          vConfigFastRunProfile.endDistance = runParams.acceleration_distance;
          vConfigFastRunProfile.targetSpeed = runParams.target_top_speed;
          // Accel
          vConfigFastRunProfile.pNext->startPower = runParams.max_power;
          vConfigFastRunProfile.pNext->endPower = runParams.max_power;
          vConfigFastRunProfile.pNext->endDistance = (runParams.timed_distance - runParams.slowdown_distance_from_end);
          vConfigFastRunProfile.pNext->targetSpeed = runParams.target_top_speed;
          // Mid Cruise
          vConfigFastRunProfile.pNext->pNext->startPower = runParams.max_power / 2;
          vConfigFastRunProfile.pNext->pNext->endPower = runParams.max_power / 2;
          vConfigFastRunProfile.pNext->pNext->endDistance = (runParams.timed_distance - runParams.slowdown_distance_from_end);
          vConfigFastRunProfile.pNext->pNext->targetSpeed = runParams.target_top_speed;
          // Decel
          vConfigFastRunProfile.pNext->pNext->pNext->startPower = runParams.initial_deceleration_power;
          vConfigFastRunProfile.pNext->pNext->pNext->endPower = runParams.max_deceleration_power;
          vConfigFastRunProfile.pNext->pNext->pNext->endDistance = (runParams.timed_distance + runParams.target_stopping_distance);
          vConfigFastRunProfile.pNext->pNext->pNext->targetSpeed = runParams.target_end_speed;
          // Coast to line
          vConfigFastRunProfile.pNext->pNext->pNext->pNext->endDistance = (runParams.timed_distance + runParams.target_stopping_distance);
          vConfigFastRunProfile.pNext->pNext->pNext->pNext->targetSpeed = runParams.target_end_speed;
          // Stop
          vConfigFastRunProfile.pStop->startPower = runParams.break_deceleration_power;
          vConfigFastRunProfile.pStop->endPower = runParams.break_deceleration_power;
        }
        break;
      case 4:
        tft.print("Race\nSlow");
        pCurrentRunProfile = &slowRunProfile;
        break;
      case 5:
        tft.print("Race\nMedium");
        pCurrentRunProfile = &mediumRunProfile;
        break;
      case 6:
        tft.print("Race\nFast");
        pCurrentRunProfile = &fastRunProfile;
        break;
      case 7:
        tft.print("Race\nVery Fast");
        pCurrentRunProfile = &veryFastRunProfile;
        break;
      case 8:
        tft.print("Race\nVV Fast");
        pCurrentRunProfile = &vVFastRunProfile;
        break;
      case 9:
        tft.print("Race\nCrazy Fast");
        pCurrentRunProfile = &crazyRunProfile;
        break;
      case 10:
        tft.print("Test Run");
        pCurrentRunProfile = &testRunProfile;
        break;
      case 11:
        tft.print("Steering\ncalibrate");
        break;
      case 12:
        tft.print("PID\ncalibrate");
        break;
      case 13:
        tft.print("Distance\ncalibrate");
        break;
      case 14:
        tft.print("Pair\nBluetooth");
        break;
      case 15:
        tft.print("Edit\nRun params");
        break;
      case 16:
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
      case 10:
        startProfileDisarmed();
        break;
      case 11:
        startSteeringCalibrate();
        break;
      case 12:
        startCalibratePID();
        break;
      case 13:
        startDistanceCalibrate();
        break;
      case 14:
        pairBluetooth();
        break;
      case 15:
        editRunConfigParams();
        // Ensure repeat last menu
        forceDisplay = true;
        break;
      case 16:
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

void DragsterControl::editRunConfigParams()
{
    int param = 0;
    bool changed = true;
    int16_t *pParam;
    int16_t increment = 0;
    const char *name = 0;
    const char *unit = 0;

    // Wait for key releases before we start
    while(PS4.Up() || PS4.Down() || PS4.Left() || PS4.Right()) 
    {
      delay(10);
    }

    DRAGSTER_NVRAM_RACE_CONFIG_T runParams;
    readRunParameters(runParams);
    
    while(!PS4.Triangle())
    {
      switch(param)
      {
        case 0: pParam = &runParams.timed_distance;
                name = "Timed\nDistance";
                unit = "mm";
                increment = 10;
                break;
        case 1: pParam = &runParams.target_stopping_distance;
                name = "Stopping\nDistance";
                unit = "mm";
                increment = 10;
                break;
        case 2: pParam = &runParams.initial_power;
                name = "Initial\nPower";
                unit = "";
                increment = 1;
                break;
        case 3: pParam = &runParams.acceleration_distance;
                name = "Acceleration\nDistance";
                unit = "mm";
                increment = 10;
                break;
        case 4: pParam = &runParams.max_power;
                name = "Max\nPower";
                unit = "";
                increment = 1;
                break;
        case 5: pParam = &runParams.target_top_speed;
                name = "Top\nSpeed";
                unit = "mm/s";
                increment = 10;
                break;
        case 6: pParam = &runParams.slowdown_distance_from_end;
                name = "Slowdown\nDistance";
                unit = "mm";
                increment = 10;
                break;
        case 7: pParam = &runParams.initial_deceleration_power;
                name = "Initial\nDecel Power";
                unit = "";
                increment = 1;
                break;
        case 8: pParam = &runParams.max_deceleration_power;
                name = "Max\nDecel Power";
                unit = "";
                increment = 1;
                break;
        case 9: pParam = &runParams.target_end_speed;
                name = "Target\nEnd Speed";
                unit = "mm/s";
                increment = 10;
                break;
        case 10: pParam = &runParams.break_deceleration_power;
                name = "Break\nDecel Power";
                unit = "";
                increment = 1;
                break;
        case -1:
                param = 10;
                continue;
        default:
                param = 0;
                continue;
      }

      if(changed)
      {
        tft.fillScreen(TFT_BLACK);
        tft.setCursor(0,0);
        tft.setTextSize(3);
        tft.printf("%s\n\n", name);
        tft.setTextSize(4);
        tft.printf("  %d%s\n", *pParam, unit);
        changed = false;
      }

      if(PS4.Right())
      {
        *pParam += increment;
        delay(100);
        changed = true;
      }
      else if(PS4.Left())
      {
        *pParam -= increment;
        delay(100);
        changed = true;
      }
      else if(PS4.Down())
      {
        param++;
        changed = true;
        // Wait for button release
        while(PS4.Down())
        {
          delay(10);
        }
      }
      else if(PS4.Up())
      {
        param--;
        changed = true;
        // Wait for button release
        while(PS4.Up())
        {
          delay(10);
        }

      }
      else
      {
        delay(10);
      }
    }

    // Save the results
    writeRunParameters(runParams);
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
 
  showSensorMeters();
  
  state = STATE_MANUAL;
//  digitalWrite(gpioIlluminationLED, HIGH);
}

void DragsterControl::manualControl()
{
  if (!PS4.isConnected()) 
  {
     // Stop motors
    motors.stopAll();
    Serial.println("Waiting for PS4 controller...");

    // Display the sensors
    getSensorReadings();
    plotSensorReadings();

    if(selectSwitch.isTriggered(digitalRead(gpioSelectButton)))
    {
      // Switch to menu mode
        state = STATE_INITIAL;
        return;
    }
    delay(20);
  }
  else if(PS4.Cross())
  {
    // Switch to menu mode
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
    getSensorReadings();
    
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
    plotSensorReadings();

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
//  digitalWrite(gpioIlluminationLED, HIGH);
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
    getSensorReadings();
    
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
//  digitalWrite(gpioIlluminationLED, HIGH);

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
    getSensorReadings();
    
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
//  digitalWrite(gpioIlluminationLED, HIGH);
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
    getSensorReadings();
    
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
//      digitalWrite(gpioIlluminationLED, HIGH);
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
    getSensorReadings();
    Serial.print("waitForProfileArmed triggered: ");
    Serial.println(sensorRadius);
    if(PS4.Square() || 
       (selectSwitch.isTriggered(digitalRead(gpioSelectButton)) && sensorRadius > markerHighThreshold))
    {
      // Light on
//      digitalWrite(gpioIlluminationLED, HIGH);
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
    getSensorReadings();
    Serial.print("waitForProfileGo: ");
    Serial.println(sensorRadius);
    if(selectSwitch.isTriggered(digitalRead(gpioSelectButton)))
    {
        // Switch back to menu mode
        state = STATE_INITIAL;
        return;
    }

    if(PS4.Circle() ||
       (!PS4.isConnected() && sensorRadius < markerLowThreshold))
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
  getSensorReadings();

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

void  DragsterControl::getSensorReadings()
{
  // Read the ambient level first
  int sensorStartFinishAbient = analogRead(gpioSensorStartFinish);
  int sensorRadiusAbient = analogRead(gpioSensorRadius);
  int sensorRightLineAbient = analogRead(gpioSensorRightLine);
  int sensorLeftLineAbient = analogRead(gpioSensorLeftLine);

  // Illuminate
  digitalWrite(gpioIlluminationLED, HIGH);
  delayMicroseconds(phototransistorsResponseTimeMicroS);  // Allow phototransitors time to react

  // Read illuminated values
  int sensorStartFinishLit = analogRead(gpioSensorStartFinish);
  int sensorRadiusLit = analogRead(gpioSensorRadius);
  int sensorRightLineLit = analogRead(gpioSensorRightLine);
  int sensorLeftLineLit = analogRead(gpioSensorLeftLine);
  // Switch illumination off
  digitalWrite(gpioIlluminationLED, LOW);

  // Store the difference
  #ifdef INVERTED_SENSORS
  sensorStartFinish = -(sensorStartFinishLit < sensorStartFinishAbient ? sensorStartFinishLit - sensorStartFinishAbient : 0);
  sensorRightLine = -(sensorRightLineLit < sensorRightLineAbient ? sensorRightLineLit - sensorRightLineAbient : 0);
  sensorLeftLine = -(sensorLeftLineLit < sensorLeftLineAbient ? sensorLeftLineLit - sensorLeftLineAbient : 0);
  sensorRadius = -(sensorRadiusLit < sensorRadiusAbient ? sensorRadiusLit - sensorRadiusAbient : 0);
  #else
  sensorStartFinish = sensorStartFinishLit > sensorStartFinishAbient ? sensorStartFinishLit - sensorStartFinishAbient : 0;
  sensorRightLine = sensorRightLineLit > sensorRightLineAbient ? sensorRightLineLit - sensorRightLineAbient : 0;
  sensorLeftLine = sensorLeftLineLit > sensorLeftLineAbient ? sensorLeftLineLit - sensorLeftLineAbient : 0;
  sensorRadius = sensorRadiusLit > sensorRadiusAbient ? sensorRadiusLit - sensorRadiusAbient : 0;
  #endif
}

void  DragsterControl::showSensorMeters()
{
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
}

void  DragsterControl::plotSensorReadings()
{
    value[0] = map(sensorRadius, 0, 4095, 0, 100);
    value[1] = map(sensorLeftLine, 0, 4095, 0, 100);
    value[2] = map(sensorRightLine, 0, 4095, 0, 100);
    value[3] = map(sensorStartFinish, 0, 4095, 0, 100);
    plotPointers();
}