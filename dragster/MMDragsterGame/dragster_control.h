#pragma once

#include <PS4Controller.h>
#include <ESP32Servo.h>
#include <ESP32Encoder.h>
#include "display.h"
#include "mm_hardware.h"
#include "pid.h"
#include "debounce.h"
#include "dragster_run.h"
#include "nvram.h"
#include "motors.h"
#include "speeds.h"
#include "run_stats.h"
#include "bluetooth_mgmt.h"

class DragsterControl
{
  private:

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
    Speeds_t speed;
    #endif //HAS_ENCODERS
    RunStats_t stats;
    
    Servo steeringServo;
    float pidInput = 0.0;
    float pidSetpoint = 0.0;
    PID pid;
    
    Debounce startFinish;
    Debounce radiusMarker;
    Debounce selectSwitch;
    Debounce enterSwitch;
    
    int startFinishCount = 0;
    bool manualAutoSteer = false;
    DRAGSTER_NVRAM_T hwconfig;

    RunProfile_t *pCurrentRunProfile;

    static RunProfile_t vConfigFastRunProfile;
    static RunProfile_t slowRunProfile;
    static RunProfile_t mediumRunProfile;
    static RunProfile_t fastRunProfile;
    static RunProfile_t veryFastRunProfile;
    static RunProfile_t vVFastRunProfile;
    static RunProfile_t crazyRunProfile;
    static RunProfile_t testRunProfile;
    
    typedef enum
    {
      STATE_INITIAL,
      STATE_MANUAL,
      STATE_PROFILE_DISARMED,
      STATE_PROFILE_ARMED,
      STATE_PROFILE_RUN,
      STATE_PROFILE_STOPPING,
      STATE_STEER_CALIBRATE,
      STATE_PID_CALIBRATE,
      STATE_POSITION_CALIBRATE
    } EStates;
    EStates state;
    
  public:
    DragsterControl() : 
      pid(PID_Kp, PID_Ki, PID_Kd, &pidInput, &pidSetpoint),
      startFinish(markerLowThreshold, markerHighThreshold, false),
      radiusMarker(markerLowThreshold, markerHighThreshold, false),
      selectSwitch(LOW, HIGH, false),
      enterSwitch(LOW, HIGH, false)
    {
    }
    
    void readConfig();
    void writeConfig();
    void readRunParameters(DRAGSTER_NVRAM_RACE_CONFIG_T &runParams);
    void writeRunParameters(DRAGSTER_NVRAM_RACE_CONFIG_T &runParams);
    void setup();
    void initialMenu();
    void waitForPS4Controller();
    void editRunConfigParams();
    void displayStats();
    int getCalibratedPosnMm();
    void startManualControl();
    void manualControl();
    void startSteeringCalibrate();
    void steeringCalibrate();
    void startDistanceCalibrate();
    void positionCalibrate();
    void startCalibratePID();
    void displayPID();
    void calibratePID();
    void startProfileDisarmed();
    void startProfileArmed();
    void startProfileRun();
    void startProfileRunSegment();
    void startProfileStopping();
    void waitForProfileArmed();
    void waitForProfileGo();
    void profileRun();
    void profileSensorActions();
    void displayLastRunTime();
    void stateMachineRun();
};
