#include "dragster_control.h"

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
    -55,
    -55,
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
    2500,
    0,
    &crazyRunProfileStop
};
RunProfile_t crazyRunProfileDecel = {
    "Decel",
    FLAG_END_ON_MIN_SPEED,
    800,
    -5,
    -60,
    (courseTimedDistance + courseTargetStoppingDistance),
    3000,
    &crazyRunProfileCoast,
    &crazyRunProfileStop
};
RunProfile_t crazyRunProfileAccel = {
    "Accel",
    0,
    2000,
    128,
    128,
    (courseTimedDistance + courseTargetStoppingDistance)*1/2, // Max 1/2 of track
    0,  // Unlimited speed
    &crazyRunProfileDecel,
    &crazyRunProfileStop
};
RunProfile_t DragsterControl::crazyRunProfile = {
    "Start",
    FLAG_IGNORE_MARKERS,
    600,
    50,
    128,
    0,
    0,  // Unlimited
    &crazyRunProfileAccel,
    &crazyRunProfileStop
};

/////////////////////////////////////
// VV fast speed
RunProfile_t vVFastRunProfileStop = {
    "Stop",
    FLAG_IGNORE_MARKERS | FLAG_END_ON_MIN_SPEED,
    2000,
    -55,
    -55,
    0,
    0,
    &allRunProfileStopDone,
    0
};
RunProfile_t vVFastRunProfileCoast = {
    "Coast S",
    FLAG_COAST_ON_MAX_SPEED,
    1000,
    20,
    20,
    0,
    2500,
    0,
    &vVFastRunProfileStop
};
RunProfile_t vVFastRunProfileDecel = {
    "Decel",
    FLAG_END_ON_MIN_SPEED,
    700,
    -20,
    -90,
    (courseTimedDistance + courseTargetStoppingDistance),
    3000,
    &vVFastRunProfileCoast,
    &vVFastRunProfileStop
};
RunProfile_t vVFastRunProfileMidCruise = {
    "Cruise",
    FLAG_COAST_ON_MAX_SPEED,
    2000,
    105,
    105,
    int((courseTimedDistance + courseTargetStoppingDistance)*0.7), // Max 70% of track
    4300,
    &vVFastRunProfileDecel,
    &vVFastRunProfileStop
};
RunProfile_t vVFastRunProfileAccel = {
    "Accel",
    FLAG_END_ON_MAX_SPEED,
    2000,
    105,
    105,
    int((courseTimedDistance + courseTargetStoppingDistance)*0.7), // Max 70% of track
    4300,
    &vVFastRunProfileMidCruise,
    &vVFastRunProfileStop
};
RunProfile_t DragsterControl::vVFastRunProfile = {
    "Start",
    FLAG_IGNORE_MARKERS | FLAG_END_ON_MAX_SPEED,
    500,
    40,
    105,
    0,
    4300,
    &vVFastRunProfileAccel,
    &vVFastRunProfileStop
};

/////////////////////////////////////
// Very fast speed
RunProfile_t veryFastRunProfileStop = {
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
RunProfile_t veryFastRunProfileCoast = {
    "Coast S",
    FLAG_COAST_ON_MAX_SPEED,
    1000,
    20,
    20,
    0,
    2000,
    0,
    &veryFastRunProfileStop
};
RunProfile_t veryFastRunProfileDecel = {
    "Decel",
    FLAG_END_ON_MIN_SPEED,
    700,
    -20,
    -90,
    (courseTimedDistance + courseTargetStoppingDistance),
    2500,
    &veryFastRunProfileCoast,
    &veryFastRunProfileStop
};
RunProfile_t veryFastRunProfileMidCruise = {
    "Cruise",
    FLAG_COAST_ON_MAX_SPEED,
    2000,
    100,
    100,
    (courseTimedDistance + courseTargetStoppingDistance)*3/4, // Max 75% of track
    4000,
    &veryFastRunProfileDecel,
    &veryFastRunProfileStop
};
RunProfile_t veryFastRunProfileAccel = {
    "Accel",
    FLAG_END_ON_MAX_SPEED,
    2000,
    100,
    100,
    (courseTimedDistance + courseTargetStoppingDistance)*3/4, // Max 75% of track
    4000,
    &veryFastRunProfileMidCruise,
    &veryFastRunProfileStop
};
RunProfile_t DragsterControl::veryFastRunProfile = {
    "Start",
    FLAG_IGNORE_MARKERS | FLAG_END_ON_MAX_SPEED,
    500,
    40,
    100,
    0,
    4000,
    &veryFastRunProfileAccel,
    &veryFastRunProfileStop
};

/////////////////////////////////////
// Fast speed
RunProfile_t fastRunProfileStop = {
    "Stop",
    FLAG_IGNORE_MARKERS | FLAG_END_ON_MIN_SPEED,
    2000,
    -40,
    -40,
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
    1000,
    0,
    &fastRunProfileStop
};
RunProfile_t fastRunProfileDecel = {
    "Decel",
    FLAG_END_ON_MIN_SPEED,
    1000,
    -40,
    -40,
    (courseTimedDistance + courseTargetStoppingDistance),
    1500,
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
RunProfile_t DragsterControl::fastRunProfile = {
    "Accel",
    FLAG_IGNORE_MARKERS | FLAG_END_ON_MAX_SPEED,
    200,
    40,
    70,
    (courseTimedDistance + courseTargetStoppingDistance)*3/4, // Max 3/4 of track
    3000,
    &fastRunProfileMidCruise,
    &fastRunProfileStop
};

/////////////////////////////////////
// Medium speed
RunProfile_t mediumRunProfileStop = {
    "Stop",
    FLAG_IGNORE_MARKERS | FLAG_END_ON_MIN_SPEED,
    2000,
    -30,
    -30,
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
    1000,
    0,
    &mediumRunProfileStop
};
RunProfile_t mediumRunProfileDecel = {
    "Decel",
    FLAG_END_ON_MIN_SPEED,
    1000,
    -40,
    -40,
    (courseTimedDistance + courseTargetStoppingDistance),
    1500,
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
RunProfile_t DragsterControl::mediumRunProfile = {
    "Accel",
    FLAG_IGNORE_MARKERS | FLAG_END_ON_MAX_SPEED,
    200,
    40,
    52,
    (courseTimedDistance + courseTargetStoppingDistance)*3/4, // Max 3/4 of track
    3000,
    &mediumRunProfileMidCruise,
    &mediumRunProfileStop
};

/////////////////////////////////////
// Slow speed
RunProfile_t slowRunProfileStop = {
    "Stop",
    FLAG_IGNORE_MARKERS | FLAG_END_ON_MIN_SPEED,
    1000,
    -30,
    -30,
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
    -20,
    -20,
    (courseTimedDistance + courseTargetStoppingDistance),
    1000,
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
RunProfile_t DragsterControl::slowRunProfile = {
    "Accel",
    FLAG_IGNORE_MARKERS | FLAG_END_ON_MAX_SPEED,
    1000,
    40,
    40,
    (courseTimedDistance + courseTargetStoppingDistance)*4/5, // Max 4/5th of track
    2000,
    &slowRunProfileMidCruise,
    &slowRunProfileStop
};


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
RunProfile_t DragsterControl::testRunProfile = {
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
DragsterControl dragsterControl;

void setup() 
{
  Serial.begin(115200);

  // Display
  tft.init();
  tft.setRotation(1);
  tft.fillScreen(TFT_BLACK);

  dragsterControl.setup();
}

void loop() 
{
  dragsterControl.stateMachineRun();
}
