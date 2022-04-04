#include <PS4Controller.h>
#include <ESP32Servo.h>
#include "mm_hardware.h"
#include "pid.h"
#include "debounce.h"
#include "dragster_run.h"

#include <TFT_eSPI.h> // Hardware-specific library

extern TFT_eSPI tft;
extern int value[6];
extern void plotPointers(void);
extern void plotLinear(const char *label, int x, int y);

typedef enum
{
  STATE_INITIAL,
  STATE_MANUAL,
  STATE_DISARMED,
  STATE_ARMED,
  STATE_ACCEL1,
  STATE_ACCEL2,
  STATE_MAX_SPEED,
  STATE_DECEL,
  STATE_FAST_STOP,
  STATE_STOPPING
} EStates;
EStates state;

unsigned long accelTimeout;
unsigned long startTime;

Servo steeringServo;
float pidInput = 0.0;
float pidSetpoint = 0.0;
PID pid(PID_Kp, PID_Ki, PID_Kd, &pidInput, &pidSetpoint);
Debounce startFinish(markerLowThreshold, markerHighThreshold, false);
int startFinishCount = 0;
int maxRunSpeed = MAX_SPEED_SLOW;
bool manualAutoSteer = false;

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

  // Steering 
  ESP32PWM::allocateTimer(2);
  steeringServo.setPeriodHertz(steeringServoFreq);
  steeringServo.attach(gpioSteeringServo, 1000, 2000);

  // PS4 controller
  //PS4.begin("44:17:93:89:84:6a"); // this esp32
  PS4.begin();  // default MAC

}

void initialMenu()
{
  tft.fillScreen(TFT_BLACK);
  tft.setCursor(0,0);
  tft.setTextSize(4);
  tft.print("Mode?");

  int menuSelected = 0;
  
  Debounce selectSwitch(LOW, HIGH, false);
  Debounce enterSwitch(LOW, HIGH, false);
  while(!state)
  {
    if(selectSwitch.isTriggered(digitalRead(gpioSelectButton)))
    {
      tft.fillScreen(TFT_BLACK);
      tft.setCursor(0,0);
      switch(++menuSelected)
      {
      case 1:
        tft.print("Manual\n(manual)");
        break;
      case 2:
        tft.print("Manual\n(auto)");
        break;
      case 3:
        tft.print("Race\n(slow)");
        maxRunSpeed = MAX_SPEED_SLOW;
        break;
      case 4:
        tft.print("Race\n(med)");
        maxRunSpeed = MAX_SPEED_MED;
        break;
      case 5:
        tft.print("Race\n(fast)");
        maxRunSpeed = MAX_SPEED_FAST;
        break;
      case 6:
        tft.print("Race\n(crazy)");
        maxRunSpeed = MAX_SPEED_CRAZY;
        break;
      default:
        menuSelected = 0;
        tft.print("Mode?");
        break;
      }
    }
    
    if(menuSelected && enterSwitch.isTriggered(digitalRead(gpioEnterButton)))
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
        startDisarmed();
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
    digitalWrite(gpioMotorA1, LOW);
    digitalWrite(gpioMotorA2, LOW);
    digitalWrite(gpioMotorB1, LOW);
    digitalWrite(gpioMotorB2, LOW);
    ledcWrite(motorRPwmChannel, 0);
    ledcWrite(motorLPwmChannel, 0);
    //digitalWrite(gpioIlluminationLED, LOW);
    
    Serial.println("Waiting for PS4 controller...");
    delay(500);
  }
  else if(PS4.Cross())
  {
    // Switch to Race mode
    stopAll();
    startDisarmed();
    return;
  }
  else
  {
    digitalWrite(gpioIlluminationLED, HIGH);
    
    int forward = PS4.LStickY();
    if(forward > 2)
    {
      // Forward
      int dutyCycle = forward*2;
      digitalWrite(gpioMotorA1, HIGH);
      digitalWrite(gpioMotorA2, LOW);
      digitalWrite(gpioMotorB1, HIGH);
      digitalWrite(gpioMotorB2, LOW);
      ledcWrite(motorRPwmChannel, dutyCycle);
      ledcWrite(motorLPwmChannel, dutyCycle);
    }
    else if(forward < -2)
    {
      // Reverse
      int dutyCycle = (-forward-1)*2;
      digitalWrite(gpioMotorA1, LOW);
      digitalWrite(gpioMotorA2, HIGH);
      digitalWrite(gpioMotorB1, LOW);
      digitalWrite(gpioMotorB2, HIGH);
      ledcWrite(motorRPwmChannel, dutyCycle);
      ledcWrite(motorLPwmChannel, dutyCycle);
    }
    else
    {
      // Stop
      ledcWrite(motorRPwmChannel, 0);
      ledcWrite(motorLPwmChannel, 0);
    }

    // Steering
    if(!manualAutoSteer)
    {
      int steer = PS4.RStickX();
      int steerServoPos =  steeringServoCentre - steer;
      steeringServo.write(steerServoPos);
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
      int steerServoPos = (forward >= -2 ? steeringServoCentre + steer : steeringServoCentre - steer);
      steeringServo.write(steerServoPos);
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

void waitForArmed()
{
    // Wait for Square button
    if(PS4.Square())
    {
      // Light on
      digitalWrite(gpioIlluminationLED, HIGH);
      startArmed();
    }
    else if(PS4.Triangle())
    {
      // Switch to manual mode
      startManualControl();
    }
}

void waitForGo()
{
    startFinishCount = 0;
    
    // Wait for Circle button
    if(PS4.Circle())
    {
      startAccelerate1();
      tft.fillScreen(TFT_GREEN);
    }
    else
    {
      // Get initial steer
      steer();
    }
}

void stopAll()
{
    // Stop motors
    digitalWrite(gpioMotorA1, LOW);
    digitalWrite(gpioMotorA2, LOW);
    digitalWrite(gpioMotorB1, LOW);
    digitalWrite(gpioMotorB2, LOW);
    ledcWrite(motorRPwmChannel, 0);
    ledcWrite(motorLPwmChannel, 0);
    digitalWrite(gpioIlluminationLED, LOW);
    
    state = STATE_INITIAL;
}

// Accelerate up to speed
void startAccelerate(int forwardSpeed, int timeout)
{
   if(forwardSpeed >= 0)
    {
      // Forward
      int dutyCycle = forwardSpeed*2;
      digitalWrite(gpioMotorA1, HIGH);
      digitalWrite(gpioMotorA2, LOW);
      digitalWrite(gpioMotorB1, HIGH);
      digitalWrite(gpioMotorB2, LOW);
      ledcWrite(motorRPwmChannel, dutyCycle);
      ledcWrite(motorLPwmChannel, dutyCycle);
    }
    else
    {
      // Reverse
      int dutyCycle = (-forwardSpeed-1)*2;
      digitalWrite(gpioMotorA1, LOW);
      digitalWrite(gpioMotorA2, HIGH);
      digitalWrite(gpioMotorB1, LOW);
      digitalWrite(gpioMotorB2, HIGH);
      ledcWrite(motorRPwmChannel, dutyCycle);
      ledcWrite(motorLPwmChannel, dutyCycle);
    }

    accelTimeout = millis() + timeout;
}

void steer()
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
  //Serial.printf("PID in = %f, out = %f\n", pidInput, pidOutput);

  int steer = (int)pidOutput;
  int steerServoPos =  steeringServoCentre + steer;
  steeringServo.write(steerServoPos);

  // End or abort run?
  if(state != STATE_STOPPING && state != STATE_FAST_STOP)
  {
    // Check to abort run if off line
    if( sensorRightLine >= maxLineDetectorThreshold && sensorLeftLine >= maxLineDetectorThreshold)
    {
      // We've lost all contact wtih the line, just abort stop
      startFastStop();
      tft.fillScreen(TFT_RED);
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
        startFastStop();
        tft.fillScreen(TFT_BLUE);
        // Show elapsed time
        tft.setCursor(0,0);
        tft.setTextSize(4);
        tft.setTextColor(TFT_BLACK, TFT_BLUE);
        tft.print(millis() - startTime);
        tft.print("mS");
        tft.setTextColor(TFT_GREEN, TFT_BLACK);
      }
    }
  }
}

void startDisarmed()
{
  waitForPS4Controller();
  tft.fillScreen(TFT_BLACK);
  tft.setCursor(0,0);
  tft.setTextSize(4);
  tft.print("Press []\nto arm");

  state = STATE_DISARMED;
}

void startArmed()
{
  waitForPS4Controller();
  tft.fillScreen(TFT_ORANGE);
  tft.setCursor(0,0);
  tft.setTextSize(4);
  tft.setTextColor(TFT_BLACK, TFT_ORANGE);
  tft.print("Press O\nto start");
  tft.setTextColor(TFT_GREEN, TFT_BLACK);

  state = STATE_ARMED;
}

void startAccelerate1()
{
  startTime = millis();
  startAccelerate(SPEED_A1, TIME_SPEED_A1_mS);
  state = STATE_ACCEL1;
  startFinishCount = 0;
}

void startAccelerate2()
{
  startAccelerate(SPEED_A2, TIME_SPEED_A2_mS);
  state = STATE_ACCEL2;
}

void startAccelerateMax()
{
  startAccelerate(SPEED_MAX, TIME_SPEED_MAX_mS);
  state = STATE_MAX_SPEED;
}

void startDecelerate()
{
  startAccelerate(SPEED_DECEL, TIME_SPEED_DECEL_mS);
  state = STATE_DECEL;
}

void startFastStop()
{
  startAccelerate(-SPEED_FAST_STOP_REVSERSE, TIME_FAST_STOP_mS);
  state = STATE_FAST_STOP;
}

void startStop()
{
  startAccelerate(0, TIME_STOPPING_mS);
  state = STATE_STOPPING;
}

// Accelerate up to speed 1
void accelerating1()
{
    // Check if we've gone far enough
    if(accelTimeout <= millis())
      startAccelerate2();
    else
      steer();
}

void accelerating2()
{
    // Check if we've gone far enough
    if(accelTimeout <= millis())
      startAccelerateMax();
    else
      steer();
}

void acceleratingMax()
{
    // Check if we've gone far enough
    if(accelTimeout <= millis())
      startDecelerate();
    else
      steer();
}

void decelerating()
{
    // Check if we've gone far enough
    if(accelTimeout <= millis())
      startFastStop();
    else
      steer();
}

void fastStopping()
{
    // Check if we've gone far enough
    if(accelTimeout <= millis())
    {
      startStop();
    }
    else
      steer();
}


void stopping()
{
    // Check if we've gone far enough
    if(accelTimeout <= millis())
    {
      stopAll();
      startDisarmed();
    }
    else
      steer();
}


// Stop using electronic braking
void breakStop()
{
    digitalWrite(gpioMotorA1, HIGH);
    digitalWrite(gpioMotorA2, HIGH);
    digitalWrite(gpioMotorB1, HIGH);
    digitalWrite(gpioMotorB2, HIGH);
    ledcWrite(motorRPwmChannel, 0);
    ledcWrite(motorLPwmChannel, 0);
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
  case STATE_DISARMED:
    waitForArmed();
    break;
  case STATE_ARMED:
    waitForGo();
    break;
  case STATE_ACCEL1:
    accelerating1();
    break;
  case STATE_ACCEL2:
    accelerating2();
    break;
  case STATE_MAX_SPEED:
    acceleratingMax();
    break;
  case STATE_DECEL:
    decelerating();
    break;
  case STATE_FAST_STOP:
    fastStopping();
    break;
  case STATE_STOPPING:
    stopping();
    break;
  default:
    state = STATE_INITIAL;
    break;
  }
  
  delay(1);
}
