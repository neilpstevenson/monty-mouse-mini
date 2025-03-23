#pragma once
//
// Hardware definitions
//
// Waveshare RP2040-Zero
//

// Indicators
static const int ledGreen = 10;
static const int ledRed = 11;
static const int neoPixel = 16;

// Buttons
static const int buttonA = 13;  // Left - Select 
static const int buttonB = 12;  // Right - Go

// Motors
static const int motorLeftA1 = 8;
static const int motorLeftA2 = 9;

static const int motorRightB1 = 7;
static const int motorRightB2 = 6;

// Sensors
static const int illuminationLeft = 14;
static const int illuminationRight = 15;
static const int leftRightSensors = A0;
static const int midLeftRightSensors = A1;
static const int centreLeftRightSensors = A2;

// Position encoders
static const int leftEncoderA = 3;
static const int leftEncoderB = 2;
static const int rightEncoderA = 4;
static const int rightEncoderB = 5;

// Debug port
static const int debugPortTx = 0;
static const int debugPortRx = 1;
