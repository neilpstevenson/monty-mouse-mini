#include <Adafruit_NeoPixel.h>
#include "quadrature.h"

int ledGreen = 10;
int ledRed = 11;
int neopixel = 16;
int buttonA = 12;
int buttonB = 13;
int motorA1 = 4;
int motorA2 = 5;
int motorB1 = 6;
int motorB2 = 7;
int illuminationForward = 14;
int illuminationSides = 15;
int leftSensor = A3;
int rightSensor = A0;
int forwardLSensor = A2;
int forwardRSensor = A1;


Adafruit_NeoPixel pixels(1, neopixel, NEO_GRB + NEO_KHZ800);
Quadrature_encoder<2, 3> encoder1; // right
Quadrature_encoder<8, 9> encoder2;


// the setup routine runs once when you press reset:
void setup() 
{
  Serial.begin(115200);
  Serial.println("Starting test");
  
  // initialize the digital pin as an output.
  pinMode(ledGreen, OUTPUT);
  pinMode(ledRed, OUTPUT);
  pinMode(buttonA, INPUT_PULLUP);
  pinMode(buttonB, INPUT_PULLUP);

  pinMode(motorA1, OUTPUT);
  pinMode(motorA2, OUTPUT);
  pinMode(motorB1, OUTPUT);
  pinMode(motorB2, OUTPUT);
  
  pinMode(illuminationForward, OUTPUT);
  pinMode(illuminationSides, OUTPUT);

  pinMode(leftSensor, INPUT);
  pinMode(rightSensor, INPUT);
  pinMode(forwardLSensor, INPUT);
  pinMode(forwardRSensor, INPUT);

  encoder1.begin(pull_direction::up, resolution::full);
  encoder2.begin(pull_direction::up, resolution::full);

  pixels.clear(); // Set all pixel colors to 'off'
}

// the loop routine runs over and over again forever:
void loop() 
{
  static int count = 0;  

  digitalWrite(ledGreen, digitalRead(buttonA));
  digitalWrite(ledRed, digitalRead(buttonB));

/*
  // Coast Mode
  if(digitalRead(buttonA))
  {
    analogWrite(motorA1, 0);  // 255 = coast mode, 1 = break-mode
    analogWrite(motorA2, (count%128));
  }
  else
  {
    analogWrite(motorA1, (count%128));
    analogWrite(motorA2, 0);
  }
    
  if(digitalRead(buttonB))
  {
    analogWrite(motorB1, 0);
    analogWrite(motorB2, (count%128));
  }
  else
  {
    analogWrite(motorB1, (count%128));
    analogWrite(motorB2, 0);
  }
*/
  // Break mode
  if(!digitalRead(buttonA))
  {
    analogWrite(motorA1, 255);  // 255 = coast mode, 1 = break-mode
    analogWrite(motorA2, 255-(count%128));
  }
  else
  {
    analogWrite(motorA1, 255-(count%128));
    analogWrite(motorA2, 255);
  }
    
  if(digitalRead(buttonB))
  {
    analogWrite(motorB1, 255);
    analogWrite(motorB2, 255-(count%128));
  }
  else
  {
    analogWrite(motorB1, 255-(count%128));
    analogWrite(motorB2, 255);
  }
    
  if(count++ % 30 == 0)
  {
    pixels.setPixelColor(0, pixels.Color(8, 0, 8));
    pixels.show();
  }
  else if(count % 30 == 15)
  {
    pixels.setPixelColor(0, pixels.Color(0, 8, 0));
    pixels.show();
  }

  // Poll sensors
  // get background illumination levels
  int backgroundForwardL = analogRead(forwardLSensor);
  int backgroundForwardR = analogRead(forwardRSensor);
  int backgroundLeft = analogRead(leftSensor);
  int backgroundRight = analogRead(rightSensor);
  
  // then forward sensors
  digitalWrite(illuminationForward, HIGH);
  delayMicroseconds(60);
  int forwardL = analogRead(forwardLSensor);
  int forwardR = analogRead(forwardRSensor);
  digitalWrite(illuminationForward, LOW);
  
  // finally sides
  digitalWrite(illuminationSides, HIGH);
  delayMicroseconds(60);
  int left = analogRead(leftSensor);
  int right = analogRead(rightSensor);
  digitalWrite(illuminationSides, LOW);

  Serial.print(backgroundLeft);     Serial.print(" ");
  Serial.print(left);               Serial.print(" ");
  Serial.print(backgroundForwardL); Serial.print(" ");
  Serial.print(forwardL);           Serial.print(" ");
  Serial.print(backgroundForwardR); Serial.print(" ");
  Serial.print(forwardR);           Serial.print(" ");
  Serial.print(backgroundRight);    Serial.print(" ");
  Serial.print(right);              Serial.print(" ");

  int count1 = encoder1.count();
  int count2 = encoder2.count();
  Serial.print(count1);
  Serial.print(" ");
  Serial.print(count2);
  Serial.print(" ");
  
  Serial.println();

  // Continue to read and print
/*  
  for(int i = 0; i < 5; i++)
  {
    delayMicroseconds(200);
    forwardL = analogRead(forwardLSensor);
    forwardR = analogRead(forwardRSensor);
    left = analogRead(leftSensor);
    right = analogRead(rightSensor);
    Serial.print("  ");
    Serial.print(left);       Serial.print(" ");
    Serial.print(forwardL);   Serial.print(" ");
    Serial.print(forwardR);   Serial.print(" ");
    Serial.print(right);
    Serial.println();
  }
*/  
  delay(10);

}
