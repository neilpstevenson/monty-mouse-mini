#include "quadrature.h"
#include <Adafruit_NeoPixel.h>

//#define LINE_SENSOR_MINI_5 //LINE_SENSOR_6
#define WALL_SENSOR_MINI_4

#define USE_USB_SERIAL_PORT
#ifdef USE_USB_SERIAL_PORT
static UART &SerialPort = Serial;    // USB Serial
#else
static UART &SerialPort = Serial1;   // UART0 (pins 0 & 1)
#endif

static const int LED_ENCODER_FLASH_RATIO = 500;

// On-board LED
static const int LED_NEOPIXEL_IO = 16;
Adafruit_NeoPixel neoPixel(1, LED_NEOPIXEL_IO, NEO_RGB + NEO_KHZ800);

// Encoders
Quadrature_encoder<3, 2> encoder_l;
Quadrature_encoder<4, 5> encoder_r;

// Motors
mbed::PwmOut mra(p7);
mbed::PwmOut mrb(p6);
mbed::PwmOut mla(p8);
mbed::PwmOut mlb(p9);

void setup() {
  SerialPort.begin(115200);

  // Switches
  pinMode(12, INPUT_PULLUP);
  pinMode(13, INPUT_PULLUP);
  // Sensors
  pinMode(A0, INPUT);
  pinMode(A1, INPUT);
  pinMode(A2, INPUT);
  pinMode(A3, INPUT);

  // LEDs
  pinMode(10, OUTPUT);
  pinMode(11, OUTPUT);

  // Motors
  // Set frequency
  const float period = 1.0 / 20000;
  mla.period(period);
  mlb.period(period);
  mra.period(period);
  mrb.period(period);
  //pinMode(6, OUTPUT);
  //pinMode(7, OUTPUT);
  //pinMode(8, OUTPUT);
  //pinMode(9, OUTPUT);

  // Encoder
  encoder_l.begin(pull_direction::up, resolution::quarter);
  encoder_r.begin(pull_direction::up, resolution::quarter);

  // LED
  neoPixel.clear();
}

void loop() 
{
  static int count = 0;
  count++;

  // Simple switch test
  int sw_left = !digitalRead(13);
  int sw_right = !digitalRead(12);

  // Encoders
  long enc_left_count = encoder_l.count();
  long enc_right_count = encoder_r.count();

#ifdef LINE_SENSOR_6
  // Sensors (raw)
  int a0 = analogRead(A0);
  int a1 = analogRead(A1);
  int a2 = analogRead(A2);
  
  // Sensors LEFT illuminated
  digitalWrite(14, 1);
  delayMicroseconds(20);
  int a0li = analogRead(A0);
  int a1li = analogRead(A1);
  int a2li = analogRead(A2);
  digitalWrite(14, 0);
  // RIGHT illuminated
  digitalWrite(15, 1);
  delayMicroseconds(20);
  int a0ri = analogRead(A0);
  int a1ri = analogRead(A1);
  int a2ri = analogRead(A2);
  digitalWrite(15, 0);

  if(count % 10 == 0)
  {
    SerialPort.print(a0li-a0);
    SerialPort.print(" ");
    SerialPort.print(a1li-a1);
    SerialPort.print(" ");
    SerialPort.print(a2li-a2);
    SerialPort.print(" ");
    SerialPort.print(a2ri-a2);
    SerialPort.print(" ");
    SerialPort.print(a1ri-a1);
    SerialPort.print(" ");
    SerialPort.print(a0ri-a0);
    SerialPort.print(" ");
    SerialPort.print(((3*(a1li-a1) + a2li-a2)-(a2ri-a2 + 3*(a1ri-a1))) / 3);
    SerialPort.print(" ");
    SerialPort.print((a2li-a2)-(a2ri-a2));
    SerialPort.print(" ");
    SerialPort.print(sw_left);
    SerialPort.print(" ");
    SerialPort.print(sw_right);
    SerialPort.print(" ");
    SerialPort.print(enc_left_count);
    SerialPort.print(" ");
    SerialPort.print(enc_right_count);
    SerialPort.println();
  }

#endif
#if defined(LINE_SENSOR_MINI_5) || defined(WALL_SENSOR_MINI_4)
  // Sensors (raw)
  int a0 = analogRead(A0);
  int a1 = analogRead(A1);
  int a2 = analogRead(A2);
  int a3 = analogRead(A3);
  
  // Sensors LEFT/RIGHT illuminated
  digitalWrite(14, 1);
  delayMicroseconds(20);
  int a0i = analogRead(A0);
  int a3i = analogRead(A3);
  digitalWrite(14, 0);

  // Sensors CENTRE illuminated
  digitalWrite(15, 1);
  delayMicroseconds(20);
  int a1i = analogRead(A1);
  int a2i = analogRead(A2);
  digitalWrite(15, 0);

  if(count % 10 == 0)
  {
    SerialPort.print(a0i-a0); // Left
    SerialPort.print(", "); 
    SerialPort.print(a1i-a1); // Centre Left
    SerialPort.print(", ");
    SerialPort.print(a2i-a2); // Centre Right
    SerialPort.print(", ");
    SerialPort.print(a3i-a3); // Right
    SerialPort.print(", ");
    SerialPort.print(a0); // Left
    SerialPort.print(", "); 
    SerialPort.print(a0i); // Left
    SerialPort.print(", "); 
    SerialPort.print(a1); // Centre Left
    SerialPort.print(", ");
    SerialPort.print(a1i); // Centre Left
    SerialPort.print(", ");
    SerialPort.print(a2); // Centre Right
    SerialPort.print(", ");
    SerialPort.print(a2i); // Centre Right
    SerialPort.print(", ");
    SerialPort.print(a3); // Right
    SerialPort.print(", ");
    SerialPort.print(a3i); // Right
    SerialPort.print(", ");
    SerialPort.print((a1i-a1) - (a2i-a2));   // Centre diff
    SerialPort.print(", ");
    SerialPort.print(enc_left_count);
    SerialPort.print(", ");
    SerialPort.print(enc_right_count);
    SerialPort.println();
  }
#endif

  // LEDs
  digitalWrite(10, sw_left);
  digitalWrite(11, sw_right);

  if(count % 20 == 0)
  {
    neoPixel.setPixelColor(0, neoPixel.Color(0, abs(enc_left_count)%LED_ENCODER_FLASH_RATIO >= LED_ENCODER_FLASH_RATIO/2 ? 64 : 0, abs(enc_right_count)%LED_ENCODER_FLASH_RATIO >= LED_ENCODER_FLASH_RATIO/2 ? 64 : 0));
    neoPixel.show();
  }

  // Motors
  // Left
  int mode = 1; // 256 == break mode
  mla.write(sw_left ? 0.5 : mode);
  mlb.write(sw_left ? mode : 0.5);
  //analogWrite(8, sw_left ? 128 : mode);
  //analogWrite(9, sw_left ? mode: 128);
  // Right
  mra.write(sw_right ? 0.5 : mode);
  mrb.write(sw_right ? mode : 0.5);
  //analogWrite(7, sw_right ? 128 : mode);
  //analogWrite(6, sw_right ? mode: 128);

  delay(1);

  if(count % 30 > 15)
  {
    digitalWrite(10, 1);
    digitalWrite(11, 1);
  }
  delay(1);
}
