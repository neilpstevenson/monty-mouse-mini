#include "quadrature.h"

#define USE_USB_SERIAL_PORT
#ifdef USE_USB_SERIAL_PORT
static UART &SerialPort = Serial;    // USB Serial
#else
static UART &SerialPort = Serial1;   // UART0 (pins 0 & 1)
#endif

// Encoders
Quadrature_encoder<3, 2> encoder_l;
Quadrature_encoder<4, 5> encoder_r;

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
  pinMode(6, OUTPUT);
  pinMode(7, OUTPUT);
  pinMode(8, OUTPUT);
  pinMode(9, OUTPUT);

  // Encoder
  encoder_l.begin(pull_direction::up, resolution::quarter);
  encoder_r.begin(pull_direction::up, resolution::quarter);
}

void loop() {
  // Simple switch test
  int sw_left = !digitalRead(13);
  int sw_right = !digitalRead(12);

  // Encoders
  long enc_left_count = encoder_l.count();
  long enc_right_count = encoder_r.count();

  // Sensors (raw)
  int a0 = analogRead(A0);
  int a1 = analogRead(A1);
  int a2 = analogRead(A2);
  
  // Sensors LEFT illuminated
  digitalWrite(14, 1);
  delay(1);
  int a0li = analogRead(A0);
  int a1li = analogRead(A1);
  int a2li = analogRead(A2);
  digitalWrite(14, 0);
  // RIGHT illuminated
  digitalWrite(15, 1);
  delay(1);
  int a0ri = analogRead(A0);
  int a1ri = analogRead(A1);
  int a2ri = analogRead(A2);
  digitalWrite(15, 0);

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

  // LEDs
  digitalWrite(10, sw_left);
  digitalWrite(11, sw_right);

  // Motors
  // Left
  int mode = 256; // 256 == break mode
  analogWrite(8, sw_left ? 128 : mode);
  //analogWrite(9, mode);
  analogWrite(9, sw_left ? mode: 128);
  // Right
  analogWrite(7, sw_right ? 128 : mode);
  //analogWrite(6, mode); //sw_right ? mode: 128);
  analogWrite(6, sw_right ? mode: 128);

  delay(50);

  digitalWrite(10, 1);
  digitalWrite(11, 1);

  delay(10);
}
