#include <Arduino.h>
#define Enc_xa 21 // Encoder pin connection x-axis a
#define Enc_xb 22 // Encoder pin connection x-axis b
#define Enc_ya 23 // Encoder pin connection y-axis a
#define Enc_yb 24 // Encoder pin connection y-axis b
#define Enc_za 25 // Encoder pin connection z-axis a
#define Enc_zb 26 // Encoder pin connection z-axis b

#define Enc_x_pot 34 // Encoder pin connection x-axis potentiometer
#define Enc_y_pot 35 // Encoder pin connection y-axis potentiometer

volatile int posZ = 0;
int xValInit = 0;
int yValInit = 0;

void setup() {
  Serial.begin(115200);
  // pinMode(Enc_xa, INPUT_PULLUP);
  // pinMode(Enc_xb, INPUT_PULLUP);
  // pinMode(Enc_ya, INPUT_PULLUP);
  // pinMode(Enc_yb, INPUT_PULLUP);
  pinMode(Enc_za, INPUT_PULLUP);
  pinMode(Enc_zb, INPUT_PULLUP);
  pinMode(Enc_x_pot, INPUT);
  pinMode(Enc_y_pot, INPUT);

  attachInterrupt(digitalPinToInterrupt(Enc_za), readEncoderZ, CHANGE);
  xValInit = analogRead(Enc_x_pot);
  yValInit = analogRead(Enc_y_pot);
}
void loop() {
  int xVal = analogRead(Enc_x_pot);  
  int yVal = analogRead(Enc_y_pot);
  int xAngle = map(xVal - xValInit, -2047, 2047, -1650, 1650);
  int yAngle = map(yVal - yValInit, -2047, 2047, -1650, 1650);

  noInterrupts(); // read posZ
  int z = posZ;
  interrupts(); 

  Serial.write(0xEF);
  // Send pos_x data
  Serial.write((byte*)&xAngle, sizeof(xAngle));
  // Send pos_y data
  Serial.write((byte*)&yAngle, sizeof(yAngle));
  // Send posZ data
  Serial.write((byte*)&posZ, sizeof(posZ));

  delay(5);
}
// void readEncoderX() {
//   static int lastX = digitalRead(Enc_xa);
//   int currentX = digitalRead(Enc_xa);
//   int xb = digitalRead(Enc_xb);
//   if (currentX != lastX) {
//     if (currentX == xb) {
//       pos_x++;
//     } else {
//       pos_x--;
//     }
//   }
//   lastX = currentX;
// }
// void readEncoderY() {
//   static int lastY = digitalRead(Enc_ya);
//   int currentY = digitalRead(Enc_ya);
//   int yb = digitalRead(Enc_yb);
//   if (currentY != lastY) {
//     if (currentY == yb) {
//       pos_y++;
//     } else {
//       pos_y--;
//     }
//   }
//   lastY = currentY;
// }
void readEncoderZ() {
  static int lastZ = digitalRead(Enc_za);
  int currentZ = digitalRead(Enc_za);
  int zb = digitalRead(Enc_zb);
  if (currentZ != lastZ) {
    if (currentZ == zb) {
      posZ++;
    } else {
      posZ--;
    }
  }
  lastZ = currentZ;
}