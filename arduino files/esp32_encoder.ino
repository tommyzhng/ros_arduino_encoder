#include <Arduino.h>
#define Enc_xa 21 // Encoder pin connection x-axis a
#define Enc_xb 22 // Encoder pin connection x-axis b
#define Enc_ya 23 // Encoder pin connection y-axis a
#define Enc_yb 24 // Encoder pin connection y-axis b
#define Enc_za 25 // Encoder pin connection z-axis a
#define Enc_zb 26 // Encoder pin connection z-axis b
volatile int pos_x = 0;
volatile int pos_y = 0;
volatile int pos_z = 0;
void setup() {
  Serial.begin(115200);
  pinMode(Enc_xa, INPUT_PULLUP);
  pinMode(Enc_xb, INPUT_PULLUP);
  pinMode(Enc_ya, INPUT_PULLUP);
  pinMode(Enc_yb, INPUT_PULLUP);
  pinMode(Enc_za, INPUT_PULLUP);
  pinMode(Enc_zb, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(Enc_xa), readEncoderX, CHANGE);
  attachInterrupt(digitalPinToInterrupt(Enc_ya), readEncoderY, CHANGE);
  attachInterrupt(digitalPinToInterrupt(Enc_za), readEncoderZ, CHANGE);
}
void loop() {
  // Print the current positions
  noInterrupts(); // Disable interrupts while reading the position to prevent inconsistency
  int x = pos_x;
  int y = pos_y;
  int z = pos_z;
  interrupts(); // Re-enable interrupts
  Serial.write(0xEF);
  // Send pos_x data
  Serial.write((byte*)&pos_x, sizeof(pos_x));
  // Send pos_y data
  Serial.write((byte*)&pos_y, sizeof(pos_y));
  // Send pos_z data
  Serial.write((byte*)&pos_z, sizeof(pos_z));
  delay(5); // Adjust the delay as needed
}
void readEncoderX() {
  static int lastX = digitalRead(Enc_xa);
  int currentX = digitalRead(Enc_xa);
  int xb = digitalRead(Enc_xb);
  if (currentX != lastX) {
    if (currentX == xb) {
      pos_x++;
    } else {
      pos_x--;
    }
  }
  lastX = currentX;
}
void readEncoderY() {
  static int lastY = digitalRead(Enc_ya);
  int currentY = digitalRead(Enc_ya);
  int yb = digitalRead(Enc_yb);
  if (currentY != lastY) {
    if (currentY == yb) {
      pos_y++;
    } else {
      pos_y--;
    }
  }
  lastY = currentY;
}
void readEncoderZ() {
  static int lastZ = digitalRead(Enc_za);
  int currentZ = digitalRead(Enc_za);
  int zb = digitalRead(Enc_zb);
  if (currentZ != lastZ) {
    if (currentZ == zb) {
      pos_z++;
    } else {
      pos_z--;
    }
  }
  lastZ = currentZ;
}