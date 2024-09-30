#include <Arduino.h>
#define Enc_xa 22 // Encoder pin connection x-axis a
#define Enc_xb 23 // Encoder pin connection x-axis b
#define Enc_ya 26 // Encoder pin connection y-axis a
#define Enc_yb 27 // Encoder pin connection y-axis b
#define Enc_za 33 // Encoder pin connection z-axis a
#define Enc_zb 25 // Encoder pin connection z-axis b


// #define Enc_x_pot 34 // Encoder pin connection x-axis potentiometer
// #define Enc_y_pot 35 // Encoder pin connection y-axis potentiometer

volatile int posX = 0;
volatile int posY = 0;
volatile int posZ = 0;
// int xValInit = 0;
// int yValInit = 0;

void setup() {
  Serial.begin(115200);
  pinMode(Enc_xa, INPUT_PULLUP);
  pinMode(Enc_xb, INPUT_PULLUP);
  pinMode(Enc_ya, INPUT_PULLUP);
  pinMode(Enc_yb, INPUT_PULLUP);
  pinMode(Enc_za, INPUT_PULLUP);
  pinMode(Enc_zb, INPUT_PULLUP);
  // pinMode(Enc_x_pot, INPUT);
  // pinMode(Enc_y_pot, INPUT);

  attachInterrupt(digitalPinToInterrupt(Enc_xa), readEncoderX, CHANGE);
  attachInterrupt(digitalPinToInterrupt(Enc_ya), readEncoderY, CHANGE);
  attachInterrupt(digitalPinToInterrupt(Enc_za), readEncoderZ, CHANGE);

  // xValInit = analogRead(Enc_x_pot);
  // yValInit = analogRead(Enc_y_pot);
}
void loop() {
  // int xVal = analogRead(Enc_x_pot);  
  // int yVal = analogRead(Enc_y_pot);
  // int xAngle = map(xVal - xValInit, -2047, 2047, -1650, 1650);
  // int yAngle = map(yVal - yValInit, -2047, 2047, -1650, 1650);

  noInterrupts(); // read posZ
  int32_t x = posX;
  int32_t y = posY;
  int32_t z = posZ;
  int32_t angleX = (((float)x/1200) * 360) * 10000;
  int32_t angleY = (((float)y/1200) * 360) * 10000;
  interrupts();
   
  Serial.write(0xEF);
  // Send pos_x data
  Serial.write((byte*)&angleX, sizeof(angleX));
  // Send pos_y data
  Serial.write((byte*)&angleY, sizeof(angleY));
  // Send posZ data
  Serial.write((byte*)&z, sizeof(z));

  delay(10);
}
void readEncoderX() {
  static int lastX = digitalRead(Enc_xa);
  int32_t currentX = digitalRead(Enc_xa);
  int32_t xb = digitalRead(Enc_xb);
  if (currentX != lastX) {
    if (currentX == xb) {
      posX++;
    } else {
      posX--;
    }
  }
  lastX = currentX;
}
void readEncoderY() {
  static int32_t lastY = digitalRead(Enc_ya);
  int32_t currentY = digitalRead(Enc_ya);
  int32_t yb = digitalRead(Enc_yb);
  if (currentY != lastY) {
    if (currentY == yb) {
      posY++;
    } else {
      posY--;
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
      posZ++;
    } else {
      posZ--;
    }
  }
  lastZ = currentZ;
}