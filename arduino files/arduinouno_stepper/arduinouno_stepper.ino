// Define the TB6600 pin connections
#define EN_PIN 8  
#define DIR_PIN 10   
#define PUL_PIN 9   
#define MAX_INPUT_LENGTH 200
#define CW 1;
#define CCW 0;

// Motor control variables
const int stepsPerRev = 200;  
float lengthperrev = 3.1415 * 0.05;
float stepDelay = 500;
float len = 0;
float vel = 0;
unsigned long previousMicros = 0;
const unsigned long interval = 1000;  // 1 ms interval between steps
float microSecondsPerStep = 500;
bool stepperEnabled = false;

int direction = 0;
int rpm = 0;

// serial recieve
char inputString[MAX_INPUT_LENGTH];  
boolean stringComplete = false;
int inputIndex = 0;

void setup() {
  pinMode(EN_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(PUL_PIN, OUTPUT);

  Serial.begin(115200);

  digitalWrite(EN_PIN, LOW);
}

void loop() {
  // parse the new serial data
  serialEvent();
  stepsConversion();

  unsigned long currentMicros = micros();

  if (currentMicros - previousMicros >= interval && stepperEnabled) {
    previousMicros = currentMicros;
    // Set direction
    digitalWrite(DIR_PIN, direction);
    digitalWrite(PUL_PIN, HIGH);
    delayMicroseconds(microSecondsPerStep/2);
    digitalWrite(PUL_PIN, LOW);
    delayMicroseconds(microSecondsPerStep/2);
  }
}

void stepsConversion() {
  if (vel > 0) {
    stepperEnabled = true;
    direction = CW;
  } else if (vel < 0) {
    stepperEnabled = true;
    direction = CCW;
  } 
  else {
    stepperEnabled = false;
  }
  float rpm = abs(vel) * 60 / lengthperrev;
  microSecondsPerStep = (60 * 1000000)/(rpm * 200);
}

void serialEvent() {
  while (Serial.available()) {
    // get the new byte:
    char inChar = (char)Serial.read();

    // add to inputString if there is space
    if (inputIndex < MAX_INPUT_LENGTH - 1) {
      inputString[inputIndex++] = inChar;
    }

    // If the incoming character is a newline, string is complete
    if (inChar == '\n') {
      inputString[inputIndex] = '\0';  // Null-terminate the string
      stringComplete = true;
      inputIndex = 0;  // Reset for the next input
      break;
    }
  }

  // process the string when its complete:
  if (stringComplete) {
    // trim the newline character
    inputString[inputIndex - 1] = '\0';

    // find space and split into len and vel
    char* spaceIndex = strchr(inputString, ' ');
    if (spaceIndex != NULL) {
      *spaceIndex = '\0';
      spaceIndex++;  // move to the start of the velocity string

      if (strlen(inputString) > 0 && strlen(spaceIndex) > 0) {
        len = atof(inputString) / 10000;
        vel = atof(spaceIndex) / 10000;
      }
    }

    // clear the input string for the next input
    memset(inputString, 0, MAX_INPUT_LENGTH);
    stringComplete = false;
  }
}
