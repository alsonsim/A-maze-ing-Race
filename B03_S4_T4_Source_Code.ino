#include <MeMCore.h>

// Pin Configurations
#define INPUT_A A0
#define INPUT_B A1
#define LDR_PIN A2
#define IR_PIN  A3
#define LDRWait 10
#define RGBWait 100

// For Movement
#define MOTORSPEED     255 
#define TURNDELAY      610
#define TURNSPEED      150
#define MAX_DISTANCE   20

// PD Variables
float Kp = 60.0;
float Kd = 20.0;
float error = 0.0;
float prevError = 0.0;
float derivative = 0.0;
float setPoint = 10.0;  
float ultrasonicDist = 0.0;  
float prevDist = 10.0;  
float smoothingFactor = 0.5;

// Control Variables
bool status = false;  // Tracks if the mBot has started
int maxSpeedLeft = MOTORSPEED;   // Maximum speed for left motor
int maxSpeedRight = MOTORSPEED;  // Maximum speed for right motor

// Using the Library
MeDCMotor leftWheel(M1);
MeDCMotor rightWheel(M2);
MeLineFollower line(PORT_2);
MeUltrasonicSensor ultrasonicSensor(PORT_1);
MeBuzzer buzzer;

// Celebratory Tune Configuration
int tempo = 140; // tempo

#define NOTE_G4  392
#define NOTE_Ab4 415
#define NOTE_Bb4 466
#define NOTE_C5  523
#define NOTE_D5  587
#define NOTE_Eb5 622
#define NOTE_E5  659
#define NOTE_F5  698
#define NOTE_G5  784
#define NOTE_Ab5 831
#define NOTE_Bb5 932
#define NOTE_C6  1047
#define NOTE_D6  1175
#define NOTE_Eb6 1245
#define NOTE_E6  1319
#define NOTE_F6  1397
#define NOTE_G6  1568
#define NOTE_Ab6 1661
#define NOTE_Bb6 1865
#define NOTE_C7  2093

// Color Definitions
float colourArray[] = {0, 0, 0};
float blackArray[] = {625.00, 971.00, 840.00}; 
float whiteArray[] = {925.00, 996.00, 961.00}; 
float greyDiff[]   = {300.00, 25.00, 121.00};    
float colourList[6][3] = {
  {255.00, 153.00, 54.79},  // Red
  {164.90, 183.60, 168.60}, // Green
  {147.90, 183.60, 236.03}, // Blue
  {255.00, 183.60, 40.04},  // Orange
  {255.00, 244.80, 223.39}, // Pink
  {255, 255, 255}           // White
};

int melody[] = {
  //melody of our celebratory tune
  NOTE_G4,16, NOTE_C5,16, NOTE_E5,16, NOTE_G5,16,
  NOTE_C6,16, NOTE_E6,16, NOTE_G6,8, NOTE_E6,16,

  NOTE_Ab4,16, NOTE_C5,16, NOTE_Eb5,16, NOTE_Ab5,16,
  NOTE_C6,16, NOTE_Eb6,16, NOTE_Ab6,8, NOTE_Eb6,16,

  NOTE_Bb4,16, NOTE_D5,16, NOTE_F5,16, NOTE_Bb5,16,
  NOTE_D6,16, NOTE_F6,16, NOTE_Bb6,8, NOTE_Bb6,16,
  NOTE_Bb6,16, NOTE_Bb6,16, NOTE_C7,8
};

int notes = sizeof(melody)/sizeof(melody[0])/2;
int wholenote = (60000 * 4) / tempo;
int divider = 0; int noteDuration = 0;

void celebrate() {
  // Code for playing celebratory tune
  // iterate over the notes of the melody
  for (int thisNote = 0; thisNote < notes * 2; thisNote += 2) {

    //calculates the duration of each note
    divider = melody[thisNote + 1];

    noteDuration = wholenote / divider;

    // play the note for 90% of the duration, the other 10% is the pause
    buzzer.tone(melody[thisNote], noteDuration*0.9);

    // wait for the specific duration before playing the next note
    delay(noteDuration);

    // stop the waveform gen before the next note
    buzzer.noTone();
  }
}

// Movement Code
void setMotorSpeeds(int leftSpeed, int rightSpeed) {
  leftWheel.run(-leftSpeed);
  rightWheel.run(rightSpeed);
}

void stopMotor() {
  // Code for stopping motor
  setMotorSpeeds(0, 0);
}

void moveForward() {
  // Code for moving forward for some short interval
  setMotorSpeeds(MOTORSPEED, MOTORSPEED - 50);
}

void turnRight() {
  // Code for turning right 90 deg
  setMotorSpeeds(TURNSPEED, -TURNSPEED);
  delay(TURNDELAY);
  stopMotor();
}

void turnLeft() {
  // Code for turning left 90 deg
  setMotorSpeeds(-TURNSPEED, TURNSPEED);
  delay(TURNDELAY);
  stopMotor();
}

void uTurn() {
  // Code for u-turn
  setMotorSpeeds(TURNSPEED, -TURNSPEED);
  delay(TURNDELAY);
  setMotorSpeeds(TURNSPEED, -TURNSPEED);
  delay(TURNDELAY);
}

void doubleLeftTurn() {
  // Code for double left turn
  setMotorSpeeds(-TURNSPEED, TURNSPEED);
  delay(TURNDELAY);
  stopMotor();
  moveForward();
  delay(800);
  setMotorSpeeds(-TURNSPEED, TURNSPEED);
  delay(TURNDELAY);
  stopMotor();
}
void doubleRightTurn() {
  // Code for double right turn
  setMotorSpeeds(TURNSPEED, -TURNSPEED);
  delay(TURNDELAY);
  stopMotor();
  moveForward();
  delay(800);
  setMotorSpeeds(TURNSPEED, -TURNSPEED);
  delay(650);
  stopMotor();
}

void shineIR() {
  digitalWrite(INPUT_B, LOW);
  digitalWrite(INPUT_A, LOW);
}

// Colour Sensing Code
void shineRed() {
  digitalWrite(INPUT_B, HIGH);
  digitalWrite(INPUT_A, HIGH);
}

void shineGreen() {
  digitalWrite(INPUT_B, LOW);
  digitalWrite(INPUT_A, HIGH);
}

void shineBlue() {
  digitalWrite(INPUT_B, HIGH);
  digitalWrite(INPUT_A, LOW);
}

void shineColour(int colour) {
  switch(colour) {
    case 0: 
      shineRed();
      break;
    case 1:
      shineGreen();
      break;
    case 2:
      shineBlue();
      break;
  }
}

void shineNone() {
  digitalWrite(INPUT_B, LOW);
  digitalWrite(INPUT_A, LOW);
}

int detectColour() {
  shineNone();

  for (int i = 0; i <= 2; i++) {
    shineColour(i);
    delay(RGBWait);
    float colourReading = getAvgReading(5);
    
    // Normalize based on black and white range
    colourArray[i] = constrain(((colourReading - blackArray[i]) / greyDiff[i]) * 255, 0, 255);
    
    shineNone();
    delay(RGBWait);
  }
  // Run algorithm for colour decoding
  float minDistance = 10000;
  int colourID = -1;
  
  for (int i = 0; i < 6; i++) {
    float distance = calculateDistance(colourArray, colourList[i]);
    if (distance < minDistance) {
      minDistance = distance;
      colourID = i;
    }
  }
  return colourID;
}

float calculateDistance(float *colour1, float* colour2) {
  float r_diff = colour1[0] - colour2[0];
  float g_diff = colour1[1] - colour2[1];
  float b_diff = colour1[2] - colour2[2];
  
  return sqrt((r_diff * r_diff) + (g_diff * g_diff) + (b_diff * b_diff));
}

void challenge(int colour) {
  switch (colour) {
    case 0: // Red
      turnLeft();
      break;
    case 1: // Green
      turnRight();
      break;
    case 2: // Blue
      doubleRightTurn();
      break;
    case 3: // Orange
      uTurn();
      break;
    case 4: // Pink
      doubleLeftTurn(); 
      break;
    case 5: // White
      stopMotor();
      celebrate();
      break;
  }
}

int getAvgReading(int times) {
  int total = 0;
  for(int i = 0; i < times; i++){
    int reading = analogRead(LDR_PIN);
    total += reading;
    delay(LDRWait);
  }
  return total / times;
}

// PD 
float smoothDist(float dist, float prevDist) {
  return (smoothingFactor * dist) + ((1.0 - smoothingFactor) * prevDist);
}

void resetPDVariables() {
  error = 0.0;
  prevError = 0.0;
  derivative = 0.0;
  ultrasonicDist = 0.0;
  prevDist = 10.0;
}

float ComputePD(float dist) {
  float smoothedDist = smoothDist(dist, prevDist);
  prevDist = smoothedDist;
  error = smoothedDist - setPoint;
  derivative = error - prevError;
  float output = (Kp * error) + (Kd * derivative);
  prevError = error;
  return output;
}

float readDistance() {
  float distance = ultrasonicSensor.distanceCm();
  if (distance > 0 && distance < MAX_DISTANCE) {
    return distance;
  }
  return -1; // Invalid reading, there is no wall
}

// Check if IR is too close to the left wall
bool IR_Detect() {
  shineRed();
  delay(20);
  float ambientIR = analogRead(IR_PIN);
  ambientIR /= 1023;
  ambientIR *= 5; // Converting it to a suitable voltage value of 0-5V

  shineIR();
  delay(20);
  float emitterIR = analogRead(IR_PIN);
  emitterIR /= 1023; 
  emitterIR *= 5;

  float diff = ambientIR - emitterIR;

  if (diff > 3.0) { // Too close to the left wall
    return true;
  }

  return false;
}

void setup() {
  // Configure pinMode for A0, A1, A2, A3
  pinMode(INPUT_A, OUTPUT);
  pinMode(INPUT_B, OUTPUT);
  pinMode(LDR_PIN, INPUT);
  pinMode(IR_PIN, INPUT);

  Serial.begin(9600);
}

void loop() {
  // Press button to start bot
  if (analogRead(A7) < 100) {
    if (!status) {
      status = true;
      delay(500);
    } else {
      stopMotor();
      resetPDVariables();
      status = false;
      delay(500);
    }
  }

  if (status) {
    float ultrasonicDist = readDistance(); 

    int sensorStatus = line.readSensors();
    if (sensorStatus == S1_IN_S2_IN) {
      // Black line detected, activate colour detection algorithm
      stopMotor();
      int colourID = detectColour();
      challenge(colourID);
    } else if (ultrasonicDist > 0) {
      float output = ComputePD(ultrasonicDist);

      // constrain output
      constrain(output, -80, 80);

      int leftSpeed = maxSpeedLeft;
      int rightSpeed = maxSpeedRight;

      // too close to the wall
      if (output > 0) {
        // steer left
        setMotorSpeeds(leftSpeed, rightSpeed - output);
      } else {
        // steer right
        setMotorSpeeds(leftSpeed + output, rightSpeed);
      }
    } else if (IR_Detect()) {
      // nudge right
      setMotorSpeeds(maxSpeedLeft, maxSpeedRight - 150);
    } else {
      moveForward();
    }
  }
}
