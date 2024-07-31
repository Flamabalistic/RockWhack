// Speed Controlled Oscillator
// Written by Tat Kalintsev
// 29/08/2022

#include <LiquidCrystal.h>
#include <BlockNot.h>
#include <EEPROM.h>

#define updateDur 200  // in ms
#define buttons A0


// EEPROM Addresses
//#define CAM_ADDRESS 10
#define REL_SPEED_ADDRESS 11
//#define STEP_DIV_ADDRESS 12

// Stepper Variables
#define stepPin 3 // ~ 
#define dirPin 11 
#define sleepPin 12

// Interrupt
#define intPin 2


// LCD Definitions
const int RS = 8;
const int EN = 9;
const int d4 = 4;
const int d5 = 5;
const int d6 = 6;
const int d7 = 7;
const int pin_BL = 10; // arduino pin wired to LCD backlight circuit

LiquidCrystal lcd( RS,  EN,  d4,  d5,  d6,  d7);
BlockNot updateTimer(updateDur);

// Speed control variables
int pulse;
int stepDiv = 8;
int no_cams = 8;
const float stepAngle = 1.8;

// UI Variables
float trueSpeed = 1.0;
int relativeSpeed = 10;

// Control Flow Variables
bool motorIsOn = false;
int state = 0;

void setup() {
  // LCD Setup + Title Screen
  lcd.begin(16,2);

  // Enable Serial
  Serial.begin(9600);

  // Load saved data
  loadData();

  // Setup pins
  pinMode(stepPin,OUTPUT); 
  pinMode(dirPin,OUTPUT);
  pinMode(sleepPin,OUTPUT);
  pinMode(intPin, INPUT);

  // Interrupt
  attachInterrupt(digitalPinToInterrupt(intPin), stopMotor, RISING);

   // Animated Loading Screen
  loadingScreen();
  
  delay(500);
  lcd.clear();
  displayInfoScreen();
  Serial.println("Setup Done");
}


void loop() {
  // To do always
  readButtons();
  calculatePulseLength();
  
  if (motorIsOn) {
    runMotor();
  } else {
    digitalWrite(sleepPin,LOW);
  }

  if (updateTimer.TRIGGERED) {
    displayInfoScreen();
  }
}

// SECTION: MOTOR CONTROL
void calculatePulseLength() {
  pulse = int((2500 * no_cams) / (relativeSpeed * stepDiv));
  trueSpeed = float(relativeSpeed)/float(no_cams);
}

void runMotor() {  
  if (motorIsOn) {
    digitalWrite(dirPin,HIGH);
    digitalWrite(sleepPin,HIGH);
    for(int x = 0; x < 200*stepDiv; x++) {
      digitalWrite(stepPin,HIGH); 
      delayMicroseconds(pulse); 
      digitalWrite(stepPin,LOW); 
      delayMicroseconds(pulse); 
    }
    
  } else {
    digitalWrite(sleepPin,LOW);
  }
  
}

void stopMotor() {
  static unsigned long prevIntTime = 0;
  unsigned long intTime = millis();
  if (abs(intTime - prevIntTime) > 200) {
    motorIsOn = false;
  }
  prevIntTime = intTime;
}

// SECTION: USER INTERFACE FUNCTIONS
void readButtons() {
 int x;
 x = analogRead(buttons);
 lcd.setCursor(0,0);
 if (x < 60) { // Right button
   state = 0; // Power off motor
   saveData(); // Save data
 }
 else if (x < 200) { // Up
//   lcd.print ("Up    ");
   relativeSpeed++;
   if (relativeSpeed > 49) {
    relativeSpeed = 50;
   }
   delay(200);
 }
 else if (x < 400){ // Down
//   lcd.print ("Down  ");
   relativeSpeed--;
   if (relativeSpeed < 1) {
    relativeSpeed = 1;
   }
   delay(200);
 }
 else if (x < 600){ // Left was pressed
   lcd.print ("Left  ");
   lcd.clear(); 
 }
 else if (x < 800){ // Select was pressed
  motorIsOn = true;
  delay(500);
//  Serial.println("State: ");
//  Serial.println(state);
//  Serial.println("");
 }
 calculatePulseLength();
}

// Save Data
void saveData() {
  EEPROM.update(REL_SPEED_ADDRESS, relativeSpeed);
//  EEPROM.update(CAM_ADDRESS, no_cams);
  lcd.clear();
  lcd.setCursor(0,0); 
  lcd.print("Data Saved!");
  delay(2000);
  lcd.clear();
  displayInfoScreen();
}

// Load Data
void loadData() {
//  no_cams = EEPROM.read(CAM_ADDRESS);
  relativeSpeed = EEPROM.read(REL_SPEED_ADDRESS);
}

// Menus
void loadingScreen() {
  lcd.setCursor(3,0);
  lcd.print("Starting");  
  for (int i = 0; i < 3; i++) { // increment the power to 200 gradually
    delay(100);
    lcd.print(".");
  }
}

void displayInfoScreen() {
  // Line 1:
  lcd.setCursor(0,0);
  lcd.print("TS:");
  lcd.print(trueSpeed);
  lcd.print(" RS:");
  lcd.setCursor(11,0);
  lcd.print(relativeSpeed);
  lcd.print("Hz  ");

  // Line 2:
  lcd.setCursor(0,1);
  lcd.print(">START/STOP");
}
