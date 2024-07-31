// Speed Controlled Oscillator
// Written by Tat Kalintsev
// 29/08/2022

#include <LiquidCrystal.h>
#include <BlockNot.h>
#include <EEPROM.h>
#include <AccelStepper.h>

#define buttonPin A0

// EEPROM Addresses
// #define CAM_ADDRESS 10
#define REL_SPEED_ADDRESS 11
// #define STEP_DIV_ADDRESS 12

// Stepper Variables
#define stepPin 3
#define dirPin 2

AccelStepper stepper(AccelStepper::DRIVER, stepPin, dirPin);

// LCD Definitions
const int RS = 8;
const int EN = 9;
const int d4 = 4;
const int d5 = 5;
const int d6 = 6;
const int d7 = 7;
const int pin_BL = 10;  // arduino pin wired to LCD backlight circuit
LiquidCrystal lcd(RS, EN, d4, d5, d6, d7);

constexpr int updateInterval_ms = 200;
BlockNot updateTimer(updateInterval_ms);

// Speed control variables
constexpr int stepDiv = 8;
constexpr int stepsPerRevolution = 200;  // How many steps in a full revolution of the axel
constexpr int numCams = 4;               // The number of cams, i.e., the number of times the sample is hit per revolution
constexpr float hitFreqMin_Hz = 1;
constexpr float hitFreqMax_Hz = 60;
constexpr float hitFreqIncrement_Hz = 0.1;  // How much to change hitFreq when pressing buttons
float hitFreq_Hz = hitFreqMin_Hz;           // How often to hit the sample, measured in Hz

// UI Variables
float measuredPressure = 0;  // The measured pressure acting on the sample.
                             // When updating this, always round to the nearest 0.1

// Control Flow Variables
bool isMotorRunning = false;

// SECTION: USER INTERFACE FUNCTIONS
void readButtons() {
  int button_val = analogRead(buttonPin);
  // lcd.setCursor(0, 0);
  if (button_val < 60)  // Right button
  {
    // Serial.println("right");
    // pass
  } else if (button_val < 200)  // Up
  {
    // Serial.println("up");
    if (hitFreq_Hz < hitFreqMax_Hz) {
      hitFreq_Hz += hitFreqIncrement_Hz;
    }
    updateMotor();
    delay(150);
  } else if (button_val < 400)  // Down
  {
    // Serial.println("down");
    if (hitFreq_Hz > hitFreqMin_Hz) {
      hitFreq_Hz -= hitFreqIncrement_Hz;
    }
    updateMotor();
    delay(150);
  } else if (button_val < 600)  // Left
  {
    // Serial.println("left");
    // pass
  } else if (button_val < 800)  // Select
  {
    // Serial.println("select");
    if (isMotorRunning)
      stopMotor();
    else
      updateMotor();
    isMotorRunning = !isMotorRunning;
  }
}

// Save Data
void saveData() {
  EEPROM.update(REL_SPEED_ADDRESS, hitFreq_Hz);
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Data Saved!");
  delay(1500);
  lcd.clear();
  displayInfoScreen();
}

// Load Data
void loadData() {
  hitFreq_Hz = EEPROM.read(REL_SPEED_ADDRESS);
}

// Menus
void loadingScreen() {
  lcd.setCursor(3, 0);
  lcd.print("Starting");
  for (int i = 0; i < 3; i++)  // increment the power to 200 gradually
  {
    delay(100);
    lcd.print(".");
  }
}

void displayInfoScreen() {
  // Line 1:
  lcd.setCursor(0, 0);
  lcd.print("SPEED: ");
  lcd.print(hitFreq_Hz);
  lcd.print("Hz");
  lcd.setCursor(0, 1);
  lcd.print("PRESS: ");
  lcd.print(measuredPressure);
  lcd.print("kPa");
}

// SECTION: MOTOR CONTROLS

// Converts a desired "hit frequency" to step speed (in steps per second)
inline float hitFreqToStepSpeed(float f) {
  return f * (stepsPerRevolution / numCams);
}

inline void updateMotor(void) {
  float stepSpeed = hitFreqToStepSpeed(hitFreq_Hz);
  // Serial.println(hitFreq_Hz);
  stepper.setSpeed(stepSpeed);
}

inline void stopMotor(void) {
  stepper.setSpeed(0);
}

void setup() {
  // LCD Setup + Title Screen
  lcd.begin(16, 2);

  // Animated Loading Screen
  // loadingScreen();

  // Enable Serial
  Serial.begin(9600);

  // Load saved data
  loadData();

  // Setup pins
  // pinMode(stepPin, OUTPUT);
  // pinMode(dirPin, OUTPUT);

  // Finishing up
  delay(100);
  lcd.clear();
  displayInfoScreen();
  Serial.println("Setup Done");
}

void loop() {
  // To do always
  readButtons();

  if (updateTimer.TRIGGERED) {
    displayInfoScreen();
  }
}
