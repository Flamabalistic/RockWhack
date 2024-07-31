// Speed Controlled Oscillator
// Written by Tat Kalintsev
// 29/08/2022

#include <LiquidCrystal.h>
#include <BlockNot.h>
#include <EEPROM.h>
#include <AccelStepper.h>

constexpr int buttonPin = A0;

// EEPROM Addresses
constexpr int hitFreqHzAddress = 11;

// Stepper Variables
constexpr int dirPin = 2;
constexpr int stepPin = 3;
AccelStepper stepper(1, stepPin, dirPin); // AccelStepper::DRIVER

// Weight variables
constexpr int weightPin = -1;

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

constexpr int buttonInterval_ms = 50;
BlockNot buttonTimer(buttonInterval_ms);

constexpr int saveDelay_ms = 5000;
BlockNot saveTimer(saveDelay_ms);  // We want to save after changing target freq, but we want to wait a bit
                                   // and ensure the user doesn't change the freq again, making us save again.

// Speed control variables
constexpr int stepDiv = 8;
constexpr int stepsPerRevolution = 200;  // How many steps in a full revolution of the axel
constexpr int numCams = 4;               // The number of cams, i.e., the number of times the sample is hit per revolution
constexpr float hitFreqMin_Hz = 1;
constexpr float hitFreqMax_Hz = 60;
constexpr float hitFreqIncrement_Hz = 0.1;    // How much to change hitFreq when pressing a button
constexpr float hitFreqFastIncrement_Hz = 1;  // How much to change hitFreq when holding a button
constexpr int numDeadZoneTicks = 15;
constexpr int numTicksPerFastIncrement = 3;  // How many ticks per increment when quickly increasing
float hitFreq_Hz = hitFreqMin_Hz;            // How often to hit the sample, measured in Hz

// UI Variables
float measuredPressure_kPa = 0;  // The measured pressure acting on the sample.
                                 // When updating this, always round to the nearest 0.1

// Control Flow Variables
bool isMotorRunning = false;

enum class ButtonPressed {
  NONE,
  RIGHT,
  UP,
  DOWN,
  LEFT,
  SELECT,
};

// SECTION: USER INTERFACE FUNCTIONS
void readButtons() {
  static ButtonPressed prevButtonPressed = ButtonPressed::NONE;  // What button was pressed last tick
  static int buttonHoldDuration = 0;                             // how long a given button is held for in ticks

  // Debouncing the button input
  static int prevButtonVal = 0;
  int buttonVal = analogRead(buttonPin);
  int buttonDiff = buttonVal - prevButtonVal;
  prevButtonVal = buttonVal;
  if (buttonDiff < -50 || buttonDiff > 50) {
    // Serial.println("Bounce detected, returning...");
    prevButtonPressed = ButtonPressed::NONE;
    return;
  }

  // Check if we're holding down the button - used to change speed faster
  ButtonPressed buttonPressed =
    buttonVal < 60 ? ButtonPressed::RIGHT : buttonVal < 200 ? ButtonPressed::UP
                                          : buttonVal < 400 ? ButtonPressed::DOWN
                                          : buttonVal < 600 ? ButtonPressed::LEFT
                                          : buttonVal < 800 ? ButtonPressed::SELECT
                                                            : ButtonPressed::NONE;
  bool wasButtonHeld = prevButtonPressed == buttonPressed;
  prevButtonPressed = buttonPressed;
  if (wasButtonHeld)
    buttonHoldDuration++;
  else
    buttonHoldDuration = 0;

  float actualIncrement = 0;
  if (buttonHoldDuration == 0)
    actualIncrement = hitFreqIncrement_Hz;
  else if (buttonHoldDuration >= numDeadZoneTicks && buttonHoldDuration % numTicksPerFastIncrement == 0)
    actualIncrement = hitFreqFastIncrement_Hz;

  switch (buttonPressed) {
    case ButtonPressed::UP:
      // Serial.println("up");
      hitFreq_Hz = min(hitFreq_Hz + actualIncrement, hitFreqMax_Hz);
      updateMotor();
      break;
    case ButtonPressed::DOWN:
      // Serial.println("down");
      hitFreq_Hz = max(hitFreq_Hz - actualIncrement, hitFreqMin_Hz);
      updateMotor();
      break;
    case ButtonPressed::LEFT:
      // Serial.println("left");
      break;
    case ButtonPressed::RIGHT:
      // Serial.println("right");
      break;
    case ButtonPressed::SELECT:
      Serial.println("select");
      if (wasButtonHeld)  // Only trigger on first press
        return;
      isMotorRunning = !isMotorRunning;
      if (isMotorRunning)
        updateMotor();
      else
        stopMotor();
      break;
    default:
      break;
  }
}

// Save Data
void saveData() {
  Serial.println("Saving data");
  // EEPROM.update(hitFreqHzAddress, hitFreq_Hz);
  // lcd.clear();
  // lcd.setCursor(0, 0);
  // lcd.print("Data Saved!");
  // delay(1500);
  // lcd.clear();
  // displayInfoScreen();
}

// Load Data
void loadData() {
  hitFreq_Hz = EEPROM.read(hitFreqHzAddress);
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

// Displays the current speed and pressure
void displayInfoScreen() {
  lcd.setCursor(0, 0);
  lcd.print("SPEED: ");
  lcd.print(hitFreq_Hz);
  lcd.print("Hz ");  // Extra space to clear out leftover 'z'
  lcd.setCursor(0, 1);
  lcd.print("PRESS: ");
  lcd.print(measuredPressure_kPa);
  lcd.print("kPa  ");  // Extra space's to clear left over 'a's
}

// SECTION: MOTOR CONTROLS

// Converts a desired "hit frequency" to step speed (in steps per second)
inline float hitFreqToStepSpeed(float f) {
  return f * (stepsPerRevolution / numCams);
}

void updateMotor(void) {
  saveTimer.RESET;
  if (isMotorRunning) {
    float stepSpeed = hitFreqToStepSpeed(hitFreq_Hz);
    // Serial.print("Updating motor: ");
    // Serial.println(stepSpeed);
    stepper.setSpeed(stepSpeed);
  }
}

void stopMotor(void) {
  Serial.println("Stopping motor");
  stepper.setSpeed(0);
}

// Updates measuredPressure_kPa
void updateWeight(void) {
  int weightReading = analogRead(weightPin);
}

void setup() {
  // LCD Setup + Title Screen
  lcd.begin(16, 2);
  saveTimer.setFirstTriggerResponse(true);

  // Animated Loading Screen
  // loadingScreen();

  // Enable Serial
  Serial.begin(9600);

  // Load saved data
  loadData();

  // Setup pins
  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);

  // Finishing up
  delay(100);
  lcd.clear();
  displayInfoScreen();
  Serial.println("Setup Done");

  stepper.setAcceleration(1); // Load-bearing setAcceleration - however it doesn't change the accel - likely due to using setSpeed, rather than run
  stepper.setMaxSpeed(3000);
  stopMotor();
}

void loop() {
  stepper.runSpeed();
  if (updateTimer.TRIGGERED) {
    displayInfoScreen();
  }
  if (buttonTimer.TRIGGERED) {
    readButtons();
  }
  // if (saveTimer.FIRST_TRIGGER) {
  //   saveData();
  // }
}
