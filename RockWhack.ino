// Speed Controlled Oscillator
// Written by Tat Kalintsev
// 29/08/2022

#include <LiquidCrystal.h>
#include <BlockNot.h>
#include <EEPROM.h>
#include <AccelStepper.h>
// EEPROM Addresses
constexpr int hitFreqHzAddress = 11;

// Stepper Variables
constexpr int dirPin = 2;
constexpr int stepPin = 3;
constexpr int enPin = 11;                  // stepper off = high, stepper on = low
AccelStepper stepper(1, stepPin, dirPin);  // AccelStepper::DRIVER

// Weight variables
constexpr int weightPin = A1;

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
constexpr int buttonPin = A0;


// Weight variables
constexpr float maxWeight_kg = 10;
constexpr float minWeight_kg = 0;
float measuredWeight_kg = minWeight_kg;  // The measured pressure acting on the sample.
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
  static int buttonHeldTicks = 0;                                // how long a given button is held for in ticks

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

  ButtonPressed buttonPressed =
    buttonVal < 60 ? ButtonPressed::RIGHT : buttonVal < 200 ? ButtonPressed::UP
                                          : buttonVal < 400 ? ButtonPressed::DOWN
                                          : buttonVal < 600 ? ButtonPressed::LEFT
                                          : buttonVal < 800 ? ButtonPressed::SELECT
                                                            : ButtonPressed::NONE;
  // Check if we're holding down the button - used to change speed faster
  bool wasButtonHeld = prevButtonPressed == buttonPressed;
  prevButtonPressed = buttonPressed;

  // Increment hold duration
  if (wasButtonHeld)
    buttonHeldTicks++;
  else
    buttonHeldTicks = 0;

  // Set increment based on the current ticks
  // - first press should be small
  // - holding it should repeatedly be large. If we increment every tick it goes way too fast, therefore we do every 3-5 ticks
  float actualIncrement = 0;
  if (buttonHeldTicks == 0)
    actualIncrement = hitFreqIncrement_Hz;
  else if (buttonHeldTicks == numDeadZoneTicks)  // avoids the x.10 problem (i.e., holding a button will always trigger the button press action first)
    actualIncrement = hitFreqFastIncrement_Hz - hitFreqIncrement_Hz;
  else if (buttonHeldTicks >= numDeadZoneTicks && (buttonHeldTicks - numDeadZoneTicks) % numTicksPerFastIncrement == 0)
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
      // Serial.println("select");
      if (wasButtonHeld)  // Only trigger on first press
        break;
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
  EEPROM.update(hitFreqHzAddress, hitFreq_Hz);
  // lcd.clear();
  // lcd.setCursor(0, 0);
  // lcd.print("Data Saved!");
  // delay(1500);
  // lcd.clear();
  // displayInfoScreen();
}

// Load Data
void loadData() {
  hitFreq_Hz = min(hitFreqMax_Hz, max(hitFreqMin_Hz, EEPROM.read(hitFreqHzAddress)));  // clamp to avoid errors if the flash dies
}

// Displays the current speed and weight
void displayInfoScreen() {
  updateWeight();
  lcd.setCursor(0, 0);
  lcd.print("SPEED:  ");
  lcd.print(hitFreq_Hz);
  lcd.print("Hz ");  // Extra space to clear out leftover letters after changing # of displayed digits
  lcd.setCursor(0, 1);
  lcd.print("WEIGHT: ");
  lcd.print(measuredWeight_kg);
  lcd.print("kg  ");  // Extra space to clear out leftover letters after changing # of displayed digits
}

// SECTION: MOTOR CONTROLS

// Converts a desired "hit frequency" to step speed (in steps per second)
inline float hitFreqToStepSpeed(float f) {
  return f * (float)(stepsPerRevolution * stepDiv) / (float)numCams;
}

void updateMotor(void) {
  saveTimer.RESET;
  if (isMotorRunning) {
    float stepSpeed = hitFreqToStepSpeed(hitFreq_Hz);
    Serial.print("Updating motor: ");
    Serial.println(stepSpeed);
    stepper.setSpeed(stepSpeed);
    digitalWrite(enPin, LOW);
  }
}

void stopMotor(void) {
  Serial.println("Stopping motor");
  stepper.setSpeed(0);
  digitalWrite(enPin, HIGH);
}

// SECTION: WEIGHT

// Converts a given analogRead() into a weight in kg
// - Coefficients were determined experimentally - https://www.wolframalpha.com/input?i=linear+fit+%5B%2F%2Fmath%3A%2836%2C+2.5%29%2C%2852%2C+5%29%2C%2868%2C+7.5%29%2F%2F%5D
inline float readingToWeight(int readValue) {
  return max(minWeight_kg, min(maxWeight_kg, 0.15625 * ((float)readValue) - 3.125));
}

// Updates measuredPressure_kPa
void updateWeight(void) {
  int weightReading = analogRead(weightPin);
  measuredWeight_kg = readingToWeight(weightReading);
}

void setup() {
  // LCD Setup + Title Screen
  lcd.begin(16, 2);
  saveTimer.setFirstTriggerResponse(false);

  // Animated Loading Screen
  // loadingScreen();

  // Enable Serial
  Serial.begin(9600);

  // Load saved data
  loadData();

  // Setup pins
  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  pinMode(enPin, OUTPUT);

  // Finishing up
  delay(100);
  lcd.clear();
  displayInfoScreen();
  Serial.println("Setup Done");

  stepper.setAcceleration(1);  // Load-bearing setAcceleration - however it doesn't change the accel - likely due to using setSpeed, rather than run
  stepper.setMaxSpeed(hitFreqToStepSpeed(hitFreqMax_Hz));
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
  if (saveTimer.FIRST_TRIGGER) {
    saveData();
  }
}
