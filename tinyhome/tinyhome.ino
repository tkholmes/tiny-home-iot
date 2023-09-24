#include <xht11.h>
#include <StateMachine.h>
#include <LiquidCrystal_I2C.h>
#include <debounce.h>

// Yellow Buttons (Menu/Interact)
#define YELLOW_BUTTON_1_PIN 16
#define YELLOW_BUTTON_2_PIN 27
#define LED_INDICATOR_PIN   12
#define TEMP_HUMID_PIN      17
#define WATER_SENSOR_PIN    34
#define FAN_MOTOR_PMW_PIN   19
#define FAN_SPEED_PIN       18

// State Machine States
#define State0Home  0
#define State1LED   1
#define State2Temp  2
#define State3Water 3
#define State4Fan   4

// LCD
LiquidCrystal_I2C lcd(0x27,16,2);

// State Machine
int STATE_DELAY = 500;
static unsigned long lastRun = 0; 
StateMachine machine = StateMachine();
State* S0 = machine.addState(&state0Home); 
State* S1 = machine.addState(&state1LED);
State* S2 = machine.addState(&state2Temp);
State* S3 = machine.addState(&state3Water);
State* S4 = machine.addState(&state4Fan);

// Define your button with a unique id (0) and handler function.
// (The ids are so one handler function can tell different buttons apart if necessary.)
static Button yellow1(0, stateChange_Handler);
static Button yellow2(1, action_Handler);

// LED
bool ledOn = false;

// Temp and Humidity
bool showHumidity = false;
xht11 xht(TEMP_HUMID_PIN);
unsigned char dht[4] = {0, 0, 0, 0}; //Only the first 32 bits of data are received, not the parity bits

// Fan
int fanMode = 0;

// State transitions handled here. In general we:
// 1) Run exit handlers for the state that's exiting.
// 2) Decide which state we're going to transition to.
// 3) Run genral exit handlers.
// 4) Transition to the new state.
static void stateChange_Handler(uint8_t btnId, uint8_t btnState) {
  if (btnState == BTN_PRESSED) {
    // Button Pressed
  } else {
    // btnState == BTN_OPEN.
    switch (machine.currentState) {
      case State0Home:
        machine.transitionTo(S1);
        break;
      case State1LED:
        machine.transitionTo(S2);
        break;
      case State2Temp:
        machine.transitionTo(S3);
        break;
      case State3Water:
        machine.transitionTo(S4);
        break;
      case State4Fan:
        machine.transitionTo(S0);
        break;
      default:
        Serial.println("No state transition found!");
    }

    exitState();
    machine.run();
  }
}

static void action_Handler(uint8_t btnId, uint8_t btnState) {
  if (btnState == BTN_PRESSED) {
    // Button Pressed
  } else {
     // btnState == BTN_OPEN.
    switch (machine.currentState) {
      case State1LED:
        ledOn = !ledOn;
        break;
      case State2Temp:
        showHumidity = !showHumidity;
        break;
      case State4Fan:
        fanMode++;
        if (fanMode > 3) {
          fanMode = 0;
        }
        break;
      default:
        Serial.println("No state transition found!");
    }
  }

  machine.run();
}

void setup() {
  Serial.begin(115200);
  
  // LCD Setup
  lcd.init();
  lcd.backlight();
  lcd.clear();

  // Button Setup
  pinMode(YELLOW_BUTTON_1_PIN, INPUT_PULLUP);
  pinMode(YELLOW_BUTTON_2_PIN, INPUT_PULLUP);
  pinMode(LED_INDICATOR_PIN, OUTPUT);  //Set pin to output mode
  pinMode(FAN_MOTOR_PMW_PIN, OUTPUT);
  pinMode(FAN_SPEED_PIN, OUTPUT);

  // Start the state machine.
  machine.run();
}

void loop() {
    pollButtons(); // Poll buttons every loop.
    delay(10); // Debounce iterations should run fairly quickly, 10's of ms, not 100's.
    
    // Exercise the state machine ever 
    if (millis() - lastRun >= STATE_DELAY) {
      lastRun += STATE_DELAY;
      machine.run();
    }
}

static void pollButtons() {
  // update() will call buttonHandler() if PIN transitions to a new state and stays there
  // for multiple reads over 25+ ms.
  yellow1.update(digitalRead(YELLOW_BUTTON_1_PIN));
  yellow2.update(digitalRead(YELLOW_BUTTON_2_PIN));
}

void state0Home(){
  if(machine.executeOnce){
    lcd.print("Home");
  }
}

void state1LED(){
  if(machine.executeOnce){
    lcd.print("LED");
  }
  lcd.setCursor(0, 1);
  if (ledOn) {
    digitalWrite(LED_INDICATOR_PIN, LOW);  //Light up the LED
    lcd.print("OFF");
  } else {
    digitalWrite(LED_INDICATOR_PIN, HIGH);  //Light up the LED
    lcd.print("ON ");
  }
}

void state2Temp(){
  if(machine.executeOnce){
    lcd.print("Temp. and Humid.");
  }
  lcd.setCursor(0, 1);
  
  if (xht.receive(dht)) { //Returns true when checked correctly
    if (showHumidity == true)
    {
      lcd.print("Humidity: ");
      lcd.print(dht[0]); //[0] is the integral part of humidity, [1] is the fractional part
      lcd.print("% ");
    } else {
      lcd.print("Temp: ");
      lcd.print(dht[2]); //[2] integral part of temperature, [3] is the fractional part
      lcd.print("C      ");
    }
  }

  delay(100); //It takes ~1000ms to wait for the device to read;
}

void state3Water() {
  if(machine.executeOnce){
    lcd.print("Water/Steam");
  }
  lcd.setCursor(0, 1);

  int water_val = analogRead(WATER_SENSOR_PIN);
  if(water_val > 1500) {
    lcd.print("WATER!  ");
  }
  else {
    lcd.print("NO WATER");
  }
}

void state4Fan() {
  if(machine.executeOnce){
    lcd.print("Fan Speed");
  }
  lcd.setCursor(0, 1);

  switch (fanMode) {
    case 0: // OFF
      lcd.print("OFF ");
      digitalWrite(FAN_MOTOR_PMW_PIN, LOW); //pwm = 0
      analogWrite(FAN_SPEED_PIN, 0);
      break;
    case 1: // LOW
      lcd.print("LOW ");
      digitalWrite(FAN_MOTOR_PMW_PIN, LOW); //pwm = 0
      analogWrite(FAN_SPEED_PIN, 90);
      break;
    case 2: // MED
      lcd.print("MED ");
      digitalWrite(FAN_MOTOR_PMW_PIN, LOW); //pwm = 0
      analogWrite(FAN_SPEED_PIN, 150);
      break;
    case 3: // HIGH
      lcd.print("HIGH");
      digitalWrite(FAN_MOTOR_PMW_PIN, LOW); //pwm = 0
      analogWrite(FAN_SPEED_PIN, 210);
      break;
  }
}

void exitState() {
  Serial.println("Exit state: ");
  Serial.println(machine.currentState);
  lcd.clear();
  lcd.setCursor(0, 0);
}