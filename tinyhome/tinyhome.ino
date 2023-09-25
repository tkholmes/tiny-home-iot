#include <ESP32Tone.h>
#include <pitches.h>

#include <xht11.h>
#include <StateMachine.h>
#include <LiquidCrystal_I2C.h>
#include <debounce.h>
#include <ESP32_Servo.h>
#include <Wire.h>
#include <MFRC522_I2C.h>
#include <Adafruit_NeoPixel.h>

// Yellow Buttons (Menu/Interact)
#define YELLOW_BUTTON_1_PIN 16
#define YELLOW_BUTTON_2_PIN 27
#define LED_INDICATOR_PIN   12
#define TEMP_HUMID_PIN      17
#define WATER_SENSOR_PIN    34
#define FAN_MOTOR_PMW_PIN   19
#define FAN_SPEED_PIN       18
#define PROXIMITY_PIN       14
#define RFID_PIN            16
#define WINDOW_SERVO_PIN    5
#define DOOR_SERVO_PIN      13
#define NEO_PIXEL_PIN       26
#define SPEAKER_PIN         25
#define GAS_DIGITAL_PIN     23
#define GAS_ANALOG_PIN      35

// State Machine States
#define State0Home      0
#define State1LED       1
#define State2Temp      2
#define State3Water     3
#define State4Fan       4
#define State5Proximity 5
#define State6RfidOpen  6
#define State7NeoPixel  7
#define State8Sound     8
#define State9Gas       9

// LCD
LiquidCrystal_I2C lcd(0x27, 16, 2);

// State Machine
int STATE_DELAY = 500;
static unsigned long lastRun = 0;
StateMachine machine = StateMachine();
State* S0 = machine.addState(&state0Home);
State* S1 = machine.addState(&state1LED);
State* S2 = machine.addState(&state2Temp);
State* S3 = machine.addState(&state3Water);
State* S4 = machine.addState(&state4Fan);
State* S5 = machine.addState(&state5Proximity);
State* S6 = machine.addState(&state6Rfid);
State* S7 = machine.addState(&state7NeoPixel);
State* S8 = machine.addState(&state8Sound);
State* S9 = machine.addState(&state9Gas);

// Button Handlers
// (The ids are so one handler function can tell different buttons apart if necessary.)
static Button yellow1(0, stateChange_Handler);
static Button yellow2(1, action_Handler);

// LED
bool ledOn = false;

// Temp and Humidity
bool showHumidity = false;
xht11 xht(TEMP_HUMID_PIN);
unsigned char dht[4] = { 0, 0, 0, 0 };  //Only the first 32 bits of data are received, not the parity bits

// Fan
int fanMode = 0;

// Door & Window
bool doorOpen = false;
bool windowOpen = true;
Servo windowMotor;
Servo doorMotor;

// RFID Reader NOTE the -1 on the RST_PIN.
// 0x28 is the i2c address of SDA, if doesn't match，please check your address with i2c. IIC pins default to GPIO21 and GPIO22 of ESP32
MFRC522_I2C mfrc522(0x28, -1);
String password = "";

// NeoPxel
Adafruit_NeoPixel neoStrip(4, NEO_PIXEL_PIN, NEO_GRB + NEO_KHZ800);
int lightMode = 0;

// Sound
int musicScale[8] = { NOTE_C4, NOTE_D4, NOTE_E4, NOTE_F4, NOTE_G4, NOTE_A5, NOTE_B5, NOTE_C5 };
String musicScaleNotes[8] = { "C4", "D4", "E4", "F4", "G4", "A5", "B5", "C5" };
int musicNoteIndex = 0;

void setup() {
  Serial.begin(115200);

  // LCD Setup
  lcd.init();
  lcd.backlight();
  lcd.clear();

  // I2C and Rfid
  Wire.begin();        // initialize I2C
  mfrc522.PCD_Init();  // initialize MFRC522

  // Servos
  windowMotor.attach(WINDOW_SERVO_PIN);
  doorMotor.attach(DOOR_SERVO_PIN);
  
  // Start Window open as it's often left open from paging through menus (ie. State3Water).
  windowMotor.write(180);
  windowOpen = true;
  doorMotor.write(0);
  doorOpen = false;

  // Pin Setup
  pinMode(YELLOW_BUTTON_1_PIN, INPUT_PULLUP);
  pinMode(YELLOW_BUTTON_2_PIN, INPUT_PULLUP);
  pinMode(LED_INDICATOR_PIN, OUTPUT);
  pinMode(FAN_MOTOR_PMW_PIN, OUTPUT);
  pinMode(FAN_SPEED_PIN, OUTPUT);
  pinMode(PROXIMITY_PIN, INPUT);
  pinMode(SPEAKER_PIN, OUTPUT);
  pinMode(GAS_DIGITAL_PIN, INPUT);
  pinMode(GAS_ANALOG_PIN, INPUT);

  neoStrip.begin();            // INITIALIZE NeoPixel strip object (REQUIRED)
  neoStrip.show();             // Turn OFF all pixels ASAP
  neoStrip.setBrightness(35);  // Set BRIGHTNESS to about 1/7 (max = 255)

  // Start the state machine.
  machine.run();
}

void loop() {
  pollButtons();  // Poll buttons every loop.
  delay(10);      // Debounce iterations should run fairly quickly, 10's of ms, not 100's.

  // Exercise the state machine every ~STATE_DELAY or so.
  if (millis() - lastRun >= STATE_DELAY) {
    lastRun += STATE_DELAY;
    machine.run();
  }
}

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
    // General exit state handler.
    exitCurrentState();

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
        machine.transitionTo(S5);
        exitState4Fan();
        break;
      case State5Proximity:
        machine.transitionTo(S6);
        break;
      case State6RfidOpen:
        machine.transitionTo(S7);
        break;
      case State7NeoPixel:
        machine.transitionTo(S8);
        exitState7NeoPixel();
        break;
      case State8Sound:
        machine.transitionTo(S9);
        break;
      case State9Gas:
        machine.transitionTo(S0);
        break;
      default:
        Serial.println("No state transition found!");
    }
    machine.run();
    Serial.print("Now in state: ");
    Serial.println(machine.currentState);
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
      case State3Water:
        // Toggle Window
        if (windowOpen) {
          windowMotor.write(0);
          windowOpen = false;
        } else {
          windowMotor.write(180);
          windowOpen = true;
        }
        // NOTE: BECAUSE OF THE MOISTURE DETECTION LOGIC IN THIS STATE, THIS TOGGLE COULD BE IMMEDIATELY REVERTED, so we delay.
        delay(750);
      case State4Fan:
        fanMode++;
        if (fanMode > 3) {
          fanMode = 0;
        }
        break;
      case State6RfidOpen:
        // If the door is open, and they press the Action button, close the door.
        if (doorOpen) {
          doorMotor.write(0);
          doorOpen = false;
        }
        break;
      case State7NeoPixel:
        lightMode++;
        if (lightMode > 8) {
          lightMode = 0;
        }
        break;
      case State8Sound:
        tone(SPEAKER_PIN, musicScale[musicNoteIndex], 250, 0);
        Serial.print("Music index:");
        Serial.println(musicNoteIndex);
        musicNoteIndex++;
        if (musicNoteIndex > 7) {
          musicNoteIndex = 0;
        }
        break;
      default:
        Serial.println("No state transition found!");
    }
  }

  machine.run();
}

static void pollButtons() {
  // update() will call buttonHandler() if PIN transitions to a new state and stays there
  // for multiple reads over 25+ ms.
  yellow1.update(digitalRead(YELLOW_BUTTON_1_PIN));
  yellow2.update(digitalRead(YELLOW_BUTTON_2_PIN));
}

void state0Home() {
  if (machine.executeOnce) {
    lcd.print("Home");
  }
}

void state1LED() {
  if (machine.executeOnce) {
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

void state2Temp() {
  if (machine.executeOnce) {
    lcd.print("Temp. and Humid.");
  }
  lcd.setCursor(0, 1);

  if (xht.receive(dht)) {  //Returns true when checked correctly
    if (showHumidity == true) {
      lcd.print("Humidity: ");
      lcd.print(dht[0]);  //[0] is the integral part of humidity, [1] is the fractional part
      lcd.print("% ");
    } else {
      lcd.print("Temp: ");
      lcd.print(dht[2]);  //[2] integral part of temperature, [3] is the fractional part
      lcd.print("C      ");
    }
  }

  delay(100);  //It takes ~1000ms to wait for the device to read;
}

void state3Water() {
  int water_val = analogRead(WATER_SENSOR_PIN);

  if (machine.executeOnce) {
    lcd.print("Water/Window");

    // Open the window automatically if no water found.
    if (water_val < 1500 && !windowOpen) {
      windowMotor.write(180);
      windowOpen = true;
    }
  }

  lcd.setCursor(0, 1);
  if (water_val > 1500) {
    lcd.print("WATER!  ");

    // CLose the window if open and water is detected.
    if (windowOpen) {
      windowMotor.write(0);
      windowOpen = false;
    }
  } else {
    lcd.print("NO WATER");
  }
}

void state4Fan() {
  if (machine.executeOnce) {
    lcd.print("Fan Speed");
  }
  lcd.setCursor(0, 1);

  switch (fanMode) {
    case 0:  // OFF
      lcd.print("OFF ");
      digitalWrite(FAN_MOTOR_PMW_PIN, LOW);  //pwm = 0
      analogWrite(FAN_SPEED_PIN, 0);
      break;
    case 1:  // LOW
      lcd.print("LOW ");
      digitalWrite(FAN_MOTOR_PMW_PIN, LOW);  //pwm = 0
      analogWrite(FAN_SPEED_PIN, 90);
      break;
    case 2:  // MED
      lcd.print("MED ");
      digitalWrite(FAN_MOTOR_PMW_PIN, LOW);  //pwm = 0
      analogWrite(FAN_SPEED_PIN, 150);
      break;
    case 3:  // HIGH
      lcd.print("HIGH");
      digitalWrite(FAN_MOTOR_PMW_PIN, LOW);  //pwm = 0
      analogWrite(FAN_SPEED_PIN, 210);
      break;
  }
}

void state5Proximity() {
  if (machine.executeOnce) {
    lcd.print("Proximity");
  }
  lcd.setCursor(0, 1);

  bool proximityFound = digitalRead(PROXIMITY_PIN);
  if (proximityFound) {
    lcd.print("Activity!     ");
  } else {
    lcd.print("No Activity.  ");
  }
}

void state6Rfid() {
  if (machine.executeOnce) {
    lcd.print("RFID Door");
  }
  lcd.setCursor(0, 1);

  // Rever LCD status
  if (!mfrc522.PICC_IsNewCardPresent() || !mfrc522.PICC_ReadCardSerial()) {
    if (doorOpen) {
      lcd.print("Open.       ");
    } else {
      lcd.print("Closed.     ");
    }
    password = "";
    return;
  }

  for (byte i = 0; i < mfrc522.uid.size; i++) {
    password = password + String(mfrc522.uid.uidByte[i]);
  }
  if (password == "693854118")  //The card number is correct, open the door
  {
    if (doorOpen) {
      // Door already open.
      lcd.print("Already Open");
      delay(500);
    } else {
      // Open door
      doorMotor.write(180);
      doorOpen = true;
    }
  } else  //The card number is wrong，LCD displays error
  {
    lcd.print("Wrong Key!  ");
    delay(500);
  }
}

void state7NeoPixel() {
  if (machine.executeOnce) {
    lcd.print("Neo Pixel Lights");
  }
  lcd.setCursor(0, 1);

  switch (lightMode) {
    case 0:  // OFF
      lcd.print("OFF       ");
      colorWipe(neoStrip.Color(0, 0, 0), 0);
      break;
    case 1:  // RED
      lcd.print("RED       ");
      colorWipe(neoStrip.Color(255, 0, 0), 0);
      break;
    case 2:  // ORANGE
      lcd.print("ORANGE    ");
      colorWipe(neoStrip.Color(200, 100, 0), 0);
      break;
    case 3:  // YELLOW
      lcd.print("YELLOW    ");
      colorWipe(neoStrip.Color(200, 200, 0), 0);
      break;
    case 4:  // GREEN
      lcd.print("GREEN     ");
      colorWipe(neoStrip.Color(0, 255, 0), 0);
      break;
    case 5:  // BLUE
      lcd.print("BLUE      ");
      colorWipe(neoStrip.Color(0, 0, 255), 0);
      break;
    case 6:  // INDIGO
      lcd.print("INDIGO    ");
      colorWipe(neoStrip.Color(100, 0, 255), 0);
      break;
    case 7:  // PURPLE
      lcd.print("PURPLE    ");
      colorWipe(neoStrip.Color(200, 0, 255), 0);
      break;
    case 8:  // RAINBOW
      lcd.print("RAINBOW   ");
      theaterChaseRainbow(2);
      break;
  }
}

void state8Sound() {
  if (machine.executeOnce) {
    lcd.print("Sound");
  }
  lcd.setCursor(0, 1);

  lcd.print(musicScaleNotes[musicNoteIndex]);
  lcd.print("    ");
}

void state9Gas() {
  if (machine.executeOnce) {
    lcd.print("Gas Detector");
  }
  lcd.setCursor(0, 1);
  
  int gasDigital = digitalRead(GAS_DIGITAL_PIN);
  float gasAnalog = analogRead(GAS_ANALOG_PIN);

  // Hazerdous gas detected!
  if (gasDigital == 0 || gasAnalog > 35) {
    tone(SPEAKER_PIN, NOTE_C2, 100, 0);
    delay(25);
    tone(SPEAKER_PIN, NOTE_C2, 100, 0);
    delay(25);
    tone(SPEAKER_PIN, NOTE_C2, 100, 0);
    lcd.print("Dangerous   ");
    delay(750); // Make sure folks can read message.
  } else {
    lcd.print("Safe Air    ");
  }
}

void exitState4Fan() {
  digitalWrite(FAN_MOTOR_PMW_PIN, LOW);  //pwm = 0
  analogWrite(FAN_SPEED_PIN, 0);
  fanMode = 0;
}

void exitState7NeoPixel() {
  colorWipe(neoStrip.Color(0, 0, 0), 0);
}

void exitCurrentState() {
  Serial.print("Exiting state: ");
  Serial.println(machine.currentState);
  lcd.clear();
  lcd.setCursor(0, 0);
}

// NEO PIXEL FUNCTIONS (to be broken out)

void colorWipe(uint32_t color, int wait) {
  for (int i = 0; i < neoStrip.numPixels(); i++) {  // For each pixel in strip...
    neoStrip.setPixelColor(i, color);               //  Set pixel's color (in RAM)
    neoStrip.show();                                //  Update strip to match
    delay(wait);                                    //  Pause for a moment
  }
}

// Rainbow cycle along whole strip. Pass delay time (in ms) between frames.
void rainbow(int wait) {
  for (long firstPixelHue = 0; firstPixelHue < 5 * 65536; firstPixelHue += 256) {
    for (int i = 0; i < neoStrip.numPixels(); i++) {  // For each pixel in strip...
      int pixelHue = firstPixelHue + (i * 65536L / neoStrip.numPixels());
      neoStrip.setPixelColor(i, neoStrip.gamma32(neoStrip.ColorHSV(pixelHue)));
    }
    neoStrip.show();  // Update strip with new contents
    delay(wait);      // Pause for a moment
  }
}

// Rainbow-enhanced theater marquee. Pass delay time (in ms) between frames.
void theaterChaseRainbow(int wait) {
  int firstPixelHue = 0;           // First pixel starts at red (hue 0)
  for (int a = 0; a < 30; a++) {   // Repeat 30 times...
    for (int b = 0; b < 3; b++) {  //  'b' counts from 0 to 2...
      neoStrip.clear();            //   Set all pixels in RAM to 0 (off)
      // 'c' counts up from 'b' to end of strip in increments of 3...
      for (int c = b; c < neoStrip.numPixels(); c += 3) {
        int hue = firstPixelHue + c * 65536L / neoStrip.numPixels();
        uint32_t color = neoStrip.gamma32(neoStrip.ColorHSV(hue));  // hue -> RGB
        neoStrip.setPixelColor(c, color);                           // Set pixel 'c' to value 'color'
      }
      neoStrip.show();              // Update strip with new contents
      delay(wait);                  // Pause for a moment
      firstPixelHue += 65536 / 90;  // One cycle of color wheel over 90 frames
    }
  }
}