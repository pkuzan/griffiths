/*Power Controller Software for :
   Windows Machine
   On and Off pushbutton switches
   Single LED
   Audio Power Control

  Paul Griffiths Auditorium

  Paul Kuzan
  20/10/2021
*/

#include <Bounce2.h>

#define DEBUG //enable/disable serial debug output

#ifdef DEBUG
#define DEBUG_PRINT(x) Serial.print(x)
#define DEBUG_PRINTDEC(x) Serial.print(x, DEC)
#define DEBUG_PRINTLN(x) Serial.println(x)
#else
#define DEBUG_PRINT(x)
#define DEBUG_PRINTDEC(x)
#define DEBUG_PRINTLN(x)
#endif

#define STATE_STANDBY 1
#define STATE_COMPUTER_STARTING 2
#define STATE_HW_STARTING 3
#define STATE_RUNNING 4
#define STATE_COMPUTER_STOPPRING 5
#define STATE_WAIT_FOR_PSU 6

#define STATE_LED_OFF 1
#define STATE_LED_ON 2
#define STATE_LED_FLASH_SLOW 3
#define STATE_LED_FLASH_FAST 4

#define SWITCH_NONE 1
#define SWITCH_PRESS_SHORT 2
#define SWITCH_PRESS_LONG 3

const int onOffSwitchPin = 1;
const int offSwitchPin = 2;

//Uses USB bus power to detect when Mac has actually started and shutdown
//Logic is inverted by opto-isolator
const int USBBusPowerPin = 5;

//Normally the servo pin
const int servoPin = 23;

const int ledPin = 22;

//Detects if audio engine is on or off (labeled Switching Voltage)
//Logic is inverted by opto-isolator
const int audioOnOffPin = 7;

//Will send MIDI to Hauptwerk to shut computer down (Servo Pin)
const int shutdownPin = 15;

//Controls audio - probably via a contactor or relay
const int audioPowerPin = 16;

//Controls power to NUC and monitors etc
const int systemPowerPin = 17;

//Power LED flash  interval
const unsigned long onFlashInterval = 1000UL;
const unsigned long offFlashInterval = 200UL;

unsigned long previousMillis = 0;
bool ledFlashState;

Bounce onOffSwitch = Bounce();
Bounce offSwitch = Bounce();

volatile byte state;
volatile byte ledState;

bool justTransitioned = false;
bool ledStateJustTransitioned = false;

//Pushbutton hold time
const unsigned long switchHoldTime = 5000UL;
unsigned long switchPressTime = 0;
byte switchState = SWITCH_NONE;
bool switchPressed = false;

//Delayed shutdown
const unsigned long delayShutdownTime = 20000UL;
unsigned long shutdownTime = 0;

void setup() {
#ifdef DEBUG
  Serial.begin(9600);
#endif

  pinMode(onOffSwitchPin, INPUT_PULLUP);
  onOffSwitch.attach(onOffSwitchPin);

  pinMode(offSwitchPin, INPUT_PULLUP);
  offSwitch.attach(offSwitchPin);

  pinMode(audioPowerPin, OUTPUT);
  pinMode(systemPowerPin, OUTPUT);
  pinMode(ledPin, OUTPUT);
  pinMode(shutdownPin, OUTPUT);
  pinMode(servoPin, OUTPUT);

  pinMode(audioOnOffPin, INPUT);
  pinMode(USBBusPowerPin, INPUT);

  transitionTo(STATE_STANDBY);
}

void loop() {
  readSwitch();
  doStateMachine();
  doLEDStateMachine();
}

void readSwitch() {
  onOffSwitch.update();
  if (onOffSwitch.fell()) {
    switchPressed = true;
    switchPressTime = millis();
  } else if (onOffSwitch.rose()) {
    if (switchPressed) {
      if ((millis() - switchPressTime) > switchHoldTime) {
        //Long press
        switchState = SWITCH_PRESS_LONG;
      } else {
        //Short press
        switchState = SWITCH_PRESS_SHORT;
      }
      switchPressed = false;
    }
  }
}

//The Main State Machine
void doStateMachine() {
  if (switchState == SWITCH_PRESS_LONG) {
    switchState = SWITCH_NONE;
    transitionTo(STATE_STANDBY);
  }

  switch (state) {
    case STATE_STANDBY: {
        if (justTransitioned) {
          DEBUG_PRINT("Standby\n");

          transitionLEDState(STATE_LED_ON);
          switchOffSystemPower();
          switchOffAudio();
          digitalWrite(shutdownPin, LOW);

          justTransitioned = false;
        }

        if (switchState == SWITCH_PRESS_SHORT) {
          DEBUG_PRINT("On Button pressed\n");
          switchState = SWITCH_NONE;
          switchOnSystemPower();

          transitionTo(STATE_COMPUTER_STARTING);
        }
        break;
      }

    case STATE_COMPUTER_STARTING: {
        if (justTransitioned) {
          DEBUG_PRINT("Waiting for Computer to Start\n");
          justTransitioned = false;
        }

        //Logic inverted by opto
        if (digitalRead(USBBusPowerPin) == LOW) {
          transitionLEDState(STATE_LED_FLASH_SLOW);
          transitionTo(STATE_HW_STARTING);
        }
        break;
      }

    case STATE_HW_STARTING: {
        if (justTransitioned) {
          DEBUG_PRINT("Waiting for Hauptwerk to Start\n");
          //Wait for 4094 to stabalise
          delay(2000);
          justTransitioned = false;
        }
        
        if (digitalRead(audioOnOffPin) == LOW) {
          transitionTo(STATE_RUNNING);
        }
        break;
      }

    case STATE_RUNNING: {
        if (justTransitioned) {
          DEBUG_PRINT("Hauptwerk Started\n");

          switchOnAudio();
          transitionLEDState(STATE_LED_OFF);

          justTransitioned = false;
        }

        if (switchState == SWITCH_PRESS_SHORT) {
          DEBUG_PRINT("Off Button pressed\n");
          switchState = SWITCH_NONE;
          transitionLEDState(STATE_LED_FLASH_FAST);
          switchOffAudio();
          sendShutdownMIDI();
          transitionTo(STATE_COMPUTER_STOPPRING);
        }
        break;
      }

    case STATE_COMPUTER_STOPPRING: {
        if (justTransitioned) {
          DEBUG_PRINT("Computer Stopping\n");

          justTransitioned = false;
        }

        if (digitalRead(USBBusPowerPin) == HIGH) {
          DEBUG_PRINT("USB OFF\n");

          transitionTo(STATE_WAIT_FOR_PSU);
        }
        break;
      }

    case STATE_WAIT_FOR_PSU: {
        if (justTransitioned) {
          DEBUG_PRINT("Waiting for PSU\n");

          shutdownTime = millis() + delayShutdownTime;
          switchOffSystemPower();

          justTransitioned = false;
        }

        if (millis() > shutdownTime) {
          transitionTo(STATE_STANDBY);
        }
        break;
      }
  }
}

//The State Machine for the Power LED
void doLEDStateMachine() {
  switch (ledState) {
    case STATE_LED_OFF: {
        if (ledStateJustTransitioned) {
          updateLED(false);

          ledStateJustTransitioned = false;
        }

        break;
      }
    case STATE_LED_ON: {
        if (ledStateJustTransitioned) {
          updateLED(true);

          ledStateJustTransitioned = false;
        }

        break;
      }
    case STATE_LED_FLASH_SLOW: {
        if (ledStateJustTransitioned) {
          //Do nothing
          ledStateJustTransitioned = false;
        }

        doFlash(onFlashInterval);

        break;
      }
    case STATE_LED_FLASH_FAST: {
        if (ledStateJustTransitioned) {
          //Do nothing
          ledStateJustTransitioned = false;
        }

        doFlash(offFlashInterval);

        break;
      }
  }
}

void doFlash(unsigned long interval) {
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis > interval) {
    previousMillis = currentMillis;
    updateLED(!ledFlashState);
  }
}


//Actually turn on or off the power led
void updateLED(bool newLEDFlashState) {
  ledFlashState = newLEDFlashState;

  if (ledFlashState) {
    digitalWrite(ledPin, HIGH);
  } else {
    digitalWrite(ledPin, LOW);
  }
}

void transitionLEDState(byte newLEDState) {
  ledStateJustTransitioned = true;
  ledState = newLEDState;
}

void transitionTo(byte newState) {
  justTransitioned = true;
  state = newState;
}

void switchOnAudio() {
  digitalWrite(servoPin, HIGH);
}

void switchOffAudio() {
  digitalWrite(servoPin, LOW);
}

void switchOnSystemPower() {
  digitalWrite(systemPowerPin, HIGH);
}

void switchOffSystemPower() {
  digitalWrite(systemPowerPin, LOW);
}

void sendShutdownMIDI() {
  digitalWrite(shutdownPin, HIGH);
  delay(200);
  digitalWrite(shutdownPin, LOW);
}
