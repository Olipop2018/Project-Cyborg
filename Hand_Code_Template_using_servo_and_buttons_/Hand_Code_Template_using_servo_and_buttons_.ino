#include <Servo.h>

Servo myservo;  // create servo object to control a servo

// constants won't change. They're used here to set pin numbers:
const int buttonPin = 2;    // the number of the pushbutton pin
const int buttonPin2 = 8;    // the number of the pushbutton pin
const int buttonPin3 = 7;    // the number of the pushbutton pin


// Variables will change:
int buttonState;             // the current reading from the input pin
int lastButtonState = LOW;   // the previous reading from the input pin
int pos = 0;    // variable to store the servo position

// the following variables are unsigned longs because the time, measured in
// milliseconds, will quickly become a bigger number than can be stored in an int.
unsigned long lastDebounceTime = 0;  // the last time the output pin was toggled
unsigned long debounceDelay = 50;    // the debounce time; increase if the output flickers

void setup() {
  myservo.attach(5);  // attaches the servo on pin 5 to the servo object
  pinMode(buttonPin, INPUT);
  pinMode(buttonPin2, INPUT);
  pinMode(buttonPin3, INPUT);
  Serial.begin(9600);
  myservo.write(pos); // tells servo to start at positon 0

}

void loop() {
  // read the state of the switch into a local variable:
  int reading = digitalRead(buttonPin2);

  // check to see if you just pressed the button
  // (i.e. the input went from LOW to HIGH), and you've waited long enough
  // since the last press to ignore any noise:

  // If the switch changed, due to noise or pressing:
  if (reading != lastButtonState) {
    // reset the debouncing timer
    lastDebounceTime = millis();
  }

  if ((millis() - lastDebounceTime) > debounceDelay) {
    // whatever the reading is at, it's been there for longer than the debounce
    // delay, so take it as the actual current state:

    // if the button state has changed:
    if (reading != buttonState) {
      buttonState = reading;

      // only toggle the servo to move by 90 degrees and reset to 0 when it's at 180 if the new button state is HIGH
      // operates the rotation og the wrist
      /*if (buttonState == HIGH) {
        if(pos == 180)
        pos=0;
        else
        pos += 90;
        }*/
        // operates the index finger
      /*if (buttonState == HIGH) {
        //Serial.print(pos);
        //Serial.print("\n");
        if (pos >= 180) {
          for (pos = 180; pos >= 0; pos -= 1) {
            myservo.write(pos);              // tell servo to go to position in variable 'pos'
            delay(15);                       // waits 15ms for the servo to reach the position
          }
        }
        else {
          for (pos = 0; pos <= 180; pos += 1) {
            myservo.write(pos);              // tell servo to go to position in variable 'pos'
            delay(15);                       // waits 15ms for the servo to reach the position
          }
        }
        }
      */
      // operates the 3 fingers
      /*if (buttonState == HIGH) {
        //Serial.print(pos);
        //Serial.print("\n");
        if (pos >= 180) {
          for (pos = 180; pos >= 0; pos -= 1) {
            myservo.write(pos);              // tell servo to go to position in variable 'pos'
            delay(15);                       // waits 15ms for the servo to reach the position
          }
        }
        else {
          for (pos = 0; pos <= 180; pos += 1) {
            myservo.write(pos);              // tell servo to go to position in variable 'pos'
            delay(15);                       // waits 15ms for the servo to reach the position
          }
        }
      }*/
    }
  }

  myservo.write(pos);

  // save the reading. Next time through the loop, it'll be the lastButtonState:
  lastButtonState = reading;
}
