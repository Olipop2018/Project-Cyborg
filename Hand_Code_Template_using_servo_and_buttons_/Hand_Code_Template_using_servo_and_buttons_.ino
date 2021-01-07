#include <Servo.h>

Servo myservo;  // create servo object to control a servo
//Servo WristServo;
//Servo IndexServo;
//Servo 3FingerServo;
//Servo ThumbFlexServo;
//Servo ThumbAbdServo;
// constants won't change. They're used here to set pin numbers:
const int buttonPin = 2;    // the number of the pushbutton pin(o/c sensor)
const int buttonPin2 = 8;    // the number of the pushbutton pin(wrist Rotation sensor)
const int buttonPin3 = 7;    // the number of the pushbutton pin(index sensor)
const int buttonPin4 = 4;    // the number of the pushbutton pin(3Fingers)
const int buttonPin5 = 12;    // the number of the pushbutton pin(thumb sensor)

////////potenttial problem: might need an extra sensor for the thumb since it uses 2 motors/////////

// Variables will change:
int buttonState;             // the current reading from the input pin(o/c sensor)
int buttonState2;             // the current reading from the input pin(wrist Rotation sensor)
int buttonState3;             // the current reading from the input pin(index sensor)
int buttonState4;             // the current reading from the input pin(3Fingers)
int buttonState5;             // the current reading from the input pin(thumb sensor)

int lastButtonState = LOW;   // the previous reading from the input pin(o/c sensor)
int lastButtonState2 = LOW;  // the previous reading from the input pin(wrist Rotation sensor)
int lastButtonState3 = LOW;  // the previous reading from the input pin(index sensor)
int lastButtonState4 = LOW;  // the previous reading from the input pin(3Fingers)
int lastButtonState5 = LOW;  // the previous reading from the input pin(thumb sensor)

int pos = 0;    // variable to store the servo position(o/c sensor)
int pos2 = 0;    // variable to store the servo position(wrist Rotation sensor)
int pos3 = 0;    // variable to store the servo position(index sensor)
int pos4 = 0;    // variable to store the servo position(3Fingers)
int pos5 = 0;    // variable to store the servo position(thumb sensor)

// the following variables are unsigned longs because the time, measured in
// milliseconds, will quickly become a bigger number than can be stored in an int.
unsigned long lastDebounceTime = 0;  // the last time the output pin was toggled
unsigned long debounceDelay = 50;    // the debounce time; increase if the output flickers

void setup() {
  myservo.attach(5);  // attaches the servo on pin 5 to the servo object
  pinMode(buttonPin, INPUT);
  pinMode(buttonPin2, INPUT);
  pinMode(buttonPin3, INPUT);
  pinMode(buttonPin4, INPUT);
  pinMode(buttonPin5, INPUT);
  Serial.begin(9600);
  myservo.write(pos); // tells servo to start at positon 0

}
void wristRotation() {

  // only toggle the servo to move by 90 degrees and reset to 0 when it's at 180 if the new button state is HIGH
  // operates the rotation og the wrist
  if (buttonState2 == HIGH) {
    if (pos2 == 180)
      pos2 = 0;
    else
      pos2 += 90;
    myservo.write(pos2);
  }
}
void indexMov() {
  if (buttonState3 == HIGH) {
    //Serial.print(pos);
    //Serial.print("\n");
    if (pos3 >= 180) {
      for (pos3 = 180; pos3 >= 0; pos3 -= 1) {
        myservo.write(pos3);              // tell servo to go to position in variable 'pos'
        delay(15);                       // waits 15ms for the servo to reach the position
      }
    }
    else {
      for (pos3 = 0; pos3 <= 180; pos3 += 1) {
        myservo.write(pos3);              // tell servo to go to position in variable 'pos'
        delay(15);                       // waits 15ms for the servo to reach the position
      }
    }
  }
}
void fingers_3F() {
  // operates the 3 fingers
  if (buttonState4 == HIGH) {
    //Serial.print(pos4);
    //Serial.print("\n");
    if (pos4 >= 180) {
      for (pos4 = 180; pos4 >= 0; pos4 -= 1) {
        myservo.write(pos4);              // tell servo to go to position in variable 'pos'
        delay(15);                       // waits 15ms for the servo to reach the position
      }
    }
    else {
      for (pos4 = 0; pos4 <= 180; pos4 += 1) {
        myservo.write(pos4);              // tell servo to go to position in variable 'pos'
        delay(15);                       // waits 15ms for the servo to reach the position
      }
    }
  }
}
void ThumbMov() {
  if (buttonState5 == HIGH) {
    //Serial.print(pos5);
    //Serial.print("\n");
    if (pos5 >= 180) {
      for (pos5 = 180; pos5 >= 0; pos5 -= 1) {
        myservo.write(pos5);              // tell servo to go to position in variable 'pos'
        delay(15);                       // waits 15ms for the servo to reach the position
      }
    }
    else {
      for (pos5 = 0; pos5 <= 180; pos5 += 1) {
        myservo.write(pos5);              // tell servo to go to position in variable 'pos'
        delay(15);                       // waits 15ms for the servo to reach the position
      }
    }
  }
}
void OC() {

}
void debounceButtons(int reading) {
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

    }
  }
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
