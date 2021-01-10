#include <Servo.h>

Servo myservo;  // create servo object to control a servo
//Servo WristServo;
//Servo IndexServo;
//Servo 3FingerServo;
//Servo ThumbFlexServo;
//Servo ThumbAbdServo;

// button lists with related variables for debouncing
typedef struct Buttons_L {
  int buttonState; // the current reading from the input pin
  int lastButtonState;// the previous reading from the input pin
  int pos;// variable to store the servo position
  unsigned long lastDebounceTime; // the last time the output pin was toggled

} Buttons;
/// initialize the button array of list
Buttons buttons[] = {
  { LOW, LOW, 0, 0}//buttons[0](o/c sensor)
  , {LOW, LOW, 0, 0}//buttons[1](wrist Rotation sensor)
  , { LOW, LOW, 0, 0}//buttons[2](index sensor)
  , {LOW, LOW, 0, 0}//buttons[3](3Fingers)
  , { LOW, LOW, 0, 0}//buttons[4](thumb sensor)
};
// constants won't change. They're used here to set pin numbers:
const int buttonPin = 2;    // the number of the pushbutton pin(o/c sensor)
const int buttonPin2 = 8;    // the number of the pushbutton pin(wrist Rotation sensor)
const int buttonPin3 = 7;    // the number of the pushbutton pin(index sensor)
const int buttonPin4 = 4;    // the number of the pushbutton pin(3Fingers)
const int buttonPin5 = 12;    // the number of the pushbutton pin(thumb sensor)

////////potenttial problem: might need an extra sensor for the thumb since it uses 2 motors/////////


// the following variables are unsigned longs because the time, measured in
// milliseconds, will quickly become a bigger number than can be stored in an int.
//unsigned long lastDebounceTime = 0;  // the last time the output pin was toggled
unsigned long debounceDelay = 50;    // the debounce time; increase if the output flickers

void setup() {
  myservo.attach(5);  // attaches the servo on pin 5 to the servo object
  //WristServo.attach();
  //IndexServo.attach();
  //3FingerServo.attach();
  //ThumbFlexServo.attach();
  //ThumbAbdServo.attach();
  pinMode(buttonPin, INPUT);
  pinMode(buttonPin2, INPUT);
  pinMode(buttonPin3, INPUT);
  pinMode(buttonPin4, INPUT);
  pinMode(buttonPin5, INPUT);
  // digitalWrite(ledPin, ledState);
  Serial.begin(9600);
  myservo.write(buttons[0].pos); // tells servo to start at positon 0
  //WristServo.write(buttons[1].pos);
  //IndexServo.write(buttons[2].pos);
  //3FingerServo.write(buttons[3].pos);
  //ThumbFlexServo.write(buttons[4].pos);
  //ThumbAbdServo.write(buttons[5].pos);
}

void OC() {
  // will close or open hand
  //all finger related servos will run
  if (buttons[0].buttonState == HIGH) {
    if (buttons[0].pos >= 180) {
      for (buttons[0].pos = 180; buttons[0].pos >= 0; buttons[0].pos -= 1) {
        myservo.write(buttons[0].pos); // tell servo to go to position in variable 'pos'
        //IndexServo.write(buttons[0].pos);
        //3FingerServo.write(buttons[0].pos);
        //ThumbFlexServo.write(buttons[0].pos);
        //ThumbAbdServo.write(buttons[0].pos);
        delay(15);                       // waits 15ms for the servo to reach the position
      }
    }
    else {
      for (buttons[0].pos = 0; buttons[0].pos <= 180; buttons[0].pos += 1) {
        myservo.write(buttons[0].pos);              // tell servo to go to position in variable 'pos'
        //IndexServo.write(buttons[0].pos);
        //3FingerServo.write(buttons[0].pos);
        //ThumbFlexServo.write(buttons[0].pos);
        //ThumbAbdServo.write(buttons[0].pos);
        delay(15);                       // waits 15ms for the servo to reach the position
      }
    }
  }
}

void wristRotation() {
  // only toggle the servo to move by 90 degrees and reset to 0 when it's at 180 if the new button state is HIGH
  // operates the rotation of the wrist
  if (buttons[1].buttonState == HIGH) {
    if (buttons[1].pos == 180)
      buttons[1].pos = 0;
    else
      buttons[1].pos += 90;
    myservo.write(buttons[1].pos);
  }
}

void indexMov() {
  // willt move the indexf finger from 0 to 180 and vice versa
  if (buttons[2].buttonState == HIGH) {
    if (buttons[2].pos >= 180) {
      for (buttons[2].pos = 180; buttons[2].pos >= 0; buttons[2].pos -= 1) {
        myservo.write(buttons[2].pos);              // tell servo to go to position in variable 'pos'
        delay(15);                       // waits 15ms for the servo to reach the position
      }
    }
    else {
      for (buttons[2].pos = 0; buttons[2].pos <= 180; buttons[2].pos += 1) {
        myservo.write(buttons[2].pos);              // tell servo to go to position in variable 'pos'
        delay(15);                       // waits 15ms for the servo to reach the position
      }
    }
  }
}

void fingers_3F() {
  // operates the 3 fingers, move the fringers from 0 to 180 and vice versa
  if (buttons[3].buttonState == HIGH) {
    if (buttons[3].pos >= 180) {
      for (buttons[3].pos = 180; buttons[3].pos >= 0; buttons[3].pos -= 1) {
        myservo.write(buttons[3].pos);              // tell servo to go to position in variable 'pos'
        delay(15);                       // waits 15ms for the servo to reach the position
      }
    }
    else {
      for (buttons[3].pos = 0; buttons[3].pos <= 180; buttons[3].pos += 1) {
        myservo.write(buttons[3].pos);              // tell servo to go to position in variable 'pos'
        delay(15);                       // waits 15ms for the servo to reach the position
      }
    }
  }
}

void ThumbMov() {
  // move the thum from 0 to 180
  if (buttons[4].buttonState == HIGH) {
    if (buttons[4].pos >= 180) {
      for (buttons[4].pos = 180; buttons[4].pos >= 0; buttons[4].pos -= 1) {
        myservo.write(buttons[4].pos);              // tell servo to go to position in variable 'pos'
        delay(15);                       // waits 15ms for the servo to reach the position
      }
    }
    else {
      for (buttons[4].pos = 0; buttons[4].pos <= 180; buttons[4].pos += 1) {
        myservo.write(buttons[4].pos);              // tell servo to go to position in variable 'pos'
        delay(15);                       // waits 15ms for the servo to reach the position
      }
    }
  }
}

void debounceButtonsExecute(int reading, int num) {
  // check to see if you just pressed the button
  // (i.e. the input went from LOW to HIGH), and you've waited long enough
  // since the last press to ignore any noise:
  // If the switch changed, due to noise or pressing:
  if (reading != buttons[num].lastButtonState) {
    // reset the debouncing timer
    buttons[num].lastDebounceTime = millis();
  }

  if ((millis() - buttons[num].lastDebounceTime) > debounceDelay) {
    // whatever the reading is at, it's been there for longer than the debounce
    // delay, so take it as the actual current state:

    // if the button state has changed:
    if (reading != buttons[num].buttonState) {
      buttons[num].buttonState = reading;
      if (num == 0)
        OC();
      else if (num == 1)
        wristRotation();
      else if (num == 2)
        indexMov();
      else if (num == 3)
        fingers_3F();
      else if (num == 4)
        ThumbMov();
    }
  }
  buttons[num].lastButtonState = reading;
}

void loop() {
  // read the state of the switch into a local variable:
  int reading = digitalRead(buttonPin);
  int reading2 = digitalRead(buttonPin2);

  int reading3 = digitalRead(buttonPin3);
  //int reading = digitalRead(buttonPin4);
  //int reading = digitalRead(buttonPin5);
  //debounceButtons(reading, 1);
  debounceButtonsExecute(reading, 0);

  debounceButtonsExecute(reading2, 1);
  debounceButtonsExecute(reading3, 2);
  //debounceButtonsExecute(reading4, 3);
  // debounceButtonsExecute(reading5, 4);

}
