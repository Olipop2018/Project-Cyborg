#include <Servo.h>

Servo WristServo;
Servo IndexFServo;
Servo MiddleFServo;
Servo RingFServo;
Servo LittleFServor;
Servo ThumbServo;

// button lists with related variables for debouncing
typedef struct Sensors_L {
  int buttonState; // the current reading from the input pin
  int lastButtonState;// the previous reading from the input pin
  int count;// variable to store the servo position
  unsigned long lastDebounceTime; // the last time the output pin was toggled

} MuscleSensors;

int readValue = 0;
int readValue2 = 0;
int readValue3 = 0;

int servPos[] = {
  0 //servPos[0](all fingers)
  , 0 //servPos[1](wrist Rotation sensor)
  , 0 //servPos[2](index sensor)
  , 0 //servPos[3](3Fingers)
  , 0 //servPos[4](thumb sensor)
};
/// initialize the button array of list
MuscleSensors Sensors[] = {
  { LOW, LOW, 0, 0}//Sensors[0](O/C sensor) and(index)
  , {LOW, LOW, 0, 0}//Sensors[1](3Fingers) and (thumb sensor)
  , { LOW, LOW, 0, 0}//Sensors[2](wrist Rotation sensor)
};

// constants won't change. They're used here to set pin numbers:
const int MSensorPin = A0;    // the number of the musscle sensor pin(O/C sensor) and(index)
const int MSensorPin2 = A1;    // the number of the pushbutton pin(3Fingers) and (thumb sensor)
const int MSensorPin3 = A3;    // the number of the pushbutton pin(wrist Rotation sensor)

// the following variables are unsigned longs because the time, measured in
// milliseconds, will quickly become a bigger number than can be stored in an int.
unsigned long debounceDelay = 50;    // the debounce time; increase if the output flickers

void setup() {

  IndexFServo.attach(2);
  ThumbServo.attach(3);
  MiddleFServo.attach(4);
  RingFServo.attach(5);
  LittleFServor.attach(6);
  WristServo.attach(7);
  Serial.begin(9600);

  IndexFServo.write(servPos[0]);
  ThumbServo.write(servPos[0]);
  MiddleFServo.write(servPos[0]);
  RingFServo.write(servPos[0]);
  LittleFServor.write(servPos[0]);
  WristServo.write(servPos[0]);

}

void OC() {// will close or open hand and all finger related servos will run
  /// will grab the current servo position for the index, thumb and the last 3 fingers
  int posIF = servPos[2];
  int pos3F = servPos[3];
  int posTF = servPos[4];
  if (servPos[0] >= 180) {
    ///this will move all relate fingers to a closed hand state according to their current prositions.
    for (servPos[0] = 180; servPos[0] >= 0; --servPos[0]) {
      if (posIF >= servPos[0])
        IndexFServo.write(servPos[0]);

      if (posTF >= servPos[0])
        ThumbServo.write(servPos[0]);

      if (pos3F >= servPos[0]) {
        MiddleFServo.write(servPos[0]);
        RingFServo.write(servPos[0]);
        LittleFServor.write(servPos[0]);
      }
      delay(15);                       // waits 15ms for the servo to reach the position
    }
  }
  else {
    ///this will move all relate fingers to a open hand state according to their current prositions.
    for (servPos[0] = 0; servPos[0] <= 180; ++servPos[0]) {

      if (posIF <= servPos[0])
        IndexFServo.write(servPos[0]);

      if (posTF <= servPos[0])
        ThumbServo.write(servPos[0]);

      if (pos3F <= servPos[0]) {
        MiddleFServo.write(servPos[0]);
        RingFServo.write(servPos[0]);
        LittleFServor.write(servPos[0]);
      }
      delay(15);                       // waits 15ms for the servo to reach the position
    }
  }
  servPos[2] = servPos[0];
  servPos[3] = servPos[0];
  servPos[4] = servPos[0];
}

void wristRotation() {
  // only toggle the servo to move by 90 degrees and reset to 0 when it's at 180 if the new button state is HIGH
  // operates the rotation of the wrist

  if (servPos[1] == 180)
    servPos[1] = 0;
  else
    servPos[1] += 90;
  WristServo.write(servPos[1]);

}

void indexMov() {
  // willt move the indexf finger from 0 to 180 and vice versa

  if (servPos[2] >= 180) {
    for (servPos[2] = 180; servPos[2] >= 0; --servPos[2]) {
      IndexFServo.write(servPos[2]);              // tell servo to go to position in variable 'pos'
      delay(15);                       // waits 15ms for the servo to reach the position
    }
  }
  else {
    for (servPos[2] = 0; servPos[2] <= 180; ++servPos[2]) {
      IndexFServo.write(servPos[2]);              // tell servo to go to position in variable 'pos'
      delay(15);                       // waits 15ms for the servo to reach the position
    }
  }

}

void fingers_3F() {
  // operates the 3 fingers, move the fringers from 0 to 180 and vice versa

  if (servPos[3] >= 180) {
    for (servPos[3] = 180; servPos[3] >= 0; --servPos[3]) {
      MiddleFServo.write(servPos[3]);
      RingFServo.write(servPos[3]);
      LittleFServor.write(servPos[3]);             // tell servo to go to position in variable 'pos'
      delay(15);                       // waits 15ms for the servo to reach the position
    }
  }
  else {
    for (servPos[3] = 0; servPos[3] <= 180; ++servPos[3]) {
      MiddleFServo.write(servPos[3]);
      RingFServo.write(servPos[3]);
      LittleFServor.write(servPos[3]);              // tell servo to go to position in variable 'pos'
      delay(15);                       // waits 15ms for the servo to reach the position
    }
  }

}

void ThumbMov() {
  // move the thum from 0 to 180
  // Serial.print("im in thumb move");
  if (servPos[4] >= 180) {
    for (servPos[4] = 180; servPos[4] >= 0; --servPos[4]) {
      ThumbServo.write(servPos[4]);              // tell servo to go to position in variable 'pos'
      delay(15);                       // waits 15ms for the servo to reach the position
    }
  }
  else {
    for (servPos[4] = 0; servPos[4] <= 180; ++servPos[4]) {
      ThumbServo.write(servPos[4]);              // tell servo to go to position in variable 'pos'
      delay(15);                       // waits 15ms for the servo to reach the position
    }
  }

}

void debounceSensorsCount(int reading, int num) {
  // check to see if you just pressed the button
  // (i.e. the input went from LOW to HIGH), and you've waited long enough
  // since the last press to ignore any noise:
  // If the switch changed, due to noise or pressing:
  if (reading != Sensors[num].lastButtonState) {
    // reset the debouncing timer
    Sensors[num].lastDebounceTime = millis();
  }

  if ((millis() - Sensors[num].lastDebounceTime) > debounceDelay) {
    // whatever the reading is at, it's been there for longer than the debounce
    // delay, so take it as the actual current state:

    // if the button state has changed:
    if (reading != Sensors[num].buttonState) {
      Sensors[num].buttonState = reading;
      //if the butteon was pressed under half a second increase the counter
      if (Sensors[num].buttonState == HIGH) {
        if (millis() - Sensors[num].lastDebounceTime < 500) {
          ++Sensors[num].count;
          /*if (num == 1)
            count2++;
            else
            count++;*/
        }
      }
    }
  }
  Sensors[num].lastButtonState = reading;
}

void loop() {
  // read the state of the switch into a local variable:
  int reading = analogRead(MSensorPin);
  int reading2 = analogRead(MSensorPin2);
  int reading3 = analogRead(MSensorPin3);
  readValue = map(reading, 0, 1023, 0, 255);
  readValue2 = map(reading2, 0, 1023, 0, 255);
  readValue3 = map(reading3, 0, 1023, 0, 255);

  debounceSensorsCount(reading, 0);
  //Serial.println(count);
  if (millis() - Sensors[0].lastDebounceTime > 500 && Sensors[0].count == 2) {
    OC();
    Sensors[0].count = 0;
  }
  else if (millis() - Sensors[0].lastDebounceTime > 500 && Sensors[0].count == 1) {

    indexMov();
    Sensors[0].count = 0;
  }

  debounceSensorsCount(reading2, 1);
  //Serial.println(count2);
  if (millis() - Sensors[1].lastDebounceTime > 500 && Sensors[1].count == 2) {
    fingers_3F();
    //Serial.println("3fingers move");
    Sensors[1].count = 0;
  }
  else if (millis() - Sensors[1].lastDebounceTime > 500 && Sensors[1].count == 1) {
    // buttons[3].buttonState = HIGH;
    ThumbMov();
    Sensors[1].count = 0;
  }

  debounceSensorsCount(reading3, 2);
  if (millis() - Sensors[2].lastDebounceTime > 500 && Sensors[2].count >= 1) {
    wristRotation();
    Sensors[2].count = 0;
  }

}
