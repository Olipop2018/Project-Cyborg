#include <Servo.h>
#include <Wire.h>
//#include <Adafruit_PWMServoDriver.h>

//Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
//#define SERVOMIN  80 ///(76 for 2)(77 for the Ring servo)// This is the 'minimum' pulse length count (out of 4096)
//#define SERVOMAX  400 //(573)(577 on ring servo) This is the 'maximum' pulse length count (out of 4096)
//#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates
Servo IndexServo;
Servo F3Servos;
Servo Thumbservo;
enum MODE {LOCK, CALLIBRATE, OPERATION};
MODE mod = LOCK;
// button lists with related variables for debouncing
typedef struct Sensors_L {
  int buttonState; // the current reading from the input pin
  int lastButtonState;// the previous reading from the input pin
  int count;// variable to store the servo position
  unsigned long lastDebounceTime; // the last time the output pin was toggled
  int thres;
} MuscleSensors;
//const int thres = 750;
int thresMax = 0;
int thresMin = 0;
int thresMax2 = 0;
int thresMin2 = 0;
int cnt = 0;
int cnt2 = 0;
bool samp = false;
int buttonState = LOW; // the current reading from the input pin
int lastButtonState = LOW; // the previous reading from the input pin
int servPos[] = {
  180 //servPos[0](all fingers)
  , 0 //servPos[1](wrist Rotation sensor)
  , 180 //servPos[2](index sensor)
  , 180 //servPos[3](3Fingers)
  , 180 //servPos[4](thumb sensor)
};
/// initialize the button array of list
MuscleSensors Sensors[] = {
  { LOW, LOW, 0, 0, 0} //Sensors[0](O/C sensor) and(index)
  , {LOW, LOW, 0, 0}//Sensors[1](3Fingers) and (thumb sensor)
};

// constants won't change. They're used here to set pin numbers:
const int MSensorPin = A0;    // the number of the musscle sensor pin(O/C sensor) and(index)
const int MSensorPin2 = A1;    // the number of the pushbutton pin(3Fingers) and (thumb sensor)
const int buttonPin = 8;
const int redpin = 2;
const int greenpin = 3;
const int bluepin = 4;

//uint8_t WristServo = 0;
//uint8_t IndexFServo = 4;
//uint8_t F3Servo = 8;
//uint8_t ThumbServo = 12;
// the following variables are unsigned longs because the time, measured in
// milliseconds, will quickly become a bigger number than can be stored in an int.
unsigned long debounceDelay = 50;    // the debounce time; increase if the output flickers
unsigned long lastDebounceTime;
int pulselen;
void setup() {
  Thumbservo.attach(5);
  IndexServo.attach(6);
  F3Servos.attach(7);
  pinMode(buttonPin, INPUT);
  Serial.begin(9600);
  IndexServo.write(servPos[0]);
  F3Servos.write(servPos[0]);
  Thumbservo.write(servPos[0]);
  delay(10);

}

void OC() {// will close or open hand and all finger related servos will run
  /// will grab the current servo position for the index, thumb and the last 3 fingers
  int posIF = servPos[2];
  int pos3F = servPos[3];
  int posTF = servPos[4];
  if (servPos[0] >= 180) {
    ///this will move all relate fingers to a closed hand state according to their current prositions.
    for (servPos[0] = 180; servPos[0] >= 0; --servPos[0]) {
//      pulselen = map (servPos[0], 0, 180, SERVOMIN, SERVOMAX);
      if (posIF >= servPos[0])
        IndexServo.write(servPos[0]);
      // pwm.setPWM(IndexFServo, 0, pulselen);

      if (posTF >= servPos[0])
        Thumbservo.write(servPos[0]);// pwm.setPWM(ThumbServo, 0, pulselen);

      if (pos3F >= servPos[0])
        F3Servos.write(servPos[0]);// pwm.setPWM(F3Servo, 0, pulselen);


      delay(15);                       // waits 15ms for the servo to reach the position
    }
  }
  else {
    ///this will move all relate fingers to a open hand state according to their current prositions.
    for (servPos[0] = 0; servPos[0] <= 180; ++servPos[0]) {

      // pulselen = map (servPos[0], 0, 180, SERVOMIN, SERVOMAX);
      if (posIF <= servPos[0])
        IndexServo.write(servPos[0]);
      // pwm.setPWM(IndexFServo, 0, pulselen);

      if (posTF <= servPos[0])
        Thumbservo.write(servPos[0]);//  pwm.setPWM(ThumbServo, 0, pulselen);

      if (pos3F <= servPos[0])
        F3Servos.write(servPos[0]);// pwm.setPWM(F3Servo, 0, pulselen);

      delay(15);                       // waits 15ms for the servo to reach the position
    }
  }
  servPos[2] = servPos[0];
  servPos[3] = servPos[0];
  servPos[4] = servPos[0];
}
void RGB(int redval, int greenval, int blueval)
{
  analogWrite(redpin, redval);
  analogWrite(greenpin, greenval);
  analogWrite(bluepin, blueval);
}


void Pinch() {
  // only toggle the servo to move by 90 degrees and reset to 0 when it's at 180 if the new button state is HIGH
  // operates the rotation of the wrist
  //int pulselenP;
  if (servPos[1] == 180)
    servPos[1] = 0;
  else
    servPos[1] = 180;
  // pulselenP = map (servPos[1], 0, 180, SERVOMIN, SERVOMAX);
  IndexServo.write(servPos[1]); //pwm.setPWM(IndexFServo, 0, pulselenP);
  Thumbservo.write(servPos[1]);// pwm.setPWM(ThumbServo, 0, pulselenP);

}
//void wristRotation() {
//  // only toggle the servo to move by 90 degrees and reset to 0 when it's at 180 if the new button state is HIGH
//  // operates the rotation of the wrist
//  int pulselenW;
//  if (servPos[1] == 180)
//    servPos[1] = 0;
//  else
//    servPos[1] += 90;
//  pulselenW = map (servPos[1], 0, 180, SERVOMIN, SERVOMAX);
//  pwm.setPWM(WristServo, 0, pulselenW);
//
//}
//void indexMov() {
//  // willt move the indexf finger from 0 to 180 and vice versa
//  int pulselenI;
//  if (servPos[2] >= 180) {
//    for (servPos[2] = 180; servPos[2] >= 0; --servPos[2]) {
//      pulselenI = map (servPos[2], 0, 180, SERVOMIN, SERVOMAX);
//      pwm.setPWM(IndexFServo, 0, pulselenI);              // tell servo to go to position in variable 'pos'
//      delay(15);                       // waits 15ms for the servo to reach the position
//    }
//  }
//  else {
//    for (servPos[2] = 0; servPos[2] <= 180; ++servPos[2]) {
//      pulselenI = map (servPos[2], 0, 180, SERVOMIN, SERVOMAX);
//      pwm.setPWM(IndexFServo, 0, pulselenI);             // tell servo to go to position in variable 'pos'
//      delay(15);                       // waits 15ms for the servo to reach the position
//    }
//  }
//
//}
//
//void fingers_3F() {
//  // operates the 3 fingers, move the fringers from 0 to 180 and vice versa
//  int pulselen3F;
//  if (servPos[3] >= 160) {
//    for (servPos[3] = 160; servPos[3] >= 0; --servPos[3]) {
//      pulselen3F = map (servPos[3], 0, 180, SERVOMIN, SERVOMAX);
//      pwm.setPWM(F3Servo, 0, pulselen3F); // tell servo to go to position in variable 'pos'
//      delay(15);                       // waits 15ms for the servo to reach the position
//    }
//  }
//  else {
//    for (servPos[3] = 0; servPos[3] <= 160; ++servPos[3]) {
//      pulselen3F = map (servPos[3], 0, 180, SERVOMIN, SERVOMAX);
//      pwm.setPWM(F3Servo, 0, pulselen3F); // tell servo to go to position in variable 'pos'
//      delay(15);                       // waits 15ms for the servo to reach the position
//    }
//  }
//
//}
//
//void ThumbMov() {
//  // move the thum from 0 to 180
//  // Serial.print("im in thumb move");
//  int pulselenT;
//  if (servPos[4] >= 180) {
//    for (servPos[4] = 180; servPos[4] >= 0; --servPos[4]) {
//      pulselenT = map (servPos[4], 0, 180, SERVOMIN, SERVOMAX);
//      pwm.setPWM(ThumbServo, 0, pulselenT);              // tell servo to go to position in variable 'pos'
//      delay(15);                       // waits 15ms for the servo to reach the position
//    }
//  }
//  else {
//    for (servPos[4] = 0; servPos[4] <= 180; ++servPos[4]) {
//      pulselenT = map (servPos[4], 0, 180, SERVOMIN, SERVOMAX);
//      pwm.setPWM(ThumbServo, 0, pulselenT);              // tell servo to go to position in variable 'pos'
//      delay(15);                       // waits 15ms for the servo to reach the position
//    }
//  }
//
//}

void debounceSensorsCount(int reading, int num, int val) {
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
        if (millis() - Sensors[num].lastDebounceTime < 1000) {
          if (mod == OPERATION)
            ++Sensors[num].count;
          else if (mod == CALLIBRATE) {
            cnt = 1;
            Sensors[num].thres = val;
            //samp = false;
            Serial.print("after: Sensors[num].thres= ");
            Serial.println(Sensors[num].thres);
          }
        }
      }
    }
  }
  Sensors[num].lastButtonState = reading;
}
void operation() {
  Serial.print("after: Sensors[0].thres= ");
  Serial.println(Sensors[0].thres);
  Serial.print("after: Sensors[1].thres= ");
  Serial.println(Sensors[1].thres);
  thresMax = Sensors[0].thres + 100;
  thresMin = Sensors[0].thres - 100;
  thresMax2 = Sensors[1].thres + 100;
  thresMin2 = Sensors[1].thres - 100;

  // static int state3=0;
  // read the state of the switch into a local variable:
  int reading = analogRead(MSensorPin);
  int reading2 = analogRead(MSensorPin2);
  Serial.print("reading = ");
  Serial.println(reading);
  Serial.print("reading2 = ");
  Serial.println(reading2);
  // Serial.println(reading);
  if ((reading >= thresMin) && (reading <= thresMax))
    debounceSensorsCount(HIGH, 0, reading);
  else
    debounceSensorsCount(LOW, 0, reading);
  Serial.print("after: Sensors[0].count= ");
  Serial.println(Sensors[0].count);
  // toohigh(reading);
  //Serial.println(count);
  /*if (millis() - Sensors[0].lastDebounceTime > 1000 && Sensors[0].count == 2) {
    if (toohigh(reading) == false)
      OC();//ThumbMov();
    Sensors[0].count = 0;
    }*/
  if (millis() - Sensors[0].lastDebounceTime > 1000 && Sensors[0].count == 1) {
    if (toohigh(reading) == false) {
      RGB(0, 255, 0);
      OC();
      delay(500);
      RGB(0, 0, 0);
    }
    Sensors[0].count = 0;
  }
  // delay(100);

  if ((reading2 >= thresMin2) && (reading2 <= thresMax2))
    debounceSensorsCount(HIGH, 1, reading2);
  else
    debounceSensorsCount(LOW, 1, reading2);
  //Serial.println(count2);
  /* if (millis() - Sensors[1].lastDebounceTime > 1000 && Sensors[1].count == 2) {
     fingers_3F();
     //Serial.println("3fingers move");
     Sensors[1].count = 0;
    }*/
  Serial.print("after: Sensors[1].count= ");
  Serial.println(Sensors[1].count);
  if (millis() - Sensors[1].lastDebounceTime > 1000 && Sensors[1].count == 1) {
    // buttons[3].buttonState = HIGH;
    if (toohigh2(reading2) == false) {
      RGB(0, 255, 0);
      Pinch();
      delay(500);
      RGB(0, 0, 0);
    }

    Sensors[1].count = 0;
  }

  delay(100);
}
bool sample(int num, int pin) {
  Serial.println("in sample function");
  if (samp == false) {
    RGB(255, 0, 255);
    Sensors[num].thres += analogRead(pin);
    ++cnt;
    if (cnt == 5) {
      Sensors[num].thres = Sensors[num].thres / 5;
      samp = true;
      cnt = 0;
      RGB(0, 0, 0);
    }
    Serial.print("cnt in samp= ");
    Serial.println(cnt);
    delay(1000);
  }

  Serial.print("sample= ");
  Serial.println(samp);
  return samp;

}
void callibrate(int num, int pin) {
  static int state = 0;
  static int reading = 0;
  Serial.println("in callibrate function");
  bool spl = sample(num, pin);
  if (spl == true) {
    Serial.print("before: Sensors[num].thres= ");
    Serial.println(Sensors[num].thres);
    RGB(255, 255, 0);//yellow
    reading = analogRead(pin);
    if (reading >= Sensors[num].thres + 100)
      state = HIGH;
    else
      state = LOW;
    Serial.print("before: Sensors[num].thres= ");
    Serial.println(Sensors[num].thres);
    debounceSensorsCount(state, num, reading);
    Serial.print("cnt= ");
    Serial.println(cnt);
    if (millis() - Sensors[num].lastDebounceTime > 1000 && (cnt == 1)) {
      samp = false;
      RGB(0, 0, 0);
      if (cnt2 == 0) {
        cnt2 = 1;
        cnt = 0;
      }
      else if (cnt2 == 1)
        cnt2 = 2;
    }

  }
}
void changeModes(int reading) {
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
      //if the butteon was pressed under half a second increase the counter
      if (buttonState == HIGH) {
        if (millis() - lastDebounceTime < 1000) {
          ++cnt;
        }
      }
    }
  }
  lastButtonState = reading;
}

bool toohigh(int val) {
  if (val > thresMax) {
    RGB(255, 0, 0);
    delay(200);
    RGB(0, 0, 0);
    return true;
  }
  else if (val < thresMin) {
    RGB(0, 0, 255);
    delay(200);
    RGB(0, 0, 0);
    return false;
  }
  else
    return false;
}
bool toohigh2(int val) {
  if (val > thresMax2) {
    RGB(255, 0, 0);
    delay(200);
    RGB(0, 0, 0);
    return true;
  }
  else if (val < thresMin2) {
    RGB(0, 0, 255);
    delay(200);
    RGB(0, 0, 0);
    return false;
  }
  else
    return false;
}
void loop() {
  int reading = digitalRead(buttonPin);
  changeModes(reading);
  if (mod == LOCK) {
    Serial.println("LOCK mode");
    if (millis() - lastDebounceTime > 1000 && (cnt == 1)) {
      mod = OPERATION;
      Serial.println("switch to operation mode");
      cnt = 0;
    }
    else  if (millis() - lastDebounceTime > 1000 && (cnt == 2)) {
      mod = CALLIBRATE;
      Serial.println("switch to callibration mode");
      Sensors[0].thres = 0;
      Sensors[1].thres = 0;
      delay(5000);
      cnt = 0;
    }

  }
  else if (mod == CALLIBRATE) {
    // if (cnt == 0)
    //  callibrate(0, MSensorPin);
    if (cnt2 == 0) {
      Serial.println("callibration mode");
      callibrate(0, MSensorPin);

    }
    else if (cnt2 == 1) {
      callibrate(1, MSensorPin2);

    }
    else if (cnt2 == 2) {
      mod = LOCK;
      cnt = 0;
      cnt2 = 0;
    }
  }
  else if (mod == OPERATION)

    if (millis() - lastDebounceTime > 1000 && (cnt == 1)) {
      mod = LOCK;
      Serial.println("switch to LOCK mode");
      cnt = 0;
    }
//    else  if (millis() - lastDebounceTime > 1000 && (cnt == 2)) {
//      mod = CALLIBRATE;
//      Serial.println("switch to callibration mode");
//      Sensors[0].thres = 0;
//      Sensors[1].thres = 0;
//      delay(5000);
//      cnt = 0;
//    }
    else
      operation();
}
