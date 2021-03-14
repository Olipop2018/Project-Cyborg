/***************************************************
  This is an example for our Adafruit 16-channel PWM & Servo driver
  Servo test - this will drive 8 servos, one after the other on the
  first 8 pins of the PCA9685

  Pick one up today in the adafruit shop!
  ------> http://www.adafruit.com/products/815

  These drivers use I2C to communicate, 2 pins are required to
  interface.

  Adafruit invests time and resources providing this open source code,
  please support Adafruit and open-source hardware by purchasing
  products from Adafruit!

  Written by Limor Fried/Ladyada for Adafruit Industries.
  BSD license, all text above must be included in any redistribution
 ****************************************************/

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// called this way, it uses the default address 0x40
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
// you can also call it with a different address you want
//Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x41);
// you can also call it with a different address and I2C interface
//Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40, Wire);

// Depending on your servo make, the pulse width min and max may vary, you
// want these to be as small/large as possible without hitting the hard stop
// for max range. You'll have to tweak them as necessary to match the servos you
// have!
#define SERVOMIN  80 ///(76 for 2)(77 for the Ring servo)// This is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  350 //(573)(577 on ring servo) This is the 'maximum' pulse length count (out of 4096)
#define USMIN  600 // This is the rounded 'minimum' microsecond length based on the minimum pulse of 150
#define USMAX  2400 // This is the rounded 'maximum' microsecond length based on the maximum pulse of 600
#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates

// our servo # counter
uint8_t servonum = 0;
int pulselen;
int buttonState = 0;
const int buttonPin = 8;
void setup() {
  Serial.begin(9600);
  Serial.println("8 channel Servo test!");
  pinMode(buttonPin, INPUT);
  pwm.begin();
  /*
     In theory the internal oscillator (clock) is 25MHz but it really isn't
     that precise. You can 'calibrate' this by tweaking this number until
     you get the PWM update frequency you're expecting!
     The int.osc. for the PCA9685 chip is a range between about 23-27MHz and
     is used for calculating things like writeMicroseconds()
     Analog servos run at ~50 Hz updates, It is importaint to use an
     oscilloscope in setting the int.osc frequency for the I2C PCA9685 chip.
     1) Attach the oscilloscope to one of the PWM signal pins and ground on
        the I2C PCA9685 chip you are setting the value for.
     2) Adjust setOscillatorFrequency() until the PWM update frequency is the
        expected value (50Hz for most ESCs)
     Setting the value here is specific to each individual I2C PCA9685 chip and
     affects the calculations for the PWM update frequency.
     Failure to correctly set the int.osc value will cause unexpected PWM results
  */
  //pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates
  pulselen = SERVOMIN;
 pwm.setPWM(servonum, 0, pulselen);
  delay(10);
}



void loop() {
   buttonState = digitalRead(buttonPin);
  // Drive each servo one at a time using setPWM()
  if (buttonState == HIGH) {
    //if (pulselen < SERVOMAX) {


      //Print the PWM value to Serial Monitor
      Serial.print("Pulse Length is:");
      Serial.println(pulselen);

      //Drive the servo with PWM
      pwm.setPWM(servonum, 0, pulselen);
      // pwm.setPWM(4, 0, pulselen);
      //  pwm.setPWM(8, 0, pulselen);
      //   pwm.setPWM(12, 0, pulselen);
      //pwm.setPWM(1, 0, pulselen);

      //Increase PWM
    if(pulselen == SERVOMAX)
     pulselen = SERVOMIN;
      else
      pulselen = SERVOMAX;
      //pulselen_1=pulselen;
      //pulselen=pulselen+5;
      delay(500);
   // }

  }
}