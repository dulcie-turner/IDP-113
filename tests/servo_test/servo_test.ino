/* Sweep
 by BARRAGAN <http://barraganstudio.com>
 This example code is in the public domain.

 modified 8 Nov 2013
 by Scott Fitzgerald
 http://www.arduino.cc/en/Tutorial/Sweep
*/

#include <Servo.h>

Servo myservo;  // create servo object to control a servo


int pos_max = 150;
int pos_min = 30;

int pos = pos_min;    // variable to store the servo position
int pos_increment = 1;

int servo_input = A0;

void setup() {
  myservo.attach(10);  // attaches the servo on pin 9 to the servo object
  pinMode(servo_input, INPUT);

  Serial.begin(9600);
}

void loop() {
  
  myservo.write(pos);              // tell servo to go to position in variable 'pos'

  pos += pos_increment;
  if (pos == pos_max) pos = pos_min;  // flip servo direction of motion
  Serial.println(analogRead(servo_input));
  delay(15);
  
}
