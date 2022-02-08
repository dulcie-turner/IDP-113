/* Sweep
 by BARRAGAN <http://barraganstudio.com>
 This example code is in the public domain.

 modified 8 Nov 2013
 by Scott Fitzgerald
 http://www.arduino.cc/en/Tutorial/Sweep
*/

#include <Servo.h>

Servo myservo;  // create servo object to control a servo


//int pos_max = 150;
int pos_max = 135;
int pos_min = 30;

int pos = pos_min;    // variable to store the servo position
int pos_increment = 1;

int servo_input = A0;

void setup() {
  myservo.attach(10);  // attaches the servo on pin 9 to the servo object
  pinMode(servo_input, INPUT);

  Serial.begin(9600);
}

int count = 0;

void loop() {
  /*
  myservo.write(pos);              // tell servo to go to position in variable 'pos'

 
  if (pos >= pos_max) { count += 1; } else {
     pos += pos_increment;
  }
  if (count == 50) {
    pos = pos_min;  // flip servo direction of motion
    count = 0;
  }
  Serial.println(analogRead(servo_input));
  delay(15);
  */
  drop_block();
  delay(1000);
  pick_up_block();
  delay(1000);
  Serial.println(indentify_block());
  delay(1000);
}


// To add to main_program-------------------

void pick_up_block() {                                               //Picks up the block
  pos = pos_max;
  myservo.write(pos);
}

void drop_block(){                                                  // Drops the block
  pos = pos_min;
  myservo.write(pos);
}

String indentify_block() {                                          // Identifies the block
  float sum_servo_input_readings = 0;
  float avg_servo_input_reading = 0;
  for (int i = 0; i < 100; i++) {                                   // Takes multiple readings
    sum_servo_input_readings += analogRead(servo_input);
  }
  avg_servo_input_reading = sum_servo_input_readings/100;           // Finds average
  if (avg_servo_input_reading > 650) {return "fine";}               // Determines if fine or coarse
    else {return "coarse";}
}
