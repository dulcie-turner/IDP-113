/* Sweep
 by BARRAGAN <http://barraganstudio.com>
 This example code is in the public domain.

 modified 8 Nov 2013
 by Scott Fitzgerald
 http://www.arduino.cc/en/Tutorial/Sweep
*/

#include <Servo.h>
#include <Adafruit_MotorShield.h>

Servo myservo;  // create servo object to control a servo

// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
// Or, create it with a different I2C address (say for stacking)
// Adafruit_MotorShield AFMS = Adafruit_MotorShield(0x61);

// Select which 'port' M1, M2, M3 or M4. In this case, M1
Adafruit_DCMotor *Motor1 = AFMS.getMotor(2);
Adafruit_DCMotor *Motor2 = AFMS.getMotor(1);

//int pos_max = 150;
int pos_max = 135;
int pos_min = 30;

int pos = pos_min;    // variable to store the servo position
int pos_increment = 1;

int servo_input = A5;

// ultrasonic definitions
const int trigPin = 3;
const int echoPin = 2;
long ultrasonic_duration;
int ultrasonic_distance;
int block_distance_readings = 0;
int block_distance;
bool block_infront = 0;

void setup() {
  myservo.attach(10);  // attaches the servo on pin 9 to the servo object

  
  pinMode(servo_input, INPUT);

    // ultrasonic pins
  pinMode(trigPin, OUTPUT); 
  pinMode(echoPin, INPUT); 

  Serial.begin(9600);
   //AFMS.begin();  // create with the default frequency 1.6KHz


  
   //Motor1->setSpeed(200);
   //Motor2->setSpeed(200);

   
}

int count = 0;

int get_distance_sensor_readings() {
  // Clears the trigPin
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  // Sets the trigPin on HIGH state for 10 micro seconds
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  // Reads the echoPin, returns the sound wave travel time in microseconds
  ultrasonic_duration = pulseIn(echoPin, HIGH);
  // Calculating the distance ignoring 0 readings
  if (ultrasonic_duration != 0){
    ultrasonic_distance = ultrasonic_duration * 0.0343 / 2;
  }
  return ultrasonic_distance;
}

bool block_detection() {
  // detects a block after 5 consecutive block readings
  // returns 1 if detected
  if (block_distance < 15){
    block_distance_readings += 1;
  }
  else{
    block_distance_readings = 0;
  }
  return (block_distance_readings > 5);
}


void motors_forward() {
  // set motors to move forward continuously
  Motor1->run(FORWARD);
  Motor2->run(FORWARD);
}


void motors_stop() {
  // stop motors
  Motor1->run(RELEASE);
  Motor2->run(RELEASE);
}




void loop() {
        /*pick_up_block();
      delay(15);
        //motors_forward();
        //delay(10000);
        motors_stop();
        drop_block();
        //delay(5000);
        delay(15);

  //motors_forward();
  
  //get_distance_sensor_readings();
  /*if (true) {

  }*/
  
  /*myservo.write(pos);
  delay(15);

  pos += 1;
  if (pos > pos_max) pos = pos_min;
  /*delay(1000);
  motors_stop();

  drop_block();
  delay(100);*/
  myservo.write(pos_max);
  delay(30);
  myservo.write(pos_min);
  delay(30);
}


// To add to main_program-------------------

void pick_up_block() {                                               //Picks up the block
  pos = pos_min;
  myservo.write(pos);
}

void drop_block(){                                                  // Drops the block
  pos = pos_max;
  myservo.write(pos);
}

String identify_block() {                                          // Identifies the block
  float sum_servo_input_readings = 0;
  float avg_servo_input_reading = 0;
  for (int i = 0; i < 100; i++) {                                   // Takes multiple readings
    sum_servo_input_readings += analogRead(servo_input);
  }
  avg_servo_input_reading = sum_servo_input_readings/100;           // Finds average
  if (avg_servo_input_reading > 650) {return "fine";}               // Determines if fine or coarse
    else {return "coarse";}
}
