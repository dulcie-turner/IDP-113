#include <Servo.h>
#include <Adafruit_MotorShield.h>

Servo servoclaw;  // create servo object to control a servo

// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield();

// Select which 'port' M1, M2, M3 or M4. In this case, M1
Adafruit_DCMotor *Motor1 = AFMS.getMotor(2);
Adafruit_DCMotor *Motor2 = AFMS.getMotor(1);

int pos_max = 140;
int pos_min = 30;
int pos = pos_min;

// ultrasonic definitions
const int trigPin = 3;
const int echoPin = 2;
long ultrasonic_duration;
int ultrasonic_distance;
int block_distance_readings = 0;
int block_distance;
bool block_infront = 0;

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
  return (block_distance_readings > 3);
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

void pick_up_block() {
  for (pos = pos_max; pos >= pos_min; pos -= 1) {
     servoclaw.write(pos);
     delay(15);
  }
}

void drop_block() {
  for (pos = pos_min; pos <= pos_max; pos += 1) {
     servoclaw.write(pos);
     delay(15);
  }

}



void setup() {
  servoclaw.attach(10);

  
  // ultrasonic pins
  pinMode(trigPin, OUTPUT); 
  pinMode(echoPin, INPUT); 

  Serial.begin(9600);
  AFMS.begin();  // create with the default frequency 1.6KHz
   
  Motor1->setSpeed(200);
  Motor2->setSpeed(200);

  drop_block();
     
}

void loop() { 

  
  motors_forward();
  block_distance = get_distance_sensor_readings();
  Serial.println(String(block_distance));
  if (block_detection()) {
    Serial.println("detected");
    motors_stop();
    pick_up_block();
    motors_forward();
    delay(1000);
    drop_block();
    motors_stop();
    delay(500);
  }
  
}
