/*
  HC-SR04 code from www.HowToMechatronics.com
*/

// ultrasonic definitions
const int trigPin = 3; // green wire
const int echoPin = 2;
long ultrasonic_duration;
int ultrasonic_distance;

void setup() {
  // ultrasonic pins
  pinMode(trigPin, OUTPUT); 
  pinMode(echoPin, INPUT); 
  
  Serial.begin(9600); // Starts the serial communication
}

void loop() {
  // Clears the trigPin
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  // Sets the trigPin on HIGH state for 10 micro seconds
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  // Reads the echoPin, returns the sound wave travel time in microseconds
  ultrasonic_duration = pulseIn(echoPin, HIGH);
  // Calculating the distance
  ultrasonic_distance = ultrasonic_duration * 0.0343 / 2;

  Serial.println(ultrasonic_distance);
  delay(100);
  
}
