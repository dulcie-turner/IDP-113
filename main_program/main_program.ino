/* MAIN PROGRAM

How to use?
  Plug in the arduino and make sure everything's powered on
  Go onto your laptop and set up a mobile hotspot using the SSID and password written in arduino_secrets.h
  Upload the program
  Open the serial terminal and wait for the wifi to connect - eventually a link will be printed
  Using the same laptop, navigate to that link and follow the instructions to control it

If the wifi won't connect?
  Try restarting the computer with the hotspot

  HC-SR04 code from www.HowToMechatronics.com
*/

 
#include <SPI.h>
#include <WiFiNINA.h>
#include <Wire.h>
#include <Adafruit_MotorShield.h>

// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
// Or, create it with a different I2C address (say for stacking)
// Adafruit_MotorShield AFMS = Adafruit_MotorShield(0x61);

// Select which 'port' M1, M2, M3 or M4. In this case, M1
Adafruit_DCMotor *Motor1 = AFMS.getMotor(1);
Adafruit_DCMotor *Motor2 = AFMS.getMotor(2);


// setup line sensors
int line_pins[] = {1, 2, 3, 4};
int n_line_sensors = 4;
bool line_reading[4] = {0,0,0,0};
float line_follow_ratio = 0;
int n_sensors_high = 0;
int n_junction_readings = 0;
int n_junctions = 0;
int n_error_readings = 0;
bool off_line = false;


// ultrasonic definitions
const int trigPin = 6;
const int echoPin = 10;
long ultrasonic_duration;
int ultrasonic_distance;
int block_distance_readings = 0;
int block_distance;
bool block_infront = 0;

String mode = "manual";

// sort out wifi security
#include "arduino_secrets.h" 
char ssid[] = SECRET_SSID;        // your network SSID (name)
char pass[] = SECRET_PASS;    // your network password (use for WPA, or use as key for WEP)
int keyIndex = 0;                 // your network key Index number (needed only for WEP)

int status = WL_IDLE_STATUS;
WiFiServer server(80);

// motor speed control
int left_speed = 255;
int right_speed = 255;

int previous_left_speed = 0;
int previous_right_speed = 0;

int stage = 0;


void motors_change_speed() {
  Motor1->setSpeed(left_speed);
  Motor2->setSpeed(right_speed);  
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

void motors_backward() {
  // set motors to move backward continuously
  Motor1->run(BACKWARD);
  Motor2->run(BACKWARD);
}

void motors_left_fast() {
  // set motors to turn left quickly
  Motor1->run(FORWARD);
  Motor2->run(BACKWARD);
}

void motors_left_slow() {
  // set motors to turn left slowly
  Motor1->run(FORWARD);
  Motor2->run(RELEASE);
}

void motors_right_fast() {
  // set motors to turn right quickly
  Motor1->run(BACKWARD);
  Motor2->run(FORWARD);
}

void motors_right_slow() {
  // set motors to turn right slowly
  Motor1->run(RELEASE);
  Motor2->run(FORWARD);
}

void motors_turn_90(String direction) {
   Motor1->setSpeed(200);
   Motor2->setSpeed(200); 
  if (direction == "left") {
    Motor1->run(FORWARD);
    Motor2->run(BACKWARD);
  } else {
    Motor1->run(BACKWARD);
    Motor2->run(FORWARD);
  }

  delay(2000);
  motors_stop();
  motors_change_speed();
}


// handle somebody connecting to wifi server
void handle_client(WiFiClient client) {


  Serial.println("new client");           // print a message out the serial port
  String currentLine = "";                // make a String to hold incoming data from the client
  while (client.connected()) {            // loop while the client's connected
    if (client.available()) {             // if there's bytes to read from the client,
      char c = client.read();             // read a byte, then
      Serial.write(c);                    // print it out the serial monitor
      if (c == '\n') {                    // if the byte is a newline character

        // if the current line is blank, you got two newline characters in a row.
        // that's the end of the client HTTP request, so send a response:
        if (currentLine.length() == 0) {
          // HTTP headers always start with a response code (e.g. HTTP/1.1 200 OK)
          // and a content-type so the client knows what's coming, then a blank line:
          client.println("HTTP/1.1 200 OK");
          client.println("Content-type:text/html");
          client.println();

          // the content of the HTTP response follows the header:
          client.print("<style>* {font-size: 26pt} </style>");
          client.print("Click <a href=\"/F\">here</a> to set the motors to FORWARD<br>");
          client.print("Click <a href=\"/B\">here</a> to set the motors to BACKWARD<br>");
          client.print("Click <a href=\"/S\">here</a> to set the motors to STOP<br><br>");
          client.print("Click <a href=\"/SL\">here</a> to set the motors to SLOW LEFT<br>");
          client.print("Click <a href=\"/FL\">here</a> to set the motors to FAST LEFT<br>");
          client.print("Click <a href=\"/SR\">here</a> to set the motors to SLOW RIGHT<br>");
          client.print("Click <a href=\"/FR\">here</a> to set the motors to FAST RIGHT<br><br>");
          client.print("Click <a href=\"/fast\">here</a> to set the speed to FAST<br>");
          client.print("Click <a href=\"/medium\">here</a> to set the speed to MEDIUM<br>");
          client.print("Click <a href=\"/slow\">here</a> to set the speed to SLOW<br><br><br>");
          client.print("Click <a href=\"/line\">here</a> to set the mode to LINE FOLLOW<br>");
          client.print("Click <a href=\"/main\">here</a> to set the mode to MAIN PROGRAM<br>");
          client.print("Click <a href=\"/manual\">here</a> to set the mode to MANUAL<br>");
          client.print("Click <a href=\"/L90\">here</a> to set the mode to LEFT 90<br>");
          client.print("Click <a href=\"/R90\">here</a> to set the mode to RIGHT 90<br>");
          

          // The HTTP response ends with another blank line:
          client.println();
          // break out of the while loop:
          break;
        } else {    // if you got a newline, then clear currentLine:
          currentLine = "";
        }
      } else if (c != '\r') {  // if you got anything else but a carriage return character,
        currentLine += c;      // add it to the end of the currentLine
      }

      // Check client request for command
      if (currentLine.endsWith("GET /F")) {
        motors_forward();
      }
      if (currentLine.endsWith("GET /S")) {
        motors_stop();
      }
      if (currentLine.endsWith("GET /B")) {
        motors_backward();
      }
      if (currentLine.endsWith("GET /SL")) {
        motors_left_slow();
      }
      if (currentLine.endsWith("GET /FL")) {
        motors_left_fast();
      }
      if (currentLine.endsWith("GET /SR")) {
        motors_right_slow();
      }
      if (currentLine.endsWith("GET /FR")) {
        motors_right_fast();
      }
      if (currentLine.endsWith("GET /fast")) {
        left_speed = 250;
        right_speed = 250;
      }
      if (currentLine.endsWith("GET /medium")) {
        left_speed = 160;
        right_speed = 160;
      }
      if (currentLine.endsWith("GET /slow")) {
        left_speed = 60;
        right_speed = 60;
      }
      if (currentLine.endsWith("GET /line")) {
        mode = "auto";
        motors_forward();
      }
      if (currentLine.endsWith("GET /main")) {
        mode = "main";
        stage = 0;
      }
      if (currentLine.endsWith("GET /manual")) {
        mode = "manual";
        motors_stop();
        n_junctions = 0;
      }
      if (currentLine.endsWith("GET /L90")) {
        motors_turn_90("left");
      }
      if (currentLine.endsWith("GET /R90")) {
        motors_turn_90("right");
      }
    }
  }

  // close the connection:
  client.stop(); 
  Serial.println("client disonnected");
}

void printWifiStatus() {
  // print the SSID of the network you're attached to:
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  // print your board's IP address:
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);

  // print the received signal strength:
  long rssi = WiFi.RSSI();
  Serial.print("signal strength (RSSI):");
  Serial.print(rssi);
  Serial.println(" dBm");
  // print where to go in a browser:
  Serial.print("To see this page in action, open a browser to http://");
  Serial.println(ip);
}

void setup() {
  Serial.begin(9600);           // set up Serial library at 9600 bps

  AFMS.begin();  // create with the default frequency 1.6KHz
  //AFMS.begin(1000);  // OR with a different frequency, say 1KHz

  for (int i = 0; i < n_line_sensors; i++) {
    pinMode(line_pins[i], INPUT);
  }

  // ultrasonic pins
  pinMode(trigPin, OUTPUT); 
  pinMode(echoPin, INPUT); 

  // ----- CONNECTING TO WIFI ------

  // check for the WiFi module:
  if (WiFi.status() == WL_NO_MODULE) {
    Serial.println("Communication with WiFi module failed!");
    // don't continue
    while (true);
  }

  String fv = WiFi.firmwareVersion();
  if (fv < "1.0.0") {
    Serial.println("Please upgrade the firmware");
  }

  // attempt to connect to Wifi network:
  while (status != WL_CONNECTED) {
    Serial.print("Attempting to connect to Network named: ");
    Serial.println(ssid);                   // print the network name (SSID);

    // Connect to WPA/WPA2 network. Change this line if using open or WEP network:
    status = WiFi.begin(ssid, pass);
    // wait 10 seconds for connection:
    delay(10000);
    
  }
  server.begin();                           // start the web server on port 80
  printWifiStatus();                        // you're connected now, so print out the status

  // ----------------------------------
}


void get_line_sensor_readings(bool printResults) {
  // take readings from line sensor and (optionally) print
  
  for (int i = 0; i < n_line_sensors; i++) {
    line_reading[i] = digitalRead(line_pins[i]);
    if (printResults) {
      Serial.print(String(line_reading[i]));
    }
  }
  if (printResults) { Serial.println("\n-------");}
  
}

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

void block_detection() {
  if (block_distance < 15){
    block_distance_readings += 1;
  }
  else{
    block_distance_readings = 0;
  }
  if (block_distance_readings > 5){
    block_infront = 1;
  }
  else{
    block_infront = 0;
  }
  
}


void decide_line_follow_speed() {
  n_sensors_high = 0;
  float line_follow_ratio = 0;

  // proportional control
  for (int i = 0; i < n_line_sensors; i++)  {
    line_follow_ratio += line_reading[i]*i;
    n_sensors_high += line_reading[i];
  }
  if (n_sensors_high == 0) {
    line_follow_ratio = 0;
  }
  else {
    line_follow_ratio = (line_follow_ratio/n_sensors_high) -1.5;
  }

  // set motor speeds based on ratio
  if (line_follow_ratio > 0) {
    left_speed = 250 - (line_follow_ratio * 50);
    right_speed = 250;
  }
  else {
    left_speed = 250;
    right_speed = 250 + (line_follow_ratio * 50);
  }  
  if (off_line) {
    left_speed *= 2/3;
    right_speed *= 2/3;
  }

  // if enough junction readings detected, robot is at a junction
  if (n_sensors_high == 4) {
    n_junction_readings += 1;
  } else {
    n_junction_readings = 0;
  }
  if (n_junction_readings == 3){
    n_junctions += 1; 
    n_junction_readings = 0;
    Serial.println(String(n_junctions) + " junctions");  
  }

  // if enough error readings detected, robot has left line
  if (n_sensors_high == 0) {
    n_error_readings += 1;
  } else {
    if (off_line) {
      // if line detected after reversing process, start going forward again
      delay(5);
      motors_forward();
    }
    n_error_readings = 0;
    off_line = false;
  }
  if (n_error_readings == 30){
    off_line = true;
    n_error_readings = 0;

    // start reversing
    motors_backward();
    Serial.println("lost line");
  }
}

void line_following(){
  get_line_sensor_readings(true);
  decide_line_follow_speed();
 }

  


void loop() {

  if (mode == "auto") {
    line_following();
  }

  block_distance = get_distance_sensor_readings();
  block_detection();
  
  /* Serial.println(block_distance); */
  if (block_infront){
    Serial.println("Block");
  }

    // ------------ WIFI CODE -----------
  WiFiClient client = server.available();   // listen for incoming clients

  if (client) {   
    Serial.println("client");   
    handle_client(client);
  }
  // -------------------------------------

  if (left_speed != previous_left_speed || right_speed != previous_right_speed) {
    motors_change_speed();

    previous_left_speed = left_speed;
    previous_right_speed = right_speed;
  }
}
