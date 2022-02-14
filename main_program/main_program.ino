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

How to update code:
  Close serial port
  Unplug power
  Upload
  Plug in power
  Open serial port
*/
 
#include <SPI.h>
#include <WiFiNINA.h>
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include <Servo.h>

// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
// Or, create it with a different I2C address (say for stacking)
// Adafruit_MotorShield AFMS = Adafruit_MotorShield(0x61);

// Select which 'port' M1, M2, M3 or M4. In this case, M1
Adafruit_DCMotor *Motor1 = AFMS.getMotor(1);
Adafruit_DCMotor *Motor2 = AFMS.getMotor(2);


// setup line sensors
int line_pins[] = {8, 12, 7, 11};
int n_line_sensors = 4;
bool line_reading[4] = {0,0,0,0};
float line_follow_ratio = 0;
int n_sensors_high = 0;
int n_junction_readings = 0;
int n_junctions = 0;
int n_not_junction_readings = 0;
bool on_junction = false;
int initial_junctions;
int n_error_readings = 0;
bool off_line = false;
bool turning_sensors = false;
int sweep_distance = 0;


// ultrasonic definitions
const int trigPin = 3;
const int echoPin = 2;
long ultrasonic_duration;
int ultrasonic_distance;
int block_distance_readings = 0;
int block_distance;
bool block_infront = 0;

// servo setup
Servo clawservo; 
int servo_input = A0;
int pos_max = 140;
int pos_min = 70;
int pos = pos_min;

String mode = "manual";

// sort out wifi
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
int return_stage = 0;
String block_type;

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
  Serial.println("stopping motors");
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
  Motor1->run(BACKWARD);
  Motor2->run(FORWARD);
}

void motors_left_slow() {
  // set motors to turn left slowly
  Motor1->run(RELEASE);
  Motor2->run(FORWARD);
}

void motors_right_fast() {
  // set motors to turn right quickly
  Motor1->run(FORWARD);
  Motor2->run(BACKWARD);
}

void motors_right_slow() {
  // set motors to turn right slowly
  Motor1->run(FORWARD);
  Motor2->run(RELEASE);
}

void motors_turn_90(String direction) {
  // turn left or right 90 degrees
   Motor1->setSpeed(200);
   Motor2->setSpeed(200); 
  if (direction == "left") {
    Motor1->run(BACKWARD);
    Motor2->run(FORWARD);
  } else {
    Motor1->run(FORWARD);
    Motor2->run(BACKWARD);
  }

  delay(2000);
  motors_stop();
  motors_change_speed();
}

void motors_turn_180() {
  // turn 180 degrees
   Motor1->setSpeed(200);
   Motor2->setSpeed(200); 
   Motor1->run(FORWARD);
   Motor2->run(BACKWARD);

  delay(3400);
  motors_stop();

  turning_sensors = false;
  sweep_distance = 0;
  
  while (turning_sensors == false) {
   sweep_distance += 50;
   motors_right_fast();
   motors_change_speed();
   delay(sweep_distance);
   get_line_sensor_readings(false);
   n_sensors_high = 0;
   for (int i = 0; i < n_line_sensors; i++)  {
     n_sensors_high += line_reading[i];
   }
   if (n_sensors_high != 0) {
    turning_sensors = true;
   }
   else {
    sweep_distance += 50;
    motors_left_fast();
    motors_change_speed();
    delay(sweep_distance);
    get_line_sensor_readings(false);
    n_sensors_high = 0;
    for (int i = 0; i < n_line_sensors; i++)  {
      n_sensors_high += line_reading[i];
   }
      if (n_sensors_high != 0) {
      turning_sensors = true;
   }
   }
  }
  
  delay(100);
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
          client.print("Click <a href=\"/T180\">here</a> to set the mode to Turn 180<br>");
          

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
    
    motors_change_speed();

      }
      if (currentLine.endsWith("GET /medium")) {
        left_speed = 160;
        right_speed = 160;
    motors_change_speed();

      }
      if (currentLine.endsWith("GET /slow")) {
        left_speed = 60;
        right_speed = 60;
    motors_change_speed();

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
      if (currentLine.endsWith("GET /T180")) {
        motors_turn_180();
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
  clawservo.attach(10);  
  pinMode(servo_input, INPUT);
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

  drop_block();
  motors_stop();
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

bool block_detection() {
  // detects a block after 5 consecutive block readings
  // returns 1 if detected
  if (block_distance < 12){
    block_distance_readings += 1;
  }
  else{
    block_distance_readings = 0;
  }
  return (block_distance_readings > 5);
}

void decide_line_follow_speed(bool region_without_line) {
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
    left_speed = 250;
    right_speed = 250 - (line_follow_ratio * 100);
  }
  else {
    left_speed = 250 + (line_follow_ratio * 100);
    right_speed = 250;
  }  

  Serial.println(String(left_speed) + " , " + String(right_speed));
  // move slower if reversing
  if (off_line) {
    left_speed = 150;
    right_speed = 150;
  }

  // if enough junction readings detected, robot is at a junction
  if (n_sensors_high >= 3) {
    n_junction_readings += 1;
    n_not_junction_readings = 0;
  } else {
    n_junction_readings = 0;
    n_not_junction_readings += 1;
  }
  if (n_junction_readings == 3 && on_junction == false){
    n_junctions += 1; 
    on_junction = true;
    Serial.println(String(n_junctions) + " junctions");  
  }
  if (n_not_junction_readings == 3){
    on_junction = false;
  }

  if (region_without_line == false) {
    // if enough error readings detected, robot has left line
    // only check for this if robot is both line following and junction detecting (rather than only searching for a junction)
    if (n_sensors_high == 0) {
      n_error_readings += 1;
    } else {
      if (off_line) {
        // if line detected after reversing process, start going forward again
        delay(10);
        motors_forward();
      }
      n_error_readings = 0;
      off_line = false;
    }
    if (n_error_readings == 90){
      off_line = true;
      n_error_readings = 0;
    
      // start reversing
      left_speed = 150;
      right_speed = 150;
      motors_backward();
      Serial.println("lost line");
    }
  } else {
    left_speed = 250;
    right_speed = 250;
  }
}

void line_following(bool region_without_line){
  get_line_sensor_readings(true);
  decide_line_follow_speed(region_without_line);
    motors_change_speed();

 }


void pick_up_block() {
  for (pos = pos_max; pos >= pos_min; pos -= 1) {
     clawservo.write(pos);
     delay(15);
  }
}

void drop_block() {
  for (pos = pos_min; pos <= pos_max; pos += 1) {
     clawservo.write(pos);
     delay(15);
  }
}

String identify_block() {
  float sum_servo_input_readings = 0;
  float avg_servo_input_reading = 0;
  for (int i = 0; i < 100; i++) {                                   // Takes multiple readings
    sum_servo_input_readings += analogRead(servo_input);
  }
  avg_servo_input_reading = sum_servo_input_readings/100;           // Finds average
  if (avg_servo_input_reading > 650) {return "fine";}               // Determines if fine or coarse
  else {return "coarse";}
}

bool return_block_complete;

bool return_block(bool block_number) {
  Serial.println(String(return_stage) + " return stage");
  switch (return_stage) {
    case 0:
      // stage 0 = turn 180 degrees
      motors_turn_180();
      motors_backward();
      delay(500);
      motors_forward();
     break;

    case 1:
      // stage 1 = follow line up until red/blue junction
      
      int junctions_to_move;
      if (block_number == 0 || block_number == 1) junctions_to_move = 1;
      if (block_number == 2) junctions_to_move = 2;

      initial_junctions = n_junctions;
      while(n_junctions != initial_junctions + junctions_to_move) {
        line_following(false);
      }
      delay(1000);
      motors_stop();
     break;

    case 2:
      // s;tage 2 = turn left/right depending on block type
      if (block_type == "coarse") motors_turn_90("right");
      else if (block_type == "fine") motors_turn_90("left");
     break;

     case 3:
      // stage 3 = move to centre of box
      goto_centre_of_box();
     break;

     case 4:
       // stage 4 = drop block
       drop_block();
       motors_backward();
       delay(1000);
       motors_stop();
      break;

     case 5:
        // stage 5 = turn to face forward
        motors_forward();
        if (block_number == 2) {
          if (block_type == "coarse") motors_turn_90("left");
          else if (block_type == "fine") motors_turn_90("right");
        } else {
          if (block_type == "coarse") motors_turn_90("right");
          else if (block_type == "fine") motors_turn_90("left");        
        }
       // stage 5 = turn around
       //motors_turn_180();
      break;
      /*
     case 6:
        // stage 6 = get to central junction
        //motors_forward();
        //delay(500);
        /*initial_junctions = n_junctions;
        while(n_junctions != initial_junctions + 2) {
          line_following(false);
        }
        delay(500);
        //motors_stop();   
        break;

    case 7:
      // stage 7 = turn to face forward
      if (block_number == 2) {
        if (block_type == "coarse") motors_turn_90("left");
        else if (block_type == "fine") motors_turn_90("right");
      } else {
        if (block_type == "coarse") motors_turn_90("right");
        else if (block_type == "fine") motors_turn_90("left");        
      }
     break;*/
  }
  return_stage += 1;

  // return true if completed
  return (return_stage == 6);
}

void find_final_block() {
  // move forward in a straight line until block found or edge of box reached
  // (edge of box condition for error handling means if block is missed / ultrasonic sensor faulty,
  // robot can hopefully still return to start)
  
  initial_junctions = n_junctions;
  while(n_junctions <= initial_junctions + 1 && !block_detection()) {
    line_following(false);
    block_distance = get_distance_sensor_readings();
  }
  motors_stop();
}

void undo_find_final_block() {
  // move robot to entrance of final box
  motors_turn_180();
  initial_junctions = n_junctions;
  while(n_junctions <= initial_junctions + 1) {
    line_following(false);
  }
}

void goto_centre_of_box() {
  // drive to edge of block then reverse a bit
  motors_forward();
  delay(1000);
  motors_stop();
  /*initial_junctions = n_junctions;
  while(n_junctions <= initial_junctions + 1) {
    line_following(false);
  }

  motors_backward();
  delay(15);
  motors_stop();*/
}

  
void main_routine() {
  Serial.println(String(stage) + String("Stage"));
  switch (stage) {
    case 0:
      // stage 0 = leave start box
      left_speed = 200;
      right_speed = 200;
      motors_change_speed();
      motors_forward();
      delay(1500);
      
      /*Serial.println(n_junctions);
      while(n_junctions != 1) {
        line_following(true);
      }*/
     break;

    case 1:
      // stage 1 = follow line until block detected
      initial_junctions = n_junctions;
      do {
        line_following(false);
        //block_distance = get_distance_sensor_readings();
      } while (n_junctions < initial_junctions + 2);

      for (int i = 0; i < 20; i++) {
        line_following(false);
        delay(15);
      }
      motors_stop();
     break;

    case 2:
      // stage 2 = pick up & identify block
      pick_up_block();
      block_type = identify_block();
     break;

    case 3:
      // stage 3 = return block to correct box
      return_block_complete = return_block(0);
      while (return_block_complete == false) {
        return_block_complete = return_block(0);
        delay(5);  
      };
     break;

    case 4:
      // stage 4 = follow line until block detected
      motors_forward();
      do {
        line_following(false);
        block_distance = get_distance_sensor_readings();
      } while (!block_detection());
      delay(500);
      motors_stop();
     break;

    case 5:
      // stage 5 = pick up & identify block
      pick_up_block();
      block_type = identify_block();
     break;

    case 6:
      // stage 6 = return block to correct box
      return_stage = 0;
      while (!return_block(1)) {};
     break;

    case 7:
      // stage 7 = line follow until final junction (at furthest box)
      motors_forward();
      initial_junctions = n_junctions;
      while(n_junctions != initial_junctions + 2) {
        line_following(false);
      }
     break;

    case 8:
      // stage 8 = search for final block
      find_final_block();
     break;  

    case 9:
      // stage 9 = pick up & identify block
      pick_up_block();
      block_type = identify_block();
     break;

    case 10:
      // stage 10 = return to entrance of furthest box
      undo_find_final_block();
     break;

    case 11:
      // stage 11 = return block to correct box
      return_stage = 0;
      while (!return_block(2)) {};
     break;

    case 12:
      // stage 12 = return to entrance of start/stop area
      initial_junctions = n_junctions;
      while(n_junctions != initial_junctions + 1) {
        line_following(true);
      }
      motors_stop();
     break;

    case 13:
      // stage 13 = go to centre of start/stop area
      goto_centre_of_box();
     break;
  }

  Serial.println(("return stage %d complete", stage));
  stage += 1;
}

void loop() {
  
  if (mode == "auto") {
    line_following(false);
  } else if (mode == "main") {
    main_routine();
  }

  /*block_distance = get_distance_sensor_readings();
 
  if (block_detection()){
    Serial.println("Block");
  }*/

    // ------------ WIFI CODE -----------
  WiFiClient client = server.available();   // listen for incoming clients

  if (client) {   
    Serial.println("client");   
    handle_client(client);
  }
  // -------------------------------------

}
