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
#define SERVOMIN  70 // This is the 'minimum' pulse length count (out of 4096)
//Original 150, changing it to 70
#define SERVOMAX  412 // This is the 'maximum' pulse length count (out of 4096)

/*Not used in this sketch
#define USMIN  600 // This is the rounded 'minimum' microsecond length based on the minimum pulse of 150
#define USMAX  2400 // This is the rounded 'maximum' microsecond length based on the maximum pulse of 600
*/

#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates

// our servo # counter
uint8_t servonum = 0;

int pulselen;
int pulselen_1;

// constants won't change. They're used here to set pin numbers:
const int buttonPin = 8;     // the number of the pushbutton pin

// variables will change:
int buttonState = 0;         // variable for reading the pushbutton status

String inString = "";    // string to hold input

void setup() {
  Serial.begin(9600);
  
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
  Serial.println("PWM Servo Calibration");
  
  // initialize the LED pin as an output:
  pinMode(LED_BUILTIN, OUTPUT);
  
  // initialize the pushbutton pin as an input:
  pinMode(buttonPin, INPUT);

  pwm.begin();
  // In theory the internal oscillator is 25MHz but it really isn't
  // that precise. You can 'calibrate' by tweaking this number till
  // you get the frequency you're expecting!
  pwm.setOscillatorFrequency(27000000);  // The int.osc. is closer to 27MHz  
  pwm.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates

  //To test min
  pulselen= SERVOMIN;

  //To test Max
  pulselen_1= SERVOMAX;
  
  pwm.setPWM(0, 0, pulselen);
  
  delay(10);
}

void loop() {
  /* This section is used to test serial input control movement, 
   * Ignore this for MIN/MAX test
   * 
  while (Serial.available() > 0) {
    int inChar = Serial.read();
    if (isDigit(inChar)) {
      // convert the incoming byte to a char and add it to the string:
      inString += (char)inChar;
    }
    // if you get a newline, print the string, then the string's value:
    if (inChar == '\n') {
      Serial.print("Value:");
      Serial.println(inString.toInt());
      Serial.print("String:");
      Serial.println(inString);
      // clear the string for new input:
      inString = "";
    }
  }
  */
  
  // read the state of the pushbutton value:
  buttonState = digitalRead(buttonPin);
  Serial.println(buttonState);

  // check if the pushbutton is pressed. If it is, the buttonState is HIGH:
  if (buttonState == HIGH){
      if (pulselen < SERVOMAX){
          //turn LED on:
          //digitalWrite(LED_BUILTIN, HIGH);
          
          //Print the PWM value to Serial Monitor
          Serial.print("Pulse Length is:");
          Serial.println(pulselen);
          
          //Drive the servo with PWM
          pwm.setPWM(0, 0, pulselen);
          //pwm.setPWM(1, 0, pulselen);

          //Increase PWM
          pulselen=pulselen+20;
          //pulselen_1=pulselen;
          //pulselen=pulselen-20;
          delay(500);
      }

      //Below section allow you to cycle through defined MIN/MAX
      //common out if needed.
      
      if (pulselen >= SERVOMAX){
          // turn LED on:
          //digitalWrite(LED_BUILTIN, HIGH);
          Serial.print("Pulse Length is:");
          Serial.println(pulselen_1);
          
          pwm.setPWM(0, 0, pulselen_1);
          //pwm.setPWM(1, 0, pulselen_1);
          pulselen_1=pulselen_1-20;
          
          delay(500);

          if(pulselen_1 <= SERVOMIN){
              pulselen = SERVOMIN;
              pulselen_1= SERVOMAX;
              
          }
      }
      
  } 
  
  
}
