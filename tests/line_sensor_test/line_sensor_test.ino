/* LINE SENSOR TEST
 *  
 *  Prints all 4 line sensor readings to the Serial terminal repeatedly
 */

int line_pins[] = {1, 2, 3, 4};
int n_line_sensors = 4;
bool line_reading[4];

void setup() {
  Serial.begin(9600);           // set up Serial library at 9600 bps

  for (int i = 0; i < n_line_sensors; i++) {
    pinMode(line_pins[i], INPUT);
  }
}

void loop() {
  for (int i = 0; i < n_line_sensors; i++) {
    line_reading[i] = digitalRead(line_pins[i]);
    Serial.println("Line sensor - " + String(i + 1) + " - Reading - " + String(line_reading[i]));
    delay(500);
  }
  Serial.println("-------");

  }
