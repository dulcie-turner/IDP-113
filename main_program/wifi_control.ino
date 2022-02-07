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
