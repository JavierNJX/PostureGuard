#include <Wire.h>
#include <WiFi.h>


const char* ssid = "abcd1234";            //replace
const char* password = "abcd1234";  //replace


WiFiServer server(80);
String header;
unsigned long currentTime = millis();
unsigned long previousTime = 0;
const long timeoutTime = 5000;
String status;
float RateRoll, RatePitch, RateYaw;
float RateRoll2, RatePitch2, RateYaw2;
float RateCalibrationRoll, RateCalibrationPitch, RateCalibrationYaw;
float RateCalibrationRoll2, RateCalibrationPitch2, RateCalibrationYaw2;
int RateCalibrationNumber;
float AccX, AccY, AccZ;
float AccX2, AccY2, AccZ2;

float AngleRoll, AnglePitch;
float AngleRoll2, AnglePitch2;
uint32_t LoopTimer;
float KalmanAngleRoll = 0, KalmanUncertaintyAngleRoll = 2 * 2;
float KalmanAngleRoll2 = 0, KalmanUncertaintyAngleRoll2 = 2 * 2;
float KalmanAnglePitch = 0, KalmanUncertaintyAnglePitch = 2 * 2;
float KalmanAnglePitch2 = 0, KalmanUncertaintyAnglePitch2 = 2 * 2;
float Kalman1DOutput[] = { 0, 0 };
void kalman_1d(float KalmanState, float KalmanUncertainty, float KalmanInput, float KalmanMeasurement) {
  KalmanState = KalmanState + 0.004 * KalmanInput;
  KalmanUncertainty = KalmanUncertainty + 0.004 * 0.004 * 4 * 4;
  float KalmanGain = KalmanUncertainty * 1 / (1 * KalmanUncertainty + 3 * 3);
  KalmanState = KalmanState + KalmanGain * (KalmanMeasurement - KalmanState);
  KalmanUncertainty = (1 - KalmanGain) * KalmanUncertainty;
  Kalman1DOutput[0] = KalmanState;
  Kalman1DOutput[1] = KalmanUncertainty;
}
void gyro_signals(int address) {
  Wire.beginTransmission(address);
  Wire.write(0x1B);
  Wire.write(0x08);
  Wire.endTransmission();
  Wire.beginTransmission(address);
  Wire.write(0x1A);
  Wire.write(0x05);
  Wire.endTransmission();
  Wire.beginTransmission(address);
  Wire.write(0x1C);
  Wire.write(0x10);
  Wire.endTransmission();
  Wire.beginTransmission(address);
  Wire.write(0x3B);
  Wire.endTransmission();
  Wire.requestFrom(address, 6);
  int16_t AccXLSB = Wire.read() << 8 | Wire.read();
  int16_t AccYLSB = Wire.read() << 8 | Wire.read();
  int16_t AccZLSB = Wire.read() << 8 | Wire.read();
  Wire.beginTransmission(address);
  Wire.write(0x43);
  Wire.endTransmission();
  Wire.requestFrom(address, 6);
  int16_t GyroX = Wire.read() << 8 | Wire.read();
  int16_t GyroY = Wire.read() << 8 | Wire.read();
  int16_t GyroZ = Wire.read() << 8 | Wire.read();

  if (address == 0x68) {
    AccX = (float)AccXLSB / 4096 - 0.02;  //Change accordingly
    AccY = (float)AccYLSB / 4096;
    AccZ = (float)AccZLSB / 4096 - 0.06;
    RateRoll = (float)GyroX / 65.5;
    RatePitch = (float)GyroY / 65.5;
    RateYaw = (float)GyroZ / 65.5;
    AngleRoll = atan(AccY / sqrt(AccX * AccX + AccZ * AccZ)) * 1 / (3.142 / 180);
    AnglePitch = -atan(AccX / sqrt(AccY * AccY + AccZ * AccZ)) * 1 / (3.142 / 180);
  } else if (address == 0x69) {
    AccX2 = (float)AccXLSB / 4096 - 0.03;  //Change accordingly
    AccY2 = (float)AccYLSB / 4096;
    AccZ2 = (float)AccZLSB / 4096 - 0.11;
    RateRoll2 = (float)GyroX / 65.5;
    RatePitch2 = (float)GyroY / 65.5;
    RateYaw2 = (float)GyroZ / 65.5;
    AngleRoll2 = atan(AccY2 / sqrt(AccX2 * AccX2 + AccZ2 * AccZ2)) * 1 / (3.142 / 180);
    AnglePitch2 = -atan(AccX2 / sqrt(AccY2 * AccY2 + AccZ2 * AccZ2)) * 1 / (3.142 / 180);
  }
}
void setup() {
  Serial.begin(57600);

  Wire.setClock(400000);
  Wire.begin();
  delay(250);
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();
  for (RateCalibrationNumber = 0; RateCalibrationNumber < 2000; RateCalibrationNumber++) {
    gyro_signals(0x68);
    RateCalibrationRoll += RateRoll;
    RateCalibrationPitch += RatePitch;
    RateCalibrationYaw += RateYaw;
    delay(1);
  }
  RateCalibrationRoll /= 2000;
  RateCalibrationPitch /= 2000;
  RateCalibrationYaw /= 2000;
  Wire.beginTransmission(0x69);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();
  for (RateCalibrationNumber = 0; RateCalibrationNumber < 2000; RateCalibrationNumber++) {
    gyro_signals(0x69);
    RateCalibrationRoll2 += RateRoll2;
    RateCalibrationPitch2 += RatePitch2;
    RateCalibrationYaw2 += RateYaw2;
    delay(1);
  }
  RateCalibrationRoll2 /= 2000;
  RateCalibrationPitch2 /= 2000;
  RateCalibrationYaw2 /= 2000;
  LoopTimer = micros();
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  // Print local IP address and start web server
  Serial.println("");
  Serial.println("WiFi connected.");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
  server.begin();
}
void loop() {
  WiFiClient client = server.available();  // Listen for incoming clients

  if (client) {  // If a new client connects,
    currentTime = millis();
    previousTime = currentTime;
    Serial.println("New Client.");  // print a message out in the serial port
    String currentLine = "";        // make a String to hold incoming data from the client
    while (client.connected() && currentTime - previousTime <= timeoutTime) {
      // loop while the client's connected
      currentTime = millis();
      if (client.available()) {  // if there's bytes to read from the client,
        gyro_signals(0x68);
        RateRoll -= RateCalibrationRoll;
        RatePitch -= RateCalibrationPitch;
        RateYaw -= RateCalibrationYaw;
        kalman_1d(KalmanAngleRoll, KalmanUncertaintyAngleRoll, RateRoll, AngleRoll);
        KalmanAngleRoll = Kalman1DOutput[0];
        KalmanUncertaintyAngleRoll = Kalman1DOutput[1];
        kalman_1d(KalmanAnglePitch, KalmanUncertaintyAnglePitch, RatePitch, AnglePitch);
        KalmanAnglePitch = Kalman1DOutput[0];
        KalmanUncertaintyAnglePitch = Kalman1DOutput[1];

        gyro_signals(0x69);
        RateRoll2 -= RateCalibrationRoll2;
        RatePitch2 -= RateCalibrationPitch2;
        RateYaw2 -= RateCalibrationYaw2;
        kalman_1d(KalmanAngleRoll2, KalmanUncertaintyAngleRoll2, RateRoll2, AngleRoll2);
        KalmanAngleRoll2 = Kalman1DOutput[0];
        KalmanUncertaintyAngleRoll2 = Kalman1DOutput[1];
        kalman_1d(KalmanAnglePitch2, KalmanUncertaintyAnglePitch2, RatePitch2, AnglePitch2);
        KalmanAnglePitch2 = Kalman1DOutput[0];
        KalmanUncertaintyAnglePitch2 = Kalman1DOutput[1];

        if (KalmanAngleRoll+KalmanAngleRoll2 > 45) {
          status = "Bad";
        } else if (KalmanAngleRoll+KalmanAngleRoll2 > 40) {
          status = "Ok";
        } else {
          status = "Good";
        }

        while (micros() - LoopTimer < 4000)
          ;
        LoopTimer = micros();

        char c = client.read();  // read a byte, then
        Serial.write(c);         // print it out the serial monitor
        header += c;
        if (c == '\n') {  // if the byte is a newline character
          // if the current line is blank, you got two newline characters in a row.
          // that's the end of the client HTTP request, so send a response:
          if (currentLine.length() == 0) {
            // HTTP headers always start with a response code (e.g. HTTP/1.1 200 OK)
            // and a content-type so the client knows what's coming, then a blank line:
            client.println("HTTP/1.1 200 OK");
            client.println("Content-type:text/html");
            client.println("Connection: close");
            client.println("");



            // Display the HTML web page
            client.println("<!DOCTYPE html><html>");
            client.println("<head><meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">");
            client.println("<link rel=\"icon\" href=\"data:,\">");
            // CSS to style the on/off buttons
            // Feel free to change the background-color and font-size attributes to fit your preferences
            client.println("<style>html { font-family: Helvetica; display: inline-block; margin: 0px auto; text-align: center;}");
            client.println("h1 { color:blue;}");
            client.println(".button { background-color: #4CAF50; border: none; color: white; padding: 16px 40px;");
            client.println("text-decoration: none; font-size: 30px; margin: 2px; cursor: pointer;}");
            client.println(".button2 {background-color: #555555;}</style></head>");

            // Web Page Heading
            client.println("<body><h1>Posture Guard</h1>");

            // Display current state, and ON/OFF buttons for GPIO 26
            client.printf("<p>Cervical Spine Angle - %.2f", KalmanAngleRoll*-1);
            client.println("&deg;");
            client.println("</p>");
            // If the output26State is off, it displays the ON button

            // Display current state, and ON/OFF buttons for GPIO 27
        

            client.printf("<p>Thoracic Spine Angle - %.2f", KalmanAngleRoll2*-1);
            client.println("&deg;");

            client.println("</p>");
            // If the output26State is off, it displays the ON button

            // Display current state, and ON/OFF buttons for GPIO 27
          
            client.println("<br/><br/><p>Your Posture :</p>");
            client.println("<h3>" + status + "</h3>");
            // If the output27State is off, it displays the ON button
            client.println("<canvas id='sketchCanvas' width='300' height='300'></canvas>");
            client.println("</body></html>");
            client.println("<script>");
            client.println("setTimeout(function() {");
            client.println("  location.reload();");
            client.println("}, 5000);");  // 5-second refresh interval
            client.println("function drawSketch(angleRoll, angleRoll2) {");
            client.println("  var canvas = document.getElementById('sketchCanvas');");
            client.println("  var ctx = canvas.getContext('2d');");
            client.println("  var centerX = canvas.width / 2;");
            client.println("  var centerY = canvas.height / 2;");
            client.println("  var length = 50; // Adjust the length of the line");
            client.println("  var endX = centerX + length * Math.cos(-angleRoll * (Math.PI / 180));");
            client.println("  var endY = centerY-30 + length * Math.sin(-angleRoll * (Math.PI / 180));");
            client.println("  var endX2 = endX + (length-30) * Math.cos(-angleRoll2 * (Math.PI / 180));");
            client.println("  var endY2 = endY + (length-30) * Math.sin(-angleRoll2 * (Math.PI / 180));");
            client.println("  var endX3 = endX + (length-20) * Math.cos(-angleRoll2 * (Math.PI / 180));");
            client.println("  var endY3 = endY + (length-20) * Math.sin(-angleRoll2 * (Math.PI / 180));");
            client.println("  ctx.clearRect(0, 0, canvas.width, canvas.height);");
            client.println("  ctx.beginPath();");
            client.println("  ctx.arc(endX3, endY3, 10, 0, 2 * Math.PI);");
            client.println("  ctx.moveTo(centerX, centerY);");
            client.println("  ctx.lineTo(centerX, centerY-30);");
            client.println("  ctx.lineTo(endX, endY);");
            client.println("  ctx.lineTo(endX2, endY2);");
            client.println("  ctx.moveTo(centerX, centerY);");
            client.println("  ctx.lineTo(centerX+50, centerY);");
            client.println("  ctx.lineTo(centerX+50, centerY+50);");
            client.println("ctx.lineTo(centerX+50, centerY+50);");
            client.println("ctx.moveTo(centerX-10, centerY-80);");
            client.println(" ctx.lineTo(centerX-10, centerY+10);");
            client.println("ctx.lineTo(centerX+40, centerY+10);");
            client.println("ctx.lineTo(centerX+40, centerY+60);");
            client.println("ctx.moveTo(centerX-10, centerY-70);");
            client.println("ctx.lineTo(centerX-10, centerY+60);");
            client.println("  ctx.lineWidth = 5;");
            client.println("  ctx.stroke();");
            client.println("}");
            client.println("function alertuser() {alert('do attempt to fix your posture');}");
            client.println("window.onload = function() {");
            client.printf("  drawSketch(%f, %f);", KalmanAngleRoll2, KalmanAngleRoll);
            client.println("var status = '" + status + "';");
            client.println("if(status=='Bad'){alertuser();}");
            client.println("};");
            client.println("</script>");

            // The HTTP response ends with another blank line
            client.println();
            // Break out of the while loop
            break;
          } else {  // if you got a newline, then clear currentLine
            currentLine = "";
          }
        } else if (c != '\r') {  // if you got anything else but a carriage return character,
          currentLine += c;      // add it to the end of the currentLine
        }
      }
    }
    // Clear the header variable
    header = "";
    // Close the connection
    client.stop();
    Serial.println("Client disconnected.");
    Serial.println("");
  }
}
