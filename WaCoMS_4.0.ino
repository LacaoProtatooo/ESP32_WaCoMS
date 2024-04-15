/* WaComs
 Inputs: 
  GPIO: 39 - 1pin: Ph-Level Sensor   
  GPIO: 34 - 1pin: Water-Flow Sensor
  GPIO: SCL 22, SDA 21 - 2pins: Real Time Clock

  Ouputs:
  GPIO: TXD 17, RXD 16 - 2pins: GSM Module
  GPIO: MISO 19, MOSI 23, SCK 18, CS 5 - 4pins: SD Card Module
  GPIO: SCL 22, SDA21 - 2pins: OLED

  Others:
  GPIO: 27 - 1pin: Float Switch
  GPIO: 32 - 1pin: Relay: DC Motor
  GPIO: 33 - 1pin: Relay: DC Motor
*/

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <SoftwareSerial.h>
#include <RTClib.h>

#include <WiFi.h>
#include <WebServer.h>

#include "FS.h"
#include "SD.h"
#include "SPI.h"

// PINS =============================================================================

#define FLOAT_SENSOR_PIN 27
#define PH_TESTER_PIN 39
#define WATER_FLOW_PIN 34
#define DC_CONTROL_PIN 32
#define DC_CONTROL_PIN_2ND 33

#define GSM_TXD_PIN 17
#define GSM_RXD_PIN 16

// CONFIG ===========================================================================

// RTC
RTC_DS3231 rtc;
char daysOfTheWeek[7][12] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};
String dateTimeString;

// GSM
SoftwareSerial mySerial(GSM_TXD_PIN, GSM_RXD_PIN); // RX , TX
String gsm_prev_msg = "";

// OLED
#define SCREEN_WIDTH 128  // OLED display width, in pixels
#define SCREEN_HEIGHT 32  // OLED display height, in pixels
#define OLED_RESET    -1  // Reset pin # (or -1 if sharing reset pin)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// WEB SERVER
WebServer server(80);

const char* ssid = "STARLINK X";
const char* password = "starlink"; 

// VARIABLES ========================================================================

// Message Stopper
bool wtrfill = false;
int sdrecord = 1;

// PH-Level Sensor
float calibration_value = 21.34 - 0.7;
unsigned long int avgval; 
int buffer_arr[10],temp;
float ph_act = 0; // PH LEVEL 
float prev_ph_act = 0;


// Water Flow Sensor
volatile int pulseCount = 0; // Pulse counter
float calibrationFactor = 4.5; // Calibration factor for the flow sensor
float flowRate;                // Flow rate in L/min
unsigned int flowMilliLitres;  // Flow in milliliters
unsigned long totalMilliLitres; // Total flow since program start
unsigned long oldTime;

// FUNCTIONS ========================================================================

// DC Motors
void runDCMotor(){
  digitalWrite(DC_CONTROL_PIN, LOW);
  digitalWrite(DC_CONTROL_PIN_2ND, LOW);
}
void stopDCMotor(){
  digitalWrite(DC_CONTROL_PIN, HIGH);
  digitalWrite(DC_CONTROL_PIN_2ND, HIGH);
}

// Real Time Clock Module (Update dateTimeString Variable)
void checkRTC() {
  DateTime now = rtc.now();
  
  // Format of Date and Time to Global Variable dateTimeString
  dateTimeString =
  String(now.year(), DEC) + "/" +
  String(now.month(), DEC) + "/" +
  String(now.day(), DEC) + " " +
  String(now.hour(), DEC) + ":" +
  String(now.minute(), DEC) ;         
}



// Oled Date and Time Display (FULL)
void displayDateTime(){
  DateTime now = rtc.now();
  checkRTC();

  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);
  display.setCursor(0, 0); // First Line
  display.print("Day: ");
  display.print(daysOfTheWeek[now.dayOfTheWeek()]);
  display.print("  Date: ");
  display.print(dateTimeString.substring(0, 10)); // Display only the date part

  display.setCursor(0, 10); // Second Line
  display.print("Time: ");
  display.println(dateTimeString.substring(11)); // Display only the time part

  display.display();
}

// Oled Display Message
void displayOLED(String message, String message2nd = ""){
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);

  display.setCursor(0, 0); // First Line
  display.println(message);

  display.setCursor(0, 10); // Second Line
  if (message2nd != "") {
    display.println(message2nd);
  }

  display.display();
  delay(3000);
  display.clearDisplay();
}

void checkWaterPHLevel(){
  for(int i=0;i<10;i++) 
    { 
    buffer_arr[i]=analogRead(PH_TESTER_PIN);
    delay(30);
    }

  for(int i=0;i<9;i++)
    {
      for(int j=i+1;j<10;j++)
      {
      if(buffer_arr[i]>buffer_arr[j])
        {
        temp=buffer_arr[i];
        buffer_arr[i]=buffer_arr[j];
        buffer_arr[j]=temp;
        }
      }
    }     

  avgval=0;
  ph_act=0;

  for(int i=2;i<8;i++){
    avgval+=buffer_arr[i];
  }
  
  float volt=(float)avgval*5.0/1024/6; 
  ph_act = -5.70 * volt + calibration_value;

  float temp= (volt - 0.5) * 100.0;

  Serial.print("pH Val: ");
  Serial.println(ph_act);

  if(ph_act > 4.0 && ph_act < 8.5) {
    // SAFE FOR CROPS GOOD PH LEVEL
    displayOLED("PH Level : " + String(ph_act),"Safe for Storing");

  } else { // STOP DC MOTOR
    stopDCMotor();
    sendSDCardMessage("PH Level out of range: Not good for Storing. Recommended PH level: (PH : 4.0 - PH : 8.5)");
    sendGSMMessage("Your PH Sensor Level out of range: Not good for Storing Water.");
    
    displayOLED("PH Level : " + String(ph_act),"PH-Level Too Acidic / Alkalotic for Crops PH Level (PH : 4.0 - PH : 8.5)");
  }
}

void checkWaterFlowRate(){
  unsigned long currentTime = millis();
  if ((currentTime - oldTime) > 1000) { // Process counters once per second
    detachInterrupt(digitalPinToInterrupt(WATER_FLOW_PIN)); // Disable interrupt
    
    // Calculate flow rate and total flow
    flowRate = (pulseCount * 1000.0) / (calibrationFactor * (currentTime - oldTime)); // in mL/min
    flowMilliLitres += flowRate; // Update total flow
    
    // Print and display flow rate and total flow
    String flowRateMsg = "Flow rate: " + String(flowRate) + " mL/min";
    String totalFlowMsg = "Total flow: " + String(flowMilliLitres) + " mL";
    displayOLED(flowRateMsg, totalFlowMsg);

    // Reset pulse counter and update time
    pulseCount = 0;
    oldTime = currentTime;

    Serial.println(flowRate);

    // Re-enable interrupt
    attachInterrupt(digitalPinToInterrupt(WATER_FLOW_PIN), pulseCounter, FALLING);
  }
}

void pulseCounter() {
  pulseCount++;
}

void sendGSMMessage(String message){
  Serial.println("Message to be sent to GSM: ");
  Serial.println(message);

  if(gsm_prev_msg == message){
    //Serial.println("Message to be sent repeated, invalidating gsm message");
    //return;
  }

  const char *smsContent = message.c_str();

  mySerial.println("AT+CMGF=1"); // Configuring TEXT mode
  updateSerial();
  mySerial.println("AT+CMGS=\"+639157513227\"\r"); // Number to be sent into
  updateSerial();
  mySerial.println(smsContent); // text content
  updateSerial();
  mySerial.println((char)26);

  gsm_prev_msg = message;
}

// SD CARD
void sendSDCardMessage(String message) {
  Serial.println("Storing Message to SD Card...");
  checkRTC();

  // Remove unsupported characters from datetime string
  String sanitizedDateTimeString = dateTimeString;
  String timeString = dateTimeString.substring(dateTimeString.indexOf(' ') + 1);
  sanitizedDateTimeString.replace("/", "_");
  sanitizedDateTimeString.replace(":", "_");
  timeString.replace(":", "_");

  String filename = "/record_" + (String)sdrecord + "_" + sanitizedDateTimeString + ".txt";
  //String filename = "/record_" + (String)sdrecord + "_" + timeString + ".txt";
  //Serial.println("SD filename: " + filename);
  
  message += " Timestamp: " + dateTimeString;

  const char* ffilename = filename.c_str();
  const char* mmessage = message.c_str();

  writeFile(SD, ffilename, mmessage);

  Serial.println("After File Write:");
  listDir(SD, "/", 0);

  sdrecord++;
}

void checkWaterLevel(){
  int floatbutton = 1;
  floatbutton = digitalRead(FLOAT_SENSOR_PIN);

  if (floatbutton == HIGH) { // NOT FULL
    wtrfill = false;
    runDCMotor();
    displayOLED("Water Reservoir:", "Not Full");
    Serial.println("Water Reservoir: Fillable");
  }
  else {  // FULL TANK : MUST STOP THE DC MOTOR
    stopDCMotor();
    Serial.println("Water Reservoir: Full");
    displayOLED("Water Reservoir:", "Full");

    if(wtrfill == false){
      sendSDCardMessage("Water Reservoir is Full");
      sendGSMMessage("Your Water Reservoir is Full");

      wtrfill = true;
    }
  }
}

void updateSerial()
{
  delay(300);
  while (Serial.available()) 
  {
    mySerial.write(Serial.read());//Forward what Serial received to Software Serial Port
  }
  while(mySerial.available()) 
  {
    Serial.write(mySerial.read());//Forward what Software Serial received to Serial Port
  }
}

// SD CARD FUNCTIONS (MODIFY) =============================== !!
void listDir(fs::FS &fs, const char * dirname, uint8_t levels){
  Serial.printf("Listing directory: %s\n", dirname);

  File root = fs.open(dirname);
  if(!root){
    Serial.println("Failed to open directory");
    return;
  }
  if(!root.isDirectory()){
    Serial.println("Not a directory");
    return;
  }

  File file = root.openNextFile();
  while(file){
    if(file.isDirectory()){
      Serial.print("  DIR : ");
      Serial.println(file.name());
      if(levels){
        listDir(fs, file.name(), levels -1);
      }
    } else {
      Serial.print("  FILE: ");
      Serial.print(file.name());
      Serial.print("  SIZE: ");
      Serial.println(file.size());
    }
    file = root.openNextFile();
  }
}

void writeFile(fs::FS &fs, const char * path, const char * message){
  Serial.printf("Writing file: %s\n", path);

  Serial.print("SD Card Message: ");
  Serial.println(message);

  File file = fs.open(path, FILE_WRITE);
  delay(3000);
  if(!file){
    Serial.println("Failed to open file for writing");
    return;
  }
  if(file.print(message)){
    Serial.println("File written");
  } else {
    Serial.println("Write failed");
  }
  file.close();
}
// END OF SD CARD FUNCTIONS ================================= !!

// SETUP FUNCTIONS (MUST IN VOID SETUP) ===========================================================

void pinSetups(){
  pinMode(DC_CONTROL_PIN, OUTPUT);
  pinMode(DC_CONTROL_PIN_2ND, OUTPUT);
  pinMode(WATER_FLOW_PIN, INPUT_PULLUP);
  pinMode(FLOAT_SENSOR_PIN, INPUT_PULLUP);
  stopDCMotor();

}

void setup_RTC(){
  rtc.begin(); 

  if (!rtc.begin()) 
  {
    Serial.println("Couldn't find RTC");
    while (1);
  }

  if (rtc.lostPower()) {
    Serial.println("RTC lost power, let's set the time!");
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }

  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { 
    Serial.println(F("SSD1306 allocation failed"));
    for(;;); // Don't proceed, loop forever
  }
  display.display();
  delay(2000);

  rtc.adjust(DateTime(__DATE__, __TIME__));
  
  Serial.println("display initialization success");
}

void setup_WaterFlow() {
  attachInterrupt(digitalPinToInterrupt(WATER_FLOW_PIN), pulseCounter, FALLING);
  pulseCount = 0;
  flowRate = 0.0;
  flowMilliLitres = 0;
  totalMilliLitres = 0;
  oldTime = 0;
}

void setup_SDCard() {
  Serial.println("Setting Up SD Card Module...");
  delay(2000);

  if(!SD.begin(5)){
    Serial.println("Card Mount Failed");
    return;
  }
  uint8_t cardType = SD.cardType();

  if(cardType == CARD_NONE) {
    Serial.println("No SD card attached");
    return;
  }
  uint64_t cardSize = SD.cardSize() / (1024 * 1024);
  Serial.printf("SD Card Size: %lluMB\n", cardSize);

  listDir(SD, "/", 0);
  Serial.println("SD Card Storage: ");
  Serial.printf("Total space: %lluMB\n", SD.totalBytes() / (1024 * 1024));
  Serial.printf("Used space: %lluMB\n", SD.usedBytes() / (1024 * 1024));

  Serial.println("Card Mount Success...");

  delay(2000);
}

void checkGSM(){
    mySerial.println("AT"); //Once the handshake test is successful, it will back to OK
  updateSerial();
  mySerial.println("AT+CSQ"); //Signal quality test, value range is 0-31 , 31 is the best
  updateSerial();
  mySerial.println("AT+CCID"); //Read SIM information to confirm whether the SIM is plugged
  updateSerial();
  mySerial.println("AT+CREG?"); //Check whether it has registered in the network
  updateSerial();
  mySerial.println("ATI");
  updateSerial();
  mySerial.println("AT+CBC");
  updateSerial();
  mySerial.println("AT+COPS?");
  updateSerial();
}

void setup_WebServer() {
  Serial.begin(9600);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  Serial.println("WiFi connected");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());

  server.on("/", HTTP_GET, handleRoot);
  server.begin();
  Serial.println("HTTP server started");
}

// WEB SERVER =====================================================

void handleRoot() {
  DateTime now = rtc.now();
  
  String dateTimeString =
    String(now.year(), DEC) + "/" +
    String(now.month(), DEC) + "/" +
    String(now.day(), DEC) + " " +
    String(now.hour(), DEC) + ":" +
    String(now.minute(), DEC) + ":" +
    String(now.second(), DEC);

  String currentDate = String(daysOfTheWeek[now.dayOfTheWeek()]) + ", " + dateTimeString.substring(0, 10);
  String currentTime = dateTimeString.substring(11);
  String flowRateStr = String(flowRate, 2);
  String phLevelStr = String(ph_act, 2);

  String webpage = "<!DOCTYPE html>";
  webpage += "<html>";
  webpage += "<head>";
  webpage += "<title>WaCoMS - Water Collecting and Monitoring System</title>";
  webpage += "<meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">";
  webpage += "<meta http-equiv=\"refresh\" content=\"1\">";
  webpage += "<style>";
  webpage += "body {";
  webpage += "text-align:center;";
  webpage += "background-color: #f2f2f2;";
  webpage += "font-family: Arial, sans-serif;";
  webpage += "}";
  webpage += ".header {";
  webpage += "max-width: 78%;";
  webpage += "margin-bottom: 20px;";
  webpage += "color: #fff;";
  webpage += "background-color: rgb(74, 83, 118);";
  webpage += "border: 2px solid #36454F;";
  webpage += "border-radius: 10px;";
  webpage += "padding: 20px;";
  webpage += "margin: 0 auto 20px auto;";
  webpage += "justify-content: center;";
  webpage += "}";
  webpage += ".title {";
  webpage += "font-size: 32px;";
  webpage += "margin-bottom: 20px;";
  webpage += "}";
  webpage += ".tagline {";
  webpage += "font-size: 18px;";
  webpage += "margin-bottom: 10px;";
  webpage += "}";
  webpage += ".names {";
  webpage += "font-size: 14px;";
  webpage += "font-style: italic;";
  webpage += "}";
  webpage += ".grid-container {";
  webpage += "max-width: 80%;";
  webpage += "display: grid;";
  webpage += "grid-template-columns: repeat(auto-fit, minmax(300px, 1fr));";
  webpage += "gap: 20px;";
  webpage += "margin: 0 auto 20px auto;";
  webpage += "justify-content: center;";
  webpage += "}";
  webpage += ".card {";
  webpage += "min-height: 150px;";
  webpage += "background-color: white;";
  webpage += "padding: 20px;";
  webpage += "border-radius: 10px;";
  webpage += "box-shadow: 0px 0px 10px rgba(0, 0, 0, 0.1);";
  webpage += "position: relative;";
  webpage += "}";
  webpage += ".title2 {";
  webpage += "padding: 5px;";
  webpage += "border-radius: 5px;";
  webpage += "font-size: 18px;";
  webpage += "margin-bottom: 15px;";
  webpage += "color: #fff;";
  webpage += "background-color: rgb(74, 83, 118);";
  webpage += "}";
  webpage += ".row {";
  webpage += "display: flex;";
  webpage += "justify-content: space-between;";
  webpage += "margin-bottom: 10px;";
  webpage += "}";
  webpage += ".row span {";
  webpage += "font-weight: bold;";
  webpage += "}";
  webpage += ".description {";
  webpage += "position: absolute;";
  webpage += "bottom: 20px;";
  webpage += "left: 0;";
  webpage += "right: 0;";
  webpage += "text-align: center;";
  webpage += "font-size: 14px;";
  webpage += "font-style: italic;";
  webpage += "color: #888;";
  webpage += "}";
  webpage += ".footer {";
  webpage += "max-width: 78%;";
  webpage += "position: fixed;";
  webpage += "bottom: 0;";
  webpage += "left: 50%;";
  webpage += "transform: translateX(-50%);";
  webpage += "width: 100%;";
  webpage += "background-color: rgb(74, 83, 118);";
  webpage += "color: #fff;";
  webpage += "padding: 10px 0;";
  webpage += "}";
  webpage += "#how-it-works {";
  webpage += "max-width: 78%;";
  webpage += "margin: 0 auto;";
  webpage += "text-align: left;";
  webpage += "padding: 20px;";
  webpage += "background-color: #fff;";
  webpage += "border-radius: 10px;";
  webpage += "box-shadow: 0px 0px 10px rgba(0, 0, 0, 0.1);";
  webpage += "}";
  webpage += "#how-it-works h2 {";
  webpage += "font-size: 24px;";
  webpage += "margin-bottom: 10px;";
  webpage += "}";
  webpage += "#how-it-works p {";
  webpage += "font-size: 16px;";
  webpage += "margin-bottom: 15px;";
  webpage += "}";
  webpage += "#how-it-works ol {";
  webpage += "margin-left: 20px;";
  webpage += "font-size: 16px;";
  webpage += "}";
  webpage += "#how-it-works li {";
  webpage += "margin-bottom: 10px;";
  webpage += "}";
  webpage += "</style>";
  webpage += "</head>";

  // JavaScript for updating data at intervals
  webpage += "<script>";
  webpage += "function updateData() {";
  webpage += "  fetch('/data')"; // Assuming '/data' is the endpoint for fetching updated data
  webpage += "    .then(response => response.json())";
  webpage += "    .then(data => {";
  webpage += "      document.getElementById('currentDate').innerText = 'Current Date: ' + data.currentDate;";
  webpage += "      document.getElementById('currentTime').innerText = 'Current Time: ' + data.currentTime;";
  webpage += "      document.getElementById('flowRate').innerText = 'Flow Rate: ' + data.flowRate + ' L/min';";
  webpage += "      document.getElementById('phLevel').innerText = 'pH Level: ' + data.phLevel;";
  webpage += "    })";
  webpage += "    .catch(error => console.error('Error fetching data:', error));";
  webpage += "}";
  webpage += "updateData();"; // Call updateData() initially when the page loads
  webpage += "setInterval(updateData, 3000);"; // Call updateData() every 3 seconds
  webpage += "</script>";


  webpage += "<body class=\"body\">";
  webpage += "<div class=\"header\">";
  webpage += "<div class=\"title\">WaCoMS - Water Collecting and Monitoring System</div>";
  webpage += "<div class=\"tagline\">Online monitoring system for water collection and quality assessment</div>";
  webpage += "<div class=\"names\">Developed by: Henrich Lacao, Andrei Co, Diana Carreon</div>";
  webpage += "</div>";
  webpage += "<div class=\"grid-container\">";
  webpage += "<div class=\"card\">";
  webpage += "<div class=\"title2\">Time and Date</div>";
  webpage += "<div class=\"row\">";
  webpage += "<span>Current Date:</span>";
  webpage += "<span id=\"currentDate\">" + currentDate + "</span>";
  webpage += "</div>";
  webpage += "<div class=\"row\">";
  webpage += "<span>Current Time:</span>";
  webpage += "<span id=\"currentTime\">" + currentTime + "</span>";
  webpage += "</div>";
  webpage += "<div class=\"description\">This card displays the current date and time</div>";
  webpage += "</div>";
  webpage += "<div class=\"card\">";
  webpage += "<div class=\"title2\">Water Flow Rate</div>";
  webpage += "<div class=\"row\">";
  webpage += "<span>Flow Rate:</span>";
  webpage += "<span id=\"flowRate\">" + flowRateStr + " L/min</span>";
  webpage += "</div>";
  webpage += "<div class=\"description\">This card shows the rate of water flow</div>";
  webpage += "</div>";
  webpage += "<div class=\"card\">";
  webpage += "<div class=\"title2\">pH Level</div>";
  webpage += "<div class=\"row\">";
  webpage += "<span>pH Level:</span>";
  webpage += "<span id=\"phLevel\">" + phLevelStr + "</span>";
  webpage += "</div>";
  webpage += "<div class=\"description\">This card indicates the pH level of the water</div>";
  webpage += "</div>";
  webpage += "</div>";
  webpage += "<div class=\"footer\">";
  webpage += "&copy; 2024 WaCoMS - Water Collecting and Monitoring System";
  webpage += "</div>";
  webpage += "<section id=\"how-it-works\">";
  webpage += "<h2>How WaCoMS Works</h2>";
  webpage += "<p>WaCoMS utilizes a combination of sensors, data processing, and communication modules to monitor water collection and quality assessment. Here's how it works:</p>";
  webpage += "<ol>";
  webpage += "<li><strong>Sensors:</strong> WaCoMS is equipped with sensors to measure water flow rate and pH level in real-time.</li>";
  webpage += "<li><strong>Data Collection:</strong> The sensor data is collected continuously and transmitted to the central processing unit.</li>";
  webpage += "<li><strong>Data Analysis:</strong> The collected data is analyzed to determine the rate of water flow and pH level, providing insights into water quality.</li>";
  webpage += "<li><strong>Alerts and Notifications:</strong> If any anomalies are detected, such as abnormal pH levels or low water flow rates, WaCoMS generates alerts and notifications for prompt action.</li>";
  webpage += "<li><strong>User Interface:</strong> Users can access the WaCoMS system through a user-friendly interface, allowing them to monitor water parameters, view historical data, and receive alerts.</li>";
  webpage += "</ol>";
  webpage += "</section>";
  webpage += "</body>";
  webpage += "</html>";

  server.send(200, "text/html", webpage);
}

// ============================================================

void setup(){
  Serial.begin(9600);
  mySerial.begin(9600); // GSM Communication
  Wire.begin(); // PH Level Sensor

  pinSetups();
  setup_RTC();
  displayOLED("WaCOMS is Starting.", "Please Wait...");
  Serial.println("WaCOMS is Starting.. Please Wait...");
  checkGSM();
  delay(15000); // 15 secs delay for GSM to startup

  setup_SDCard();
  setup_WaterFlow();

  // Undermaintenance
  setup_WebServer();
}

void loop(){
  server.handleClient();

  checkRTC(); // Update DATE TIME NOW
  displayDateTime();
  delay(1000);

  // Reservoir / PH Level / Flow Sensor
  checkWaterLevel();
  checkWaterPHLevel();
  checkWaterFlowRate(); 
}

