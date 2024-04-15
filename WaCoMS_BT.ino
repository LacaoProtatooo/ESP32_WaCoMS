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
#include "BluetoothSerial.h"

#include <WiFi.h>

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

// BT
BluetoothSerial SerialBT;

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

#if !defined(CONFIG_BT_SPP_ENABLED)
#error Serial Bluetooth not available or not enabled. It is only available for the ESP32 chip.
#endif

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

// VARIABLES ========================================================================

// Message Stopper
bool wtrfill = false;
int sdrecord = 1;

// PH-Level Sensor
float calibration_value = 21.34 - 0.7;
unsigned long int avgval; 
int buffer_arr[10],temp;
float ph_act = 0; // PH LEVEL 


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
    Serial.println("Message to be sent repeated, invalidating gsm message");
    return;
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

void updateBluetooth(){
  if (Serial.available()) {
    SerialBT.write(Serial.read());
  }

  while(SerialBT.available()){
 Serial.write(SerialBT.read());

    char command = SerialBT.read();
    if(command == '1'){
      SerialBT.println("Running DC Motor");
      runDCMotor();
    }
    if(command == '2'){
      SerialBT.println("Stopping DC Motor");
      stopDCMotor();
    }
    if(command == '3'){
      SerialBT.println("Printing Values: ");
      SerialBT.println("PH Level: ");
      SerialBT.print(ph_act);
      SerialBT.println("GSM Module previous message: ");
      SerialBT.print(gsm_prev_msg);
      SerialBT.println("Date Time : ");
      SerialBT.print(dateTimeString);
    }

  }
  delay(20);
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

void setup_Bluetooth(){
  SerialBT.begin("ESP32_WaCoMS"); //Bluetooth device name
  delay(10000);
  Serial.println("The device started, now you can pair it with bluetooth!");
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

// ============================================================

void setup(){
  Serial.begin(9600);
  mySerial.begin(9600); // GSM Communication
  Wire.begin(); // PH Level Sensor

  pinSetups();
  setup_RTC();
  setup_Bluetooth();
  displayOLED("WaCOMS is Starting.", "Please Wait...");
  Serial.println("WaCOMS is Starting.. Please Wait...");
  checkGSM();
  delay(15000); // 15 secs delay for GSM to startup

  setup_SDCard();
  setup_WaterFlow();

  // Undermaintenance
}

void loop(){

  checkRTC(); // Update DATE TIME NOW
  updateBluetooth();
  displayDateTime();
  delay(1000);

  // Reservoir / PH Level / Flow Sensor
  checkWaterLevel();
  updateBluetooth();
  checkWaterPHLevel();
  updateBluetooth();
  checkWaterFlowRate(); 
  updateBluetooth();
}

