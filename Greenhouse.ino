/*
Water override protection, will only stay overridden for 30 minutes
If water should stop flowing, the light switches off
Switches on light for 12 or 18 hours
Switches water on for 0, 1, 5, 15 or 60 minutes per hour
Switches fan to full when over a certain temp, then switches back again after dropping 2 degrees
Switches fan off below a set temp
Switches Heater on/off at set temp
Has Light and Water overrides
Has LCD to display 'program', water and pump status, LED, errors and override status

Changelog
9.2
Changed Sensorcheck to 15 seconds
Added void Serial2Digits and LCD2Digits
Added LCD Stuff
Noticed waterPin didn't have pinMode set
During Error2, Light didn't stay off, fixed by adding 'if errorr statement'

9.3
Updated the LCD Display
Added errorlog for counting errors, resets at 6pm
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
9.4
Removed the sensortimer etc. errors 2 and 3 (No water, No light) replaced with errorcounters
Changed the variable overheat to errorcount[1]
Added fan OFF when below fanLow (16)
Added RF Switch3 on/off (for Heater)
Added Heater on at heatTrigger (14)
Added toggles for LEDZ, LCD_DISPLAY, RF, RFERROR

9.5
Removed toggles for LCD_DISPLAY, RFERROR
Added a Menu system, changed some buttons
Variables fanLow, heatTrigger now changeable
Updated the way the RF code is transmitted
Added a Low/Hi temp count, display and reset option

9.6
Changed alternating Display
Removed Grow/Flower from button 1
Removed Water times from button 2
Added alternate menus on button 1
Added Light override to button 2
Display Alternates when error is detected

9.7
Added a PlantPot to the setup, checks moisture and pumps water
Added IF statement for PlantPot LCD
Modified the +60 minutes routine
Added TLog[]s for Fan and Heater run times
Added Menu's for above
Removed the debugging stuff

9.8
Not a lot, just tidied up the comments

9.9
Added 'Logging' to another arduino
Changed all the 'strings' to (F("blah")) to save memory
Change some int to boolean again to save memory
Added 0 and 1 minute to the Pump Cycle

10.0
Removed Logging, what was I thinking?
Removed Serial commands, no need for it.
Fixed the water so that if it's not running when not on full, it stays off for the hour
Added light override timer
Implemented a 56 day counter for Flowering (in case I forget)
Changed the LCD Display around
Removed Autosave
Added PotToggle to allow Pot watering for X seconds at 0 and 30 minutes past the hour.

10.1
Made RF toggle permanent (easier to mess with menus)
Added 2 new menus for Potmoist and PotDelay
Added value check after EEPROM load
Shuffled Menus a bit

10.2
Removed LEDS and associated pins, variables etc.
Created a new menu system for a Rotary Encoder
Removed old 4 button system

10.3
Changed light,pump,int and fullfan toggles for arrays with previous settings
Commented out section near line 325 regarding error checking for water.

10.4
Added ability to set clock via menu system

10.5
Change to an Arduino Mega
Added WiFi, pain in the arse

*** To Do

*/
#define TESTBOX 1             // Toggle for manual temp and Water Tgl        |     | Testbox
// Libraries
#include <Time.h>
#include <DS1307RTC.h>
#include <Wire.h>
#include <EEPROM.h>
#include "DHT.h"
#include <LiquidCrystal_I2C.h>
#include <ClickEncoder.h>
#include <TimerOne.h>
#include <ArduinoJson.h>

// The following is for the ESP Wifi stuff
#include <WiFiEspClient.h>
#include <WiFiEsp.h>
#include <WiFiEspUdp.h>
#include <PubSubClient.h>

#define WIFI_AP "Skynet2.4"
#define WIFI_PASSWORD "Nn4zxvjt6Yvp"
#define CLIENT_NAME "GardenTest"

/****************************************FOR JSON***************************************/
#define MQTT_MAX_PACKET_SIZE 512

char mqttserver[] = "192.168.0.10";
// Initialize the Ethernet client object
WiFiEspClient espClient;

PubSubClient client(espClient);

int status = WL_IDLE_STATUS;
LiquidCrystal_I2C lcd(0x27, 20, 4); // addr, EN, RW, RS, D4, D5, D6, D7, Backlight, POLARITY
//LiquidCrystal_I2C lcd(0x27, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE); // addr, EN, RW, RS, D4, D5, D6, D7, Backlight, POLARITY
ClickEncoder *encoder;
// Analog Pins
#define PotPin 0               // pin for Pot Plant (input)
#define lightsensorPin 7       // pin for light sensor (input)
#define PinA A2                // Encoder PinA
#define PinB A3                // Encoder PinB
#if TESTBOX
#define VarPin 4               // pin for Variable Potentiometer (input) (Testbox Only)
#endif
// Digital Pins
#define DHTPIN 2               // pin for DHT/temp (input)
#define lightPin 4             // pin for light (output)
#if TESTBOX
#define waterPin 11             // pin for water running sensor (input)
#define pumpPin 5              // pin for pump (output)
#define fanPin 7               // pin for FULL Fan (output)
#define fanPin2 6              // pin for INTERMEDIATE Fan (output)
#else
#define waterPin 5             // pin for water running sensor (input)
#define pumpPin 3              // pin for pump (output)
#define fanPin 6               // pin for FULL Fan (output)
#define fanPin2 7              // pin for INTERMEDIATE Fan (output)
#endif
#define PotPout 8              // pin for Pot Pump (output)
#define PinC 9                 // Encoder Button
#define rfTransmitPin 10       // RF Transmitter pin = digital pin 10
DHT dht(DHTPIN, DHT11);         // Set HDHT sensor
// Variables
#define menus 12                // Number of LCD Options.
int codeToTransmit[]= {1,2,3,2,2,2,3,3,2,2,3,2,2,2,2,3,3,3,2,3,3,0,0,0,0};                        // The array used to hold the RF code
const int lightB[]= {3,3,3,3,2,3,3,3,3,2,3,3,2,2,3,3,3,3,2,3,2,3,2,3,3,3,3,2,2,3,3,2};  // 4 Digits each, 1on, 1off, 2on, 2off, 3on, 3off, 4on, 4off
int errorlog[] = {0,0,0,0,0,0};
int errorcount[] = {0,0,0,0};
int number[] = {0,0,0,0};       // Temporary for LCDisplay3
const char* errorrs[] = {"No Error", "Overheating", "Pump ON, no water", "Light ON, bulb OFF", "NO CLOCK", "Light Delay WAIT"};
int promloc = 20;               // Location of EEPROM data
int tempLimits[] = {50,0};      // Temperature low/high limits
int TLog[] = {0,0,0,0};         // TLog file (Fan Off, Int Fan, Full Fan, Heater On)
int YLog[] = {0,0,0,0};         // Yesterdays Log Files
int RLog = 0;                   // Toggle used to reset to logfile
int menuSet = 0;                // Menu select
int fanTrigger = 26;            // Temperature to increase fan speed
int fanLow = 16;                // Temperature to turn off fan
int heatTrigger = 12;           // Temperature to turn on Heater
int lcdscreen = 0;              // Variable to toggle LCD Displays
int timenow[] = {0,0,0,0,0,0};  // used for storing time hour, min, seconds, date
int lightoff = 6;               // light timer off
int pumpoff = 5;                // pump timer off
float tempC;                    // Set tempC as float value
float humy;                     // Set humy for Humidity
int lightreading = 0;           // variable used to store state of input
#if TESTBOX
boolean lightToggle[] = {1,0};        // lightToggle / Previous lightToggle
boolean pumpToggle[] = {1,0};         // pumpToggle
boolean FullFanToggle[] = {1,0};      // Fan Toggle for FULL SPEED
boolean IntFanToggle[] = {1,0};       // Fan Toggle for INTERMEDIATE SPEED
#else
boolean lightToggle[] = {0,1};        // lightToggle / Previous lightToggle
boolean pumpToggle[] = {0,1};         // pumpToggle
boolean FullFanToggle[] = {0,1};      // Fan Toggle for FULL SPEED
boolean IntFanToggle[] = {0,1};       // Fan Toggle for INTERMEDIATE SPEED
#endif
int heatToggle = 3;             // Heater Toggle 2 on, 3 off
int prevheatToggle = 3;         // previous Heater Toggle 2 on, 3 off
int overrideTime[] = {0,0,0};   // Set variable for override timer (water,light,Menu)
boolean lightOveride = 0;       // Light override toggle
boolean pumpOveride = 0;        // Pump override toggle
int buttonState = 0;            // the current reading from the input pin
int prevbuttonState = 0;        // previous button state
boolean waterToggle = 0;        // Toggle for water running or not
long mincounter = 0;            // Timer for logg counter
long fivesecondtimer = 0;       // Timer for serial display
long lightDelay = 0;            // Timer for light delay
int moisture = 0;               // Set variable for PlantPot moisture
int errorr = 0;                 // errorlevel
int preverrorr = 0;             // previous errorlevel
boolean bulbCheck = 0;          // Is the bulb lit?
boolean waterhourtoggle = 0;    // water toggle hour thing
int daysleft = 0;               // Days until Harvest
int prevday = 0;                // previous hour
boolean PotToggle = 1;          // Toggle for Potplant
int PotDelay = 0;               // seconds for potplant
int Potmoist = 85;              // trigger for potplant
int16_t last, value;            // last and current value for Rotary Encoder
int submenu = 0;                // submenu toggle

int Ulimit[] = {23,59,59,31,12,2030};  // Timeset upper limit
int Llimit[] = {0,0,0,1,1,2019};       // Timeset lower limit
int pos[] = {0,0,3,0,6,0,0,1,3,1,6,1}; // 
int Sett = 0;
int SetaClock = 0;
int heartbeat = 0;
long blinktimer = 0;
int tempboo = 0;
String temp_str; //see last code block below use these to convert the float that you get back from DHT to a string =str
String hum_str;
String daysleft_str;
char tempp[50];
char humm[50];
char daysleftt[50];

void timerIsr() {
  encoder->service();
}

int sortthehour(int x, int y){  // x is time, y is minutes to add
  x = x + y;
  if (x >= 60) x = x - 60;      // If overridetimer is over 60, fix it
  return x;
}

int minstohoursmins(int x, int y){ // x input minutes, y = 0 Hours or 1 Minutes
  int xx = x % 60;              // Minutes
  int yy = (x - xx)/60;         // Hours
  if(y == 0) return yy;         // If y=0 return Hours
  if(y == 1) return xx;         // If y=1 return Minutes
}

void setup() {
#if TESTBOX
  pinMode(waterPin, INPUT_PULLUP);
#endif
  pinMode(PinC, INPUT_PULLUP);
  pinMode(lightPin, OUTPUT);
  pinMode(pumpPin, OUTPUT);
  pinMode(fanPin, OUTPUT);
  pinMode(fanPin2, OUTPUT);
  digitalWrite(pumpPin, 0);
  digitalWrite(lightPin, 0);
  Serial.begin(9600);
  pinMode(rfTransmitPin, OUTPUT);                                       // Transmit pin is an output
  encoder = new ClickEncoder(PinA, PinB, PinC);                         // Pin A, Pin B, Button (Can be A0 etc.)
  encoder->setAccelerationEnabled(0);                                   // Set encoder acceleration off
  Timer1.initialize(500);
  Timer1.attachInterrupt(timerIsr); 
  pinMode(PotPout, OUTPUT);
  lcd.begin(20, 4);                                                     // initialize the lcd as 20x4
  lcd.setBacklight(1);                                                  // switch on the backlight
  lcd.clear();
  dht.begin();                                                          // Start the Temp/Humidity sensor
  if (EEPROM.read(promloc+0) == 6 || EEPROM.read(promloc+0) == 12) {    // If EEPROM isn't empty, load eeprom
    readepprom();                                                       // Read Previous Pumpoff
  } else {
    writeepprom();                                                      // else write data to eeprom
  }
  for(int i=0; i<4; i++){                                               // Read Yesterdays Logg from EEPROM
    YLog[i] = EEPROM.read(promloc+5+i);
  }
  gettemp();
  lcd.setCursor(4,0);
  lcd.print(F("The Ultimate"));
  lcd.setCursor(1,1);
  lcd.print(F("Automated Growroom"));
  lcd.setCursor(3,2);
  lcd.print(F("v10.6 - 2016/9"));
  lcd.setCursor(1,3);
  lcd.print(F("** STARTING  UP **"));
  delay(1500);
  InitWiFi();
  client.setServer( mqttserver, 1883 );
  digitalWrite(lightPin, 0);     
  digitalWrite(pumpPin, 0);         
  digitalWrite(fanPin, 0); 
  digitalWrite(fanPin2, 0);   
}

void loop() {
  unsigned long currentMillis = millis();
  if (currentMillis - fivesecondtimer > 5000) gettemp();                  // Get Temp Timer
  if(PotDelay > 0){
    PlantPot();                                                           // Check the plantpot
  }
  checktimer();
  keyboard();
  if (currentMillis - fivesecondtimer > 5000) sensortest();               // Sensortest every 5 seconds
  if(errorr == 2) {                                                       // If errorr 2 (water not running) 
    lightToggle[0] = 0;                                                      // Turn light off
    lightDelay = currentMillis;                                           // LightDelay = currentMillis
  } else if(currentMillis - lightDelay < 30000 && currentMillis > 30000){ // If time is less than lightdelay (and Arduino has been running for 30 seconds)
    lightToggle[0] = 0;                                                      // Toggle light off
    errorr = 5;                                                           // Set error level to 5 (Wait, light delay)
  }
  if (overrideTime[0] == timenow[1] && pumpOveride == 1) pumpOveride = 0;  // If time is overridetime & Pump override is on, cancel override
  if (overrideTime[1] == timenow[1] && lightOveride == 1) lightOveride = 0;// If time is overridetime & Light override is on, cancel override
  if (overrideTime[2] == timenow[1] && menuSet > 0) {                      // If time is menuoverridetime & menu is on, cancel override
    buttonState = 3; menuSet = 11;                                         // Set button to 'click' and menulist to 'cancel'
    Keyboard2();
  }
  if (lightOveride == 1) lightToggle[0] = !lightToggle[0];                      // if light override set, invert light
  if (pumpOveride == 1) pumpToggle[0] = !pumpToggle[0];                         // if pump override set, invert pump
  if (errorcount[1] >= 25) {                                              // if overheat counter is greater than 24 (24 x 5 Seconds = 2 Minutes)
    lightToggle[0] = 0;                                                      // turn off light
    pumpToggle[0] = 0;                                                       // turn off pump
    errorr = 1;                                                           // set errorlevel 1
  }
  if(currentMillis - mincounter > 60000){                                 // After 60 seconds has passed
    if(FullFanToggle[0] == 1){                                               // If Full Fan is on add 1 min to log counter
      TLog[2] += 1;
    } else if(IntFanToggle[0] == 1){                                         // If Int Fan is on
      TLog[1] += 1;
    } else {
      TLog[0] += 1;                                                       // Fan is off
    }
    if(heatToggle == 2) TLog[3] += 1;                                     // If Heater is on
      mincounter = currentMillis;                                         // Reset counter
      tempboo = 0;
      for(int i=0; i<10; i++){
        status = WiFi.status();
        tempboo+= status;
        delay(1);
      }
      if(tempboo > 0){
        status = 1;
      } else {
        status = 0;
      }
      if ( status != WL_CONNECTED && menuSet == 0) {
          lcd.clear();
          for(int i=0; i<3; i++) {
          Serial.print("Attempting to connect to WPA SSID: ");
          Serial.println(WIFI_AP);
          lcd.setCursor(0,0);
          lcd.print("Connecting");
          lcd.setCursor(0,1);
          lcd.print(WIFI_AP);
          // Connect to WPA/WPA2 network
          status = WiFi.begin(WIFI_AP, WIFI_PASSWORD);
          delay(500);
          if(status == WL_CONNECTED){
            break;
          }
        }
        if(status == WL_CONNECTED){
        Serial.println("Connected to AP");
        lcd.setCursor(0,2);
        lcd.print("Connected");
      } else {
        lcd.setCursor(0,2);
        lcd.print(" **NOT**");
        lcd.setCursor(0,3);
        lcd.print("Connected");
      }
      delay(1000);
    }
    if ( !client.connected() ) {
      reconnect();
    }
  SendData();
  }
#if TESTBOX
  if(lightToggle[0] != lightToggle[1]) digitalWrite(lightPin, lightToggle[0]);                 // WRITE LIGHT PIN
  if(pumpToggle[0] != pumpToggle[1]) digitalWrite(pumpPin, pumpToggle[0]);                     // WRITE PUMP PIN
  if(FullFanToggle[0] != FullFanToggle[1]) digitalWrite(fanPin, FullFanToggle[0]);             // WRITE FULL FAN PIN
  if(IntFanToggle[0] != IntFanToggle[1]) digitalWrite(fanPin2, IntFanToggle[0]);               // WRITE INTERMEDIATE FAN PIN
#else
  if(lightToggle[0] != lightToggle[1]) digitalWrite(lightPin, !lightToggle[0]);                 // WRITE LIGHT PIN
  if(pumpToggle[0] != pumpToggle[1]) digitalWrite(pumpPin, !pumpToggle[0]);                     // WRITE PUMP PIN
  if(FullFanToggle[0] != FullFanToggle[1]) digitalWrite(fanPin, !FullFanToggle[0]);             // WRITE FULL FAN PIN
  if(IntFanToggle[0] != IntFanToggle[1]) digitalWrite(fanPin2, !IntFanToggle[0]);               // WRITE INTERMEDIATE FAN PIN
#endif
  if(heatToggle != prevheatToggle){                                       // If heatToggle is different to prevheatToggle
    transmitCode(heatToggle,3);                                           // Turn heater on/off
    delay(500);                                                           // wait 1/2 second
    transmitCode(heatToggle,3);                                           // Turn heaton on/off
  }
  if (currentMillis - fivesecondtimer > 5000) {                           // Serial Display Timer
    fivesecondtimer = currentMillis;
    if (errorr > 0) {                                                     // If Error, alternate display
      if(lcdscreen > 0){
        lcdscreen = 0;
      } else {
        lcdscreen = 1;
      }
    }
    if(menuSet == 0){
      if(lcdscreen == 0){
        LCDisplay1();
      } else if(lcdscreen == 1){
        LCDisplay2();
      } else if(lcdscreen == 2 || lcdscreen == 3){
        LCDisplay3();
      }
    }

//    client.loop();

  }
  prevheatToggle = heatToggle;                                          // prevheatToggle = heatToggle
  if(preverrorr != errorr && errorr > 0){                               // If previous error does not match current error
    errorlog[errorr]++;                                                 // increase error
  }
  preverrorr = errorr;                                                  // Set previous error to current error
  lightToggle[1] = lightToggle[0];                                      // Set Previous lighttoggle to current lighttoggle
  pumpToggle[1] = pumpToggle[0];                                        // Set Previous pumptoggle to current pumptoggle
  FullFanToggle[1] = FullFanToggle[0];                                  // Set Previous FullFanToggle to current FullFanToggle
  IntFanToggle[1] = IntFanToggle[0];                                    // Set Previous IntFanToggle to current IntFanToggle
} 

void sensortest() {
  errorr = 0;
  if (tempC > 45 && errorcount[1] < 30) {                               // If temperature gets above 45
    errorcount[1]++;                                                    // increase errorcount (overheat)
  }
  if(tempC <= 45 && errorcount[1] > 25) {
    errorcount[1]--;
  }
/*  if (pumpToggle[0] == 1 && waterToggle == 0 && pumpOveride == 0) {        // If pump is running and water sensor dry and pumpOveride is off
    errorcount[2]++;                                                    // Set errorcount +1
    if(errorcount[2] > 20){
      errorcount[2]--;
    }
  } else if (waterhourtoggle == 0 || waterToggle == 1) {                // If waterhourtoggle = 0 or water sensor is wet
    errorcount[2]--;                                                    // Set errorcount -1
    if(errorcount[2] < 0){
      errorcount[2]++;
    }
  }
*/
  if (lightToggle[0] == 1 && bulbCheck == 0 && lightOveride == 0) {        // If light is switched on and no light present and LightOveride is off
    errorcount[3]++;                                                    // Set errorcount +1
    if(errorcount[3] > 20){
      errorcount[3]--;
    }
  } else {                                                              // else
    errorcount[3] = 0;                                                  // reset errorcount
  }
  if (errorcount[3] >= 10) errorr = 3;                                  // If errorcount3 >= X set errorrlevel 3 (3 before 2, it just fixes things) *Light*
  if (errorcount[2] >= 10) {
    errorr = 2;                                                         // If errorcount2 >= X set errorrlevel 2 *Water*
    waterhourtoggle = 1;                                                // Set the waterhourtoggle to 1
  } else {
    waterhourtoggle = 0;
  }
  tmElements_t tm;                                                      // Read clock
  if (!RTC.read(tm)) errorr = 4;                                        // if not working, set errorlevel 4
  if(IntFanToggle[0] == 0) tempC = tempC - 2;                              // If Intermediate Fan is off, Decrease temperature by 2 degrees (stops fan from switching too often)
  if (tempC >= fanLow) {                                                // If tempC >= fanLow
    IntFanToggle[0] = 1;                                                   // Intermediate fan on
  } else if (tempC < fanLow && timenow[1] < 5){                         // else if tempC < fanLow and minute is < 5 (Allows fan to run for 5 mins at start of each hour)
    IntFanToggle[0] = 1;                                                   // Intermediate fan on
  } else {                                                              // else
    IntFanToggle[0] = 0;                                                   // Intermediate fan off
    tempC = tempC + 2;                                                  // Increase temperature by 2 degrees
  }
  if(FullFanToggle[0] == 1) tempC = tempC + 2;                             // If FULL Fan is on, Increase temperature by 2 degrees (stops fan from switching too often)
  if (tempC > fanTrigger) {                                             // If temperature is higher than trigger
    FullFanToggle[0] = 1;                                                  // FULL fan on
    tempC = tempC - 2;                                                  // Decrease temperature by 2 degrees
  } else {
    FullFanToggle[0] = 0;                                                  // else FULL fan off
  }
  if(heatToggle == 2) tempC = tempC - 2;                                // If Heater is on, Decrease temperature by 2 degrees (stops heater from switching too often)
  if(tempC <= heatTrigger){                                             // If tempC <= heatTrigger
    heatToggle = 2;                                                     // set heatToggle on(2)
    tempC = tempC + 2;                                                  // Increase temperature by 2 degrees
  } else {                                                              // else
    heatToggle = 3;                                                     // set heatToggle off(3)
  }
}

void checktimer() {                                                     // Check timers
  tmElements_t tm;
  if (RTC.read(tm) && SetaClock == 0) {
    timenow[0] = tm.Hour;
    timenow[1] = tm.Minute;
    timenow[2] = tm.Second;
    timenow[3] = tm.Day;
    if(menuSet == 0 && lcdscreen == 0 && millis() > 5000){
      LCDTime();
    }
    if(timenow[0] == 17) RLog = 1;                                         // If hour 17 set RLog 1
    if(timenow[0] == 18 && RLog == 1){                                     // If Hour 18 and RLog = 1, reset the errorlogs and the counter log
      RLog = 0;
      for(int i=0; i<6; i++){                                              // Reset error counters
        errorlog[i] = 0;
      }
      for(int i=0; i<4; i++){                                              // Reset the Fan and Heater logs
        YLog[i] = TLog[i];                                                 // copy TLog to YLog
        EEPROM.update(promloc+5+i,TLog[i]);                                        // Write log to EEPROM
        delay(20);
        TLog[i] = 0;                                                       // set TLog to 0
      }
      if(daysleft > 0){
        daysleft--;                                                        // Take 1 day off
      }
      EEPROM.update(promloc+10, daysleft);                                          // Update EEPROM
    }
    if (timenow[0] >= 18 || timenow[0] < lightoff) {                       // Check time>on and time<off
      lightToggle[0] = 1;
    } else {
      lightToggle[0] = 0;
    }
    if (timenow[1] >= 0 && timenow[1] < pumpoff && lightToggle[0] == 1) {      // If Pump timer is matched and LightToggle is on ..
      pumpToggle[0] = 1;                                                       // turn on pump
    } else {
      pumpToggle[0] = 0;                                                       // Reset pump toggle
    }
    if (timenow[1] == 30) {                                                 // If minute is 30
      if (timenow[2] < 5) PotToggle = 1;                                    // and if seconds are less than 5 Set PotToggle to 1
      if (errorcount[1] < 30) errorcount[1] = 0;                            // Reset the overheat counter 30 minutes after the hour only if errors are less than 30
    }
    if (timenow[1] == 0) {
      if (timenow[2] < 5) PotToggle = 1;                                    // and if seconds are less than 5 Set PotToggle to 1
      if (waterhourtoggle == 1){                                            // If minute is 0 and waterhourtoggle is on
        waterhourtoggle = 0;                                                // Set toggle off
        errorcount[2] = 0;                                                  // Reset errorcount(water) to 0
      }
    }
  }
}

void gettemp() {
#if TESTBOX
  tempC = analogRead(VarPin)/22;                                            // Read Variable pot and divide by 22
  lightreading = analogRead(lightsensorPin)-102;                            // Read state of the light input pin
  waterToggle = digitalRead(waterPin);                                      // Read Water pin
#else
  tempC = dht.readTemperature();                                            // Read temperature as Celsius
  humy = dht.readHumidity();                                                // Read Humidity
  if (isnan(tempC) || isnan(humy)){
    Serial.println(F("Failed to read from DHT"));
  }
  lightreading = analogRead(lightsensorPin);                                // Read state of the light input pin
  waterToggle = !digitalRead(waterPin);                                     // Read Water pin (Inverted because 1 is OFF !?!?!)
//  waterToggle = 1;                                     // Read Water pin (Inverted because 1 is OFF !?!?!)
#endif
  lightreading = map(lightreading, 0, 1023, 0, 100);                        // convert 0-1024 to 0-100
  if (lightreading > 40) {                                                  // If bulb is on
    bulbCheck = 1;                                                          // bulb is on
  } else {
    bulbCheck = 0;                                                          // bulb is off
  }
  if(tempC < tempLimits[0]){                                                // If temperature is less than recorded low temp
    tempLimits[0] = tempC;                                                  // Set low record to temperature
  }
  if(tempC > tempLimits[1]){                                                // If temperature is more than recorded high temp
    tempLimits[1] = tempC;                                                  // Set high record to temperature
  }
}
void keyboard() {                                                                                                                      //     %%%   %  %  %%%%  %%%%
  buttonState = 0;                                                                                                                     //     %  %  %  %   %     %
  value += encoder->getValue();                                             // Read value from Encoder                                 //     %%%   %  %   %     %
  if(value > last){                                                                                                                    //     %  %  %  %   %     %
    buttonState = 1;                                                                                                                   //     %%%    %%    %     %
  }
  if(value < last){
    buttonState = 2;
  }
//  if(value != last){        
//    Serial.print("Last:"); Serial.print(last); Serial.print(" Current:"); Serial.println(value);
//    Serial.print("Button:"); Serial.println(buttonState);
//  }
  last = value;
  ClickEncoder::Button b = encoder->getButton();                    // Don't know the code but this reads the Rotary Encoder button
  if (b != ClickEncoder::Open) {                                    // If button is pressed (not open circuit)
    #define VERBOSECASE(label) case label:; break;
    switch (b) {
      case ClickEncoder::Clicked:
          buttonState = 3;
        break;
      case ClickEncoder::Held:
          buttonState = 4;
        break;
    }
  }    
  if(prevbuttonState != 4){                                           // Stops HELD from being repeated.
    if (buttonState > 0) {                                            // If any button is pressed ________
      if(submenu == 1){
        Keyboard3();
      } else if(menuSet == 0){
        Keyboard1();
      } else if(SetaClock == 1){
        keyboard4();
      } else {
        Keyboard2();
      }
    }
  }
  if(SetaClock == 1) MenuPrint4();
  prevbuttonState = buttonState;
}

void writeepprom() {
  EEPROM.update(promloc+0, lightoff);                               // Store it in eeprom
  EEPROM.update(promloc+1, pumpoff);
  EEPROM.update(promloc+2, fanTrigger);
  EEPROM.update(promloc+3, fanLow);
  EEPROM.update(promloc+4, heatTrigger);
  EEPROM.update(promloc+10, daysleft);
  EEPROM.update(promloc+12, Potmoist);
  EEPROM.update(promloc+13, PotDelay);
  delay(200);
}

void readepprom() {
  lightoff = EEPROM.read(promloc+0);                            // Read Previous Lightoff
  pumpoff = EEPROM.read(promloc+1);                             // Read Previous Pumpoff
  fanTrigger = EEPROM.read(promloc+2);                          // Read Previous fanTrigger
  fanLow = EEPROM.read(promloc+3);                              // Read Previous fanLow
  heatTrigger = EEPROM.read(promloc+4);                         // Read Previous heatTrigger
  daysleft = EEPROM.read(promloc+10);                           // Read Previous daysleft
  Potmoist = EEPROM.read(promloc+12);                           // Read Previous Potmoist
  PotDelay = EEPROM.read(promloc+13);                           // Read Previous PotDelay
  if(lightoff > 12) lightoff = 12;
  if(pumpoff > 60) pumpoff = 60;
  if(fanTrigger > 28) fanTrigger = 28;
  if(fanLow > 25) fanLow = 25;
  if(heatTrigger > 16) heatTrigger = 16;
  if(daysleft > 56) daysleft = 0;
  if(Potmoist > 95 || Potmoist <85) Potmoist = 90;
  if(PotDelay > 10) PotDelay = 0;
  delay(200);
}

void LCD2digits(int number) {
  if (number >= 0 && number < 10) {
    lcd.write('0');
  }
  lcd.print(number);
}

void SetDate(int days) {
  daysleft = days;
  EEPROM.update(promloc+10, daysleft);
  EEPROM.update(promloc+11, prevday);
}

void transmitCode(int State, int timez) {             // Transmit  0=1on 1=1off 2=2on 3=2off 4=3on 5=3off -- Second number is times to repeat, 3 for switch, 8 for program
  for (int i = 0; i < 4; i++) {                       // Add on the 4 digits that correspond to the command
    codeToTransmit[i+21] = lightB[State*4+i];
  }
  int highLength = 0;
  int lowLength = 0;
  for (int j = 0; j < timez; j++) {                   //The signal is transmitted X times in succession
    for (int i = 0; i < 25; i++) {
      switch (codeToTransmit[i]) {
        case 1: // SH + VLL
          highLength = 2;
          lowLength = 85;
          break;
        case 2: // SH + LL
          highLength = 2;
          lowLength = 7;
          break;
        case 3: // LH + DL
          highLength = 7;
          lowLength = 2;
          break;
      }
      digitalWrite(rfTransmitPin, HIGH);              // Transmit a HIGH signal - the duration of transmission will be determined  by the highLength and timeDelay variables
      delayMicroseconds(highLength * 120);
      digitalWrite(rfTransmitPin, LOW);               // Transmit a LOW signal - the duration of transmission will be determined by the lowLength and timeDelay variables
      delayMicroseconds(lowLength * 120);
    }
  }
  delay(100);
}
void PlantPot() {
//  Serial.println("PlantPot1");
  moisture = analogRead(PotPin);                                     // Read Moisture
  moisture = map(moisture, 0, 1024, 100, 0);
  if(moisture < Potmoist && PotToggle == 1 && lightToggle[0] == 1){     // If it's dry and the PotToggle and light is on
    digitalWrite(PotPout, 1);                                        // Water on
    delay (PotDelay*1000);                                           // Wait X Seconds
    digitalWrite(PotPout, 0);                                        // Water off
    PotToggle = 0;                                                   // PotToggle 0
  } else {
    digitalWrite(PotPout, 0);                                        // Water off
  }
//  Serial.println("PlantPot0");
}
void LCDTime() {
  lcd.setCursor(0,0);
  LCD2digits(timenow[0]);
  lcd.print(F(":"));
  LCD2digits(timenow[1]);
  lcd.print(F(":"));
  LCD2digits(timenow[2]);
}
void LCDisplay1() {
  lcd.clear();
  lcd.setCursor(10,0);
  lcd.print(F("Harvest:"));
  if(lightoff == 6){
    LCD2digits(daysleft);
  } else {
    lcd.print(F("NA"));
  }
  lcd.setCursor(0,1);
  lcd.print(F("Light:"));
  lcd.print(lightToggle[0]);
  lcd.print(F("/"));
  lcd.print(bulbCheck);
  lcd.print(F(" Temp:"));
  LCD2digits(tempC);
  lcd.setCursor(0,2);
  lcd.print(F("Pump :"));
  lcd.print(pumpToggle[0]);
  lcd.print(F("/"));
  lcd.print(waterToggle);
  if(lightoff == 6){
    lcd.print(F(" Flwr"));
  } else {
    lcd.print(F(" Grow"));
  }
  lcd.print(F("*"));
  if(pumpoff == 1){
    lcd.print(F("1min"));
  } else if(pumpoff == 5){
    lcd.print(F("5min"));
  } else if(pumpoff == 15){
    lcd.print(F("15min"));
  } else if(pumpoff == 60){
    lcd.print(F("Full"));
  } else if(pumpoff == 0){
    lcd.print(F("Off"));
  }
  lcd.setCursor(0,3);
  lcd.print(F("Fan:"));
  if(FullFanToggle[0] == 1){
    lcd.print(F("Full  "));
  } else if(IntFanToggle[0] == 1){
    lcd.print(F("Int   "));
  } else if (heatToggle == 2){
    lcd.print(F("Heater "));
  } else{
    lcd.print(F("Off   "));
  }
  if(pumpOveride == 1){
    lcd.print(F("PumpOv:"));
    LCD2digits(overrideTime[0]);
  } else if(lightOveride == 1){
    lcd.print(F("LiteOv:"));
    LCD2digits(overrideTime[1]);
  } else{
    lcd.print(F("           "));
  }
}
void LCDisplay2() {
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print(F("Photo:"));
  LCD2digits(lightreading);
  if(PotDelay == 0){
    lcd.print(F("  Hum:"));
    lcd.print(humy);
  } else {
    lcd.print(F(" Pot:"));
    lcd.print(moisture);
    lcd.print(F("/"));
    lcd.print(Potmoist);
    lcd.print(F(":"));
    lcd.print(PotDelay);
  }
  lcd.setCursor(0,1);
  lcd.print(F("OHeat:"));
  LCD2digits(errorcount[1]);
  lcd.print(F(" Lo:"));
  LCD2digits(tempLimits[0]);
  lcd.print(F(" Hi:"));
  LCD2digits(tempLimits[1]);
  lcd.setCursor(0,2);
  lcd.print(F("NoBulb:"));
  LCD2digits(errorlog[3]);
  lcd.print(F(" NWaterA:"));
  LCD2digits(errorlog[2]);
  lcd.setCursor(0,3);
  if(errorr > 1){
    lcd.print(errorrs[errorr]);     
  } else{
    lcd.print(F("           "));
  }
  lcd.setCursor(10,3);
  if(pumpOveride == 1){
    lcd.print(F("PumpOv:"));
    LCD2digits(overrideTime[0]);
  } else if(lightOveride == 1){
    lcd.print(F("LiteOv:"));
    LCD2digits(overrideTime[1]);
  }
}
void LCDisplay3() {
  for(int i=0; i<4; i++){
    if(lcdscreen == 2){
      number[i] = TLog[i];
    } else {
      number[i] = YLog[i];
    }
  }
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print(F("Fan  Off  "));
  LCD2digits(minstohoursmins(number[0],0));
  lcd.print(F(":"));
  LCD2digits(minstohoursmins(number[0],1));
  lcd.setCursor(0,1);
  lcd.print(F("Int  Fan  "));
  LCD2digits(minstohoursmins(number[1],0));
  lcd.print(F(":"));
  LCD2digits(minstohoursmins(number[1],1));
  lcd.setCursor(0,2);
  lcd.print(F("Full Fan  "));
  LCD2digits(minstohoursmins(number[2],0));
  lcd.print(F(":"));
  LCD2digits(minstohoursmins(number[2],1));
  lcd.setCursor(0,3);
  lcd.print(F("Heater On "));
  LCD2digits(minstohoursmins(number[3],0));
  lcd.print(F(":"));
  LCD2digits(minstohoursmins(number[3],1));
  lcd.setCursor(16,0);
  if(lcdscreen == 2){
    lcd.print("Tody");
  } else {
    lcd.print("Ysty");
  }
}

void Stars() {
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print(F("********************"));
    lcd.setCursor(0,1);
    lcd.print(F("*                  *"));
    lcd.setCursor(0,2);
    lcd.print(F("*                  *"));
    lcd.setCursor(0,3);
    lcd.print(F("********************"));
}

void MenuPrint() {
  lcd.clear();
  lcd.setCursor(3,0);
  lcd.print(F("Season: ")); 
  if(lightoff == 6){
    lcd.print(F("Flowering"));
  } else {
    lcd.print(F("Growing"));
  }
  lcd.setCursor(3,1);
  lcd.print(F("Pump on "));
  if (pumpoff == 60) {
    lcd.print(F("FULL"));
  } else if(pumpoff == 0){
    lcd.print(F("OFF"));
  } else {
    lcd.print(F("for "));
    LCD2digits(pumpoff);
  }
  lcd.setCursor(3,2);
  lcd.print(F("Fan Full: "));
  lcd.print(fanTrigger);
  lcd.setCursor(3,3);
  lcd.print(F("Fan Off: "));
  lcd.print(fanLow);
  lcd.setCursor(0,menuSet-1);
  if(submenu == 1){
    lcd.print(F("-O-"));
  } else {
    lcd.print(F(">>>"));
  }
}
void MenuPrint2() {
  lcd.clear();
  lcd.setCursor(3,0);
  lcd.print(F("Heater on: "));
  lcd.print(heatTrigger); 
  lcd.setCursor(3,1);
  lcd.print(F("Program Heater"));
  lcd.setCursor(3,2);
  lcd.print(F("PlantPot%: "));
  lcd.print(Potmoist); 
  lcd.setCursor(3,3);
  if(PotDelay == 0){
    lcd.print(F("Pot Routine OFF"));
  } else {
    lcd.print(F("Pot Delay: "));
    lcd.print(PotDelay);
    lcd.print("s");
  }
  lcd.setCursor(0,menuSet-5);
  if(submenu == 1){
    lcd.print(F("-O-"));
  } else {
    lcd.print(F(">>>"));
  }
}
void MenuPrint3() {
  lcd.clear();
  lcd.setCursor(3,0);
  lcd.print(F("Reset Hi/Lo Temps")); 
  lcd.setCursor(3,1);
  lcd.print(F("Save Settings")); 
  lcd.setCursor(3,2);
  lcd.print(F("Cancel")); 
  lcd.setCursor(3,3);
  lcd.print(F("Set Clock")); 
  lcd.setCursor(0,menuSet-9);
  if(submenu == 1){
    lcd.print(F("-O-"));
  } else {
    lcd.print(F(">>>"));
  }
}
void MenuPrint4() {
  unsigned long blinkMillis = millis();
  if (blinkMillis - blinktimer > 250) {
    heartbeat = !heartbeat;                                               // invert heartbeat toggle
    blinktimer = blinkMillis;
  }
  if(heartbeat == 1){
    blinking();
  } else {
    LCDTime2Set();
  }
}

void Keyboard1() {
  if (buttonState == 1) {                                       // If encoder is increased
    lcdscreen++;
  }
  if (buttonState == 2) {                                       // If encoder is decreased
    lcdscreen--;
  }
  if(lcdscreen < 0){
    lcdscreen = 3;
  }
  if(lcdscreen > 3){
    lcdscreen = 0;
  }
  if(lcdscreen == 0){
    LCDisplay1();
  } else if(lcdscreen == 1){
    LCDisplay2();
  } else if(lcdscreen == 2){
    LCDisplay3();
  } else if(lcdscreen == 3){
    LCDisplay3();
  }
  if (buttonState == 3) {                                       // If encoder is Clicked
    menuSet = 1;                                                // Start Menu System
    overrideTime[2] = sortthehour(timenow[1], 5);               // Add 5 mins to current time for overrideTime
    lcd.clear();
    MenuPrint();                                                // Print Menu
  }
  if (buttonState == 4 && lcdscreen == 0) {                     // If encoder is Held down
    lightOveride = !lightOveride;
      Stars();
      lcd.setCursor(8,1);
      lcd.print(F("Light"));
      lcd.setCursor(6,2);
      lcd.print(F("Override"));
      delay(1000);
      LCDisplay1();
    if (lightOveride == 1) {                                    // If override switched on
      overrideTime[1] = sortthehour(timenow[1], 30);            // Add 30 mins to current time for overrideTIme
    } 
  }
  if (buttonState == 4 && lcdscreen == 1) {                     // If encoder is Held down
    pumpOveride = !pumpOveride;
      Stars();
      lcd.setCursor(8,1);
      lcd.print(F("Pump"));
      lcd.setCursor(6,2);
      lcd.print(F("Override"));
      delay(1000);
      LCDisplay2();
    if (pumpOveride == 1) {                                     // If override switched on
      overrideTime[0] = sortthehour(timenow[1], 30);            // Add 30 mins to current time for overrideTIme
    } 
  }
}
void Keyboard2() {
  if (buttonState == 1) {                                       // If Encoder increased
    menuSet++;                                                  // Next menu
    if(menuSet > menus){                                        // If menuSet > menus
      menuSet = 1;                                              // Set menuSet to 1
    }
  }
  if (buttonState == 2) {                                       // If Encoder decreased
    menuSet--;                                                  // Previous menu
    if(menuSet < 1){                                            // If menuSet < 1
      menuSet = menus;                                          // Set menuSet to menus
    }
  }
  if(buttonState == 3){                                         // If button is pressed
    if (menuSet == 1) {
      if (lightoff == 6) {                                      // If Flowering, then set to Grow
        lightoff = 12;                                          //
        SetDate(0);                                             // Set Date for Finish to 0
      } else {                                                  // Else
        lightoff = 6;                                           // If Growing, then set to Flower
        SetDate(56);                                            // Set Date for Finish
      }
    }
    if (menuSet == 2) {                                         // If menu item is ...
      submenu = 1;
    }
    if (menuSet == 3) {                                         // If menu item is ...
      submenu = 1;
    }
    if (menuSet == 4) {                                         // If menu item is ...
      submenu = 1;
    }
    if (menuSet == 5) {                                         // If menu item is ...
      submenu = 1;
    }
    if (menuSet == 6) {                                         // If menu item is ...
      lcd.clear();
      Stars();
      lcd.setCursor(4,1);
      lcd.print(F("Programming"));
      lcd.setCursor(8,2);
      lcd.print(F("Heater"));
      transmitCode(2,8);
      delay(500);
      lcdscreen = 0;
      menuSet = 0;
      LCDisplay1();
      LCDTime();
    }
    if (menuSet == 7) {                                         // If menu item is ...
      submenu = 1;
    }
    if (menuSet == 8) {                                         // If menu item is ...
      submenu = 1;
    }
    if (menuSet == 9) {                                         // If menu item is ...
      tempLimits[0] = tempC;                                    // Set Lowtemp to temperature
      tempLimits[1] = tempC;                                    // Set Hightemp to temperature
      Stars();
      lcd.setCursor(6,1);
      lcd.print(F("Resetting"));
      lcd.setCursor(1,2);
      lcd.print(F("Hi/Lo Temperatures"));
      delay(1500);
      menuSet = 0;
      lcdscreen = 0;
      LCDisplay1();
      LCDTime();
    }
    if (menuSet == 10) {                                        // If menu item is ...
      lcd.clear();
      Stars();
      lcd.setCursor(3,1);
      lcd.print(F("Saving Settings"));
      lcd.setCursor(6,2);
      lcd.print(F("To EEPROM"));
      writeepprom();
      delay(500);
      menuSet = 0;
      lcdscreen = 0;
      LCDisplay1();
      LCDTime();
    }
    if (menuSet == 11) {                                        // If menu item is ...
      lcd.clear();
      Stars();
      lcd.setCursor(5,1);
      lcd.print(F("Cancelling"));
      readepprom();
      delay(500);
      lcdscreen = 0;
      menuSet = 0;
      LCDisplay1();
      LCDTime();
    }
    if (menuSet == 12) {                                        // If menu item is ...
      lcd.clear();
      Stars();
      lcd.setCursor(7,1);
      lcd.print(F("Setting"));
      lcd.setCursor(7,2);
      lcd.print(F("Clock"));
      delay(500);
      SetaClock = 1;
      buttonState = 0;
      lcd.clear();
      tmElements_t tm;                                                      // Read clock
      if (RTC.read(tm)) {
        timenow[0] = tm.Hour;
        timenow[1] = tm.Minute;
        timenow[2] = tm.Second;
        timenow[3] = tm.Day;
        timenow[4] = tm.Month;
        timenow[5] = tmYearToCalendar(tm.Year);
      }
    }
  }
  if(SetaClock == 1){
      keyboard4();
  } else if(menuSet > 0 && menuSet < 5){
    MenuPrint();
  } else if(menuSet >4 && menuSet <9){
    MenuPrint2();
  } else if(menuSet >8 && menuSet <13){
    MenuPrint3();
  }
}

void Keyboard3() {
  if (buttonState == 1) {                                       // If Encoder increased
    if (menuSet == 2) {                                         // If menu item is ...
      if (pumpoff == 0) {                                       // If set to 0 minutes
        pumpoff = 1;                                            // change to 1
      } else if (pumpoff == 1) {                                // If set to 1 minutes
        pumpoff = 5;                                            // change to 5
      } else if (pumpoff == 5) {                                // If set to 5 minutes
        pumpoff = 15;                                           // change to 15
      } else if (pumpoff == 15) {                               // else if 15
        pumpoff = 60;                                           // change to FULL
      } else if (pumpoff == 60) {                               // else if FULL
        pumpoff = 0;                                            // change to 0
      }
    }
    if (menuSet == 3) {                                         // If menu item is ...
      fanTrigger++;                                             // Increase fanTrigger
      if(fanTrigger > 28){                                      // If fanTrigger > 28
        fanTrigger = 23;                                        // Set fanTrigger to 23
      }
    }
    if (menuSet == 4) {                                         // If menu item is ...
      fanLow++;                                                 // Increase fanLow
      if(fanLow > 25){                                          // If fanLow > 25
        fanLow = 12;                                            // Set fanLow to 12
      }
    }
    if (menuSet == 5) {                                         // If menu item is ...
      heatTrigger++;                                            // Increase heatTrigger
      if(heatTrigger > 16){                                     // If heatTrigger > 16
        heatTrigger = 10;                                       // Set heatTrigger to 10
      }
    }
    if (menuSet == 7) {                                         // If menu item is ...
      Potmoist++;                                               // Increase Potmoist
      if(Potmoist > 95){                                        // If Potmoist > 28
        Potmoist = 85;                                          // Set Potmoist to 23
      }
    }
    if (menuSet == 8) {                                         // If menu item is ...
      PotDelay++;                                               // Increase PotDelay
      if(PotDelay > 10){                                        // If PotDelay > 10secs
        PotDelay = 0;                                           // Set PotDelay to 0secs
      }
    }
  }
  if (buttonState == 2) {                                       // If Encoder decreased
    if (menuSet == 2) {                                         // If menu item is ...
      if (pumpoff == 0) {                                       // If set to 0 minutes
        pumpoff = 60;                                           // change to FULL
      } else if (pumpoff == 60) {                               // If set to FULL minutes
        pumpoff = 15;                                           // change to 15
      } else if (pumpoff == 15) {                               // If set to 15 minutes
        pumpoff = 5;                                            // change to 5
      } else if (pumpoff == 5) {                                // else if 5
        pumpoff = 1;                                            // change to 1
      } else if (pumpoff == 1) {                                // else if 1
        pumpoff = 0;                                            // change to 0
      }
    }
    if (menuSet == 3) {                                         // If menu item is ...
      fanTrigger--;                                             // Increase fanTrigger
      if(fanTrigger < 23){                                      // If fanTrigger > 28
        fanTrigger = 28;                                        // Set fanTrigger to 23
      }
    }
    if (menuSet == 4) {                                         // If menu item is ...
      fanLow--;                                                 // Increase fanLow
      if(fanLow < 12){                                          // If fanLow > 25
        fanLow = 25;                                            // Set fanLow to 12
      }
    }
    if (menuSet == 5) {                                         // If menu item is ...
      heatTrigger--;                                            // Increase heatTrigger
      if(heatTrigger < 10){                                     // If heatTrigger > 16
        heatTrigger = 16;                                       // Set heatTrigger to 10
      }
    }
    if (menuSet == 7) {                                         // If menu item is ...
      Potmoist--;                                               // Increase Potmoist
      if(Potmoist < 85){                                        // If Potmoist > 28
        Potmoist = 95;                                          // Set Potmoist to 23
      }
    }
    if (menuSet == 8) {                                         // If menu item is ...
      PotDelay--;                                               // Increase PotDelay
      if(PotDelay < 0){                                         // If PotDelay > 10secs
        PotDelay = 10;                                          // Set PotDelay to 0secs
      }
    }
  }
  if(buttonState == 3){                                         // If button is pressed
    submenu = 0;
  }
  if(menuSet > 0 && menuSet < 5){
    MenuPrint();
  } else if(menuSet >4 && menuSet <9){
    MenuPrint2();
  } else if(menuSet >8){
    MenuPrint3();
  } else {
    // Do fuck all
  }
}
void keyboard4() {

    if (buttonState == 1) {
      timenow[Sett]++;
      if(timenow[Sett] > Ulimit[Sett]){
        timenow[Sett] = Llimit[Sett];
      }
    }
    else if (buttonState == 2) {
      timenow[Sett]--;
      if(timenow[Sett] < Llimit[Sett]){
        timenow[Sett] = Ulimit[Sett];
      }
    }
    if (buttonState == 3) {
      Sett++;
      if(Sett > 5){
        SetClock();
      }
    }
}
void blinking(){
  lcd.setCursor(pos[Sett*2],pos[(Sett*2)+1]);
  lcd.print("  ");
  if(pos[Sett*2] == 6 && pos[(Sett*2+1)] == 1){
    lcd.print("  ");
  }
}
void LCDTime2Set() {
  lcd.setCursor(0,0);
  LCD2digits(timenow[0]);                                                      // Hour
  lcd.print(":");
  LCD2digits(timenow[1]);                                                      // Minute
  lcd.print(":");
  LCD2digits(timenow[2]);                                                      // Second
  lcd.setCursor(0,1);
  LCD2digits(timenow[3]);                                                      // Day
  lcd.print("/");
  LCD2digits(timenow[4]);                                                      // Month
  lcd.print("/");
  lcd.print(timenow[5]);                                                       // Year
  lcd.setCursor(11,0);
  lcd.print("Set  ");
}
void SetClock() {
  Sett = 0;
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Setting Clock");
  lcd.setCursor(0,1);
  lcd.print(" Please Wait");
  setTime(timenow[0],timenow[1],timenow[2],timenow[3],timenow[4],timenow[5]-2000);
  RTC.set(now());
  delay(2000);
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Clock Set");
  delay(2000);
  lcd.clear();
  SetaClock = 0;
  menuSet = 0;
}

void InitWiFi()
{
  Serial1.begin(9600);
  // initialize ESP module
  lcd.clear();
  lcd.print("Attempting WiFi");
  WiFi.init(&Serial1);
  // check for the presence of the shield
  if (WiFi.status() == WL_NO_SHIELD) {
    Serial.println("WiFi shield not present");
    lcd.setCursor(0,1);
    lcd.print("No Shield");
    lcd.setCursor(0,2);
    lcd.print("(Hardware Issue)");
    // don't continue
    while (true);
  }

  Serial.println("Connecting to AP ...");
  lcd.setCursor(0,1);
  lcd.print("Connecting");
  // attempt to connect to WiFi network
  while ( status != WL_CONNECTED) {
    Serial.print("Attempting to connect to WPA SSID: ");
    Serial.println(WIFI_AP);
    lcd.setCursor(0,2);
    lcd.print("Trying AP ");
    lcd.print(WIFI_AP);
    // Connect to WPA/WPA2 network
    status = WiFi.begin(WIFI_AP, WIFI_PASSWORD);
    delay(500);
  }
  Serial.println("Connected to AP");
  lcd.setCursor(0,3);
  lcd.print("Connected");
  delay(1000);
}

void reconnect() {
  // Loop until we're reconnected
  for(int i=0; i<3; i++){
    Serial.print("Connecting to HA MQTT Server ...");
/*
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("MQTT Conn Attempt:"); lcd.print(i+1);
*/
    // Attempt to connect (clientId, username, password)
    if ( client.connect(CLIENT_NAME, "harry", NULL) ) {
      Serial.println( "[DONE]" );
//      lcd.setCursor(0,1);
//      lcd.print("[DONE]");
      break;
    } else {
      Serial.print( "[FAILED] [ rc = " );
      Serial.print( client.state() );
      Serial.println( " : retrying in 1 second]" );
      // Wait 5 seconds before retrying
/*
    lcd.setCursor(0,1);
    lcd.print("[FAILED]");
    lcd.setCursor(0,2);
    lcd.print( client.state() );
*/
      delay( 1000 );
    }
  }
}
void SendData()
{
  StaticJsonDocument<200> doc;
//  JsonObject root = doc.as<JsonObject>();

  // Add values in the object
  //
  // Most of the time, you can rely on the implicit casts.
  // In other case, you can do root.set<long>("time", 1351824120);
  if(lightoff == 6){
    doc["season"] = "Flowering";
  } else {
    doc["season"] = "Growing";
  }
  doc["error"] = errorrs[errorr];
  doc["light"] = lightToggle[0];
  doc["pump"] = pumpToggle[0];

  if(FullFanToggle[0] == 1){
    doc["fan"] = 2;
  } else if(IntFanToggle[0] == 1){
    doc["fan"] = 1;
  } else if (heatToggle == 2){
    doc["fan"] = 3;
  } else{
    doc["fan"] = 0;
  }

  doc["temp"] = tempC;
  doc["humi"] = humy;
  doc["daysleft"] = daysleft;

  char buffer[512];
  serializeJsonPretty(doc, buffer);
//  serializeJsonPretty(doc, Serial);
  size_t len = measureJson(doc);
  client.publish("garden/", buffer, len);
}
