/*
* Self-winding clock synchronizer software
*
* The purpose of this software is to maintain an accurate timebase and send a synchronizing pulse once an hour to a
* self-winding clock.
*
* The accurate timebase is obtained from GPS satellites.
*
* Hardware is based on the following:
*
* - Adafruit ESP32-S2 Reverse TFT Feather ( https://www.adafruit.com/product/5345 )
*   - Contains:
*     - ESP32-S2 Tensilica SOC, 240 MHz single-core, native USB, WiFi
*     - 1.14" 240x135 pixel color TFT display with ST7789 chipset
*     - 4 MB flash memory
*     - 2 MB PSRAM memory
*     - MAX17048 lithium-ion battery charge/fuel gauge IC
*     - STEMMA QT connector (JST SH 4-pin) for I2C devices
*     - NeoPixel RGB LED status indicator
*     - Three user interface buttons
*     - USB-C connector for charging lithium-ion battery and for serial communications with a PC
*
* - Adafruit Ultimate GPS FeatherWing ( https://www.adafruit.com/product/3133 )
*
* - Adafruit Power Relay FeatherWing ( https://www.adafruit.com/product/3191 )
*
*
* Arduino libraries:
* - Adafruit_MAX1704X - Library for MAX17048 lithium-ion fuel gauge IC
* - Adafruit_NeoPixel - Library for Adafruit NeoPixel single-wire based RGB LED pixel
* - Adafruit_ST7789   - Library for Adafruit 1.14" color TFT display with SPI and ST7789 TFT driver
* - Adafruit_GPS      - Library for Adafruit GPS module
* - ESP32Time         - Library for setting and retrieving internal RTC time on ESP32 boards, by fbiego
*
* Inputs:
*   D0 : ENABLE_DISABLE_OUTPUT - Enable/disable sync relay output (pin pulled HIGH by default, goes LOW when pressed)
*   D1 : TRIGGER_OUTPUT        - Manually trigger sync relay output (for as long as the button is held down) (pin pulled LOW by default, goes HIGH when pressed)
*
* Outputs:
*   Pin 10: Output signal to coil of relay for energizing the synchronizer of the self-winding clock (also red LED on relay board)
*   Pin 13: Output to red LED on ESP32 board (next to USB-C connector) (HIGH is on)
*
*
* TO DO:
* - Add failsafe to disable relay signal if it is detected to be active for more than 3 seconds
*/


// #defines
// Need to include this #define to have GPS library use hardware serial instead of software serial
#define NO_SW_SERIAL
#define GPSSerial Serial1
#define ONE_DAY_IN_SECONDS (60 * 60 * 24)
#define ONE_DAY_IN_MILLISECONDS (1000 * 60 * 60 * 24)
#define NUM_GPS_MSGS_RCVD_FOR_TIME_TO_BE_VALID    3
// Constants for GPIO outputs
#define ENABLE_DISABLE_OUTPUT             0
#define TRIGGER_OUTPUT                    1
#define CLOCK_SYNC                        10    /* Output pin for clock synchronizer relay control */
#define RED_LED                           13
#define ENABLE_DISABLE_OUTPUT_ACTIVE      LOW
#define TRIGGER_OUTPUT_ACTIVE             HIGH
#define RED_LED_ACTIVE                    HIGH
#define RED_LED_INACTIVE                  LOW

// #includes
#include <Arduino.h>
#include <Adafruit_GPS.h>
#include "Adafruit_MAX1704X.h"
#include <Adafruit_NeoPixel.h>
#include <Adafruit_ST7789.h>
#include "Adafruit_TestBed.h"
#include <ESP32Time.h>
#include <Fonts/FreeSans12pt7b.h>
#include <Fonts/FreeSans9pt7b.h>


// Global variables
uint8_t gMainLoopCounter = 0;                         // Main loop counter
bool gSynchronizerOutputManuallyDisabled = false;     // True if the synchronizer output is manually disabled
bool gRTCTimeValid = false;                           // True if the RTC time is valid
bool gGPSTimeValid = false;                           // True if the GPS time is valid
bool gRTCNeverSyncedToGPS = true;                     // Set to false the first time the RTC gets synced to GPS time base
nmea_float_t gSecondsSinceLastValidGPSTime;           // Seconds since the last valid GPS time was received (equal to 0 if never received)
unsigned long gTimeRTCLastSyncedToGPS = 0;            // Time (in milliseconds since board booted up) when RTC was last synced to GPS time base
uint32_t gNumGPSMsgsRcvd = 0;                         // Number of times a message has been received from the GPS module
bool gMAX17048_OK = true;                             // True if MAX17048 battery charger/fuel gauge IC is okay
bool gTriggerOutputActivePreviously = false;          // True if trigger output button was active previous time thru the loop
int  gPreviousPinValEnableDisableOutput;              // Previous pin value of enable/disable output button
int  gPreviousPinValTriggerOutput;                    // Previous pin value of trigger output button
extern Adafruit_TestBed TB;
Adafruit_MAX17048 gLiPo;                              // Adafruit lithium-ion battery charger/fuel gauge object
Adafruit_ST7789 gDisplay = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);  // Adafruit TFT LCD display object
GFXcanvas16 gCanvas(240, 135);                        // A GFX 16-bit canvas context for graphics, part of the Adafruit GFX graphics class
Adafruit_GPS GPS(&GPSSerial);                         // Adafruit GPS object for communicating with GPS module
ESP32Time rtc;                                        // ESP32 RTC object


//Function prototypes
void readGPS();                                       // Get data from GPS module
void syncRTCtoGPS();                                  // Sync the RTC to the GPS time base to keep the RTC accurate
void updateDisplay();                                 // Update the LCD display
void updateRTCTimeWithGPSTime();                      // Update the internal RTC time with the latest GPS time


// Setup
void setup() {
  // Set clock synchronizer GPIO pin to be an output and turn it off
  pinMode(CLOCK_SYNC, OUTPUT);
  digitalWrite(CLOCK_SYNC, LOW);

  Serial.begin(115200);
  //while (! Serial) delay(10);

  // Delay to allow serial to become ready
  delay(500);

  // Initialize the RTC
  rtc.setTime(0, 0, 14, 26, 2, 2024);                     // Default date/time to 2/26/2024 2:00 PM

  // Start the GPS software at 9600 baud rate
  GPS.begin(9600);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);           // Turn on RMC (recommended minimum) and GGA (fix data)
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_100_MILLIHERTZ);   // 10 second update time
  GPS.sendCommand(PGCMD_ANTENNA);                         // Request updates on antenna status
  delay(1000);
  GPSSerial.println(PMTK_Q_RELEASE);                      // Ask for GPS firmware version

  // Start the NeoPixel RGB LED
  TB.neopixelPin = PIN_NEOPIXEL;
  TB.neopixelNum = 1; 
  TB.begin();
  TB.setColor(WHITE);

  // Start the LCD display
  gDisplay.init(135, 240);            // Init ST7789 240x135
  gDisplay.setRotation(3);
//  gCanvas.setFont(&FreeSans12pt7b);
  gCanvas.setFont(&FreeSans9pt7b);
  gCanvas.setTextColor(ST77XX_WHITE); 
  delay(500);

  // Activate the LCD backlight
  pinMode(TFT_BACKLITE, OUTPUT);
  digitalWrite(TFT_BACKLITE, HIGH);

  // Start the lithium ion battery charger/monitor
  if (!gLiPo.begin()) {
    Serial.println(F("Could not find Adafruit MAX17048?\nMake sure a battery is plugged in!"));
    gMAX17048_OK = false;
  }
  if (gMAX17048_OK) {
    Serial.print(F("Found MAX17048"));
    Serial.print(F(" with Chip ID: 0x")); 
    Serial.println(gLiPo.getChipID(), HEX);
  }

  // Set user buttons to be inputs
  pinMode(ENABLE_DISABLE_OUTPUT, INPUT_PULLUP);
  pinMode(TRIGGER_OUTPUT, INPUT_PULLDOWN);
  pinMode(2, INPUT_PULLDOWN);

  // Set red LED to be output
  pinMode(RED_LED, OUTPUT);
  digitalWrite(RED_LED, RED_LED_INACTIVE);

  // Set the previous state of the buttons to the current value so we don't detect a button press/release at startup
  gPreviousPinValEnableDisableOutput = digitalRead(ENABLE_DISABLE_OUTPUT);
  gPreviousPinValTriggerOutput       = digitalRead(TRIGGER_OUTPUT);

  Serial.println();
  Serial.println();
  Serial.println();
  Serial.println();
  Serial.println();
  Serial.println("************************************************");
  Serial.println("************************************************");
  Serial.println("********** Clock synchronizer startup **********");
  Serial.println("************************************************");
  Serial.println("************************************************");
}


// Main loop
void loop() {

  // Get data from the GPS module
  readGPS();

  // Update GPS time valid/not valid state
  // GPS time valid logic:
  // 1) When program first starts running, GPS time is invalid
  // 2) GPS time becomes valid when "time since last valid GPS time" becomes less than or equal to 120 seconds AND at least NUM_GPS_MSGS_RCVD_FOR_TIME_TO_BE_VALID GPS messages have been received
  // 3) GPS time becomes invalid when "time since last valid GPS time" becomes greater than 120 seconds

  // See how long it has been since we last received a valid GPS time
  gSecondsSinceLastValidGPSTime = GPS.secondsSinceTime();

  if ((gGPSTimeValid) && (gSecondsSinceLastValidGPSTime > 120)) {
    // Time is now invalid
    gGPSTimeValid = false;
    Serial.println("ERR: GPS TIME NOT VALID");
    Serial.print("Sec since valid: ");
    Serial.println(gSecondsSinceLastValidGPSTime);
  }

  if ( (!gGPSTimeValid) && (gSecondsSinceLastValidGPSTime <= 120) && (gNumGPSMsgsRcvd >= NUM_GPS_MSGS_RCVD_FOR_TIME_TO_BE_VALID) ) {
    // Time is now valid
    gGPSTimeValid = true;
    Serial.println("INFO: GPS TIME VALID");
    Serial.print("Sec since valid: ");
    Serial.println(gSecondsSinceLastValidGPSTime);
  }

  // Update RTC time valid/not valid state
  // RTC time valid logic:
  // 1) When program first starts running, RTC time is invalid
  // 2) RTC time becomes valid when RTC time gets synced to GPS time (handled lower down in the main loop)
  // 3) RTC time becomes invalid when "time since RTC time last synced to GPS time" becomes greater than one day

  if ((gRTCTimeValid) && ((millis() - gTimeRTCLastSyncedToGPS) > ONE_DAY_IN_MILLISECONDS)) {
    // Time is now invalid
    gRTCTimeValid = false;
//    gSynchronizerOutputEnabled = false;
    Serial.println("ERR: RTC TIME NOT VALID");
    Serial.print("Sec since RTC synced to GPS: ");
    Serial.println(gTimeRTCLastSyncedToGPS / 1000);
  }


  // Check if we need to sync the RTC time to the GPS time base
  // RTC time gets synced to GPS time in the following cases:
  // 1) After program start if RTC has never been synced to GPS and the GPS time is valid and at least NUM_GPS_MSGS_RCVD_FOR_TIME_TO_BE_VALID GPS messages have been received
  // 2) Once a day if GPS time is valid
  if ( ( ( (gRTCNeverSyncedToGPS) && (gNumGPSMsgsRcvd >= NUM_GPS_MSGS_RCVD_FOR_TIME_TO_BE_VALID)) || ((millis() % ONE_DAY_IN_MILLISECONDS) == 0) ) && gGPSTimeValid ) {
    gRTCNeverSyncedToGPS = false;
    gTimeRTCLastSyncedToGPS = millis();
    syncRTCtoGPS();
    // If RTC had been previously invalid, then do some stuff now that RTC is valid
    if (!gRTCTimeValid) {
      // Time is now valid
      gRTCTimeValid = true;
//      gSynchronizerOutputEnabled = true;
      Serial.println("INFO: RTC TIME VALID");
    }
  }


  // Check for user inputs
  // Check for enable/disable output button
  int pinVal = digitalRead(ENABLE_DISABLE_OUTPUT);
  if ( (pinVal == ENABLE_DISABLE_OUTPUT_ACTIVE) && (pinVal != gPreviousPinValEnableDisableOutput) ){
    // Enable/disable output button is newly pressed, so toggle enable/disable state of output
    gSynchronizerOutputManuallyDisabled = !gSynchronizerOutputManuallyDisabled;         // Toggle the enable/disable flag
  }
  gPreviousPinValEnableDisableOutput = pinVal;        // Remember the current reading for next time thru the loop

  // Check for manual output trigger button
  pinVal = digitalRead(TRIGGER_OUTPUT);
  if (pinVal == TRIGGER_OUTPUT_ACTIVE) {
    if (pinVal != gPreviousPinValTriggerOutput) {
      // Turn on the sync relay output
      digitalWrite(CLOCK_SYNC, HIGH);
      digitalWrite(RED_LED, RED_LED_ACTIVE);
      Serial.println("PULSE ON MANUAL");
    }
  } else if (pinVal != gPreviousPinValTriggerOutput){
    digitalWrite(CLOCK_SYNC, LOW);
    digitalWrite(RED_LED, RED_LED_INACTIVE);
    Serial.println("PULSE OFF MANUAL");
  }
  gPreviousPinValTriggerOutput = pinVal;              // Remember the current reading for next time thru the loop


  // Activate/deactivate clock synchronizer relay
  // If the RTC time is valid, then see if need to activate or deactivate the clock synchronizer relay
  uint8_t seconds = rtc.getSecond();
  uint8_t minutes = rtc.getMinute();
  if ( (gRTCTimeValid) && (!gSynchronizerOutputManuallyDisabled) ) {
    // Once an hour, one second before the top of the hour, activate the self-winding clock synchronizer circuit to make sure clock is set properly
    if ( (minutes == 59) && (seconds == 59) )
//    if ( (seconds == 59) || (seconds == 29) )
    {
      if (digitalRead(CLOCK_SYNC) == LOW)
      {
        digitalWrite(CLOCK_SYNC, HIGH);   // Activate the clock synchronizer circuit (will get de-activated one second later)
        digitalWrite(RED_LED, RED_LED_ACTIVE);
        Serial.print("PULSE ON    ");
        Serial.print("Min: ");
        Serial.print(minutes);
        Serial.print("  Sec: ");
        Serial.println(seconds);
      }
    }
  }

  // NOTE: Always want to try to deactivate the clock sync output (even if RTC time is not valid) to handle the
  // corner case where RTC time had been valid when the clock sync output was activated, but it became invalid
  // before it was time to deactivate it.
  // Once an hour, at the top of the hour, deactivate the self-winding clock synchronizer circuit
  if ( (minutes == 0) && (seconds == 0) )
  //if ( (seconds == 0) || (seconds == 30) )
  {
    if (digitalRead(CLOCK_SYNC) == HIGH)
    {
      digitalWrite(CLOCK_SYNC, LOW);    // De-activate the clock synchronizer circuit
      digitalWrite(RED_LED, RED_LED_INACTIVE);
      Serial.print("PULSE OFF   ");
      Serial.print("Min: ");
      Serial.print(minutes);
      Serial.print("  Sec: ");
      Serial.println(seconds);
    }
  }
  
  // TODO: add failsafe to deactivate the relay if it is detected to be on for more than 3 seconds

  // Update the LCD display about every 200 milliseconds
  if (gMainLoopCounter % 2 == 0) {
    updateDisplay();
  }

  // Set main loop timing (approximate is fine)
  delay(100);             // Delay about 100 milliseconds
  gMainLoopCounter++;
}


// Get data from GPS module
void readGPS()
{
  bool done = false;
  int  counter = 0;
  while (!done && (counter < 1000)) {
    counter++;
    // Read data from the GPS
    char c = GPS.read();
    if (c == 0) {
      done = true;
    } else {
      Serial.print(c);
    }
    // Check for new NMEA data received from the GPS module
    if (GPS.newNMEAreceived()) {
      if (GPS.parse(GPS.lastNMEA())) {
        // Increment the number of GPS messages received
        // It seems like the GPS time does not stabilize right away, so attempting to compensate by not
        // syncing the RTC to the GPS time until a few GPS messages have been received
        gNumGPSMsgsRcvd++;
        Serial.print("GPS Msgs: ");
        Serial.println(gNumGPSMsgsRcvd);
      } else {
        Serial.println("No GPS parse");
      }
    }
  }
}


// Sync the RTC to the GPS time base to keep the RTC accurate
void syncRTCtoGPS()
{
  // Note: Need to add the "time since last valid GPS time" value (in seconds) to the stored GPS time to get the actual current time
  // to be stored in the RTC time

  Serial.println("*** SYNCING RTC TO GPS TIME ***");

  // Get the current GPS time values and put them in a tm struct
  struct tm gps_tm;
  gps_tm.tm_sec = GPS.seconds;
//  Serial.print("Sec: ");
  Serial.println(gps_tm.tm_sec);
  gps_tm.tm_min = GPS.minute;
//  Serial.print("Min: ");
  Serial.println(gps_tm.tm_min);
  gps_tm.tm_hour = GPS.hour;
//  Serial.print("Hr : ");
  Serial.println(gps_tm.tm_hour);
  gps_tm.tm_mday = GPS.day;
//  Serial.print("Day: ");
  Serial.println(gps_tm.tm_mday);
  gps_tm.tm_mon = GPS.month - 1;
//  Serial.print("Mon: ");
  Serial.println(gps_tm.tm_mon);
  gps_tm.tm_year = GPS.year + 100;
//  Serial.print("Yr : ");
  Serial.println(gps_tm.tm_year);

  // Convert to a time_t value
  time_t t = mktime(&gps_tm);
  Serial.print("mktime: ");
  Serial.println(asctime(&gps_tm));
  Serial.print("t: ");
  Serial.println(t);
  Serial.print("GPSsecSinceTime: ");
  nmea_float_t sst = GPS.secondsSinceTime();
  Serial.println(sst);

  // Add in the "time since last valid GPS time" value (in seconds)
  t += (int)sst;
  Serial.print("t(updated): ");
  Serial.println(t);

  // Save to RTC
  rtc.setTime(t);

  // Output to serial
  Serial.println(ctime(&t));
}


// Update the LCD display
/* LCD display:
*   Line 1:  "UTC: 13:27:42"            (Current UTC time, with "*" added at the end if time is not valid)
*   Line 2:  "Mon Mar 4 13:27:42 2024"  (Current UTC date and time)
*   Line 3:  "GPS Age: 12  Sats: 12"    (GPS Age (seconds since last valid GPS time was received) and number of GPS satellites being used)
*   Line 4:  "Lat: 42.45  Long: -88.42" (Latitude and longitude)
*   Line 5:  "Sync Out: ENABLED"        (Clock sync pulse output state, possible values: "ENABALED", "DISABLED", "HIGH")
*   Line 6:  "Battery: 4.1 V / 95%"     (Internal Lithium-ion battery voltage and state of charge)
*/
void updateDisplay()
{
  gCanvas.fillScreen(ST77XX_BLACK);
  gCanvas.setCursor(0, 17);

  // Line 1
  gCanvas.setTextColor(ST77XX_RED);
  gCanvas.print("UTC: ");
  int hour = rtc.getHour(true);         // true indicates 24 hour time
  if (hour < 10) { gCanvas.print('0'); }
  gCanvas.print(hour);
  gCanvas.print(":");
  int minute = rtc.getMinute();
  if (minute < 10) { gCanvas.print('0'); }
  gCanvas.print(minute);
  gCanvas.print(":");
  int second = rtc.getSecond();
  if (gRTCTimeValid) {
    if (second < 10) { gCanvas.print('0'); }
    gCanvas.println(second);
  } else {
    if (second < 10) { gCanvas.print('0'); }
    gCanvas.print(second);
    gCanvas.println(" *");
  }

  // Line 2
  time_t t = rtc.getEpoch();
  gCanvas.print(ctime(&t));

  // Line 3
  gCanvas.setTextColor(ST77XX_GREEN);
  gCanvas.print("GPS Age: ");
  if (gSecondsSinceLastValidGPSTime < 0.001) {
    // Value is zero, which means no valid GPS time has been received
    gCanvas.print("N/A sec");
  } else if (gSecondsSinceLastValidGPSTime > 9999.0) {
    gCanvas.print("9999+ sec");
  } else {
    gCanvas.print((int)gSecondsSinceLastValidGPSTime);
    gCanvas.print(" sec");
  }
  gCanvas.print("   Sats: ");
  gCanvas.println(GPS.satellites);

  // Line 4
  gCanvas.print("Lat: ");
  gCanvas.print(GPS.latitudeDegrees);
  gCanvas.print("   Long: " );
  gCanvas.println(GPS.longitudeDegrees);

  // Line 5
  gCanvas.setTextColor(ST77XX_BLUE); 
  gCanvas.print("Sync Out: ");
  gCanvas.setTextColor(ST77XX_WHITE);
  if (digitalRead(CLOCK_SYNC) == HIGH) {
    gCanvas.println("HIGH");
  } else if ( (gRTCTimeValid) && (!gSynchronizerOutputManuallyDisabled) ) {
    gCanvas.println("ENABLED");
  } else {
    gCanvas.println("DISABLED");
  }

  // Line 6
  gCanvas.setTextColor(ST77XX_MAGENTA);
  gCanvas.print("Battery: ");
  gCanvas.setTextColor(ST77XX_WHITE);
  if (gMAX17048_OK) {
    gCanvas.print(gLiPo.cellVoltage(), 1);
    gCanvas.print(" V  /  ");
    float cp = gLiPo.cellPercent();
    if (cp > 100.0) cp = 100.0;       // Limit max battery percentage to 100%
    gCanvas.print(cp, 0);
    gCanvas.println("%");
  }

  gDisplay.drawRGBBitmap(0, 0, gCanvas.getBuffer(), 240, 135);
}
