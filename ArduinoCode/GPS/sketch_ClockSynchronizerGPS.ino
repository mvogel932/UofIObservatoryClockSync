/*
* Self-winding clock synchronizer software
*
* The purpose of this software is to maintain an accurate timebase and send a synchronizing pulse once an hour to a
* self-winding clock.
*
* The accurate timebase is obtained over the Internet from an NTP server via a WiFi connection.
*
* Outputs:
*   Pin 15: Output signal to coil of relay for energizing the synchronizer of the self-winding clock (also red LED on relay board)
*   Pin 2 : Blue LED, blinks at different rates to indicate status of WiFi and NTP time
*/
#include <ESP8266WiFi.h>
#include <NTPClient.h>
#include <WiFiUdp.h>

// WiFi network constants
const char* ssid     = "MViPhone6s";            // TODO: Put in correct ssid for U of I Observatory WiFi network
const char* password = "Observatory1896";       // TODO: Put in correct password for U of I Observatory WiFi network
//const char* ssid     = "Spike2";                // TODO: Put in correct ssid for U of I Observatory WiFi network
//const char* password = "uias8427";              // TODO: Put in correct password for U of I Observatory WiFi network

//unsigned int gWifiStatus = WL_DISCONNECTED;   // WiFi status, init to WL_DISCONNECTED

// Constants for GPIO outputs
const int CLOCK_SYNC = 15;
const int BLUE_LED   = 2;

// Main loop timing
const unsigned long MAIN_LOOP_INTERVAL = 500;
unsigned long       currentMsec        = 0;
unsigned long       previousMsec       = 0;

// NTP time values
const long utcOffsetInSeconds = -18000;   // Offset from UTC to Central Daylight Time (minus 5 hours)
int  gMinutes        = 0;
int  gSeconds        = 0;

// Constants for the blink period of the blue LED
// Note that the main loop period is about 500 msec, so the values need to be twice the number of seconds desired
const int LED_PERIOD_NO_WIFI_NO_NTPTIME = 8;
const int LED_PERIOD_WIFI_OK_NO_NTPTIME = 4;
const int LED_PERIOD_WIFI_OK_NTPITME_OK = 1;

// Blue LED variables
int  ledPeriod       = LED_PERIOD_NO_WIFI_NO_NTPTIME;   // Blink period of the blue LED
int  ledCounter      = ledPeriod;                       // Blink counter for blue LED
bool blueLedOn       = false;                           // Indicates blue LED is on


char daysOfTheWeek[7][12] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};

// Define NTP Client to get time
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org", utcOffsetInSeconds);

// Function prototypes
void toggleBlueLED();   // Function to toggle the blue LED

//
// Setup function
//
void setup()
{
  // Set blue LED GPIO pin to be an output and turn it off
  pinMode(BLUE_LED, OUTPUT);
  digitalWrite(BLUE_LED, HIGH);
  blueLedOn = false;

  // Set clock synchronizer GPIO pin to be an output and turn it off
  pinMode(CLOCK_SYNC, OUTPUT);
  digitalWrite(CLOCK_SYNC, LOW);

  // Start the serial port (UART) at 115200 bps
  Serial.begin(115200);
  delay(100);

  // Attempt to connnect to the WiFi network
  Serial.println();
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);

  // Loop until WiFi status is connected (WL_CONNECTED)  
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }
  
  // Got here, which means the WiFi got connected, so print out the details of the WiFi connection
  ledPeriod = LED_PERIOD_WIFI_OK_NO_NTPTIME;
  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
  Serial.println();

  Serial.print("ESP8266 Board MAC Address:  ");
  Serial.println(WiFi.macAddress());
  Serial.println();

  // Start the NTPClient
  timeClient.begin();

  // Setup main loop timing counters
  currentMsec  = millis();
  previousMsec = currentMsec;
}


//
// Main loop (period is about 500 msec)
//
void loop()
{
  static bool firstTime  = true;   // Indicates first time through the loop()
  static bool halfSecond = false;  // Indicates second time thru the loop for each second (since loop runs every 500 msec)

  wl_status_t wifiStatus = WiFi.status();

  // Update the NTP time (only if connected to WiFi).  By default a network update from the NTP Server is only made every 60 seconds.
  if (wifiStatus == WL_CONNECTED)
  {
    timeClient.update();
  }

  // Get the latest time
  gMinutes = timeClient.getMinutes();
  gSeconds = timeClient.getSeconds();

  // Once an hour, one second before the top of the hour, activate the self-winding clock synchronizer circuit to make sure clock is set properly
//  if ( (gMinutes == 59) && (gSeconds == 59) )
//  if ( (gMinutes == 00) && (gSeconds == 59) )
  if ( (gSeconds == 59) || (gSeconds == 29) )
  {
    if (digitalRead(CLOCK_SYNC) == LOW)
    {
      digitalWrite(CLOCK_SYNC, HIGH);   // Activate the clock synchronizer circuit (will get de-activated one second later)
      Serial.println("Sync pulse ON");
    }
  }

//  if ( (gMinutes == 0) && (gSeconds == 0) )
//  if ( (gMinutes == 01) && (gSeconds == 0) )
  if ( (gSeconds == 0) || (gSeconds == 30) )
  {
    if (digitalRead(CLOCK_SYNC) == HIGH)
    {
      digitalWrite(CLOCK_SYNC, LOW);    // De-activate the clock synchronizer circuit
      Serial.println("Sync pulse OFF");
    }
  }

  // Once a minute (and the first time here) print out the day of the week and the current time
  if ( (firstTime == true) || ( (gSeconds == 0) && (halfSecond == false) ) )
  {
    firstTime = false;
    Serial.print(daysOfTheWeek[timeClient.getDay()]);
    Serial.print(", ");
    Serial.println(timeClient.getFormattedTime());
  }

  // Update the blue LED period if necessary
  if (wifiStatus != WL_CONNECTED)
  {
    // WiFi not connected (and therefore also no NTP connection)
    ledPeriod = LED_PERIOD_NO_WIFI_NO_NTPTIME;
    if ( (gSeconds % 10) == 0 )
    {
      Serial.println("NO WIFI!!!");
    }
  }
  else if (!timeClient.isTimeSet())
  {
    // NTP time invalid
    ledPeriod = LED_PERIOD_WIFI_OK_NO_NTPTIME;
    if ( (gSeconds % 10) == 0 )
    {
      Serial.println("NO NTP TIME***");
    }
  }
  else
  {
    // WiFi and NTP time both good
    ledPeriod = LED_PERIOD_WIFI_OK_NTPITME_OK;
  }

  // Toggle the blue LED with a period based on the status of the WiFi connection and the NTP time status
  if (--ledCounter <= 0)
  {
    // Time to blink the LED
    toggleBlueLED();
    ledCounter = ledPeriod;   // Reset the blink counter
  }

  // Toggle the half second flag
  halfSecond = !halfSecond;

  // Wait until it has been 500 msec since the last time we got here
  bool doneWithLoop = false;
  while (!doneWithLoop)
  {
    currentMsec = millis();
    if (currentMsec - previousMsec >= MAIN_LOOP_INTERVAL)
    {
      previousMsec = currentMsec;
      doneWithLoop = true;
    }
  }
}


//
// Toggle the blue LED
//
void toggleBlueLED()
{
  // Toggle the blue LED
    if (blueLedOn == true)
    {
      digitalWrite(BLUE_LED, HIGH);   // Turn off the LED
      blueLedOn = false;
    }
    else
    {
      digitalWrite(BLUE_LED, LOW);    // Turn on the LED
      blueLedOn = true;
    }
}