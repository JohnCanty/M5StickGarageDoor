/*
*******************************************************************************
* Value Readings:
*       = 0  : Do not connect to WiFi
*       < 100: Connect to WiFi to close door
*      >= 100: Connect to WiFi to open door
* 
*
* 
* 
*******************************************************************************
*/
#include <M5StickCPlus.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <time.h>
#include <Preferences.h>
#include <TinyGPSPlus.h>
#include <SoftwareSerial.h>



WiFiClient espClient;
PubSubClient client(espClient);
Preferences preferences;
// The TinyGPSPlus object
TinyGPSPlus gps;

// Configure the name and password of the connected wifi and your NTP, MQTT Serve host.
const char* ssid = "SSID";
const char* password = "PASSWORD";
const char* mqtt_server = "IP OF MQTT Server";
const char* ntpServer = "IP OF NTP Server"; //Set the connect NTP server. 
const long  gmtOffset_sec = -28800;
const int   daylightOffset_sec = 3600;
constexpr float speedSP = 10.0;
constexpr float sp = 4.0;
RTC_TimeTypeDef RTC_TimeStruct;
RTC_DateTypeDef RTC_DateStruct;
// For stats that happen every 5 seconds
constexpr int RXPin = 33, TXPin = 32;
constexpr int Relay1 = 26, Relay2 = 25;
constexpr uint32_t GPSBaud = 9600;
unsigned long last = 0UL;
unsigned long lastMsg = 0;
constexpr uint16_t MSG_BUFFER_SIZE = 50;
char date[MSG_BUFFER_SIZE];
char tyme[MSG_BUFFER_SIZE];
int value = 0;
int count = 0;
int distanceCount = 0;
float accX = 0;
float accY = 0;
float accZ = 0;
float accXh = 0;
float accYh = 0;
float accZh = 0;
float currentSpeed = 0;
double lastDistance = 0; //if last distance - current distance > 0 getting closer to home
double currentDistance = 0; //if last distance - current distance < 0 Moving further away
bool speed;
bool doorOpen = false;
bool doorClos = false;

// The serial connection to the GPS device
SoftwareSerial ss(RXPin, TXPin);

void setup() 
{
  //Serial.begin(115200); //Serial monitor
  M5.begin(); //start the M5 device
  ss.begin(GPSBaud); //Connect to the GPS
  M5.Lcd.fillScreen(TFT_BLACK); //Clear the screen
  M5.Lcd.setRotation(3); //set screen rotation
  M5.Lcd.setTextFont(2);
  M5.Imu.Init();  //Init IMU.
  pinMode(Relay1, OUTPUT); // Set the two relay ports
  pinMode(Relay2, OUTPUT);
  preferences.begin("lastAction", false); //Setup write to RTC memory 
  WiFi.mode(WIFI_STA);  //Set the mode to WiFi station mode.
  WiFi.disconnect();
  //Serial.println("End of Setup");
  value = preferences.getUInt("counter", 0); //pull in the NVM and put it somewhere we can use it

  if ( value > 0 ) { setupWifi(); } // if the value is greater than 0 we need to connect to the wifi
}

void setupWifi() 
{
  //Serial.println("Wifi Setup");
  WiFi.begin(ssid, password); //Start Wifi connection. 
  while (WiFi.status() != WL_CONNECTED) //while the wifi isn't connected
  {
    delay(5000); //Wait 5 seconds
    M5.Lcd.print("Status Value: ");
    M5.Lcd.println(value);
    //Serial.print("Value Value: ");
    //Serial.println(value);
    M5.Lcd.print("Wifi Status: ");
    M5.Lcd.print(WiFi.status());
    count++;
    if (count == 2) { //If we have tried 2 times
      ESP.restart(); //Reboot
    }
  M5.Lcd.setTextColor(TFT_GREEN, TFT_BLACK);
  M5.Lcd.printf("\n****\n");
  M5.Lcd.print("Wifi Status: ");
  M5.Lcd.println(WiFi.status());
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer); //init and get the time.
  ntpGetLocalTime();
  delay(500);
  client.setServer(mqtt_server, 1883);  //Sets the server details.
  delay(500);
  client.setCallback(callback); //Sets the message callback function.
  }
}

void ntpGetLocalTime()//Output current time. 
{  
  struct tm timeinfo; 
  RTC_TimeTypeDef TimeStruct;
  RTC_DateTypeDef DateStruct;
  while(!getLocalTime(&timeinfo)) //Return 1 when the time is successfully obtained. 
  { 
    M5.Lcd.println("Failed to obtain time");
  }
  TimeStruct.Hours   = timeinfo.tm_hour;  //Set the time.
  TimeStruct.Minutes = timeinfo.tm_min;
  TimeStruct.Seconds = timeinfo.tm_sec;
  M5.Rtc.SetTime(&TimeStruct);  //writes the set time to the real time clock.
  DateStruct.Month = timeinfo.tm_mon + 1;
  DateStruct.Date = timeinfo.tm_mday;
  DateStruct.Year = timeinfo.tm_year + 1900;
  M5.Rtc.SetDate(&DateStruct);  //writes the set date to the real time clock.
}

void callback(char* topic, byte* payload, unsigned int length) {
  //for (int i = 0; i < length; i++) 
  //{
  //  Serial.print((char)payload[i]);
  //}
  //Serial.println();
  if ((char)payload[0] == 'o')
  {
    if ((char)payload[1] == 'p')
    {
      if ((char)payload[2] == 'e')
      {
        if ((char)payload[3] == 'n')
        {
          M5.Lcd.setTextColor(TFT_RED, TFT_BLACK);
          M5.Lcd.println("The door is open");
          doorOpen = true;
          doorClos = false;
        }
      }
    }
  }
  if ((char)payload[0] == 'c')
  {
    if ((char)payload[1] == 'l')
    {
      if ((char)payload[2] == 'o')
      {
        if ((char)payload[3] == 's')
        {
          M5.Lcd.setTextColor(TFT_BLUE, TFT_BLACK);
          M5.Lcd.println("The door is Closed");
          doorClos = true;
          doorOpen = false;
        }
      }
    }
  }
}



void reConnect() {
  while (!client.connected()) {
    M5.Lcd.setTextColor(TFT_YELLOW, TFT_BLACK);
    M5.Lcd.print("Attempting MQTT connection...");
    // Create a random client ID. 
    String clientId = "M5Stack-";
    clientId += String(random(0xffff), HEX);
    // Attempt to connect. 
    if (client.connect(clientId.c_str())) {
      M5.Lcd.setTextColor(TFT_GREEN, TFT_BLACK);
      M5.Lcd.printf("\nSuccess\n");
      // Once connected, publish an announcement to the topic.
      client.publish("Motorcycle/Vehicle/LWT", "Online");
      // ... and resubscribe. 
      client.subscribe("GarageDoor/Status");
    } else {
      delay(250);
    }
  }
}

double distanceToHome(double currentLAT, double currentLON) {
  static const double HOME_LAT = 30.0000, HOME_LON = -110.00000; //Random Lat Long used, place Home LAT Long here.
  return TinyGPSPlus::distanceBetween(currentLAT, currentLON, HOME_LAT, HOME_LON);
}

void openDoor() {
  if(!client.connected()){reConnect();}
  while(!doorOpen)
  {
    int i = 0;
    client.publish("Motorcycle/Vehicle/Arrived", "Home"); //was Home
    delay(500);
    client.publish("GarageDoor/CMND", "open"); //was open
    delay(1000);
    client.loop(); //We need to update the mqtt data to get out of the loop
    if ( i == 4 ) // If we tried 4 times, let's toggle the state of the door
    {
      client.publish("GarageDoor/CMND", "clos"); //toggle to closed
      delay(50);
    }
    i++;
  }
  M5.Lcd.fillScreen(TFT_BLACK);
  M5.Lcd.println("Door is open");
  resetCounter(10);
  delay(1000);
  M5.Axp.PowerOff();
}

void closeDoor() {
  if(!client.connected()){reConnect();}
  while(!doorClos)
  {
    int i = 0;
    client.publish("Motorcycle/Vehicle/Arrived", "Gone"); //was Gone
    delay(500);
    client.publish("GarageDoor/CMND", "clos"); //was clos
    delay(1000);
    client.loop(); //We need to update the mqtt data to get out of the loop
    if ( i == 4 )
    {
      client.publish("GarageDoor/CMND", "open"); //toggle to open
      delay(50);
    }
    i++;
  }
  M5.Lcd.fillScreen(TFT_BLACK);
  M5.Lcd.println("Door is closed");
  resetCounter(0);
  value = 0;
  doorClos = false;
  doorOpen = false;
  return;
}

void resetCounter( int x ) {
  preferences.putUInt("counter", x);
  preferences.end();
}

void loop() 
{
  M5.update(); //Read the press state of the key.
  unsigned long now = millis(); //Obtain the host startup duration.  
  while (ss.available() > 0) //GPS available?
  {
    if(gps.encode(ss.read())) //Encode the last read
    {
      //Serial.println("we got an encode");
      currentDistance = distanceToHome(gps.location.lat(), gps.location.lng()) / 1000.0;
      if (gps.speed.isUpdated())
      {
        //Serial.print(F("SPEED      Fix Age="));
        //Serial.print(gps.speed.age());
        //Serial.print(F(" MPH="));
        currentSpeed = gps.speed.mph();
        //Serial.println(currentSpeed);
      }
    }
  }
  if (M5.BtnB.wasReleased()) //If the button B is pressed.
  {  
    if (client.connected())
    {
      openDoor();
    }
    else
    {
      value = 100; // Set the reboot state for door opening
      resetCounter(value); // store this into the NVM
      ESP.restart(); //Reboot
    }
  }
  if (M5.BtnA.wasReleased()) //If the button A is pressed.
  {
    if (client.connected())
    {
      closeDoor();
    }
    else
    {
      value = 10; // Set the reboot state for door closing
      resetCounter(value); // store this into the NVM
      ESP.restart(); //Reboot
    }   
  }
  M5.Imu.getAccelData(&accX,&accY,&accZ);
  if ( value == 0 );
  {
    if (speed)
    {
      M5.Lcd.setTextColor(TFT_GREEN, TFT_BLACK);
    } else
    {
      M5.Lcd.setTextColor(TFT_RED, TFT_BLACK);
    }
    M5.Lcd.setTextFont(7);
    M5.Lcd.drawFloat(currentSpeed, 2, 70, 40);
  }
  if ( value > 0 ) 
  { 
    client.loop(); //This function is called periodically to allow clients to process incoming messages and maintain connections to the server.
    if(!client.connected())
    {
      reConnect();
    }
    else
    {
      M5.Rtc.GetTime(&RTC_TimeStruct);  //Gets the time in the real-time clock.
      M5.Rtc.GetDate(&RTC_DateStruct);  //Gets the date in the real-time clock.
      int year = RTC_DateStruct.Year;
      int month = RTC_DateStruct.Month;
      int day = RTC_DateStruct.Date;
      int hour = RTC_TimeStruct.Hours;
      int minute = RTC_TimeStruct.Minutes;
      int second = RTC_TimeStruct.Seconds;
      if (now - lastMsg > 2000) 
      {
        lastMsg = now;
        snprintf (date, MSG_BUFFER_SIZE, "%d%02d%02d", year, month, day); //Format and store it in date.
        snprintf (tyme, MSG_BUFFER_SIZE, "%02d%02d%02d", hour, minute, second); //Format and store it in tyme.
        char distc[10];
        char val[8];
        char distance[10];
        itoa(currentDistance, distance, 10);
        itoa(value, val, 10);
        itoa(distanceCount, distc, 10);
        client.publish("Motorcycle/Vehicle/Date", date);  //Publishes a message to the specified topic.
        client.publish("Motorcycle/Vehicle/Time", tyme);  //Publishes a message to the specified topic.
        client.publish("Motorcycle/Vehicle/distance", distance);  //Publishes a message to the specified topic.
        client.publish("Motorcycle/Vehicle/distanceCount", distc);  //Publishes a message to the specified topic.
        client.publish("Motorcycle/Vehicle/Value", val);  //Publishes a message to the specified topic.
      }
    }
  } 
  if (currentSpeed > speedSP)
  {
    digitalWrite(Relay1, LOW);
    speed = false;
  } 
  else 
  {
    digitalWrite(Relay1, HIGH);
    speed = true;
  }

  if(value >= 100) //This opens the door automatically if the value has something in it.
  {
      openDoor();
  }
  if (accX > accXh) {accXh = accX;}
  if (accY > accYh) {accYh = accY;}
  if (accZ > accZh) {accZh = accZ;}
  //M5.Lcd.printf("%.2f   %.2f   %.2f      ",accXh,accYh,accZh);
  if (accYh > sp || accXh > sp || accZh > sp) // If we see a shake
  {
    if(client.connected() && value >= 10 && value < 100) // and the value is lower than 100 but greater than 10 we are in close mode
    {
      closeDoor();
    }
    else if ( value < 10 ) // Value is likely 0 so in GPS only mode This gets us into close mode
    {
      value = 10; // Set the reboot state for door closing
      resetCounter(value); // store this into the NVM
      ESP.restart(); //Reboot
    }
    else // value must be greater than 100 meaning the door should be opening so no close operations
    {
      accXh = 0;
      accYh = 0;
      accZh = 0;
    }
  }

// Handle distance to the house before setting the mode

  if (lastDistance != currentDistance) // The distance has changed
  {
    if (lastDistance == 0.0) // The distance has changed and this is likely a first boot
    {
      lastDistance = currentDistance; // Just slug it in there
    }
    else if (lastDistance - currentDistance < 0) // We are moving further away
    {
      distanceCount = 0; // Reset the distance count, all closing must be consecutive
      lastDistance = currentDistance; // make current distance last distance
    }
    else if (lastDistance - currentDistance > 10 && currentDistance < 100.0) // We are getting closer
    {
      distanceCount++; // We now know we are getting closer, let's debounce this a bit
      lastDistance = currentDistance; // make current distance the last distance
    }
  }
  //Serial.println(value);
  if (currentDistance < 100.0 && distanceCount >= 10) // We got there, closing distance and 10 consecutive counts
  {
    value = 100; // Set the reboot state for door opening
    resetCounter(value); // store this into the NVM
    ESP.restart(); //Reboot
  }
}
