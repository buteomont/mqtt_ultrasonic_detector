/**
 * This is an ESP8266 program to measure distance with an HC-SR04
 * ulrasonic ranger, and report over MQTT whether or not some object
 * is within a specified distance window.  It utilizes the ESP8266's  
 * sleep mode to maximize battery life.
 *
 * Configuration is done via serial connection.  Enter:
 *  broker=<broker name or address>
 *  port=<port number>   (defaults to 1883)
 *  topicRoot=<topic root> (something like buteomont/water/pressure/ - must end with / and 
 *  "state" or "period" will be added)
 *  user=<mqtt user>
 *  pass=<mqtt password>
 *  ssid=<wifi ssid>
 *  wifipass=<wifi password>
 *  
 */
#define VERSION "20.10.13.1"  //remember to update this after every change! YY.MM.DD.REV
 
#include <PubSubClient.h> 
#include <ESP8266WiFi.h>
#include <EEPROM.h>
#include "mqtt_ultrasonic_detector.h"

//PubSubClient callback function header.  This must appear before the PubSubClient constructor.
void incomingMqttHandler(char* reqTopic, byte* payload, unsigned int length);

WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);

// These are the settings that get stored in EEPROM.  They are all in one struct which
// makes it easier to store and retrieve from EEPROM.
typedef struct 
  {
  unsigned int validConfig=0; 
  char ssid[SSID_SIZE] = "";
  char wifiPassword[PASSWORD_SIZE] = "";
  char mqttBrokerAddress[ADDRESS_SIZE]=""; //default
  int mqttBrokerPort=1883;
  char mqttUsername[USERNAME_SIZE]="";
  char mqttPassword[PASSWORD_SIZE]="";
  char mqttTopicRoot[MQTT_TOPIC_SIZE]="";
  } conf;

conf settings; //all settings in one struct makes it easier to store in EEPROM
boolean settingsAreValid=false;

String commandString = "";     // a String to hold incoming commands from serial
bool commandComplete = false;  // goes true when enter is pressed


void setup() 
  {
  pinMode(TRIG_PIN, OUTPUT);  // The trigger pin will tell the sensor to send a pulse
  digitalWrite(TRIG_PIN, LOW);// normally low
  pinMode(ECHO_PIN, INPUT);   // The echo pin is where the reflected pulse comes back
  pinMode(LED_BUILTIN,OUTPUT);// The blue light on the board shows activity

  Serial.begin(115200);
  Serial.setTimeout(10000);
  Serial.println();
  
  while (!Serial); // wait here for serial port to connect.

  EEPROM.begin(sizeof(settings)); //fire up the eeprom section of flash
  Serial.print("Settings object size=");
  Serial.println(sizeof(settings));
    
  commandString.reserve(200); // reserve 200 bytes of serial buffer space for incoming command string

  loadSettings(); //set the values from eeprom
  if (settings.mqttBrokerPort < 0) //then this must be the first powerup
    {
    Serial.println("\n*********************** Resetting All EEPROM Values ************************");
    initializeSettings();
    saveSettings();
    delay(2000);
    ESP.restart();
    }

  if (settingsAreValid)
    {
    // ********************* attempt to connect to Wifi network
    Serial.print("Attempting to connect to WPA SSID \"");
    Serial.print(settings.ssid);
    Serial.println("\"");
    
    WiFi.mode(WIFI_STA); //station mode, we are only a client in the wifi world
    WiFi.begin(settings.ssid, settings.wifiPassword);
    while (WiFi.status() != WL_CONNECTED) 
      {
      // not yet connected
      Serial.print(".");
      checkForCommand(); // Check for input in case something needs to be changed to work
      delay(500);
      }
  
    Serial.println("Connected to network.");
    Serial.println();

    // ********************* Initialize the MQTT connection
    mqttClient.setServer(settings.mqttBrokerAddress, settings.mqttBrokerPort);
    mqttClient.setCallback(incomingMqttHandler);
    
//    delay(2000);  //give wifi a chance to warm up
    reconnect();  // connect to the MQTT broker   
    }
    
  Serial.println("\nConfiguration is done via serial connection.  You can enter:\n");
  
  
  }

 
void loop()
  {
  int distance=measure();
  Serial.print("Nearest object in centimeters: ");
  Serial.println(distance);
  delay(5000);    
  }

// Read the distance 10 times and return the dominant value
int measure()
  {
  int vals[10];
  int answer,answerCount=0;

  //get 10 samples
  for (int i=0;i<10;i++)
    {
    // Turn on the LED to show activity
    digitalWrite(LED_BUILTIN,LED_ON);
    
    vals[i]=getDistance();

    // Turn off the LED
    digitalWrite(LED_BUILTIN,LED_OFF);
    
    delay(50); //give it some space
    }

//  for (int i=0;i<10;i++)
//    {
//    Serial.print(vals[i]);
//    Serial.print(" ");
//    }
//  Serial.println();


  //find the most common value within the sample set
  //This code is not very efficient but hey, it's only 10 values
  for (int i=0;i<9;i++) //using 9 here because the last one can only have a count of 1
    {
    int candidate=vals[i];
    int candidateCount=1;  
    for (int j=i+1;j<10;j++)
      {
      if (candidate==vals[j])
        {
        candidateCount++;
        }
      }
    if (candidateCount>answerCount)
      {
      answer=candidate;
      answerCount=candidateCount;
      }
    }
  return answer;
  }

//Take a measurement
int getDistance()
  {  
  // Trigger the HC-SR04 to send out a pulse by giving it a 10us kick
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  //measure the duration of the echo pulse to see how far away the nearest thing is
  long duration = pulseIn(ECHO_PIN, HIGH);
  
  //Calculate the distance (in cm) based on the speed of sound.  This is nominally at sea level
  return duration/58.2;
  }
