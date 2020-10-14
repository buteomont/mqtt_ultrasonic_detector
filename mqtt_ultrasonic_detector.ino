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
 *  mindistance=<minimum presence distance>
 *  maxdistance=<maximum presence distance>
 *  sleepTime=<seconds to sleep between measurements> (set to zero for continuous readings)
 */
#define VERSION "20.10.14.2"  //remember to update this after every change! YY.MM.DD.REV
 
#include <PubSubClient.h> 
#include <ESP8266WiFi.h>
#include <EEPROM.h>
#include "mqtt_ultrasonic_detector.h"

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
  int minimumPresenceDistance=0;  // Item is present if distance is greater than this
  int maximumPresenceDistance=400;// and distance is less than this
  int sleepTime=10; //seconds to sleep between distance checks
  char mqttClientId[MQTT_CLIENTID_SIZE]=""; //will be the same across reboots
  } conf;

conf settings; //all settings in one struct makes it easier to store in EEPROM
boolean settingsAreValid=false;

String commandString = "";     // a String to hold incoming commands from serial
bool commandComplete = false;  // goes true when enter is pressed

int distance=0; //the last measurement
boolean itemPresent=false; //TRUE if some object is within range window
unsigned long doneTimestamp=0; //used to allow publishes to complete before sleeping

char* clientId = settings.mqttClientId;
  

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
    reconnect();  // connect to the MQTT broker
     
  // let's do this 
    distance=measure();
    itemPresent=distance>settings.minimumPresenceDistance 
                && distance<settings.maximumPresenceDistance;
    report();
    doneTimestamp=millis(); //this is to allow the publish to complete before sleeping
    }
  else
    {
    showSettings();
    }
  }

 
void loop()
  {
  checkForCommand(); // Check for input in case something needs to be changed to work
  if (settingsAreValid && settings.sleepTime==0) //if sleepTime is zero then don't sleep
    {
    reconnect();  // may need to reconnect to the MQTT broker
    distance=measure();
    itemPresent=distance>settings.minimumPresenceDistance 
                && distance<settings.maximumPresenceDistance;
    report();    
    } 
  else if (settingsAreValid                        //setup has been done and
          && millis()-doneTimestamp>PUBLISH_DELAY) //waited long enough for report to finish
    {
    Serial.print("Sleeping for ");
    Serial.print(settings.sleepTime);
    Serial.println(" seconds");
    ESP.deepSleep(settings.sleepTime*1000000);
    } 
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


void showSettings()
  {
  Serial.print("broker=<MQTT broker host name or address> (");
  Serial.print(settings.mqttBrokerAddress);
  Serial.println(")");
  Serial.print("port=<port number>   (");
  Serial.print(settings.mqttBrokerPort);
  Serial.println(")");
  Serial.print("topicRoot=<topic root> (");
  Serial.print(settings.mqttTopicRoot);
  Serial.println(")  Note: must end with \"/\"");  
  Serial.print("user=<mqtt user> (");
  Serial.print(settings.mqttUsername);
  Serial.println(")");
  Serial.print("pass=<mqtt password> (");
  Serial.print(settings.mqttPassword);
  Serial.println(")");
  Serial.print("ssid=<wifi ssid> (");
  Serial.print(settings.ssid);
  Serial.println(")");
  Serial.print("wifipass=<wifi password> (");
  Serial.print(settings.wifiPassword);
  Serial.println(")");
  Serial.print("mindistance=<minimum presence distance> (");
  Serial.print(settings.minimumPresenceDistance);
  Serial.println(")");
  Serial.print("maxdistance=<maximum presence distance> (");
  Serial.print(settings.maximumPresenceDistance);
  Serial.println(")");
  Serial.print("sleeptime=<seconds to sleep between measurements> (");
  Serial.print(settings.sleepTime);
  Serial.println(")");
  Serial.print("MQTT Client ID is ");
  Serial.println(settings.mqttClientId);
  Serial.println("\n*** Use \"factorydefaults=yes\" to reset all settings ***\n");
  }

/*
 * Reconnect to the MQTT broker
 */
void reconnect() 
  {
  // Loop until we're reconnected
  while (!mqttClient.connected()) 
    {
    Serial.print("Attempting MQTT connection...");
    
    // Attempt to connect
    if (mqttClient.connect(clientId,settings.mqttUsername,settings.mqttPassword))
      {
      Serial.println("connected to MQTT broker.");

      //resubscribe to the incoming message topic
      char topic[MQTT_TOPIC_SIZE];
      strcpy(topic,settings.mqttTopicRoot);
      strcat(topic,MQTT_TOPIC_COMMAND_REQUEST);
      mqttClient.subscribe(topic);
      }
    else 
      {
      Serial.print("failed, rc=");
      Serial.println(mqttClient.state());
      Serial.println("Will try again in a second");
      
      // Wait a second before retrying
      // In the meantime check for input in case something needs to be changed to make it work
      checkForCommand(); 
      
      delay(1000);
      }
    }
  }

  
/*
 * Check for configuration input via the serial port.  Return a null string 
 * if no input is available or return the complete line otherwise.
 */
String getConfigCommand()
  {
  if (commandComplete) 
    {
    Serial.println(commandString);
    String newCommand=commandString;

    commandString = "";
    commandComplete = false;
    return newCommand;
    }
  else return "";
  }

void processCommand(String cmd)
  {
  const char *str=cmd.c_str();
  char *val=NULL;
  char *nme=strtok((char *)str,"=");
  if (nme!=NULL)
    val=strtok(NULL,"=");

  //Get rid of the carriage return
  if (val!=NULL && strlen(val)>0 && val[strlen(val)-1]==13)
    val[strlen(val)-1]=0; 

  if (nme==NULL || val==NULL || strlen(nme)==0 || strlen(val)==0)
    {
    showSettings();
    
    return;  
    }
  else if (strcmp(nme,"broker")==0)
    {
    strcpy(settings.mqttBrokerAddress,val);
    saveSettings();
    }
  else if (strcmp(nme,"port")==0)
    {
    settings.mqttBrokerPort=atoi(val);
    saveSettings();
    }
  else if (strcmp(nme,"topicRoot")==0)
    {
    strcpy(settings.mqttTopicRoot,val);
    saveSettings();
    }
  else if (strcmp(nme,"user")==0)
    {
    strcpy(settings.mqttUsername,val);
    saveSettings();
    }
  else if (strcmp(nme,"pass")==0)
    {
    strcpy(settings.mqttPassword,val);
    saveSettings();
    }
  else if (strcmp(nme,"ssid")==0)
    {
    strcpy(settings.ssid,val);
    saveSettings();
    }
  else if (strcmp(nme,"wifipass")==0)
    {
    strcpy(settings.wifiPassword,val);
    saveSettings();
    }
  else if (strcmp(nme,"mindistance")==0)
    {
    settings.minimumPresenceDistance=atoi(val);
    saveSettings();
    }
  else if (strcmp(nme,"maxdistance")==0)
    {
    settings.maximumPresenceDistance=atoi(val);
    saveSettings();
    }
  else if (strcmp(nme,"sleeptime")==0)
    {
    settings.sleepTime=atoi(val);
    saveSettings();
    }
 else if ((strcmp(nme,"factorydefaults")==0) && (strcmp(val,"yes")==0)) //reset all eeprom settings
    {
    Serial.println("\n*********************** Resetting EEPROM Values ************************");
    initializeSettings();
    saveSettings();
    delay(2000);
    ESP.restart();
    }
  else
    showSettings();
  return;
  }

void initializeSettings()
  {
  settings.validConfig=0; 
  strcpy(settings.ssid,"");
  strcpy(settings.wifiPassword,"");
  strcpy(settings.mqttBrokerAddress,""); //default
  settings.mqttBrokerPort=1883;
  strcpy(settings.mqttUsername,"");
  strcpy(settings.mqttPassword,"");
  strcpy(settings.mqttTopicRoot,"");
  settings.minimumPresenceDistance=0;
  settings.maximumPresenceDistance=400;
  settings.sleepTime=10;
  strcpy(settings.mqttClientId,strcat("UltrasonicDetector",String(random(0xffff), HEX).c_str()));
  }

void checkForCommand()
  {
  if (Serial.available())
    {
    serialEvent();
    String cmd=getConfigCommand();
    if (cmd.length()>0)
      {
      processCommand(cmd);
      }
    }
  }


/************************
 * Do the MQTT thing
 ************************/
void report()
  {  
  char topic[MQTT_TOPIC_SIZE];
  char reading[18];
  boolean success=false;

  //publish the distance measurement
  strcpy(topic,settings.mqttTopicRoot);
  strcat(topic,MQTT_TOPIC_DISTANCE);
  sprintf(reading,"%d",distance); 
  success=publish(topic,reading);
  if (!success)
    Serial.println("************ Failed publishing distance measurement!");

  //publish the object detection state
  strcpy(topic,settings.mqttTopicRoot);
  strcat(topic,MQTT_TOPIC_STATE);
  sprintf(reading,"%s",itemPresent?"YES":"NO"); //item within range window
  success=publish(topic,reading);
  if (!success)
    Serial.println("************ Failed publishing sensor state!");
  }

boolean publish(char* topic, char* reading)
  {
  Serial.print(topic);
  Serial.print(" ");
  Serial.println(reading);
  return mqttClient.publish(topic,reading);
  }

  
/*
*  Initialize the settings from eeprom and determine if they are valid
*/
void loadSettings()
  {
  EEPROM.get(0,settings);
  if (settings.validConfig==VALID_SETTINGS_FLAG)    //skip loading stuff if it's never been written
    {
    settingsAreValid=true;
    Serial.println("Loaded configuration values from EEPROM");
//    showSettings();
    }
  else
    {
    Serial.println("Skipping load from EEPROM, device not configured.");    
    settingsAreValid=false;
    }
  }

/*
 * Save the settings to EEPROM. Set the valid flag if everything is filled in.
 */
boolean saveSettings()
  {
  if (strlen(settings.ssid)>0 &&
    strlen(settings.wifiPassword)>0 &&
    strlen(settings.mqttBrokerAddress)>0 &&
    settings.mqttBrokerPort!=0 &&
    strlen(settings.mqttTopicRoot)>0 &&
    strlen(settings.mqttClientId)>0)
    {
    Serial.println("Settings deemed complete");
    settings.validConfig=VALID_SETTINGS_FLAG;
    settingsAreValid=true;
    }
  else
    {
    Serial.println("Settings still incomplete");
    settings.validConfig=0;
    settingsAreValid=false;
    }
    
  //The mqttClientId is not set by the user, but we need to make sure it's set  
  if (strlen(settings.mqttClientId)==0)
    {
    strcpy(settings.mqttClientId,strcat("UltrasonicDetector",String(random(0xffff), HEX).c_str()));
    Serial.println("Remember to remove the temporary code in the loadSettings() function");
    }
    
    
  EEPROM.put(0,settings);
  return EEPROM.commit();
  }

  
/*
  SerialEvent occurs whenever a new data comes in the hardware serial RX. This
  routine is run between each time loop() runs, so using delay inside loop can
  delay response. Multiple bytes of data may be available.
*/
void serialEvent() 
  {
  while (Serial.available()) 
    {
    // get the new byte
    char inChar = (char)Serial.read();

    // if the incoming character is a newline, set a flag so the main loop can
    // do something about it 
    if (inChar == '\n') 
      {
      commandComplete = true;
      }
    else
      {
      // add it to the inputString 
      commandString += inChar;
      }
    }
  }
