/**
 * This is an ESP8266 program to measure distance with an HC-SR04
 * ulrasonic ranger, and report over MQTT whether or not some object
 * is within a specified distance window.  It utilizes the ESP8266's  
 * sleep mode to maximize battery life.  It will wake up at least
 * once per hour to let you know it's still alive.
 *
 * Configuration is done via serial connection.  Enter:
 *  broker=<broker name or address>
 *  port=<port number>   (defaults to 1883)
 *  topicroot=<topic root> (something like buteomont/water/pressure/ - must end with / and 
 *  "state" or "period" will be added)
 *  user=<mqtt user>
 *  pass=<mqtt password>
 *  ssid=<wifi ssid>
 *  wifipass=<wifi password>
 *  mindistance=<minimum presence distance>
 *  maxdistance=<maximum presence distance>
 *  sleepTime=<seconds to sleep between measurements> (set to zero for continuous readings)
 */
#define VERSION "20.11.11.2"  //remember to update this after every change! YY.MM.DD.REV
 
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
  int minimumPresenceDistance=0;  // Item is present if distance is greater than this
  int maximumPresenceDistance=400;// and distance is less than this
  int sleepTime=10; //seconds to sleep between distance checks
  char mqttClientId[MQTT_CLIENTID_SIZE]=""; //will be the same across reboots
  bool debug=false;
  } conf;

conf settings; //all settings in one struct makes it easier to store in EEPROM
boolean settingsAreValid=false;

String commandString = "";     // a String to hold incoming commands from serial
bool commandComplete = false;  // goes true when enter is pressed

int distance=0; //the last measurement; this goes to EEPROM after each measurement
boolean itemPresent=false; //TRUE if some object is within range window
unsigned long doneTimestamp=0; //used to allow publishes to complete before sleeping

//The distance measurement is stored in EEPROM right after the settings
const unsigned int measurementLocation=sizeof(settings); 

//We should report at least once per hour, whether we have a package or not.  This
//will also let us retrieve any outstanding MQTT messages.  Since the internal millis()
//counter is reset every time it wakes up, we need to save it before sleeping and restore
//it when waking up. To keep from killing our flash memory, we'll store it in the RTC
//memory, which is kept alive by the battery or power supply.
typedef struct
  {
  long nextHealthReportTime=0; 
  unsigned long rtc=0;
  } MY_RTC;
  
MY_RTC myRtc;

ADC_MODE(ADC_VCC); //so we can use the ADC to measure the battery voltage

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

  system_rtc_mem_read(64, &myRtc, sizeof(myRtc)); //read the last saved timestamp before we slept
  
  EEPROM.begin(sizeof(settings)+sizeof(distance)); //fire up the eeprom section of flash
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
//    if (settings.debug)
//      {
//      Serial.print("Settings object size=");
//      Serial.println(sizeof(settings));
//      Serial.print("Read RTC: ");
//      Serial.println(myRtc.rtc);
//      Serial.print("Read nextHealthReportTime: ");
//      Serial.println(myRtc.nextHealthReportTime);
//      }
      
    //Get a measurement and compare it with the last one stored in EEPROM.
    //If they are the same, no need to phone home. Unless an hour has passed since
    //the last time home was phoned. 
    Serial.print("Measured distance: ");
    distance=measure(); 

    int changepct=saveMeasurement(distance); //returns true if new value is different from last one
    
    Serial.print(distance);
    Serial.print(" cm (");
    Serial.print(changepct);
    Serial.println("% changed)");

    Serial.print("Battery voltage: ");
    Serial.println(convertToVoltage(readBattery()));
    
    if (changepct>MAX_CHANGE_PCT || myMillis()>myRtc.nextHealthReportTime)
      {
      // ********************* attempt to connect to Wifi network
      connectToWiFi();
        
      // ********************* Initialize the MQTT connection
      reconnect();  // connect to the MQTT broker
       
      // let's do this 
      itemPresent=distance>settings.minimumPresenceDistance 
                  && distance<settings.maximumPresenceDistance;
      report();
      doneTimestamp=millis(); //this is to allow the publish to complete before sleeping
      myRtc.nextHealthReportTime=myMillis()+ONE_HOUR;
      }
    }
  else
    {
    showSettings();
    }
  }

/*
 * If not connected to wifi, connect.
 */
void connectToWiFi()
  {
  if (WiFi.status() != WL_CONNECTED)
    {
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
    }
  }

/**
 * Handler for incoming MQTT messages.  The payload is the command to perform. 
 * The MQTT message topic sent is the topic root plus the command.
 * Implemented commands are: 
 * MQTT_PAYLOAD_SETTINGS_COMMAND: sends a JSON payload of all user-specified settings
 * MQTT_PAYLOAD_REBOOT_COMMAND: Reboot the controller
 * MQTT_PAYLOAD_VERSION_COMMAND Show the version number
 * MQTT_PAYLOAD_STATUS_COMMAND Show the most recent flow values
 */
void incomingMqttHandler(char* reqTopic, byte* payload, unsigned int length) 
  {
  if (settings.debug)
    {
    Serial.println("====================================> Callback works.");
    }
  payload[length]='\0'; //this should have been done in the caller code, shouldn't have to do it here
  boolean rebootScheduled=false; //so we can reboot after sending the reboot response
  char charbuf[100];
  sprintf(charbuf,"%s",payload);
  char* response;
  
  
  //if the command is MQTT_PAYLOAD_SETTINGS_COMMAND, send all of the settings
  if (strcmp(charbuf,MQTT_PAYLOAD_SETTINGS_COMMAND)==0)
    {
    char tempbuf[35]; //for converting numbers to strings
    char jsonStatus[JSON_STATUS_SIZE];
    
    strcpy(jsonStatus,"{");
    strcat(jsonStatus,"\"broker\":\"");
    strcat(jsonStatus,settings.mqttBrokerAddress);
    strcat(jsonStatus,"\", \"port\":");
    sprintf(tempbuf,"%d",settings.mqttBrokerPort);
    strcat(jsonStatus,tempbuf);
    strcat(jsonStatus,", \"topicroot\":\"");
    strcat(jsonStatus,settings.mqttTopicRoot);
    strcat(jsonStatus,"\", \"user\":\"");
    strcat(jsonStatus,settings.mqttUsername);
    strcat(jsonStatus,"\", \"pass\":\"");
    strcat(jsonStatus,settings.mqttPassword);
    strcat(jsonStatus,"\", \"ssid\":\"");
    strcat(jsonStatus,settings.ssid);
    strcat(jsonStatus,"\", \"wifipass\":\"");
    strcat(jsonStatus,settings.wifiPassword);
    strcat(jsonStatus,"\", \"minimumPresenceDistance\":\"");
    sprintf(tempbuf,"%d",settings.minimumPresenceDistance);
    strcat(jsonStatus,tempbuf);
    strcat(jsonStatus,"\", \"maximumPresenceDistance\":\"");
    sprintf(tempbuf,"%d",settings.maximumPresenceDistance);
    strcat(jsonStatus,tempbuf);
    strcat(jsonStatus,"\", \"sleepTime\":\"");
    sprintf(tempbuf,"%d",settings.sleepTime);
    strcat(jsonStatus,tempbuf);
    strcat(jsonStatus,"\", \"mqttClientId\":\"");
    strcat(jsonStatus,settings.mqttClientId);
    
    strcat(jsonStatus,"\"}");
    response=jsonStatus;
    }
  else if (strcmp(charbuf,MQTT_PAYLOAD_VERSION_COMMAND)==0) //show the version number
    {
    char tmp[15];
    strcpy(tmp,VERSION);
    response=tmp;
    }
  else if (strcmp(charbuf,MQTT_PAYLOAD_STATUS_COMMAND)==0) //show the latest value
    {
    report();
    
    char tmp[25];
    strcpy(tmp,"Status report complete");
    response=tmp;
    }
  else if (strcmp(charbuf,MQTT_PAYLOAD_REBOOT_COMMAND)==0) //reboot the controller
    {
    char tmp[10];
    strcpy(tmp,"REBOOTING");
    response=tmp;
    rebootScheduled=true;
    }
  else if (processCommand(charbuf))
    {
    response="OK";
    }
  else
    {
    char badCmd[18];
    strcpy(badCmd,"(empty)");
    response=badCmd;
    }
    
  char topic[MQTT_TOPIC_SIZE];
  strcpy(topic,settings.mqttTopicRoot);
  strcat(topic,charbuf); //the incoming command becomes the topic suffix

  if (!publish(topic,response))
    Serial.println("************ Failure when publishing status response!");
  
  if (rebootScheduled)
    {
    delay(2000); //give publish time to complete
    ESP.restart();
    }
  }
 
void loop()
  {
  mqttClient.loop(); //This has to happen every so often or we get disconnected for some reason
  checkForCommand(); // Check for input in case something needs to be changed to work
  if (settingsAreValid && settings.sleepTime==0) //if sleepTime is zero then don't sleep
    {
    connectToWiFi(); //may need to connect to the wifi
    reconnect();  // may need to reconnect to the MQTT broker
    distance=measure();
    itemPresent=distance>settings.minimumPresenceDistance 
                && distance<settings.maximumPresenceDistance;
    report();    
    } 
  else if (settingsAreValid                        //setup has been done and
          && millis()-doneTimestamp>PUBLISH_DELAY) //waited long enough for report to finish
    {
    if (settings.debug)
      {
      Serial.print("Next report in ");
      Serial.print((myRtc.nextHealthReportTime-myMillis())/1000);
      Serial.println(" seconds.");
      }
    Serial.print("Sleeping for ");
    Serial.print(settings.sleepTime);
    Serial.println(" seconds");

    //save the wakeup time so we can keep track of time across sleeps
    myRtc.rtc=myMillis()+settings.sleepTime*1000;
    system_rtc_mem_write(64, &myRtc, sizeof(myRtc)); //save the timing before we sleep
    
      
    ESP.deepSleep(settings.sleepTime*1000000);
    } 
  }

/*
 * This returns the elapsed milliseconds, even if we've been sleeping
 */
unsigned long myMillis()
  {
  return millis()+myRtc.rtc;
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
  Serial.print("topicroot=<topic root> (");
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
  Serial.print("debug=1|0 (");
  Serial.print(settings.debug);
  Serial.println(")");
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

    mqttClient.setBufferSize(JSON_STATUS_SIZE); //default (256) isn't big enough
    mqttClient.setServer(settings.mqttBrokerAddress, settings.mqttBrokerPort);
    mqttClient.setCallback(incomingMqttHandler);
    
    // Attempt to connect
    if (mqttClient.connect(settings.mqttClientId,settings.mqttUsername,settings.mqttPassword))
      {
      Serial.println("connected to MQTT broker.");

      //resubscribe to the incoming message topic
      char topic[MQTT_TOPIC_SIZE];
      strcpy(topic,settings.mqttTopicRoot);
      strcat(topic,MQTT_TOPIC_COMMAND_REQUEST);
      bool subgood=mqttClient.subscribe(topic);
      showSub(topic,subgood);
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
  mqttClient.loop(); //This has to happen every so often or we get disconnected for some reason
  }

void showSub(char* topic, bool subgood)
  {
  if (settings.debug)
    {
    Serial.print("++++++Subscribing to ");
    Serial.print(topic);
    Serial.print(":");
    Serial.println(subgood);
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

bool processCommand(String cmd)
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
    return false;   //not a valid command, or it's missing
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
  else if (strcmp(nme,"topicroot")==0)
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
  else if (strcmp(nme,"debug")==0)
    {
    settings.debug=atoi(val)==1?true:false;
    saveSettings();
    }
  else if ((strcmp(nme,"resetmqttid")==0)&& (strcmp(val,"yes")==0))
    {
    generateMqttClientId();
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
    {
    showSettings();
    return false; //command not found
    }
  return true;
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
  strcpy(settings.mqttClientId,generateMqttClientId());
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

int readBattery()
  {
  int raw=ESP.getVcc(); //This commandeers the ADC port
  return raw;
  }

float convertToVoltage(int raw)
  {
  int vcc=map(raw,0,FULL_BATTERY,0,330);
  return ((float)vcc)/100.0;
  }


/************************
 * Do the MQTT thing
 ************************/
void report()
  {  
  char topic[MQTT_TOPIC_SIZE];
  char reading[18];
  boolean success=false;

  //publish the battery voltage
  strcpy(topic,settings.mqttTopicRoot);
  strcat(topic,MQTT_TOPIC_BATTERY);
  sprintf(reading,"%.2f",convertToVoltage(readBattery())); 
  success=publish(topic,reading);
  if (!success)
    Serial.println("************ Failed publishing battery voltage!");

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
  return mqttClient.publish(topic,reading,true); //retained
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
    if (settings.debug)
      {
      Serial.println("Loaded configuration values from EEPROM");
//      showSettings();
      }
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
    strcpy(settings.mqttClientId,generateMqttClientId());
    }
    
    
  EEPROM.put(0,settings);
  return EEPROM.commit();
  }

/* saves the measurement and returns the 
 * absolute value of the change in percent.
 */
int saveMeasurement(int dist)
  {
  int reading=0;
  EEPROM.get(measurementLocation,reading);//read the last measurement
  if (reading!=dist) //if not changed, don't do anything
    {
    EEPROM.put(measurementLocation,dist);
    EEPROM.commit();
    }
  int change=(1-(float)dist/(float)reading)*100;
  return abs(change);  
  }

//Generate an MQTT client ID.  This should not be necessary very often
char* generateMqttClientId()
  {
  char tmpbuf[MQTT_CLIENTID_SIZE]="";
  strcpy(tmpbuf,strcat(MQTT_CLIENT_ID_ROOT,String(random(0xffff), HEX).c_str()));
  if (settings.debug)
    {
    Serial.print("New MQTT userid is ");
    Serial.println(tmpbuf);
    }
  return tmpbuf;
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
