/*
 * ESP8266 wireless sensor and control module
 *  - connects to predefined WiFi network
 *  - connects to MQTT server
 *  - publishes on the ESP8266/BUTTON topic the values of: desired light power
 *  - controls the on/off state and desired power using round capacitive buttons
 *  - controls desired power using capacitive slider
 *  - holds constant illumination level using proximity sensor
 *  - publish occurs periodically every 15s or when desired power level changes 
 */

#include <ESP8266WiFi.h> // WiFi
#include <PubSubClient.h> // MQTT
#include <ArduinoJson.h> // JSON objects
#include <Adafruit_VCNL4010.h> // proximity, ambient light
#include <Wire.h> //  I2C
#include <Ticker.h> // timer

// WiFi and MQTT parameters
const char* ssid = "Turris"; 
const char* password = "zettor11"; 
const char* mqtt_server = "192.168.1.163";
const int16_t port = 41235;

// I2C settings for capacitive boards
const uint8_t address = 40; // 7-bit address of the device B0101000
const uint8_t quantity = 1; // number of bytes to request

// Capacitive board data structures and constants
int16_t touchData[8]; // storing capacitive buttons data
const uint8_t sensitivity = 8; // arbitrary - no of steps per 1 slider button
const uint8_t threshold = 20; // absolute threshold indicating pressed button

// VCNL 4010 (ambient and proximity) data structures and settings
uint16_t proximity; // proximity value
uint16_t ambLight; // ambient light value
const uint16_t proximityThreshold = 2700; // threshold indicating user presence

// Control logic values and constants
unsigned long toggleTime = 0; // the last time the light was toggled
const uint16_t debounce = 500; // the debounce time
boolean lightOn = true; // current status
boolean toSend = false; // indicates change in parameters to be send
boolean alreadyPressed = false; // indicates change in on/off button
long somethingPressed; // time when something was pressed

const int16_t defaultPower = 60; // used when turned on
int16_t power = defaultPower; // light power
int16_t defDesiredIllumination; // default desired illumination, determined when turned on from the default power
int32_t desiredIllumination; // for determining change in illumination
const uint16_t maxPower = 600; // arbitrary value to prevent flickering
const uint16_t timeLimit = 2000; // waiting time for adjusting desired ambLight

// JSON structure values and constants (to be sent via MQTT)
const int8_t msgSize = 24; // JSON minimal message size
char msg[msgSize]; // JSON structure

// Constants for control sensitivity tuning
const uint16_t tolerance = 300; // tolerance for ambient adjusting
const uint16_t powerStep = 5; // power step for adjusting according to ambLight
const uint8_t stepSize = 5; // power step size for fading with buttons
const uint8_t sliderStepSize = 10; // power step size for fading with slider

int16_t previousSliderPosition = -1; // last position on slider

WiFiClient espClient; // WiFi object
PubSubClient client(espClient); // MQTT ebject

Adafruit_VCNL4010 vcnl; // ambient and proximity sensor object

// timer objects
Ticker readTimer;
Ticker periodicTimer; 

/*
 * Main periodic function. 
 * By default, it only adjust power according to ambient light.
 * When user presence is detected, it evaluates the capacitive buttons values and controls power accordingly.
 */
void tick() {
  ambLight = vcnl.readAmbient();
  proximity = vcnl.readProximity();
  // Serial.println(proximity);
  
  if (proximity >= proximityThreshold) {
    Serial.println("Presence detected...");
    readTouchData();
    evaluateTouchButtons();
    evaluateSlider();
  }

  if (millis() - somethingPressed < timeLimit) { // for 1 second, setting the desired illumination
    desiredIllumination = ambLight;  
  } else {  // after longer period, adjust power according to current illumination
    adjustPower();
  }

  /*
  Serial.print("Ambient: ");
  Serial.println(ambLight);
  Serial.print("DesiredAmb: ");
  Serial.println(desiredIllumination);  
  Serial.print("Power: ");
  Serial.println(power);
  Serial.print("Proximity: ");
  Serial.println(proximity);
  */
  
  if (toSend) {
    sendMessage();   
  }
}

/*
 *  Reads all capacitive button values and stores in global array touchData[].
 */
void readTouchData() {
  touchData[0] = readFromTouch(0x11);
  touchData[1] = readFromTouch(0x12);
  touchData[2] = readFromTouch(0x13);
  touchData[3] = readFromTouch(0x14);
  touchData[4] = readFromTouch(0x15);
  touchData[5] = readFromTouch(0x16); // on/off
  touchData[6] = readFromTouch(0x17); // fading -
  touchData[7] = readFromTouch(0x10); // fading +  
}

/*
 * Reading from capacitive board through I2C with specific address.
 */
int16_t readFromTouch(uint8_t regAddress) {
  Wire.beginTransmission(address); // CAP address
  Wire.write(regAddress); // set pointer to register with this address
  Wire.requestFrom(address, quantity);
  int16_t touchValue = Wire.read();
  Wire.endTransmission();
  if (touchValue > 127) {
    return(touchValue - 256);  
  } else {
    return(touchValue);  
  }    
}

/*
 * Check touch button values for on/off and fading
 */
void evaluateTouchButtons() {
  if(touchData[5] > threshold || touchData[6] > threshold || touchData[7] > threshold) {
    somethingPressed = millis();  
    desiredIllumination = ambLight;
  }
  
  if(touchData[7] > threshold) { // touched after more than debounce
    if (millis()-toggleTime > debounce && alreadyPressed == false) {
      if (!lightOn) { // turning on
        if (power == 0) {
          power = defaultPower;
        }
      }
      lightOn = !lightOn;
      toggleTime = millis();
      toSend = true;
    }
    alreadyPressed = true;
  } else {
      alreadyPressed = false;
  }

  if(touchData[6] > threshold && lightOn) { // button 7 fading -
      if (power-stepSize >= 0) {
        power -= stepSize;
        toSend = true;
      }
      if (power == 0) {
        lightOn = false;  
      }
  }

  if(touchData[5] > threshold) { // button 6 fading +
      if (lightOn && power + stepSize <=maxPower) {
        power += stepSize;
        toSend = true;
      } else if (!lightOn) {
        lightOn = true;
        power = 0;
        power += stepSize;
      }
  }    
}

/*
 * Adjust power according to current illumination
 */
void adjustPower() {
  if (desiredIllumination < ambLight && abs(ambLight - desiredIllumination) > tolerance && lightOn) {
    power -= powerStep;
    toSend = true;
    if (power <= 0) {
      power = 0;  
    }
  } else if (desiredIllumination > ambLight && abs(ambLight - desiredIllumination)> tolerance && lightOn) {
    power += powerStep;  
    toSend = true;
    if (power >= maxPower) {
      power = maxPower;  
    }
  }
}

/*
 * Filter slider position from raw slider data.
 */
void evaluateSlider() {
  
  int16_t sliderPosition = getSliderPosition();
  if(sliderPosition != -1) {
    somethingPressed = millis();  
    desiredIllumination = ambLight;
  }
  if (previousSliderPosition != -1 && sliderPosition != -1 && previousSliderPosition != sliderPosition) {
    power = power + (sliderPosition - previousSliderPosition) * sliderStepSize;
    if (power <= 0) {
      power = 0;
      lightOn = false;  
    } else if (power > maxPower) {
      power = maxPower;  
    } else if (power > 0 && !lightOn) {
      power = sliderStepSize;
      lightOn = true;  
    }
    toSend = true;
  }
  previousSliderPosition = sliderPosition;
}

/*
 * Returns slider position according to global data in touchData[].
 * If no presence is detected, returns -1.
 */
int16_t getSliderPosition() {
  // find maxValue and maxIndex in slider touch data
  uint16_t maxValue = 0;
  byte maxIndex = 0;
  for(int i = 0; i < 5; i++) {
    if (touchData[i] >= maxValue && touchData[i] > threshold) {
        maxValue = touchData[i];
        maxIndex = i;
    }
  }
  
  int16_t sliderPosition = -1;
  if (maxValue > threshold && maxIndex != 4) { // determine slider position
    sliderPosition = (maxIndex * sensitivity) + (sensitivity * touchData[maxIndex + 1])/maxValue; 
  } else if (maxValue > threshold && maxIndex == 4) {
    sliderPosition = maxIndex * sensitivity;
  }    
  return sliderPosition;
}

/*
 * Sending a mqtt message from global values.
 */
void sendMessage() {
  StaticJsonBuffer<msgSize> jsonBuffer; // Reserved memory space for json object
  JsonObject& root = jsonBuffer.createObject();
     
  if(!lightOn) {
    root["power"] = 0;
  } else {
    root["power"] = power; 
  }
  root.printTo(msg, sizeof(msg)); // Generate json string
    
  Serial.print("Publish message: ");
  Serial.println(msg);
  client.publish("ESP8266/BUTTON", msg);

  toSend = false; // data sent
}

/*
 * Reestablishing mqtt connection, when fails.
 */
void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {

    // int32_t startTime = millis(); 
    
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect("ESP_sensor_module")) {
      Serial.println("connected");

      // Serial.print("MQQT starting time: "); 
      // Serial.println(millis()-startTime); 
      
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void setup() { 
  Serial.begin(9600);
  Serial.println("Sensor module running");

  // connect to wifi
  setup_wifi();
  client.setServer(mqtt_server, port);

  // start I2C communication with VCNL4010
  Serial.print("Found VCNL4010: ");
  Serial.println(vcnl.begin());
  
  // connect to mqtt
  reconnect();

  sendMessage();
  delay(1000); // increase for ambient sensor adjusting?
  ambLight = vcnl.readAmbient();
  defDesiredIllumination = ambLight - (ambLight % 100);
  Serial.print("Default power: ");
  Serial.println(power);
  Serial.print("Default desired Illumination: ");
  Serial.println(defDesiredIllumination);
  desiredIllumination = defDesiredIllumination;

  // set timer for data reading
  readTimer.attach(0.125, tick);  
  periodicTimer.attach(15, periodicPublish);
}

/*
 * Sets up WiFi connection.
 */
void setup_wifi() {
  delay(10);
  Serial.print("Connecting to ");
  Serial.println(ssid);

  // int32_t startTime = millis();

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  // Serial.print("WiFi start time: ");
  // Serial.println(millis() - startTime);

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

/*
 * Publish every 5 seconds in case that mqtt failed or new client connected.
 */
void periodicPublish() {
    sendMessage();
    reconnect();
    }

/*
 * Obligatory loop.
 */
void loop() {
}
