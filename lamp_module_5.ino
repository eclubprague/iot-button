/*
 * ESP8266 wireless lamp module
 *  - connects to predefined WiFi and MQTT server
 *  - subscribes to the ESP8266/BUTTON topic and reads desired power level
 *  - sends heartbeat every 5 seconds containing unique chip ID and current power level on ESP8266/LAMP topic
 *  - sets illumination level via PWM
 */

#include <ESP8266WiFi.h> // WiFi
#include <PubSubClient.h> // MQTT
#include <ArduinoJson.h> // JSON objects
#include <Ticker.h> // timer

// WiFi and MQTT values and constants
const char* ssid = "Turris";
const char* password = "zettor11"; 
const char* mqtt_server = "192.168.1.163";
const uint16_t port = 41235; 

// JSON objects values and constants
const int8_t msgSize = 24; // ESP8266/BUTTON instruction size
const int8_t msg2Size = 50; // ESP8266/LAMP heartbeat size
char msg[msgSize]; // for json object
char msg2[msg2Size]; 

// PWM output
uint8_t outPin = 13; // led diode or lamp

// power control 
uint16_t defaultPower = 60; 
int16_t volatile power = defaultPower; // desired light power
int16_t currentPower = defaultPower;
uint16_t ambLight;

// WiFi, MQTT and timer objects
WiFiClient espClient;
PubSubClient client(espClient);
Ticker heartbeatTimer;
Ticker adjustTimer;

/*
 * Sets up wifi, mqtt, pwm.
 */
void setup() {
  Serial.begin(115200);
  Serial.println("Lamp module running...");

  setup_wifi();
  
  client.setServer(mqtt_server, port);
  client.setCallback(callback);
  
  pinMode(outPin, OUTPUT);
  analogWrite(outPin, defaultPower);  

  heartbeatTimer.attach(5, heartbeat);
  adjustTimer.attach(0.004, adjustPower);

  reconnect();  
}

/*
 * Sets up wifi connection.
 */
void setup_wifi() {   
  
  delay(10);
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

/*
 * Client callback function. Called when new data arrives. 
 * Reads data from JSON and sets pwm for light brightness.
 */
void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message on topic [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < length; i++) {
    //Serial.print((char)payload[i]);
    msg[i] = (char)payload[i];
  }
  Serial.println(msg);

  StaticJsonBuffer<msgSize> jsonBuffer; // reserve space
  JsonObject& root = jsonBuffer.parseObject(msg); // deserialize
  
  if (!root.success()) {
  Serial.println("parseObject() failed");
  }

  power = root["power"];
  /*
  proximity = root["proximity"];
  ambLight = root["ambLight"];
  Serial.print("Power: ");
  Serial.println(power);
  Serial.print("Proximity: ");
  Serial.println(proximity);
  Serial.print("Ambient light: ");
  Serial.println(ambLight);  
  */
}

/*
 * MQTT reconnecting.
 */
void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect("ESP_lamp_module")) {
      Serial.println("connected");
      // ... and resubscribe
      client.subscribe("ESP8266/BUTTON", 1); // QoS 1 = at least once
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

/*
 * Obligatory loop, used for connection maintenance
 */
void loop() {
  if (!client.connected()) {
    reconnect();
  }
  
  client.loop(); // mqtt reload
}

/*
 * Adjusting power, controled by timer
 */
void adjustPower() {
  if (currentPower < power) {
    currentPower++;  
  } else if (currentPower > power) {
    currentPower--;
  }
  /* 
  Serial.print("Time: ");
  Serial.println(millis());
  Serial.print("Current power: ");
  Serial.println(currentPower);
  */
  analogWrite(outPin, currentPower);  
  
  }

  /*
   * ESP lamp module heartbeat, sending power info and module unique ID.
   */
  void heartbeat() {
    StaticJsonBuffer<msg2Size> jsonBuffer; // Reserved memory space for json object
    JsonObject& root = jsonBuffer.createObject();    

    root["power"] = currentPower;
    root["chipID"] = ESP.getChipId();

    root.printTo(msg2, sizeof(msg2)); // Generate json string
    
    Serial.print("Sending heartbeat on topic [ESP8266/LAMP] ");
    Serial.println(msg2);
    client.publish("ESP8266/LAMP", msg2);
  }
