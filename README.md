# eClub - button
eClub Summer Camp 2016 - Capacitive User Interface. 

Sensor_module.ino contains source code for Capacitive User Interface - ESP8266 powered control device, consisting of ambient and proximity VCNL4010 sensor and custom made capacitive board with slider and buttons (based on CAP1208 chip).
It controls the light module using MQTT broker and implements simple capacitive slider filtering. Ambient light sensor is used for automatic illumination adjustment and proximity sensor to reduce power requirements.

Sensor Module pinout:

- I2C bus (to connect VCNL4010 and CAP1208 in series)

  - GPIO4 --> SCL
  
  - GPIO5 --> SDA
  
- GND should be common for all three components
  
- ESP8266 is only 3.3V tolerant!
  
Lamp_module.ino contains source code for custom made prototype of smart light - ESP8266 controling a 12V led band via PWM. It is controlled via MQTT.

Lamp Module pinout

- pwm output on GPIO13
