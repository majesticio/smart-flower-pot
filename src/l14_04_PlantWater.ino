/*
 * Project l14_04_PlantWater
 * Description: IoT Plant Watering System
 * Author: Gabriel Fosse
 * Date: 4/19/21
 */

SYSTEM_MODE(SEMI_AUTOMATIC);

#include"Grove_Air_quality_Sensor.h"
#include"Arduino.h"
#include <Wire.h>
// #include <SPI.h>
#include <Adafruit_MQTT.h>
#include "Adafruit_MQTT/Adafruit_MQTT.h" 
#include "Adafruit_MQTT/Adafruit_MQTT_SPARK.h" 
#include "Adafruit_MQTT/Adafruit_MQTT.h" 
#include "credentials.h"
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include "Adafruit_SSD1306.h"
#define OLED_RESET D4
#define SEALEVELPRESSURE_HPA (1013.25)

const int moisturePin = A0;
const int AIRSENSORPIN = A2;
const int PUMPPIN = 3;
const int DUSTPIN = A1;
const char RELAY = 3;

AirQualitySensor airqualitysensor(AIRSENSORPIN); // define objects
Adafruit_SSD1306 display(OLED_RESET);
String airQualityStatus, DateTime, TimeOnly;
Adafruit_BME280 bme;

char currentDateTime[25], currentTime[9];
int current_quality =-1;
char percent = 0x25;
float pumpState;
int moistVal;
float temp,pressure,altitude, humidity;
int timer = 0;
int timeout = 0 ;
int lastTime;
unsigned long last;
int waterTimer = 0; 
unsigned long duration;
unsigned long starttime;
unsigned long sampletime_ms = 30000;//sample 30s 
unsigned long lowpulseoccupancy = 0;
float ratio = 0;
float concentration = 0;
int airQualityVal;

/************ Global State (you don't need to change this!) ******************/ 
TCPClient TheClient; 

// Setup the MQTT client class by passing in the WiFi client and MQTT server and login details. creds from credentials.h
Adafruit_MQTT_SPARK mqtt(&TheClient,AIO_SERVER,AIO_SERVERPORT,AIO_USERNAME,AIO_KEY); 

/****************************** Feeds ***************************************/ 
// Setup Feeds to publish or subscribe 
// Notice MQTT paths for AIO follow the form: <username>/feeds/<feedname> 
// Adafruit_MQTT_Subscribe LED_buttonState = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/LED_buttonState"); 
Adafruit_MQTT_Publish tempBME = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/tempBME");
// Adafruit_MQTT_Publish LPO = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/LPO");
// Adafruit_MQTT_Publish dustRatio = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/dustRatio");
Adafruit_MQTT_Publish dustConcentration = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/dustConcentration");
Adafruit_MQTT_Publish humidityBME = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/humidityBME");
Adafruit_MQTT_Publish moistureValue = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/moistureValue");
Adafruit_MQTT_Publish airQuality = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/airQuality");
Adafruit_MQTT_Subscribe pump_switch = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/pump_switch"); 

void setup() {

  pinMode(DUSTPIN,INPUT);
  pinMode(PUMPPIN,OUTPUT);
  pinMode(RELAY,OUTPUT);

  // airqualitysensor.init(AIRSENSORPIN);

  Serial.begin(9600);
    // while (!Serial)
        ; // time to get serial running
    Serial.println(F("BME280 test"));

    unsigned status;

    // default settings
    // (you can also pass in a Wire library object like &Wire2)
    status = bme.begin();
    if (!status)
    {
        Serial.println("Could not find a valid BME280 sensor, check wiring, address, sensor ID!");
        Serial.print("SensorID was: 0x");
        Serial.println(bme.sensorID(), 16);
        Serial.print("        ID of 0xFF probably means a bad address, a BMP 180 or BMP 085\n");
        Serial.print("   ID of 0x56-0x58 represents a BMP 280,\n");
        Serial.print("        ID of 0x60 represents a BME 280.\n");
        Serial.print("        ID of 0x61 represents a BME 680.\n");
        while (1)
            ;
    }

  Serial.println("-- Default Test --");
  // delayTime = 1000;

  Serial.println();
  Time.zone ( -7) ; // MST = -7, MDT = -6
  Particle.syncTime () ; // Sync time with Particle Cloud
  Serial.begin(9600);
  pinMode(moisturePin,INPUT);
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);  // initialize with the I2C addr 0x3D (for the 128x64)
  // init done
  
  display.display(); // show splashscreen
  delay(2000);
   Serial.printf("Connecting to Internet \n");
  WiFi.connect(); // just internet no particle cloud
  while(WiFi.connecting()) {
    Serial.printf(".");
    delay(100);
  }
  Serial.printf("\n Connected!!!!!! \n"); // breathing green is connected to internet only :)
  // Setup MQTT subscription for onoff feed.
  //mqtt.subscribe(&TempF);
  mqtt.subscribe(&pump_switch); // "turns on" subscription <NEED FEED VAR NOT FEEDNAME> <&> points to --> VAR
  // mqtt.subscribe(&LED_slider); 
  starttime = millis();
  displayLargeText("MAJESTIC\n   I/O");
}

void loop() {
  moistVal = analogRead(moisturePin);
  timer = millis();
  dustSensor();
  airSensor();
  timeStuff();
  bmeStats();
  serialPrint(); 
  displayText("Moisture Value:", moistVal);
  MQTT_connect();
  ping();
  subscribes();
  publishing();
  autoPump(); //moisture activated pump 

} //end void loop

void displayLargeText(char desiredString[]) { //takes args and prints them to OLED

  display.clearDisplay();
  display.setCursor(0,0);             // Start cursor at top-left corner
  display.setRotation(0) ;
  display.setTextSize(2); 
  display.setTextColor(WHITE); // Draw 'inverse' text
  display.printf("%s", desiredString);
  display.display();
  display.startscrollright(0x00, 0x07);
  delay(1500);
  display.stopscroll();
}
void displayText(char desiredString[], int desiredOutput) { //takes args and prints them to OLED

  display.clearDisplay();
  display.setCursor(0,0);             // Start cursor at top-left corner
  display.setRotation(0) ;
  display.setTextSize(1); 
  display.setTextColor(WHITE); // Draw 'inverse' text
  display.printf("*****SMART POT******\n\n%s %i\n%0.1f C\nHumidity %0.1f%c\n%s\nDust level %0.02f\nTime is %s",
     desiredString, desiredOutput, temp, humidity, percent, airQualityStatus.c_str(), concentration, currentTime);
  display.display();
}
void MQTT_connect() {
  int8_t ret;
 
  // Stop if already connected.
  if (mqtt.connected()) {
    return;
  }
 
  Serial.print("Connecting to MQTT... ");
 
  while ((ret = mqtt.connect()) != 0) { // connect will return 0 for connected
       Serial.println(mqtt.connectErrorString(ret));
       Serial.println("Retrying MQTT connection in 5 seconds...");
       mqtt.disconnect();
       delay(5000);  // wait 5 seconds
  }
  Serial.println("MQTT Connected!");
}
void autoPump() {
  while ((timer-waterTimer)>60000)
  {
    waterTimer=millis();
    if (moistVal > 3350) {
      digitalWrite(PUMPPIN,HIGH);
      if ((timer - timeout)>300) {
        timeout = millis();
        digitalWrite(PUMPPIN,LOW);
        waterTimer = 0;
      }
    }
  } 
}
void publishing() {
    if((millis()-lastTime > 60000)) { // how to PUBLISH <don't over-publish, once or twice a min is good
    if(mqtt.Update()) {
      tempBME.publish(temp); //publish your <variable> to <feedvar2>
      humidityBME.publish(humidity);
      humidityBME.publish(humidity);
      moistureValue.publish(moistVal);
      // LPO.publish(lowpulseoccupancy); // publishing limit reached 
      // dustRatio.publish(ratio);
      dustConcentration.publish(concentration);
      airQuality.publish(airQualityVal);
      Serial.printf("Publishing stats\n"); // tells you that you have published in Serial Monitor
      } 
    lastTime = millis();
  }
}
void subscribes() {
  // this is our 'wait for incoming subscription packets' busy subloop 

  Adafruit_MQTT_Subscribe *subscription; // how to SUBCRIBE 
  while ((subscription = mqtt.readSubscription(1000))) { // checks every arg time eg 1000 millis()
    if (subscription == &pump_switch) {
      pumpState = atof((char *)pump_switch.lastread);

    }
    }
    if (pumpState) {
      digitalWrite(PUMPPIN,HIGH);
    }
    else {
      digitalWrite(PUMPPIN,LOW);
    }
}
void ping() {
  // Ping MQTT Broker every 2 minutes to keep connection alive
  if ((millis()-last)>120000) {
      Serial.printf("Pinging MQTT \n");
      if(! mqtt.ping()) {
        Serial.printf("Disconnecting \n");
        mqtt.disconnect();
      }
      last = millis();
  }
}
void bmeStats() {
  temp = bme.readTemperature();
  pressure = bme.readPressure();
  altitude = bme.readAltitude(SEALEVELPRESSURE_HPA);
  humidity = bme.readHumidity();
}
void dustSensor() {
      //This section is for the dust sensor
  duration = pulseIn(DUSTPIN, LOW);
  lowpulseoccupancy = lowpulseoccupancy+duration;
 
  if ((millis()-starttime) > sampletime_ms)//if the sample time == 30s
  {
      ratio = lowpulseoccupancy/(sampletime_ms*10.0);  // Integer percentage 0=>100
      concentration = 1.1*pow(ratio,3)-3.8*pow(ratio,2)+520*ratio+0.62; // using spec sheet curve
      // Serial.printf("LPO %i\nRatio %0.02f\nConcentration %0.02f\n",lowpulseoccupancy,ratio,concentration);
      
      lowpulseoccupancy = 0;
      starttime = millis();
  }
}
void airSensor() {
   //This is for the air sensor
   airQualityVal = airqualitysensor.getValue();
   current_quality=airqualitysensor.slope();
    if (current_quality >= 0)// if a valid data returned.
    {
      Serial.printf("airQualityVal = %i\n",airQualityVal);
        if (current_quality==0) {
            airQualityStatus = "Danger high pollution";
            Serial.println("High pollution! Force signal active");}
        else if (current_quality==1){
            airQualityStatus = "High pollution level";
            Serial.println("High pollution!");}
        else if (current_quality==2){
            airQualityStatus = "Low pollution";
            Serial.println("Low pollution!");}
        else if (current_quality ==3){
            airQualityStatus = "Fresh Air";
            Serial.println("Fresh air");}
    }//end air sensor section
}
void timeStuff() {
   // Convert String to char arrays - this is needed for formatted print
  DateTime.toCharArray (currentDateTime, 25) ;
  TimeOnly.toCharArray (currentTime, 9) ;
  DateTime = Time.timeStr () ; // Current Date and Time from Particle Time class
  TimeOnly = DateTime.substring (11 ,19) ; // Extract the Time from the DateTime String
}
void serialPrint() {
   if ((timer - timeout)>1000) {
    timeout = millis();
    Serial.printf("temp %0.02f\npressure %0.02f\naltitude %0.02f\nhumidity %0.02f\nmoisture level %i\n", 
    temp, pressure, altitude, humidity, moistVal);
     // Print using formatted print
    Serial.printf ("Date and time is %s\n", currentDateTime );
    Serial.printf ("Time is %s\n", currentTime );
    Serial.printf("pump value is %0.02f\n",pumpState);
  }
}