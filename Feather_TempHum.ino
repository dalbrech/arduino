/*
 * Connect pin 1 (on the left) of the sensor to +5V
 * Connect pin 2 of the sensor to whatever your DHTPIN is
 * Connect pin 4 (on the right) of the sensor to GROUND
 * Connect a 10K resistor from pin 2 (data) to pin 1 (power) of the sensor

 * Note:
 * You should modify the wifi account: WIFI_SSID   WIFI_PWD
 * Update ThingSpeak API: WIFI_PWD below first
*/

#include <ESP8266WiFi.h>
#include "DHT.h"
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define DHTPIN 13     // what digital pin we're connected to
#define DHTTYPE DHT22   // DHT 22  (AM2302), AM2321

#define LOG_INTERVAL 60  // seconds

//WIFI Setting
//const char* WIFI_SSID = "jds-44219";
//const char* WIFI_PWD = "ijmp-3zid-cti7-efim";
const char* WIFI_SSID = "WPI-Open";
const char* WIFI_PWD = "";
//const char* WIFI_SSID = "Liberty Farm Retreat";
//const char* WIFI_PWD = "abbykelleyfoster";
WiFiClient client;

//Thingspeak.com Setting
const char *host = "api.thingspeak.com";                  //IP address of the thingspeak server
const char *api_key ="0A55V9P1FB7SYMHY";                  //Your own thingspeak api_key
const int httpPort = 80;

void uploadDataToWeb();

Adafruit_SSD1306 display = Adafruit_SSD1306(128, 32, &Wire);

// OLED FeatherWing buttons map to different pins depending on board:
#if defined(ESP8266)
  #define BUTTON_A  0
  #define BUTTON_B 16
  #define BUTTON_C  2
#endif

// Initialize DHT sensor.
DHT dht(DHTPIN, DHTTYPE);

//float tempC = 25.0;
//float RH = 50.0;
int errorNum = 0;

long readTime = 0; 
long uploadTime = 0;

void setup(){
 
  Serial.begin(9600);
  delay(500);
 
  Serial.println();
  Serial.print("MAC: ");
  Serial.println(WiFi.macAddress());
  
  //wifi connect
  WiFi.begin(WIFI_SSID, WIFI_PWD); 

  Serial.print("OLED FeatherWing...");
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C); // Address 0x3C for 128x32
  Serial.println("OLED begun");

  // Show image buffer on the display hardware.
  // Since the buffer is intialized with an Adafruit splashscreen
  // internally, this will display the splashscreen.
  display.display();
  delay(1000);

  // Clear the buffer.
  display.clearDisplay();
  display.display();
   
  dht.begin();

  pinMode(BUTTON_A, INPUT_PULLUP);
  pinMode(BUTTON_B, INPUT_PULLUP);
  pinMode(BUTTON_C, INPUT_PULLUP);
}

void loop(){
  
  //Read Data every 5 seconds
  if(millis() - readTime > 5000){

    readTime = millis();  
      
    // Reading temperature or humidity takes about 250 milliseconds!
    // Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
    float RH = dht.readHumidity();
    // Read temperature as Celsius (the default)
    float tempC = dht.readTemperature();

    // Check if any reads failed and exit early (to try again).
    if (isnan(RH) || isnan(tempC)) {
      errorNum += 1;
      Serial.print("Failed to read from DHT sensor! #");
      Serial.println(errorNum);
      return;
    }
  
    Serial.print("Humidity: ");
    Serial.print(RH);
    Serial.print(" %\t");
    Serial.print("Temperature: ");
    Serial.print(tempC);
    Serial.println(" *C ");
  
    updateDisplay(tempC, RH);
    
    //Upload Data every LOG_INTERVAL seconds
    if(millis() - uploadTime > LOG_INTERVAL*1000){
      uploadTime = millis();
      uploadDataToWeb(tempC, RH);
    }  
  }
}

void uploadDataToWeb(float tempC, float RH){

  // Two values(field1 field2) have been set in thingspeak.com 
  String command = String("GET ") + "/update?api_key="+api_key+"&field1="+tempC+"&field2="+RH+" HTTP/1.1\r\n" +"Host: " + host + "\r\n" + "Connection: close\r\n\r\n";
  Serial.println(command); 
   
  if(!client.connect(host, httpPort)){
    Serial.println("connection failed");
    return;
  }
  
  client.print(command);
  while(client.available()){
    String line = client.readStringUntil('\r');
    Serial.print(line);
  }
}


void updateDisplay(float tempC, float RH) {
  display.clearDisplay();
  
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0,0); display.print("Temp: 'C");
  display.setCursor(72,0); display.print("Humid: %");
  
  display.setTextSize(2);
  display.setCursor(0,16); display.println(tempC);
  display.setCursor(72,16); display.println(RH);
    
  display.setCursor(0,0);
  display.display();
}
