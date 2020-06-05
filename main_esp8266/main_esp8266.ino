//ESP8266WiFi 1.5.0 by ESP8266 Community
#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <ESP8266WebServer.h>
//Arduino _JSON 0.10 by Arduino
#include <Arduino_JSON.h>
//String 1.0 by Arduino
#include <String.h>
//Time 1.6.0 by Michael Margolis
#include <TimeLib.h>
//DHT sensor library 1.3.0 by Adafruit
#include <DHT.h>

#define DHTTYPE DHT22
#define DHTPIN D2
#define MINIMUM_VOLTAGE 11.22
#define MINIMUM_TEMPERATURE 8
#define MAXIMUM_HTTP_REQUEST_LENGTH 255
#define HTTP_STATUSCODE_GET 1
#define HTTP_STATUSCODE_ERROR -1

//PI constants
double kp = 2;
double ki = 5;
//PI variables
double elapsedTime;
double error;
double lastError;
double input, output, setPoint;
double cumError, rateError;
double temperature;
unsigned long previousTime;


const char* ssid = "Tempmonitor";
int ledPin = 03;
WiFiServer server(80);
unsigned long t;
char clientBuffer;
String readString;
int httpStatus;
DHT dht(DHTPIN, DHTTYPE);
//JSON array containing sensordata
JSONVar dataJSON;
//WifiClient for handeling connected clients
WiFiClient client;
//Setting status
int status;



void setup() {
  Serial.begin(115200);
  delay(1000);
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, LOW);
  wifiInit(ssid);
  dht.begin();
  dataJSON["time"], dataJSON["temp"], dataJSON["volt"], dataJSON["statusCode"];
  Serial.println("HTTP server started");
}

void loop() {
  WiFiClient client = server.available();
  if (client)
  {
    Serial.println("\n[Client connected]");
    while (client.connected()){
        if (client.available()){
          httpAgent(client);
      }
    }
  }


  
  Serial.println("No clients");
  delay(2000);
}


  /**
     Function for computing PI-regulation, by Oskar N

     @param input value
     @return ouput value of the calculated PI-regulation
  */

  /**
     Function for initializing the Wifi-module of the MKR1010

     @param char array with the specified SSID name
     @return true if the wifi iinitialized succesfully, false otherwise

    /**
     Simple serial print function for debugging, prints the Wifi SSID and IP-adress in the serial connection

  */
  void printWiFiStatus() {
    // print the SSID of the network you're attached to:
    Serial.print("SSID: ");
    Serial.println(WiFi.SSID());

    // print your WiFi shield's IP address:
    IPAddress ip = WiFi.softAPIP();
    Serial.print("IP Address: ");
    Serial.println(ip);
  }
  /**
     Http-handeler which should be called upon when there is http data in the client buffer, dependending on content of the recieved data, it will act accordingly

     @param specified client connected to the network
  */
  void httpAgent(WiFiClient client) {

    while (client.available()) {
      clientBuffer = client.read();
      if (readString.length() < MAXIMUM_HTTP_REQUEST_LENGTH) {
        readString.concat(clientBuffer);
      }
    }
    Serial.println("----Incoming HTTP REQUEST:----");
    Serial.print(readString);
    if (readString.indexOf("GET") > -1) {
      httpStatus = HTTP_STATUSCODE_GET;
    }
    else {
      Serial.print("Invalid request format!");
      httpStatus = HTTP_STATUSCODE_ERROR;
    }

    if (httpResponse(httpStatus, client)) {
      Serial.print("----RESPONSE SENT----");
    }
    readString = "";
  }
  /**
     Function for generating an HTTP-response to the client, depending on the input status will generate diffrent responses, at the moment only GET and ERROR is implemented.

     @param statuscode for the response, predefined in library. see HTTP_STATUSCODE_xxx
     @param specified client connected to the network
  */
  bool httpResponse(int status, WiFiClient client) {

    switch (status) {

      case HTTP_STATUSCODE_GET:
        dataJSON["time"] = (String) millis();
        dataJSON["temp"] = (String) dht.readTemperature();
        dataJSON["votl"] = "12";
        client.println("HTTP/1.1 200 OK");
        client.println("Server: Arduino");
        client.println("Content-Type: application/json");
        client.println("Connection: close");
        client.println("");
        client.println(JSON.stringify(dataJSON));
        client.println("");
        Serial.println("Data transmitted");
        break;

      case HTTP_STATUSCODE_ERROR:
        client.println("HTTP/1.1 400 Bad Request");
        client.println("");
        Serial.println("Incorrect request");
        break;



    }
    client.stop();
    return true;
  }

  /**
     Function for initializing the Wifi-module of the MKR1010

     @param char array with the specified SSID name
     @return true if the wifi iinitialized succesfully, false otherwise
  */
  bool wifiInit(const char ssid[]) {
    WiFi.disconnect();
    WiFi.persistent(false);
    WiFi.mode(WIFI_AP);
    IPAddress local_IP(192,168,4,1);
    IPAddress gateway(192,168,1,1);
    IPAddress subnet(255,255,255,0);
    Serial.println("AP INITIALIZED");
    Serial.print("Creating access point named: ");
    Serial.println(ssid);
    Serial.print("Setting soft-AP configuration ... ");
    Serial.println(WiFi.softAPConfig(local_IP, gateway, subnet) ? "Ready" : "Failed!");
    status = WiFi.softAP(ssid);   
    Serial.println(status);
    server.begin();
    printWiFiStatus();
    return true;
  }
