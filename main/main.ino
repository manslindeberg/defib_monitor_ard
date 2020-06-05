
#include <Arduino.h>
//WiFiNINA 1.5.0 by Arduino
#include <WiFiNINA.h>
//SPI 1.0 by Arduino
#include <SPI.h>
//String 1.0 by Arduino
#include <String.h>
//Arduino _JSON 0.10 by Arduino
#include <Arduino_JSON.h>
//Time 1.6.0 by Michael Margolis
#include <TimeLib.h>
//DHT sensor library 1.3.0 by Adafruit
#include <DHT.h>

//Defenitions
#define DHTTYPE DHT11
#define DHTdataPin 10
#define controllerPin 5
#define tempPin A6
#define voltagePin 17
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

//DHT Sensor class
DHT dht(DHTTYPE, DHTdataPin);
//SSID of the Wi-Fi
char ssid[] = {'T', 'e', 'm', 'p', 'm', 'o', 'n', 'i', 't', 'o', 'r'} ;
int val;
//Setting status
int status = WL_IDLE_STATUS;
//Server on port 80
WiFiServer server(80);
//String for databuffer on
String readString = String(MAXIMUM_HTTP_REQUEST_LENGTH);
//JSON array containing sensordata
JSONVar dataJSON;
//Bool for keeping track of WiFistatus
bool wifiStatus;
//Integer for different Httpstatuses
int httpStatus;
//Buffer storage for connected client on the network
char clientBuffer;
//TimeKeeping varialbe
unsigned long t;
//WifiClient for handeling connected clients
WiFiClient client;


void setup() {
  Serial.begin(9600);
  while (!Serial) {};
  pinMode(voltagePin, INPUT);
  pinMode(tempPin, INPUT);

  dht.begin();
  dataJSON["time"], dataJSON["temp"], dataJSON["volt"], dataJSON["statusCode"];
  wifiStatus = wifiInit(ssid);
  t = millis();

}

void loop() {



  // compare the previous status to the current status
  if (status != WiFi.status()) {
    // it has changed update the variable
    status = WiFi.status();

    if (status == WL_AP_CONNECTED) {
      // a device has connected to the AP
      Serial.println("Device connected to AP");
    } else {
      // a device has disconnected from the AP, and we are back in listening mode
      Serial.println("Device disconnected from AP");
    }
  }
  client = server.available();
  while (client.connected()) {
    if (client.available()) {
      httpAgent(client);
    }
    //Check sensors while Client is connected
    if (millis() - t > 10000) {
      t = millis();
      if (analogRead(voltagePin) < MINIMUM_VOLTAGE) {
        //Some Warnng function;
      }
      temperature = (double) dht.readTemperature();
      output = computePID(temperature);
      delay(100);

      if (output < 0) {
        output = 0;
      }
      else if ( output > 8) {
        output = 8;
      }
      else {
        output = output;
      }
      output = (int)((output / 8) * 255);
      analogWrite(controllerPin, output);
    }

  }
  // Kollar om 10 Sek har gått
  if (millis() - t > 10000) {
    t = millis();
    if (analogRead(voltagePin) < MINIMUM_VOLTAGE) {
      //Some Warnng function;
    }
    temperature = (double) dht.readTemperature();
    output = computePID(temperature);
    delay(100);

    if (output < 0) {
      output = 0;
    }
    else if ( output > 8) {
      output = 8;
    }
    else {
      output = output;
    }
    output = (int)((output / 8) * 255);
    analogWrite(controllerPin, output);
  }
}



/**
 * Function for computing PI-regulation, by Oskar N  
 * 
 * @param input value
 * @return ouput value of the calculated PI-regulation
 */
double computePID (double inp) {
  t = millis();                                    //get current time
  elapsedTime = (double)(t - previousTime);        //compute time elapsed from previous computation

  error = setPoint - inp;                                    // determine error
  cumError += error * elapsedTime;                           // compute integral

  double out = kp * error + ki * cumError;                   //PI output

  lastError = error;                                         //remember current error
  previousTime = t;                                //remember current time

  return out;
}


/**
 * Function for initializing the Wifi-module of the MKR1010
 * 
 * @param char array with the specified SSID name
 * @return true if the wifi iinitialized succesfully, false otherwise
 */
bool wifiInit(char ssid[]) {
  Serial.println("AP INITIALIZED");
  if (WiFi.status() == WL_NO_MODULE) {
    Serial.println("Communication with WiFi module failed!");
    return false;
  }
  if (WiFi.firmwareVersion() < WIFI_FIRMWARE_LATEST_VERSION) {
    Serial.println("Please upgrade the firmware");
  }
  Serial.print("Creating access point named: ");
  Serial.println(ssid);
  //Init the AP
  status = WiFi.beginAP(ssid);
  if (status != WL_AP_LISTENING) {
    Serial.println("Creating access point failed");
    // don't continue
    return false;
  }
  server.begin();
  printWiFiStatus();
  return true;
}
/**
 * Simple serial print function for debugging, prints the Wifi SSID and IP-adress in the serial connection
 *
 */
void printWiFiStatus() {
  // print the SSID of the network you're attached to:
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  // print your WiFi shield's IP address:
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);
}
/**
 * Http-handeler which should be called upon when there is http data in the client buffer, dependending on content of the recieved data, it will act accordingly
 * 
 * @param specified client connected to the network
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
 * Function for generating an HTTP-response to the client, depending on the input status will generate diffrent responses, at the moment only GET and ERROR is implemented.
 * 
 * @param statuscode for the response, predefined in library. see HTTP_STATUSCODE_xxx
 * @param specified client connected to the network
 */
bool httpResponse(int status, WiFiClient client) {

  switch (status) {

    case HTTP_STATUSCODE_GET:
      dataJSON["temp"] = (String) dht.readTemperature();
      dataJSON["volt"] = (String) analogRead(voltagePin);
      dataJSON["time"] = (String) millis();
      client.println("HTTP/1.1 200 OK");
      client.println("Server: Arduino");
      client.println("Content-Type: application/json");
      client.println("Connection: close");
      client.println("");
      client.println(JSON.stringify(dataJSON));
      client.println("");
      break;

    case HTTP_STATUSCODE_ERROR:
      client.println("HTTP/1.1 400 Bad Request");
      client.println("");
      break;



  }
  client.stop();
  return true;
}
