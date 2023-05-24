#include <SoftwareSerial.h>
#include <stdio.h>
#include <ESP8266WiFi.h>
#include <PubSubClient.h>

EspSoftwareSerial::UART testSerial;

const char* ssid = "#JS";
const char* password = "qwertyui";
const char* mqtt_server = "broker.netpie.io";
const int mqtt_port = 1883;
const char* mqtt_Client = "79729277-a7b6-4565-8ca7-5643ddca9cad";  //"79729277-a7b6-4565-8ca7-5643ddca9cad";
const char* mqtt_username = "u7SSZbBteP82mAoGZc7hsiQbvnes4Kss";
const char* mqtt_password = "xQ$UnaNpv_rQu7wx8QcIqgeQ_X6#c1-w";
WiFiClient espClient;
PubSubClient client(espClient);


long lastMsg = 0;
int value = 0;
char msg[100];
int tempVal = 0;
int moistVal = 0;
int lightVal = 0;

String readString = "";

void setup() {

  testSerial.begin(9600, EspSoftwareSerial::SWSERIAL_8N1, D7, D8, false, 120, 120);
  Serial.begin(9600);
  //Serial.println();
  //Serial.print("Connecting to ");
  //Serial.println(ssid);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.println("please wait, I try to connect to wifi. If I cannot connect, that not my business.");
  }
  Serial.println("");
  Serial.println("OH f**k Yeah, WiFi connected");
  Serial.print("Your f**king IP address: ");
  Serial.println(WiFi.localIP());
  client.setServer(mqtt_server, mqtt_port);
}
void reconnect() {
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    if (client.connect(mqtt_Client, mqtt_username, mqtt_password)) {
      Serial.println("connected");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println("try again in 5 seconds");
      delay(5000);
    }
  }
}

void updateSensorValue(String readString) {
  Serial.println(readString);
  String temperature = "";
  String moist = "";
  String light = "";
  int temperatureInt = -999;
  int moistInt = -999;
  int lightInt = -999;
  int count = 0;
  for (int i = 0; i < readString.length(); i++) {
    if (readString[i] == ',') {
      //Serial.println(readString[i]);
      i++;
      while (readString[i] != '_') {
        if (readString[i] == ',') {
          count++;
          i++;
        }
        if (count == 0) {
          temperature += readString[i];
        } else if (count == 1) {
          moist += readString[i];
        } else if (count == 2) {
          light += readString[i];
        }
        i++;
        if (i >= readString.length()) break;
      }
    }
  }
  temperatureInt = temperature.toInt();
  moistInt = moist.toInt();
  lightInt = light.toInt();
  if (temperatureInt > 0) {
    tempVal = temperatureInt;
  }
  if (moistInt > 0) {
    moistVal = moistInt;
  }
  if (lightInt > 0) {
    lightVal = lightInt;
  }
  Serial.println(tempVal);
  Serial.println(moistVal);
  Serial.println(lightVal);
  Serial.println("Hello");
}

void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();
  // readString = "";
  while (testSerial.available()) {
    char ch;
    ch = testSerial.read();
    //Serial.print(ch);
    if (ch != '\0')
      readString += ch;
    else if (ch == '\0' && readString.length() > 0) {
      Serial.println("Read : " + readString);
      if (readString.length() > 8) {
        updateSensorValue(readString);
      }
      readString = "";
    }
  }
  /*
  if (readString.length() > 6) {
    updateSensorValue(readString);
  }
  */

  String data = "{\"data\": {\"humidity\": " + String(moistVal) + ", \"temperature\": " + String(tempVal) + ", \"light\" : " + String(lightVal) + "}}";

  Serial.println(data);
  
  data.toCharArray(msg, (data.length() + 1));
  client.publish("@shadow/data/update", msg);
  
  delay(30000);
}

/*if (!client.connected()) {
    reconnect();
    }
    client.loop();
}*/
