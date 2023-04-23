#include <SPIFFS.h>

#include <WiFi.h>
#include <PubSubClient.h>
#include <Wire.h>
#include <Keypad.h>

// Replace the next variables with your SSID/Password combination
const char* ssid = "YOUR_SSID";
const char* password = "YOUR_PASSWORD";

// Add your MQTT Broker IP address, example:
//const char* mqtt_server = "192.168.1.144";
const char* mqtt_server = "192.168.0.176";

//keypad set up
const int ROW_NUM = 4; //four rows
const int COLUMN_NUM = 3; //four columns
const int ENA_PIN = 21; // the Arduino pin connected to the EN1 pin L298N
const int IN1_PIN = 19; // the Arduino pin connected to the IN1 pin L298N
const int IN2_PIN = 18; // the Arduino pin connected to the IN2 pin L298N

char keys[ROW_NUM][COLUMN_NUM] = {
  {'1','2','3'},
  {'4','5','6'},
  {'7','8','9'},
  {'*','0','#'}
};

byte pin_rows[ROW_NUM] = {25, 14, 12, 27}; //connect to the row pinouts of the keypad
byte pin_column[COLUMN_NUM] = {26, 33, 13}; //connect to the column pinouts of the keypad

Keypad keypad = Keypad(makeKeymap(keys), pin_rows, pin_column, ROW_NUM, COLUMN_NUM);

char table_no[] = {'1','2','3','4','5','6'};
int table_arr_len = sizeof(table_no)/sizeof(table_no[0]);
char key_pressed;

//WIFI and MQTT client initializaiton
WiFiClient espClient;
PubSubClient client(mqtt_server, 1883, espClient);
long lastMsg = 0;
char msg[50];
int value = 0;

// Debug LED Pin
const int ledPin = 2;

volatile bool keypad_status = true;

void setup_wifi() {
  delay(10);
  // We start by connecting to a WiFi network
  Serial.println();
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

// Callback function for MQTT receive message
void callback(char* topic, byte* message, unsigned int length) {
  Serial.print("Message arrived on topic: ");
  Serial.print(topic);
  Serial.print(". Message: ");
  String messageTemp;
  
  for (int i = 0; i < length; i++) {
    Serial.print((char)message[i]);
    messageTemp += (char)message[i];
  }
  Serial.println();

  if (String(topic) == "esp32/status"){
    Serial.print("ESP32 Status: ");
    if (messageTemp == "standby"){
      digitalWrite(ledPin, LOW);
      Serial.println("Standby");
      keypad_status = false;
    }
    else if (messageTemp == "start"){
      digitalWrite(ledPin, HIGH);
      Serial.println("Start");
      keypad_status = true;
    }
  }

  // If a message is received on the topic esp32/led, you check if the message is either "on" or "off". 
  // Changes the output state according to the message
//  if (String(topic) == "esp32/led") {
//    Serial.print("Changing output to ");
//    if(messageTemp == "on"){
//      Serial.println("on");
//      digitalWrite(ledPin, HIGH);
//    }
//    else if(messageTemp == "off"){
//      Serial.println("off");
//      digitalWrite(ledPin, LOW);
//    }
//  }
}

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect("ESP32Client")) {
      Serial.println("connected");
      // Subscribe
      client.subscribe("esp32/led");
      client.subscribe("esp32/status");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
  digitalWrite(ledPin, HIGH);
}

void setup() {
  Serial.begin(115200);
  setup_wifi();
  //client.setServer(mqtt_server, 1883);
  client.setCallback(callback);

  pinMode(ledPin, OUTPUT);
  pinMode(ENA_PIN, OUTPUT);
  pinMode(IN1_PIN, OUTPUT);
  pinMode(IN2_PIN, OUTPUT);
  digitalWrite(ENA_PIN, HIGH);

  keypad.setHoldTime(300);
}

void loop() {
  if (!client.connected()) {
    reconnect();
  }
  // Go through MQTT callbacks
  client.loop();

  while(!keypad_status)
    client.loop();
    
  // Test: Publish esp32 topic after every 5 seconds
//  long now = millis();
//  if (now - lastMsg > 5000) {
//    lastMsg = now;
//    Serial.print("Publishing: ");
//    Serial.println("Hello!");
//    client.publish("esp32/test", "Hello!");
//  }

  // Polling for Keypad input (getKey() can return only once, reason for the temp_key)
  char temp_key = keypad.getKey();
  if(temp_key != NO_KEY)
    key_pressed = temp_key;

  // If the key is pressed down for 0.3s, publish it
  if (keypad.getState() == HOLD){
    
    String keyString = String(key_pressed); // Convert the char to a String
    const char* keyArray = keyString.c_str();
    for (int i = 0; i < table_arr_len; i++) {
      if (key_pressed == table_no[i]){
        digitalWrite(ledPin, LOW);
        Serial.print("Publishing: ");
        Serial.println(key_pressed);
        client.publish("table", keyArray);
        digitalWrite(IN1_PIN, HIGH);
        digitalWrite(IN2_PIN, LOW);
        delay(7000);
        digitalWrite(IN1_PIN, LOW);
        digitalWrite(IN2_PIN, HIGH);
        delay(7000);
        String tempStatus = "standby";
        const char* statusArray = tempStatus.c_str();
        client.publish("esp32/status", statusArray);
        break;
      }
    }
  }
}
