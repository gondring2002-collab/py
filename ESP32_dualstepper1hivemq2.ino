#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>

// ================= WIFI =================
const char* ssid     = "Tselhome-2453";
const char* password = "72699064";

// ================= MQTT =================
const char* mqttServer = "f4159c80eb524405b694dc0e6a311fe4.s1.eu.hivemq.cloud";
const int   mqttPort   = 8883;
const char* mqttUser   = "hivemq.webclient.1765435980051";
const char* mqttPass   = "2:E?Q;RJ08&1cijeFbaT";

const char* topicStepper1 = "home/esp32/stepper/1";
const char* topicStepper2 = "home/esp32/stepper/2";
const char* topicStatus   = "home/esp32/status";

WiFiClientSecure wifiClient;
PubSubClient client(wifiClient);

// ================= STEPPER CONFIG =================
const int stepDelayUs = 1000;
const int maxSteps    = 30000;

// ===== STEPPER 1 =====
const int stepPin1 = 5;
const int dirPin1  = 2;
const int enPin1   = 15;
const int limit1   = 19;

// ===== STEPPER 2 =====
const int stepPin2 = 25;
const int dirPin2  = 26;
const int enPin2   = 27;
const int limit2   = 33;

// ===== STATE =====
int  pos1 = 0;
int  pos2 = 0;
bool stop1 = false;
bool stop2 = false;

// =================================================
//                 WIFI & MQTT
// =================================================
void connectWiFi() {
  Serial.print("Connecting WiFi");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi Connected");
}

void connectMQTT() {
  while (!client.connected()) {
    Serial.print("Connecting MQTT...");
    String clientId = "ESP32-" + String(random(0xffff), HEX);

    if (client.connect(clientId.c_str(), mqttUser, mqttPass)) {
      Serial.println("OK");
      client.subscribe(topicStepper1);
      client.subscribe(topicStepper2);
      client.publish(topicStatus, "ESP32 Online");
    } else {
      Serial.print("FAIL rc=");
      Serial.println(client.state());
      delay(2000);
    }
  }
}

// =================================================
//               STEPPER FUNCTIONS
// =================================================
void moveForward(int stepPin, int dirPin, int enPin,
                 int &pos, bool &stopFlag, int steps) {

  stopFlag = false;
  digitalWrite(dirPin, HIGH);
  digitalWrite(enPin, LOW);

  for (int i = 0; i < steps; i++) {
    if (stopFlag) {
      client.publish(topicStatus, "forward_stopped");
      break;
    }

    digitalWrite(stepPin, HIGH);
    delayMicroseconds(stepDelayUs);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(stepDelayUs);

    pos++;
    client.loop();
  }

  digitalWrite(enPin, HIGH);
}

void moveBackwardToLimit(int stepPin, int dirPin, int enPin,
                         int limitPin, int &pos, bool &stopFlag) {

  stopFlag = false;
  digitalWrite(dirPin, LOW);
  digitalWrite(enPin, LOW);

  while (digitalRead(limitPin) == HIGH) {
    if (stopFlag) {
      client.publish(topicStatus, "backward_stopped");
      digitalWrite(enPin, HIGH);
      return;
    }

    digitalWrite(stepPin, HIGH);
    delayMicroseconds(stepDelayUs);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(stepDelayUs);

    pos--;
    client.loop();
  }

  digitalWrite(enPin, HIGH);
  pos = 0;
  client.publish(topicStatus, "backward_done");
}

// =================================================
//                  MQTT CALLBACK
// =================================================
void callback(char* topic, byte* payload, unsigned int length) {
  String msg;
  for (int i = 0; i < length; i++) msg += (char)payload[i];

  Serial.print(topic);
  Serial.print(" => ");
  Serial.println(msg);

  // ===== STEPPER 1 =====
  if (String(topic) == topicStepper1) {
    if (msg == "forward") {
      client.publish(topicStatus, "S1 forward");
      moveForward(stepPin1, dirPin1, enPin1, pos1, stop1, maxSteps - pos1);
    }
    else if (msg == "backward") {
      client.publish(topicStatus, "S1 backward");
      moveBackwardToLimit(stepPin1, dirPin1, enPin1, limit1, pos1, stop1);
    }
    else if (msg == "stop") {
      stop1 = true;
      digitalWrite(enPin1, HIGH);
      client.publish(topicStatus, "S1 stop");
    }
  }

  // ===== STEPPER 2 =====
  else if (String(topic) == topicStepper2) {
    if (msg == "forward") {
      client.publish(topicStatus, "S2 forward");
      moveForward(stepPin2, dirPin2, enPin2, pos2, stop2, maxSteps - pos2);
    }
    else if (msg == "backward") {
      client.publish(topicStatus, "S2 backward");
      moveBackwardToLimit(stepPin2, dirPin2, enPin2, limit2, pos2, stop2);
    }
    else if (msg == "stop") {
      stop2 = true;
      digitalWrite(enPin2, HIGH);
      client.publish(topicStatus, "S2 stop");
    }
  }
}

// =================================================
//                     SETUP
// =================================================
void setup() {
  Serial.begin(115200);

  pinMode(stepPin1, OUTPUT);
  pinMode(dirPin1, OUTPUT);
  pinMode(enPin1, OUTPUT);
  pinMode(limit1, INPUT_PULLUP);

  pinMode(stepPin2, OUTPUT);
  pinMode(dirPin2, OUTPUT);
  pinMode(enPin2, OUTPUT);
  pinMode(limit2, INPUT_PULLUP);

  digitalWrite(enPin1, HIGH);
  digitalWrite(enPin2, HIGH);

  connectWiFi();
  wifiClient.setInsecure();

  client.setServer(mqttServer, mqttPort);
  client.setCallback(callback);
  connectMQTT();

  client.publish(topicStatus, "System Ready");
}

// =================================================
//                      LOOP
// =================================================
void loop() {
  if (!client.connected()) connectMQTT();
  client.loop();
}
