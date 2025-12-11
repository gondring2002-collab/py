#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>

// ===== WIFI =====
const char* ssid = "Brin-Net";
const char* password = "";

// ===== MQTT Hivemq Cloud =====
const char* mqttServer = "broker.hivemq.com"; // contoh HiveMQ public broker
const int mqttPort = 8883; // TLS port
const char* mqttTopic = "home/steppermotor";
const char* relayTopic = "home/vacuum";

WiFiClientSecure wifiClient;
PubSubClient client(wifiClient);

// ===== STEP MOTOR PINS =====
const int stepPin = 5;     
const int dirPin  = 2;     
const int enablePin = 15;  
const int limitSwitchPin = 19;  // aktif LOW
const int relayPin = 18;         // relay vacuum

const int stepDelayUs = 1000;      
const int maxForwardSteps = 10000;  // batas maju setelah homing

int currentPosition = 0;  // posisi relatif setelah homing

// ===== WIFI =====
void setupWiFi() {
  Serial.println("Connecting to WiFi...");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi Connected");
  Serial.print("IP: ");
  Serial.println(WiFi.localIP());
}

// ===== MQTT =====
void connectMQTT() {
  while (!client.connected()) {
    Serial.println("Connecting to MQTT...");
    String clientId = "ESP32-B-" + String(random(0xffff), HEX);
    if (client.connect(clientId.c_str())) {
      Serial.println("Connected to MQTT");
      client.subscribe(mqttTopic);
      client.subscribe(relayTopic);
    } else {
      Serial.print("Failed, rc=");
      Serial.println(client.state());
      delay(2000);
    }
  }
}

void callback(char* topic, byte* payload, unsigned int length) {
  String message = "";
  for (int i = 0; i < length; i++) message += (char)payload[i];
  Serial.print("MQTT Message: ");
  Serial.println(message);

  // Stepper commands
  if (message.startsWith("forward")) {
    int stepsToGo = maxForwardSteps - currentPosition;
    if (stepsToGo > 0) moveForward(stepsToGo);
    else Serial.println("Already at maximum forward position!");
  }
  else if (message.startsWith("backward")) moveBackwardToLimit();

  // Relay commands
  if (String(topic) == relayTopic) {
    if (message == "ON") {
      digitalWrite(relayPin, HIGH);
      Serial.println("Relay Vacuum: ON");
    } else if (message == "OFF") {
      digitalWrite(relayPin, LOW);
      Serial.println("Relay Vacuum: OFF");
    }
  }
}

// ===== STEP MOTOR FUNCTIONS =====
void moveForward(int steps) {
  digitalWrite(dirPin, HIGH);
  digitalWrite(enablePin, LOW);  
  Serial.print("Moving Forward ");
  Serial.print(steps);
  Serial.println(" steps...");

  for (int i = 0; i < steps; i++) {
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(stepDelayUs);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(stepDelayUs);
  }

  currentPosition += steps;
  digitalWrite(enablePin, HIGH);  
  Serial.print("Forward done. Current position: ");
  Serial.println(currentPosition);
}

void moveBackwardToLimit() {
  digitalWrite(dirPin, LOW);
  digitalWrite(enablePin, LOW);
  int stepCount = 0;

  Serial.println("Moving Backward... Waiting for limit switch...");

  while (digitalRead(limitSwitchPin) == HIGH) { // HIGH = switch belum ditekan
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(stepDelayUs);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(stepDelayUs);
    stepCount++;
  }

  digitalWrite(enablePin, HIGH);
  currentPosition = 0; // reset posisi
  Serial.print("Motor stopped at limit switch after ");
  Serial.print(stepCount);
  Serial.println(" steps. Position reset to 0.");
}

// ===== SETUP =====
void setup() {
  Serial.begin(115200);

  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  pinMode(enablePin, OUTPUT);
  digitalWrite(enablePin, HIGH);  

  pinMode(limitSwitchPin, INPUT_PULLUP); 

  pinMode(relayPin, OUTPUT);
  digitalWrite(relayPin, LOW); // OFF default

  Serial.println("Stepper Ready. Performing homing...");

  // -------- Homing --------
  digitalWrite(enablePin, LOW);
  digitalWrite(dirPin, LOW); // mundur
  int stepCount = 0;
  while (digitalRead(limitSwitchPin) == HIGH) {
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(stepDelayUs);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(stepDelayUs);
    stepCount++;
  }
  digitalWrite(enablePin, HIGH);
  currentPosition = 0;
  Serial.print("Homing done! Limit switch pressed after ");
  Serial.print(stepCount);
  Serial.println(" steps.");

  // -------- Maju 10000 langkah setelah homing --------
  moveForward(maxForwardSteps);

  // -------- Setup WiFi / MQTT setelah home & forward --------
  setupWiFi();
  client.setServer(mqttServer, mqttPort);
  client.setCallback(callback);
  connectMQTT();

  Serial.println("Stepper ready for MQTT commands.");
}

// ===== LOOP =====
void loop() {
  if (!client.connected()) connectMQTT();
  client.loop();
}
