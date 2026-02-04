#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>

// ================= WIFI =================
//const char* ssid     = "itel P65";
//const char* password = "Salsa3349@";

const char* ssid     = "Tselhome-2453";
const char* password = "72699064";
// ================= MQTT =================
const char* mqttServer = "f4159c80eb524405b694dc0e6a311fe4.s1.eu.hivemq.cloud";
const int   mqttPort   = 8883;
const char* mqttUser   = "hivemq.webclient.1765435980051";
const char* mqttPass   = "2:E?Q;RJ08&1cijeFbaT";

const char* topicStepper1 = "home/esp32/stepper/1";
const char* topicStepper2 = "home/esp32/stepper/2";
const char* topicRelay    = "home/esp32/relay";
const char* topicStatus   = "home/esp32/status";

// ================= STEPPER =================
const int STEP_DELAY_US = 800;
const int MAX_STEPS     = 10000;
const long MANUAL_MAX_POS = 10000;

// ===== STEPPER 1 =====
#define STEP1 5
#define DIR1  2
#define EN1   15
#define LIM1  22

// ===== STEPPER 2 =====
#define STEP2 25
#define DIR2  26
#define EN2   27
#define LIM2  19

// ===== RELAY =====
#define RELAY1 18
#define RELAY2 21
#define RELAY3 23
bool relayState[3] = {false,false,false}; // r1,r2,r3

// ===== LIMIT SWITCH STATE =====
bool limState[2] = {true,true}; // s1,s2 (HIGH = tidak ditekan, LOW = ditekan)

// ================= STATE =================
//enum State { IDLE, HOMING, OPENING, READY };
enum State { IDLE, HOMING, OPENING, READY, MANUAL_FWD, MANUAL_BWD };
struct Stepper {
  int stepPin, dirPin, enPin, limitPin;
  long pos;
  State state;
};

Stepper s1 = {STEP1, DIR1, EN1, LIM1, 0, HOMING};
Stepper s2 = {STEP2, DIR2, EN2, LIM2, 0, HOMING};


WiFiClientSecure wifiClient;
PubSubClient client(wifiClient);

// ================= STATUS PUBLISH =================
void publishStatus() {
  char buf[200];
  sprintf(buf,
          "s1_state=%d,s1_pos=%ld,s1_lim=%d,"
          "s2_state=%d,s2_pos=%ld,s2_lim=%d,"
          "r1=%d,r2=%d,r3=%d",
          s1.state, s1.pos, limState[0],
          s2.state, s2.pos, limState[1],
          relayState[0],
          relayState[1],
          relayState[2]);
  client.publish(topicStatus, buf);
}

// ================= RELAY =================
void setRelay(int idx, bool on) {
    switch(idx){
        case 0: digitalWrite(RELAY1,on?HIGH:LOW); break;
        case 1: digitalWrite(RELAY2,on?HIGH:LOW); break;
        case 2: digitalWrite(RELAY3,on?HIGH:LOW); break;
    }
    relayState[idx] = on;   // update state internal
    publishStatus();        // langsung update GUI
}

// ================= WIFI =================
void connectWiFi() {
  Serial.print("Connecting WiFi");
  WiFi.begin(ssid, password);
  while(WiFi.status()!=WL_CONNECTED){
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected");
}

// ================= MQTT =================
void connectMQTT() {
  while(!client.connected()){
    Serial.print("Connecting MQTT...");
    String id = "ESP32-" + String(random(0xffff),HEX);
    if(client.connect(id.c_str(), mqttUser,mqttPass)){
        Serial.println("OK");
        client.subscribe(topicStepper1);
        client.subscribe(topicStepper2);
        client.subscribe(topicRelay);
        client.publish(topicStatus,"MQTT connected");
        publishStatus();
    }else{
        Serial.print("FAIL rc=");
        Serial.println(client.state());
        delay(2000);
    }
  }
}

// ================= STEPPER =================
void stepOnce(Stepper &s,bool dir){
  digitalWrite(s.dirPin,dir);
  digitalWrite(s.enPin,LOW);
  digitalWrite(s.stepPin,HIGH);
  delayMicroseconds(STEP_DELAY_US);
  digitalWrite(s.stepPin,LOW);
  delayMicroseconds(STEP_DELAY_US);
}

// ================= STATE MACHINE =================

void handleStepper(Stepper &s, int idx, const char* name) {

  // update limit switch
  bool lastLim = limState[idx];
  limState[idx] = digitalRead(s.limitPin);
  if (limState[idx] != lastLim) publishStatus();

  switch (s.state) {

    case HOMING:
      if (limState[idx] == HIGH) {
        if (idx == 0) {
          // Stepper 1 → MAJU
          stepOnce(s, LOW);
        } else {
          // Stepper 2 → MUNDUR
          stepOnce(s, LOW);
        }
      } else {
        s.pos = 0;
        s.state = OPENING;
        publishStatus();
        Serial.print(name);
        Serial.println(" HOMING DONE");
      }
      break;

    case OPENING:
      if (s.pos < MAX_STEPS) {
        stepOnce(s, HIGH);
        s.pos++;
      } else {
        digitalWrite(s.enPin, HIGH);
        s.state = READY;
        publishStatus();
        Serial.print(name);
        Serial.println(" READY");
      }
      break;

    case READY:
    case IDLE:
      // tidak melakukan apa-apa
      break;
    case MANUAL_FWD:
        if(s.pos >= MANUAL_MAX_POS){
            digitalWrite(s.enPin, HIGH);
            s.state = IDLE;
            publishStatus();
            Serial.println("MAX 30000 STEP");
        }else{
            stepOnce(s, HIGH);
            s.pos++;
        }
        break;

    case MANUAL_BWD:
        if(limState[idx] == LOW){
            digitalWrite(s.enPin, HIGH);
            s.pos = 0;
            s.state = IDLE;
            publishStatus();
            Serial.println("LIMIT HIT");
        }else{
            stepOnce(s, LOW);
            if(s.pos > 0) s.pos--;
        }
        break;


  }
}

// ================= MQTT CALLBACK =================
void callback(char* topic,byte* payload,unsigned int len){
    String msg;
    for(int i=0;i<len;i++) msg += (char)payload[i];

    // stepper
    Stepper* s=nullptr;
    int idx=-1;
    if(String(topic)==topicStepper1){s=&s1; idx=0;}
    if(String(topic)==topicStepper2){s=&s2; idx=1;}
    //if(s){
    //    if(msg=="stop"){ digitalWrite(s->enPin,HIGH); s->state=IDLE; publishStatus();}
    //    else if(msg=="home"){ s->pos=0; s->state=HOMING; publishStatus();}
    //   else if(msg=="open"){ s->state=OPENING; publishStatus();}
    //}
    if(s){
    if(msg=="stop"){
        digitalWrite(s->enPin, HIGH);
        s->state = IDLE;
        publishStatus();
    }
    else if(msg=="home"){
        s->pos = 0;
        s->state = HOMING;
        publishStatus();
    }
    else if(msg=="open"){
        s->state = OPENING;
        publishStatus();
    }
    else if(msg=="forward"){
        s->state = MANUAL_FWD;
        digitalWrite(s->enPin, LOW);
        publishStatus();
    }
    else if(msg=="backward"){
        s->state = MANUAL_BWD;
        digitalWrite(s->enPin, LOW);
        publishStatus();
    }
    }

    
    // relay per tombol GUI
    if(String(topic)==topicRelay){
        if(msg=="ON1") setRelay(0,true);
        else if(msg=="OFF1") setRelay(0,false);
        else if(msg=="ON2") setRelay(1,true);
        else if(msg=="OFF2") setRelay(1,false);
        else if(msg=="ON3") setRelay(2,true);
        else if(msg=="OFF3") setRelay(2,false);
    }
}

// ================= SETUP =================
void setup(){
    Serial.begin(115200);
    pinMode(STEP1,OUTPUT); pinMode(DIR1,OUTPUT); pinMode(EN1,OUTPUT);
    pinMode(STEP2,OUTPUT); pinMode(DIR2,OUTPUT); pinMode(EN2,OUTPUT);

    pinMode(LIM1,INPUT_PULLUP);
    pinMode(LIM2,INPUT_PULLUP);

    pinMode(RELAY1,OUTPUT);
    pinMode(RELAY2,OUTPUT);
    pinMode(RELAY3,OUTPUT);
     
    digitalWrite(EN1,HIGH);
    digitalWrite(EN2,HIGH);
    

    // === MATIKAN SEMUA RELAY SAAT BOOT (AKTIF LOW) ===
    digitalWrite(RELAY1, LOW);  // Relay 1 (pompa) MATI
    digitalWrite(RELAY2, LOW);  // Relay 2 MATI //vent buka
    digitalWrite(RELAY3, HIGH);  // Relay 3 MATI //V1 tutup

    relayState[0] = true;
    relayState[1] = true;
    relayState[2] = false;

    // === MATIKAN SEMUA RELAY SAAT BOOT (AKTIF LOW) ===
    digitalWrite(RELAY1, HIGH);  // Relay 1 (pompa) MATI
    digitalWrite(RELAY2, HIGH);  // Relay 2 MATI
    digitalWrite(RELAY3, HIGH);  // Relay 3 MATI

    connectWiFi();
    wifiClient.setInsecure();
    client.setServer(mqttServer,mqttPort);
    client.setCallback(callback);
    connectMQTT();

    publishStatus();
    Serial.println("SYSTEM READY");
}

// ================= LOOP =================
void loop(){
    if(!client.connected()) connectMQTT();
    client.loop();
    handleStepper(s1,0,"Stepper1");
    handleStepper(s2,1,"Stepper2");
}
