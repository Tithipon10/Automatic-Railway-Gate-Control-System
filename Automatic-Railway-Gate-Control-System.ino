#define BLYNK_PRINT Serial

#define BLYNK_TEMPLATE_ID "TMPL6ZcESYSNX"
#define BLYNK_TEMPLATE_NAME "Railway Gateway"
#define BLYNK_AUTH_TOKEN "rQ5mRaATS8pQSrRMHv0HnoIOBOc6JuZC"

#include <Servo.h>
#include <ESP8266WiFi.h>
#include <BlynkSimpleEsp8266.h>
#include <PubSubClient.h>

// NETPIE Configuration
const char* publish_topic = "@msg/railway_status";
const char* ssid = "Username";
const char* password = "Password";
const char* mqtt_server = "mqtt.netpie.io";
const int mqtt_port = 1883;
const char* mqtt_Client = "59095768-6d0a-4df4-b690-d94d6c3f57f7";  // เปลี่ยนเป็น Client ID ของคุณ
const char* mqtt_username = "8PwPk8VfD6CohzX27MFFqAgijrvdh7AZ";    // เปลี่ยนเป็น Token ของคุณ
const char* mqtt_password = "4eZZxYgEiC8wK5kGpqYAZAdT76Mb73u1";    // เปลี่ยนเป็น Secret ของคุณ

WiFiClient espClient;
PubSubClient client(espClient);

// อุปกรณ์และพิน
Servo myservo1;
Servo myservo2;

const int ledPinG = D1;
const int ledPinR = D2;
const int irSensorPin1 = D4;
const int irSensorPin2 = D5;
const int buzzPin = D6;
const int servoPin1 = D7;
const int servoPin2 = D8;

// ตัวแปรสถานะ
bool isTrainRunning = false;
bool prevTrainRunning = false;
bool isReversedMode = false;
bool prevReversedMode = false;
int trainPassCount = 0;
int prevTrainPassCount = 0;

unsigned long lastMsg = 0;
const long interval = 5000;  // ส่งข้อมูลทุก n วินาที

// ไม้กั้น
bool isClose = false;
bool prevIsClose = false;

// สำหรับคำนวณความเร็ว
unsigned long timeA = 0, timeB = 0;
const float distance_m = 0.52;  // ระยะทางระหว่างเซนเซอร์ (เมตร)
float trainSpeed = 0.0;
float prevTrainSpeed = 0.0;

// เสียงเตือน
unsigned long lastAlarmTime = 0;
const int ALARM_INTERVAL = 400;
bool isHighTone = true;

// Virtual Pins
#define VP_GATE1_CONTROL V0
#define VP_GATE2_CONTROL V1
#define VP_GATE1_STATUS V2
#define VP_GATE2_STATUS V3
#define VP_IR1_STATUS V4
#define VP_IR2_STATUS V5
#define VP_DIRECTION V6
#define VP_TRAIN_COUNT V7

BLYNK_CONNECTED() {
  Serial.println("Blynk Connected!");
  Blynk.virtualWrite(VP_GATE1_STATUS, myservo1.read() == 0 ? 1 : 0);
  Blynk.virtualWrite(VP_TRAIN_COUNT, trainPassCount);
}

BLYNK_WRITE(VP_GATE1_CONTROL) {
  int value = param.asInt();
  if (value == 1) {
    closeGate();
    isTrainRunning = true;
    Serial.println("Close the Gate immediately by the operator");
    sendToNETPIE();
  } else {
    openGate();
    isTrainRunning = false;
    Serial.println("Open the Gate immediately by the operator");
    sendToNETPIE();
  }
}

BLYNK_WRITE(VP_DIRECTION) {
  bool newMode = param.asInt();
  if (newMode != isReversedMode) {
    isReversedMode = newMode;
    sendToNETPIE();
  }
}



bool checkStatusChange() {
  if (prevTrainRunning != isTrainRunning ||
      prevReversedMode != isReversedMode ||
      prevTrainPassCount != trainPassCount ||
      prevIsClose != isClose ||
      abs(prevTrainSpeed - trainSpeed) > 0.1) {  // ตรวจสอบความเร็วที่เปลี่ยนไปเกิน 0.1
    
    // อัพเดทค่าก่อนหน้า
    prevTrainRunning = isTrainRunning;
    prevReversedMode = isReversedMode;
    prevTrainPassCount = trainPassCount;
    prevIsClose = isClose;
    prevTrainSpeed = trainSpeed;
    
    return true;  // มีการเปลี่ยนแปลง
  }
  return false;  // ไม่มีการเปลี่ยนแปลง
}

// ฟังก์ชันส่งข้อมูลไป NETPIE
void sendToNETPIE() {
  // ส่งข้อมูลไปที่ shadow data เหมือนเดิม
  String shadowData = "{\"data\":{";
  shadowData += "\"train_count\":" + String(trainPassCount) + ",";
  shadowData += "\"gate_status\":\"" + String(isClose ? "closed" : "open") + "\",";
  shadowData += "\"direction\":\"" + String(isReversedMode ? "reverse" : "normal") + "\",";
  shadowData += "\"train_detected\":" + String(isTrainRunning ? "true" : "false") + ","; 
  shadowData += "\"train_speed\":" + String(trainSpeed);
  shadowData += "}}";

  char shadowMsg[300];
  shadowData.toCharArray(shadowMsg, (shadowData.length() + 1));
  client.publish("@shadow/data/update", shadowMsg);

  String statusMsg = "Status update: ";
  statusMsg += isTrainRunning ? "Train detected, " : "No train, ";
  statusMsg += "Gate " + String(isClose ? "closed" : "open") + ", ";
  statusMsg += "Direction: " + String(isReversedMode ? "reverse" : "normal") + ", ";
  statusMsg += "Train count: " + String(trainPassCount);
  
  char msg[300];
  statusMsg.toCharArray(msg, (statusMsg.length() + 1));
  client.publish(publish_topic, msg);


}

void reconnect() {
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    if (client.connect(mqtt_Client, mqtt_username, mqtt_password)) {
      Serial.println("connected");
      client.subscribe("@msg/#");
      client.subscribe("@msg/command");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      delay(5000);
    }
  }
}

void setup() {
  Serial.begin(115200);

  // ตั้งค่า Servo และ GPIO
  myservo1.attach(servoPin1);
  myservo2.attach(servoPin2);
  myservo1.write(90);
  myservo2.write(90);
  pinMode(ledPinG, OUTPUT);
  pinMode(ledPinR, OUTPUT);
  pinMode(buzzPin, OUTPUT);
  pinMode(irSensorPin1, INPUT);
  pinMode(irSensorPin2, INPUT);

  // เชื่อมต่อ WiFi
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

  // ตั้งค่า MQTT
  client.setServer(mqtt_server, mqtt_port);
  Blynk.begin(BLYNK_AUTH_TOKEN, ssid, password);

  sendToNETPIE();
}

void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  Blynk.run();

  // เก็บค่าสถานะปัจจุบันก่อนตรวจสอบ
  bool prevTrainDetected = isTrainRunning;
  bool prevGateStatus = isClose;
  int prevCount = trainPassCount;

  // ตรวจสอบการทำงานของเซนเซอร์ในแต่ละโหมด
  if (!isReversedMode) {
    checkNormalMode();
  } else {
    checkReversedMode();
  }
  lightStatus();
  useAlarm();

  if (checkStatusChange()) {
    sendToNETPIE();
    Serial.println("Status changed - Immediate update sent");
  }
  // ส่งข้อมูลไป NETPIE ทุก 5 วินาที
  unsigned long now = millis();
  if (now - lastMsg > interval) {
    lastMsg = now;
    sendToNETPIE();
  }
}

void checkNormalMode() {
  // เมื่อเซนเซอร์ 1 ตรวจจับรถไฟและยังไม่ได้ตรวจจับอยู่
  if (digitalRead(irSensorPin1) == 0 && !isTrainRunning) {
    isTrainRunning = true;
    timeA = millis();  // เริ่มบันทึกเวลา
    closeGate();
    Serial.println("Train detected at Sensor 1");
    sendToNETPIE();
  }

  // เมื่อเซนเซอร์ 2 ตรวจจับว่ารถไฟผ่านแล้ว
  if (digitalRead(irSensorPin2) == 0 && isTrainRunning) {
    isTrainRunning = false;
    timeB = millis();  // จบบันทึกเวลา
    trainPassCount++;
    calculateSpeed(); 
    openGate();
    Serial.println("Train passed - Count: " + String(trainPassCount));
    sendToNETPIE();
  }
}

void checkReversedMode() {
  // เมื่อเซนเซอร์ 2 ตรวจจับรถไฟในโหมด reverse
  if (digitalRead(irSensorPin2) == 0 && !isTrainRunning) {
    isTrainRunning = true;
    timeA = millis();  // เริ่มบันทึกเวลา
    closeGate();
    Serial.println("Train detected at Sensor 2 (Reversed mode)");
    sendToNETPIE();
  }

  // เมื่อเซนเซอร์ 1 ตรวจจับว่ารถไฟผ่านแล้วในโหมด reverse
  if (digitalRead(irSensorPin1) == 0 && isTrainRunning) {
    isTrainRunning = false;
    timeB = millis();  // จบบันทึกเวลา
    trainPassCount++;
    calculateSpeed();
    openGate();
    Serial.println("Train passed (Reversed mode) - Count: " + String(trainPassCount));
    sendToNETPIE();
  }
}

void calculateSpeed() {
  if (timeA > 0 && timeB > timeA) {
    float time_sec = (timeB - timeA) / 1000.0;
    trainSpeed = (distance_m / time_sec) * 3.6;
    Serial.print("Train Speed: ");
    Serial.print(trainSpeed);
    Serial.println(" km/h");
  }
}



void lightStatus() {
  if (!isClose) {
    digitalWrite(ledPinR, LOW);
    digitalWrite(ledPinG, HIGH);
  } else {
    digitalWrite(ledPinR, HIGH);
    digitalWrite(ledPinG, LOW);
  }
}

void closeGate() {
  myservo1.write(0);
  myservo2.write(0);
  isClose = true;
}

void openGate() {
  myservo1.write(90);
  myservo2.write(90);
  isClose = false;
  noTone(buzzPin);
}

void useAlarm() {
  if (isTrainRunning) {
    unsigned long currentTime = millis();
    if (currentTime - lastAlarmTime >= ALARM_INTERVAL) {
      if (isHighTone) {
        tone(buzzPin, 430);
      } else {
        tone(buzzPin, 350);
      }
      isHighTone = !isHighTone;
      lastAlarmTime = currentTime;
    }
  }
}
