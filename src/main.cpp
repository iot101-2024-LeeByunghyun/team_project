#include <Arduino.h>
#include <SPI.h>
#include <MFRC522.h>
#include <ConfigPortal32.h>
#include <PubSubClient.h>
#include <ESP32Servo.h>

char*               ssid_pfix = (char*)"parking_ESP";
String              user_config_html = ""
    "<p><input type='text' name='broker' placeholder='MQTT Server'>";

char                mqttServer[100];
 
// 핀 설정
#define RST_PIN 22 // Reset 핀
#define SS_PIN 21  // SDA 핀

const int trigPin = 4; // Trig 핀
const int echoPin = 5; // Echo 핀
const int servoPin = 2; // 서보모터 핀

float dist = 300.0; // 초음파 센서 거리
float distanceValue;
int angle;
int currentAngle = 0;

MFRC522 rfid(SS_PIN, RST_PIN); // RFID 객체 생성

bool tagPresent = false; // 태그가 현재 감지된 상태인지 여부
String lastTagUID = "";  // 마지막으로 감지된 UID
unsigned long tagLastSeen = 0; // 태그가 마지막으로 감지된 시간
const unsigned long TAG_REMOVAL_DELAY = 600; // 태그 제거를 확정하는 대기 시간(ms)

const int           mqttPort = 1883;

unsigned long       interval = 1000;
unsigned long       lastPublished = - interval;
 
WiFiClient wifiClient;
PubSubClient client(wifiClient);
void msgCB(char* topic, byte* payload, unsigned int length); // message Callback
void pubStatus(float dist);
Servo myServo;


void loop() {
    long duration;
    
    client.loop();
    
    unsigned long currentMillis = millis();
    if(currentMillis - lastPublished >= interval) {
        lastPublished = currentMillis;
        pubStatus(dist);
    }

    // 새로운 태그가 감지되었는지 확인
    if (rfid.PICC_IsNewCardPresent() && rfid.PICC_ReadCardSerial()) {
        String currentUID = "";

        // UID를 문자열로 저장
        for (byte i = 0; i < rfid.uid.size; i++) {
            currentUID += String(rfid.uid.uidByte[i], HEX);
        }

        // 새로운 태그 감지 또는 기존 태그 유지 중
        if (!tagPresent || currentUID != lastTagUID) {
            Serial.println("새로운 태그 감지!");
            Serial.print("UID: ");
            Serial.println(currentUID);
            lastTagUID = currentUID;
            tagPresent = true; // 태그 감지 상태로 설정
        }

        // 태그가 감지된 마지막 시간을 갱신
        tagLastSeen = millis();
        
    } else {
        // 태그 감지 이후 일정 시간 동안 신호 없음 -> 태그 제거로 간주
        if (tagPresent && (millis() - tagLastSeen > TAG_REMOVAL_DELAY)) {
            Serial.println("태그가 제거되었습니다.");
            tagPresent = false; // 태그 상태 초기화
            lastTagUID = "";    // UID 초기화
        }
    }

    // 초음파/서보모터 코드 추가
    

    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);

    duration = pulseIn(echoPin, HIGH);
    dist = duration * 0.034 / 2;

    delay(50); // 루프 딜레이
}

void setup() {
    Serial.begin(115200);
    SPI.begin();             // SPI 초기화
    rfid.PCD_Init();         // RFID 초기화
    Serial.println("RFID 리더 초기화 완료");

    // 핀 모드 설정
    pinMode(trigPin, OUTPUT);
    pinMode(echoPin, INPUT);

    // 서보모터 초기화
    myServo.attach(servoPin);
    myServo.write(0); // 초기 각도: 90도 (중앙)

    Serial.begin(115200); // 디버깅용 시리얼 통신

    loadConfig();
    // *** If no "config" is found or "config" is not "done", run configDevice ***
    if(!cfg.containsKey("config") || strcmp((const char*)cfg["config"], "done")) {
        configDevice();
    }
    WiFi.mode(WIFI_STA);
    WiFi.begin((const char*)cfg["ssid"], (const char*)cfg["w_pw"]);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    // main setup
    Serial.printf("\nIP address : "); Serial.println(WiFi.localIP());

    if (cfg.containsKey("broker")) {
            sprintf(mqttServer, (const char*)cfg["broker"]);
    }
    client.setServer(mqttServer, mqttPort);
    client.setCallback(msgCB);

    while (!client.connected()) {
        Serial.println("Connecting to MQTT...");
        if (client.connect("ESP32Parking")) {
            Serial.println("connected");  
        } else {
            Serial.print("failed with state "); Serial.println(client.state());
            delay(2000);
        }
    }
 
    client.subscribe("id/yourname/parking/tag");
    client.subscribe("id/yourname/parking/distance");
    client.subscribe("id/yourname/servo/control");
}

void pubStatus(float dist) {
    char tag[10];
    char distance[10];
    if (tagPresent) {
        sprintf(tag, "tag");
    } else {
        sprintf(tag, "untag");
    }

    if (dist >0 && dist < 400) {
        sprintf(distance, "%.2f", dist);    // 소수점 두 자리까지
    } else {
        sprintf(distance, "600.0");       // 유효하지 않은 값
    }
    client.publish("id/yourname/parking/tag", tag);
    client.publish("id/yourname/parking/distance", distance);
}

void msgCB(char* topic, byte* payload, unsigned int length) {
 
    char msgBuffer[20];
    for(int i = 0; i < (int)length; i++) {
        msgBuffer[i] = payload[i];
    } 
    msgBuffer[length] = '\0';
    Serial.printf("\n%s -> %s", topic, msgBuffer);
    

    // 수신된 토픽에 따라 동작 처리
    if (!strcmp(topic, "id/yourname/servo/control")) {
        if (!strcmp(msgBuffer, "on")) {
            if (currentAngle != 90) { // 현재 각도가 90도가 아니라면 이동
                for (int angle = currentAngle; angle <= 90; angle++) {
                    myServo.write(angle); // 각도 변경
                    delay(10);            // 각도 변경 간 대기 시간(속도 제어)
                }
                currentAngle = 90; // 현재 각도를 90도로 설정

            }
        } else if (!strcmp(msgBuffer, "off")) {
            if (currentAngle != 0) { // 현재 각도가 0도가 아니라면 이동
                for (int angle = currentAngle; angle >= 0; angle--) {
                    myServo.write(angle); // 각도 변경
                    delay(10);            // 각도 변경 간 대기 시간(속도 제어)
                }
                currentAngle = 0; // 현재 각도를 0도로 설정
            }
        }
    }
}