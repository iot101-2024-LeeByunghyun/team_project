import paho.mqtt.client as mqtt

# 토픽 설정
subscribe_tag_topic = "id/yourname/parking/tag"  # ESP32에서 태그 정보를 수신할 토픽
subscribe_distance_topic = "id/yourname/parking/distance"  # ESP32에서 거리 정보를 수신할 토픽
publish_servo_topic = "id/yourname/servo/control"  # ESP32로 서보 모터 제어 명령을 보낼 토픽
server = "192.168.0.248"

# 태그 상태 저장 변수
tag_present = False

# MQTT 연결 시 콜백
def on_connect(client, userdata, flags, rc):
    print("Connected with result code: " + str(rc))
    # 토픽 구독
    client.subscribe(subscribe_tag_topic)
    client.subscribe(subscribe_distance_topic)

# 메시지 수신 시 콜백
def on_message(client, userdata, msg):
    global tag_present  # 태그 상태를 전역 변수로 사용

    topic = msg.topic
    payload = msg.payload.decode('utf-8')
    print(f"Received message from {topic}: {payload}")

    # 태그 상태 처리
    if topic == subscribe_tag_topic:
        if payload == "tag":
            print("Tag detected. Distance processing enabled.")
            tag_present = True  # 태그가 감지되었음을 설정
        elif payload == "untag":
            print("Tag removed. Distance processing disabled.")
            tag_present = False  # 태그가 제거되었음을 설정

    # 거리 값 처리 (태그 상태가 "tag"일 때만 실행)
    elif topic == subscribe_distance_topic and tag_present:
        try:
            distance = float(payload)
            if distance <= 8.0:
                print(f"Distance {distance}cm is within range. Sending servo ON command.")
                client.publish(publish_servo_topic, "on")  # 서보 제어 명령 전송
            else:
                print(f"Distance {distance}cm is out of range. Sending servo OFF command.")
                client.publish(publish_servo_topic, "off")  # 서보 제어 명령 전송
        except ValueError:
            print(f"Invalid distance value received: {payload}")

# MQTT 클라이언트 설정
client = mqtt.Client()
client.on_connect = on_connect
client.on_message = on_message

# MQTT 서버 연결
client.connect(server, 1883, 60)
client.loop_forever()