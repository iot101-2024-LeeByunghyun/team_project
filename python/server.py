import paho.mqtt.client as mqtt
import sys
import time


# 토픽 설정
# 이병현
subscribe_tag_topic = "id/semicon/yard/RFID/tag"  # ESP32에서 태그 정보를 수신할 토픽
subscribe_distance_topic = "id/semicon/yard/ultrasonic/distance"  # ESP32에서 거리 정보를 수신할 토픽
publish_servo_topic = "id/semicon/door/servo/control"  # ESP32로 서보 모터 제어 명령을 보낼 토픽
subscribe_press_btn_topic = "id/semicon/door/button/info"  # ESP32로 버튼 정보를 수신할 토픽

# 박종현
humidity_topic = "id/semicon/bathroom/dht22/humidity"
temperature_topic = "id/semicon/bathroom/dht22/temperature"
motor_topic = "id/semicon/bathroom/motor/control" # 욕실 모터 제어 토픽

# 박종하
publish_NeoPixel_topic= "id/semicon/yard/NeoPixel/control" # ESP32로 네오 픽셀 door/box 명령을 보낼 토픽
subscribe_press_topic= "id/semicon/yard/fsr402/press" # ESP32로 압력 정보를 수신할 토픽


# 이지훈
topic_temp = "id/semicon/livingroom/dht22/temperature"  # 온도 데이터 토픽
topic_humi = "id/semicon/livingroom/dht22/humidity"  # 습도 데이터 토픽
topic_lux = "id/semicon/livingroom/bh1750/lux"  # 조도 데이터 토픽
topic_stepper = "id/semicon/livingroom/step/control"  # 스텝모터 제어 토픽

previous_state = None

server = "192.168.0.248"

# 태그 상태 저장 변수
tag_present = False

distance = 600
hit_distance = 12.0      # 사람 감지 거리
humidity_threshold = 36 # 습도 임계값
# 버튼 동작 이후 초음파 감지 무시 시간을 설정
btn = ""
debounce_time = 6  # 초 단위
last_exit_time = time.time() # 마지막으로 나간 시간

humid_living = 100

# 출입 상태 저장 변수
human_last = False
enter_present = False
human_present = True
enter_mode = False

# MQTT 연결 시 콜백
def on_connect(client, userdata, flags, rc):
    print("Connected with result code: " + str(rc))
    # 토픽 구독
    # 이병현
    client.subscribe(subscribe_tag_topic)
    client.subscribe(subscribe_distance_topic)
    client.subscribe(subscribe_press_btn_topic)
    # 박종현
    client.subscribe(humidity_topic)
    client.subscribe(temperature_topic)
    # 박종하
    client.subscribe(subscribe_press_topic)
    # 이지훈
    client.subscribe(topic_temp)
    client.subscribe(topic_humi)
    client.subscribe(topic_lux)


# 메시지 수신 시 콜백
def on_message(client, userdata, msg):
    global tag_present  # 태그 상태를 전역 변수로 사용
    global enter_present  # 출입 상태를 전역 변수로 사용
    global distance  # 거리 정보를 전역 변수로 사용
    global human_last # 이전 출입 상태를 전역 변수로 사용
    global human_present # 현재 출입 상태를 전역 변수로 사용4
    global btn # 버튼 정보를 전역 변수로 사용
    global last_exit_time # 마지막으로 나간 시간을 전역 변수로 사용
    global humid_bath # 욕실 습도를 전역 변수로 사용
    global weight # 무게 정보를 전역 변수로 사용
    global previous_state   # 이전 상태를 전역 변수로 사용
    global humid_living # 거실 습도를 전역 변수로 사용
    global enter_mode

    topic = msg.topic
    payload = msg.payload.decode('utf-8')
    print(f"Received message from {topic}: {payload}")

    # 태그 상태 처리
    if topic == subscribe_tag_topic:
        if payload == "tag":
            tag_present = True  # 태그가 감지 플래그
        elif payload == "untag":
            tag_present = False  # 태그가 제거 플래그
            enter_mode = False  # 사람이 나감

    # 출입 상태 처리
    if tag_present == 1:    # 태그 상태에서만
        if topic == subscribe_distance_topic or topic == subscribe_press_btn_topic:
            if topic == subscribe_distance_topic:
                distance = float(payload)
            if topic == subscribe_press_btn_topic:
                btn = payload

            if not enter_present and time.time() - last_exit_time > debounce_time:
                if distance <= hit_distance:
                    print("출입 가능")
                    client.publish(publish_servo_topic, "on")   # 서보 제어 명령 전송
                    enter_present = True    # 사람이 들어감
                    enter_mode = True      # 들어간 상태
                else:
                    print("출입 불가")
                    client.publish(publish_servo_topic, "off")   # 서보 제어 명령 전송
                
            elif enter_present:
                if btn == "press":
                    print("출입 가능")
                    client.publish(publish_servo_topic, "on")   # 서보 제어 명령 전송
                    last_exit_time = time.time()                # 마지막 나간 시간 기록
                    enter_present = False   # 사람이 나감
                else:
                    if distance >= hit_distance:
                        print("출입 불가")
                        client.publish(publish_servo_topic, "off")  # 서보 제어 명령 전송


    if tag_present == 0 and enter_present == 0:
        client.publish(motor_topic, "OFF")

    elif tag_present == 1:
        if msg.topic == topic_temp:
            print(f"Temperature: {msg.payload.decode('utf-8')}°C")
        elif msg.topic == topic_humi:
            humid_living = float(msg.payload.decode('utf-8'))
        elif msg.topic == topic_lux:
            lux_value = float(msg.payload.decode('utf-8'))  # 조도 값
            print(f"Light Intensity: {lux_value} lux")

        # Lux 값에 따라 스텝모터 제어 명령 전송, 이전 상태와 비교
            if lux_value >= 400 and previous_state != "up": # 조도 값이 400 이상이면 스텝모터를 위로 회전
                print("Lux >= 400: Publishing 'up' to stepper topic.")
                client.publish(topic_stepper, "up")
                previous_state = "up"
            elif lux_value < 400 and previous_state != "down":
                print("Lux < 400: Publishing 'down' to stepper topic.")
                client.publish(topic_stepper, "down")
                previous_state = "down"
        else:
            print(f"Unknown topic {msg.topic}: {msg.payload.decode('utf-8')}")



        if msg.topic == humidity_topic:
            humid_bath = float(msg.payload.decode('utf-8'))
            if humid_bath >= humid_living:
                print("Humidity exceeded threshold. Sending motor ON command.")
                client.publish(motor_topic, "ON")
            else:
                print("Humidity below threshold. Sending motor OFF command.")
                client.publish(motor_topic, "OFF")
    
        elif msg.topic == temperature_topic:
            print("Temperature received but no specific action defined.")

        if enter_present == 0 and enter_mode == 0:

            if topic == subscribe_press_topic:
                weight= float(payload)

                if weight > 100:
                    print(f"Weight : {weight}, Sending box NeoPixel command.")
                    client.publish(publish_NeoPixel_topic, "box")  # box 네오픽셀 작동
                else:
                    print(f"Weight : {weight}, Sending door NeoPixel command.")
                    client.publish(publish_NeoPixel_topic, "door")  # door 네오픽셀 작동
            
        
        else:
            print(f"DISTANCE: {distance}, ENTER: {enter_present}")
            print("Sending shutdown NeoPixel command.")
            client.publish(publish_NeoPixel_topic, "shutdown")  # 네오픽셀 shutdown


      

# MQTT 클라이언트 설정
client = mqtt.Client()
client.on_connect = on_connect
client.on_message = on_message

# MQTT 서버 연결
client.connect(server, 1883, 60)
client.loop_forever()