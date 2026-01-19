import cv2              #OpenCV. 카메라 프레임 읽기, 이미지 전처리(리사이즈/색변환), 화면 출력 등에 사용
import mediapipe as mp  #얼굴/손/포즈 같은 “실시간 랜드마크(좌표)” 추출
import face_recognition #얼굴 인식(누구인지 식별) 라이브러리.
import serial           #UART 시리얼 통신
import time             #프레임 처리 루프 속도 조절
import numpy as np
import os
import struct
from PIL import Image
from multiprocessing import Process, Queue # ★ 핵심: 분신술 도구
화면 에러 방지
if "DISPLAY" not in os.environ:
os.environ["DISPLAY"] = ":0"
================= [설정 구역] =================
1. 파일 경로 (준비한 사진 이름과 똑같아야 함!)
PATH_BASE = "/home/user17/face_tracking/"
IMG_NORMAL = "normal.jpg" # 평소 (탐색 중)
IMG_HAPPY  = "happy.jpg"  # 주인 발견 (따라가기)
IMG_STOP   = "stop.jpg"   # 정지 (손바닥)
2. 통신 포트 설정
모터 (UART0 - GPIO 14/15)
MOTOR_PORT = '/dev/ttyS0'
MOTOR_BAUD = 115200
LCD (UART4 - GPIO 8/9)
LCD_PORT = '/dev/ttyAMA4'
LCD_BAUD = 460800
LCD_W, LCD_H = 240, 320
3. 주행 튜닝
SERVO_STEP = -1
CENTER_MIN = 87
CENTER_MAX = 93
FWD_SPEED_BASE = 100
TANK_TURN_SPEED = 100
DEADZONE = 10
TARGET_FACE_SIZE = 0.15
==============================================
---------------- [Process 2] LCD 담당 일꾼 (화가) ----------------
메인 프로세스와 별개로 돌아가서 렉을 유발하지 않음
def lcd_worker(cmd_q, ack_q):
try:
ser_lcd = serial.Serial(LCD_PORT, LCD_BAUD, timeout=1)
print("📺 [LCD] 연결 성공")
except:
print("❌ [LCD] 연결 실패 (포트 확인 필요)")
return
current_img_name = ""
while True:
우편함(Queue)에 새로운 "표정 명령"이 왔는지 확인
if not cmd_q.empty():
img_name = cmd_q.get() # 명령 꺼내기 ('normal', 'happy' 등)
중복된 표정이면 바로 끝났다고 보고하고 스킵
if img_name == current_img_name:
ack_q.put("DONE")
continue
current_img_name = img_name
full_path = os.path.join(PATH_BASE, img_name)
if os.path.exists(full_path):
try:
이미지 변환 (RGB888 -> RGB565)
img = Image.open(full_path).convert('RGB')
★ [수정] 평소 표정(IMG_NORMAL)일 때만 좌우 반전 시키기
if img_name == IMG_NORMAL:
img = img.transpose(Image.FLIP_TOP_BOTTOM)
img = img.resize((LCD_W, LCD_H))
pixel_data = list(img.getdata())
buffer = bytearray()
def to_rgb565(r, g, b):
return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3)
for r, g, b in pixel_data:
buffer.extend(struct.pack('>H', to_rgb565(r, g, b)))
FPGA 전송
ser_lcd.reset_output_buffer()
time.sleep(0.02) # 안정화 대기
ser_lcd.write(buffer)
ser_lcd.flush()
print(f"📺 [LCD] 표정 변경 완료: {img_name}")
except Exception as e:
print(f"⚠️ [LCD] 전송 중 에러: {e}")
else:
print(f"❌ [LCD] 파일 없음: {full_path}")
★ 핵심: 전송 다 했다고 보고!
ack_q.put("DONE")
time.sleep(0.05) # CPU 과부하 방지용 휴식
---------------- [Process 1] 메인 두뇌 (모터 & 판단) ----------------
def main():
큐 2개 생성 (명령용, 보고용)
lcd_cmd_q = Queue()
lcd_ack_q = Queue()
p = Process(target=lcd_worker, args=(lcd_cmd_q, lcd_ack_q))
p.start()
1. 모터 UART 연결
try:
ser_motor = serial.Serial(MOTOR_PORT, MOTOR_BAUD, timeout=0.01)
print("✅ [Main] 모터 연결 성공")
except:
ser_motor = None
print("⚠️ [Main] 모터 UART 없음 (테스트 모드)")
2. AI 모델 로드
mp_face = mp.solutions.face_detection.FaceDetection(model_selection=0, min_detection_confidence=0.5)
mp_hands = mp.solutions.hands.Hands(max_num_hands=1, min_detection_confidence=0.7)
★★★ [추가 1] 손 관절 그리기 도구 & FPS 시간 변수 ★★★
mp_drawing = mp.solutions.drawing_utils
pTime = 0 # 이전 시간 저장용
is_owner = False
cmd_mode = "STOP"
last_emotion_cmd = "" # 마지막으로 내린 표정 명령
★ [추가] 얼굴 놓침 카운트 변수 (깜빡임 방지용)
loss_cnt = 0
3. 카메라 시작
cap = cv2.VideoCapture(0)
cap.set(3, 320)
cap.set(4, 240)
servo_angle = 90
초기 표정 설정 (시작하자마자 잠깐 멈출 수 있음)
lcd_cmd_q.put(IMG_NORMAL)
초기화 때는 굳이 대기 안해도 됨 (루프 진입 전이므로)
last_emotion_cmd = IMG_NORMAL
print("🚀 로봇 가동 시작! (종료: q)")
while cap.isOpened():
ret, frame = cap.read()
if not ret: break
★★★ [추가 2] FPS 계산 로직 ★★★
cTime = time.time()
fps = 1 / (cTime - pTime) if (cTime - pTime) > 0 else 0
pTime = cTime
frame = cv2.flip(frame, 1)
h, w, c = frame.shape
center_x = w // 2
rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
모터 명령 초기화 (정지 상태)
l_speed = 0
r_speed = 0
dir_bit = 0
--- 상황 판단 로직 ---
target_emotion = last_emotion_cmd # 기본 표정
[1] 손 인식 (STOP 명령)
results_hand = mp_hands.process(rgb)
if is_owner and results_hand.multi_hand_landmarks:
for hand_lms in results_hand.multi_hand_landmarks:
★★★ [추가 3] 화면에 손 21개 관절 그리기 ★★★
mp_drawing.draw_landmarks(frame, hand_lms, mp.solutions.hands.HAND_CONNECTIONS)
fingers = []
손가락 펴짐 감지
for id in [8, 12, 16, 20]:
if hand_lms.landmark[id].y < hand_lms.landmark[id-2].y: fingers.append(1)
else: fingers.append(0)
손바닥(3개 이상) 펴면 정지
if sum(fingers) >= 3:
cmd_mode = "STOP"
target_emotion = IMG_STOP # ★ 정지 표정
else:
cmd_mode = "FOLLOW"
[2] 얼굴 인식
results_face = mp_face.process(rgb)
dist_score = 0
if results_face.detections:
★ 얼굴 찾았으면 카운트 0으로 초기화
loss_cnt = 0
target = max(results_face.detections, key=lambda d: d.location_data.relative_bounding_box.width)
bbox = target.location_data.relative_bounding_box
x = int(bbox.xmin * w); bw = int(bbox.width * w)
face_cx = x + bw // 2
face_size_ratio = bbox.width
(A) 목(Servo) 제어
err_x = face_cx - center_x
if abs(err_x) > 10:
if err_x > 0: servo_angle -= SERVO_STEP
else:         servo_angle += SERVO_STEP
servo_angle = max(0, min(180, servo_angle))
(B) 몸통 회전 방향 판단
turn_action = 0
if servo_angle < CENTER_MIN:  turn_action = -1 # 좌회전 필요
elif servo_angle > CENTER_MAX: turn_action = 1  # 우회전 필요
dist_score = int(100 / bbox.width) # 거리 추정
... (몸통 제어 로직 끝난 후) ...
★★★ [추가 4] 주인 여부에 따른 박스 색상 & 그리기 ★★★
주인이면 초록(0, 255, 0), 아니면 빨강(0, 0, 255)
box_color = (0, 255, 0) if is_owner else (0, 0, 255)
얼굴 박스 그리기
cv2.rectangle(frame, (x, int(bbox.ymin*h)), (x+bw, int((bbox.ymin+bbox.height)*h)), box_color, 2)
화면에 상태 텍스트 띄우기 (이름 표시)
name_text = "Kong Hyungkon" if is_owner else "Unknown"
cv2.putText(frame, name_text, (x, int(bbox.ymin*h)-10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, box_color, 2)
(C) 주인님 따라가기 로직
if is_owner and cmd_mode == "FOLLOW":
target_emotion = IMG_HAPPY # ★ 따라갈 땐 행복 표정
if turn_action != 0: # 1. 각도 안 맞으면 제자리 회전
l_speed = TANK_TURN_SPEED
r_speed = TANK_TURN_SPEED
dir_bit = 10 if turn_action == -1 else 5
elif face_size_ratio < TARGET_FACE_SIZE: # 2. 각도 맞고 멀면 전진
l_speed = FWD_SPEED_BASE
r_speed = FWD_SPEED_BASE
dir_bit = 9
else: # 3. 가까우면 정지
l_speed = 0; r_speed = 0; dir_bit = 0
else:
주인 아니거나 STOP 모드 -> 제자리에서 쳐다보기만 함
if turn_action == -1: l_speed=100; r_speed=100; dir_bit=10
elif turn_action == 1: l_speed=100; r_speed=100; dir_bit=51
else: l_speed=0; r_speed=0; dir_bit=0
★ [수정 3] 얼굴을 못 찾았을 때 (else 구문 추가)
else:
loss_cnt += 1 # 못 본 시간 1 증가
얼굴이 약 0.5초(15프레임) 이상 안 보일 때만 NORMAL로 변경
if loss_cnt > 15:
target_emotion = IMG_NORMAL
l_speed = 0; r_speed = 0 # 안 보이면 멈춤
else:
잠깐(0.5초 이내) 안 보인 거면, 이전 표정(HAPPY) 유지!
pass
=================================================================
★★★ [핵심 변경] 표정이 바뀔 때는 모든 걸 멈추고 대기한다 ★★★
=================================================================
if target_emotion != last_emotion_cmd:
print(f"\n🔄 표정 변경 중... ({target_emotion}) 대기하세요.")
1. LCD 담당자에게 명령 전달
lcd_cmd_q.put(target_emotion)
last_emotion_cmd = target_emotion
2. 로봇 강제 정지 명령 전송 (대기하는 동안 움직이면 안 되니까)
if ser_motor:
속도 0 패킷 전송
stop_pkt = bytearray([0xFF, int(servo_angle), 0, 0, 0])
ser_motor.write(stop_pkt)
3. 완료 신호(ACK)가 올 때까지 무한 대기 루프 (Blocking)
wait_start = time.time()
while True:
혹시 모르니 계속 정지 신호 보냄
if ser_motor: ser_motor.write(bytearray([0xFF, int(servo_angle), 0, 0, 0]))
LCD 담당자가 "다 했어요(DONE)"라고 신호를 보냈는지 확인
if not lcd_ack_q.empty():
_ = lcd_ack_q.get() # 신호 확인 및 큐 비우기
print("✅ 표정 변경 완료! 다시 움직입니다.")
break # 대기 루프 탈출 -> 다시 메인 루프로 복귀
화면이 멈추면 답답하니까 "변경 중" 텍스트 띄우며 화면 갱신은 유지
(하지만 카메라는 새로 읽지 않고 마지막 프레임만 유지하거나,
새로 읽어도 처리는 안 함)
cv2.putText(frame, "Changing Face...", (center_x-50, h//2), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 3)
cv2.imshow("Robot Eye", frame)
if cv2.waitKey(1) & 0xFF == ord('q'):
cap.release(); cv2.destroyAllWindows(); return # 강제 종료 지원
--- 모터 명령 전송 (UART0) ---
if ser_motor:
packet = bytearray([0xFF, int(servo_angle), int(l_speed), int(r_speed), int(dir_bit)])
ser_motor.write(packet)
★★★ [추가 5] 터미널에 상태 정보 출력 ★★★
\r을 쓰면 줄바꿈 없이 한 줄에서 숫자가 바뀜 (보기 깔끔함)
print(f"\rFPS: {int(fps)} | Mode: {cmd_mode} | Angle: {servo_angle} | Dist: {dist_score} | Owner: {is_owner}", end="")
--- 화면 디버깅 ---
status_text = f"Mode: {cmd_mode}"
if not is_owner: status_text = "Who are you?"
cv2.putText(frame, status_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
cv2.imshow("Robot Eye", frame)
key = cv2.waitKey(1) & 0xFF
if key == ord('q'): break
elif key == ord('1'): # 주인 등록 키
if not is_owner:
try:
encs = face_recognition.face_encodings(rgb)
if len(encs) > 0:
is_owner = True
print("🎉 주인님(Kong Hyungkon) 등록 완료!")
except: pass
종료 처리
if ser_motor: ser_motor.close()
p.terminate() # 일꾼 퇴근시키기 (중요!)
cap.release()
cv2.destroyAllWindows()
if name == 'main':
main()
