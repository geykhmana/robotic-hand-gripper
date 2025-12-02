import cv2
import mediapipe as mp
import serial
import time

# ----------------------------------------
# 1. SERIAL SETUP (WINDOWS)
# ----------------------------------------

# Change COM6 to whatever your Arduino shows in Device Manager
try:
    arduino = serial.Serial('COM6', 115200, timeout=1)
    print("Connected to Arduino on COM6")
except:
    print("ERROR: Could not open COM6")
    exit()

time.sleep(2)   # Allow Arduino to reset

def send_to_arduino(angle):
    """Send angle as bytes + ';' delimiter."""
    arduino.write(bytes(str(int(angle)), "utf-8"))
    arduino.write(b';')
    print("Sent to Arduino:", angle)


# ----------------------------------------
# 2. HAND TRACKING SETUP
# ----------------------------------------

mp_hands = mp.solutions.hands
hands = mp_hands.Hands(
    max_num_hands=1,
    min_detection_confidence=0.7,
    min_tracking_confidence=0.7
)
mp_draw = mp.solutions.drawing_utils


def count_fingers(hand_landmarks):
    finger_tip_ids = [8, 12, 16, 20]
    count = 0

    # Four fingers (not thumb)
    for tip_id in finger_tip_ids:
        tip = hand_landmarks.landmark[tip_id]
        pip = hand_landmarks.landmark[tip_id - 2]
        if tip.y < pip.y:
            count += 1

    # Thumb
    thumb_tip = hand_landmarks.landmark[4]
    thumb_ip = hand_landmarks.landmark[3]
    if thumb_tip.x < thumb_ip.x:
        count += 1

    return count


# ----------------------------------------
# 3. START VIDEO LOOP
# ----------------------------------------

cap = cv2.VideoCapture(0)
last_sent_angle = None

while True:
    success, frame = cap.read()
    if not success:
        break

    frame = cv2.flip(frame, 1)
    rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    results = hands.process(rgb_frame)

    if results.multi_hand_landmarks:
        for hand_landmarks in results.multi_hand_landmarks:
            mp_draw.draw_landmarks(frame, hand_landmarks, mp_hands.HAND_CONNECTIONS)

            finger_count = count_fingers(hand_landmarks)

            # Map finger count (0–5) to servo angle (0–180)
            angle = int((finger_count / 5) * 180)

            cv2.putText(frame, f'Fingers: {finger_count}', (10, 40),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0,255,0), 2)

            if angle != last_sent_angle:
                send_to_arduino(angle)
                last_sent_angle = angle

    cv2.imshow("Hand Control Servo", frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
arduino.close()
cv2.destroyAllWindows()