import cv2
import mediapipe as mp
import serial
import time

# -----------------------------
# SERIAL CONNECTION TO ARDUINO
# -----------------------------
try:
    arduino = serial.Serial('COM6', 115200, timeout=1)   # <— change if needed
    print("Connected to Arduino on COM6")
    time.sleep(2)
except:
    print("ERROR: Could not open COM6")
    exit()

# -----------------------------
# MEDIAPIPE HAND INITIALIZATION
# -----------------------------
mp_hands = mp.solutions.hands
hands = mp_hands.Hands(
    max_num_hands=1,
    min_detection_confidence=0.7,
    min_tracking_confidence=0.7
)
mp_draw = mp.solutions.drawing_utils


# -----------------------------
# COUNT FINGERS (from hand.py)
# -----------------------------
def count_fingers(hand_landmarks):
    finger_tip_ids = [8, 12, 16, 20]
    count = 0

    # Standard fingers
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


# -----------------------------
# SEND ANGLE TO ARDUINO
# -----------------------------
def send_angle(angle):
    angle = int(angle)
    message = f"{angle};"

    arduino.write(message.encode('utf-8'))
    # print("Sent:", message)


# -----------------------------
# VIDEO CAPTURE LOOP
# -----------------------------
cap = cv2.VideoCapture(0)

while True:
    success, frame = cap.read()
    if not success:
        break

    frame = cv2.flip(frame, 1)
    rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    results = hands.process(rgb)

    if results.multi_hand_landmarks:
        for hand_landmarks in results.multi_hand_landmarks:
            mp_draw.draw_landmarks(frame, hand_landmarks, mp_hands.HAND_CONNECTIONS)

            # Count fingers
            finger_count = count_fingers(hand_landmarks)

            # Map number of fingers to servo angle
            # 0 fingers → 0° (closed)
            # 5 fingers → 180° (open)
            angle = int((finger_count / 5) * 180)

            # Display
            cv2.putText(frame, f'Fingers: {finger_count}', (10, 40),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            cv2.putText(frame, f'Servo Angle: {angle}', (10, 80),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 0), 2)

            # Send to Arduino
            send_angle(angle)

    cv2.imshow("Hand Tracking → Arduino Servo", frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()