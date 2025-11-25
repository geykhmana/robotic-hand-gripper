import cv2
import mediapipe as mp
import serial
import threading
import tkinter
import customtkinter
import time

# ------------------------------ #
#  SERIAL SETUP
# ------------------------------ #
arduino = serial.Serial(port='COM3', baudrate=115200, timeout=1)

def send_to_arduino(value):
    """Sends a number + delimiter to Arduino."""
    msg = f"{int(value)};"
    arduino.write(msg.encode('utf-8'))


# ------------------------------ #
#  MEDIAPIPE SETUP
# ------------------------------ #
mp_hands = mp.solutions.hands
hands = mp_hands.Hands(
    max_num_hands=1,
    min_detection_confidence=0.7,
    min_tracking_confidence=0.7
)
mp_draw = mp.solutions.drawing_utils

def count_fingers(hand_landmarks):
    """Returns number of extended fingers."""
    finger_tip_ids = [8, 12, 16, 20]  # Index, middle, ring, pinky tips
    count = 0

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


# ------------------------------ #
#  HAND TRACKING THREAD
# ------------------------------ #
def hand_tracking_thread():
    cap = cv2.VideoCapture(0)

    last_sent = -1   # Avoid sending the same value over and over

    while True:
        success, frame = cap.read()
        if not success:
            continue

        frame = cv2.flip(frame, 1)
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        results = hands.process(rgb)

        if results.multi_hand_landmarks:
            for hand_landmarks in results.multi_hand_landmarks:

                # Draw hand skeleton
                mp_draw.draw_landmarks(frame, hand_landmarks, mp_hands.HAND_CONNECTIONS)

                # Count fingers
                fingers = count_fingers(hand_landmarks)

                # Update GUI label (thread-safe using after)
                app.after(0, lambda f=fingers: label_fingers.configure(text=f"Fingers: {f}"))

                # Send to Arduino (only when changed)
                if fingers != last_sent:
                    send_to_arduino(fingers)
                    last_sent = fingers

        cv2.imshow("Hand Tracking - press q to close", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()


# ------------------------------ #
#  GUI SETUP
# ------------------------------ #
customtkinter.set_appearance_mode("System")
customtkinter.set_default_color_theme("blue")

app = customtkinter.CTk()
app.geometry("800x480")
app.title("Merged Hand Tracking + Servo GUI")

servo_value = 0

def slider_event(value):
    global servo_value
    servo_value = int(value)
    label_slider.configure(text=f"Slider Angle: {servo_value}")
    send_to_arduino(servo_value)

# Labels
label_slider = customtkinter.CTkLabel(app, text="Slider Angle: 0")
label_slider.place(relx=0.1, rely=0.15)

label_fingers = customtkinter.CTkLabel(app, text="Fingers: 0")
label_fingers.place(relx=0.1, rely=0.25)

# Slider Widget
slider = customtkinter.CTkSlider(app, from_=0, to=180, command=slider_event, width=480)
slider.place(relx=0.5, rely=0.35, anchor=tkinter.CENTER)

# ------------------------------ #
#  START HAND TRACKING THREAD
# ------------------------------ #
thread = threading.Thread(target=hand_tracking_thread, daemon=True)
thread.start()

# ------------------------------ #
#  START GUI (MAIN THREAD)
# ------------------------------ #
app.mainloop()
