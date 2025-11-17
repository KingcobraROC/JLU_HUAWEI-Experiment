import cv2
import numpy as np
import time
import socket


ESP8266_IP = "10.176.218.154"
TCP_PORT = 8888
SEND_INTERVAL = 0.2
CAMERA_URL = "http://admin:123456@10.176.218.182:8081"


sock = None
last_send_time = 0


def get_tcp_socket():
    global sock
    if sock is None:
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.settimeout(3.0)
            sock.connect((ESP8266_IP, TCP_PORT))
            print(f"[INFO]Connected to ESP8266 at {ESP8266_IP}:{TCP_PORT}")
        except Exception as e:
            print(f"[ERROR]Failed to connect: {e}")
            sock = None
    return sock


def send_command(cmd: str):
    global last_send_time, sock
    current = time.time()
    if current - last_send_time < SEND_INTERVAL:
        return
    last_send_time = current

    s = get_tcp_socket()
    if s is None:
        return

    try:
        s.send((cmd + "\n").encode("utf-8"))
        print(f"[SENT]'{cmd}'")
    except (BrokenPipeError, ConnectionResetError, OSError) as e:
        print(f"[WARN]Connection lost: {e}")
        sock.close()
        sock = None


cap = cv2.VideoCapture(CAMERA_URL)
if not cap.isOpened():
    print("[FATAL]Cannot open camera stream!")
    exit(1)

start_time = time.time()
frame_counter = 0
fps = 0


lower_pink = np.array([169, 68, 94])
upper_pink = np.array([180, 137, 255])

turn_threshold = 50

try:
    while True:
        ret, frame = cap.read()
        if not ret:
            print("Camera frame read failed, retrying...")
            time.sleep(1)
            continue

        frame_counter += 1
        elapsed = time.time() - start_time
        if elapsed >= 1.0:
            fps = frame_counter / elapsed
            start_time = time.time()
            frame_counter = 0

        cv2.putText(
            frame,
            f"FPS: {fps:.1f}",
            (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            1,
            (0, 255, 0),
            2,
        )

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, lower_pink, upper_pink)
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        detected_cx = None
        distance_cm = None

        for cnt in contours:
            if cv2.contourArea(cnt) > 10000:
                x, y, w, h = cv2.boundingRect(cnt)
                KNOWN_WIDTH_CM = 3.0
                FOCAL_LENGTH = 1000
                distance_cm = (KNOWN_WIDTH_CM * FOCAL_LENGTH) / w
                detected_cx = x + w // 2

                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                cv2.putText(
                    frame,
                    f"Dist: {distance_cm:.1f}cm",
                    (x, y - 30),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    (255, 0, 255),
                    2,
                )
                break

        if detected_cx is not None and distance_cm is not None:
            center_x = frame.shape[1] // 2
            offset = detected_cx - center_x

            if distance_cm > 5.0:
                if offset > turn_threshold:
                    send_command("R")
                elif offset < -turn_threshold:
                    send_command("L")
                else:
                    send_command("F")
            else:
                send_command("S")

        # cv2.imshow('Control View', frame)
        # if cv2.waitKey(1) & 0xFF == ord('q'):
        #   break

finally:
    cap.release()
    # cv2.destroyAllWindows()
    if sock:
        sock.close()
    print("[INFO]Program exited cleanly.")
