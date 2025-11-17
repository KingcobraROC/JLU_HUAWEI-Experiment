import time
import logging
import socket

logging.basicConfig(level=logging.INFO)

# ========== TCP 配置 ==========
ESP8266_IP = "10.176.218.154"  # ←←← 务必替换为你的 ESP8266 实际 IP！
TCP_PORT = 8888
# =============================


def ser_init():
    """
    模拟串口初始化，实际返回一个已连接的 TCP socket（长连接）
    """
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(3.0)
        sock.connect((ESP8266_IP, TCP_PORT))
        logging.info(f"✅ TCP connected to ESP8266 at {ESP8266_IP}:{TCP_PORT}")
        return sock
    except Exception as e:
        logging.error(f"❌ Failed to connect to ESP8266: {e}")
        raise


def send_msg(ser, msg: str):
    """
    通过 TCP socket 发送消息（兼容原函数签名）
    参数:
        ser: 实际是 socket 对象（由 ser_init 返回）
        msg: 要发送的字符串（如 'A', 'Z' 等）
    """
    if not isinstance(ser, socket.socket):
        logging.error("⚠️ Invalid socket object!")
        return

    try:
        # 发送原始字节（与串口行为一致）
        ser.send((msg + "\n").encode("ascii"))
        logging.debug(f"[SENT] '{msg}'")
    except Exception as e:
        logging.error(f"❌ Send failed: {e}")
        # 可选：尝试重连？此处简化处理，由上层控制循环
        pass


def car_movement(distance_cm, center_x, frame_width):
    DISTANCE_THRESHOLD = 28.0
    DISTANCE_TOO_CLOSE = 18.0
    X_CENTER_OFFSET = 90

    img_center_x = frame_width // 2
    x_diff = center_x - img_center_x

    if x_diff > X_CENTER_OFFSET:
        horizontal_direction = "right"
    elif x_diff < -X_CENTER_OFFSET:
        horizontal_direction = "left"
    else:
        horizontal_direction = ""

    if distance_cm < DISTANCE_TOO_CLOSE:
        distance_command = "backward"
    elif distance_cm > DISTANCE_THRESHOLD:
        distance_command = "forward"
    else:
        distance_command = "stop"

    if distance_command == "stop":
        return "stop"
    elif distance_command == "backward":
        if horizontal_direction == "right":
            return f"{distance_command}_left"
        else:
            return f"{distance_command}_right"
    else:
        if horizontal_direction:
            return f"{distance_command}_{horizontal_direction}"
        else:
            return distance_command


def cmd2msg(car_command: str) -> str:
    command_map = {
        "forward": "A",
        "forward_right": "B",
        "backward_right": "D",
        "backward": "E",
        "backward_left": "F",
        "forward_left": "H",
        "stop": "Z",
    }
    if car_command not in command_map:
        logging.warning(f"⚠️ Unknown Command: {car_command}")
        return "Z"
    return command_map[car_command]


def main():
    ser = ser_init()  # 实际是 TCP socket
    try:
        sequence = ["A", "B", "C", "D", "E"]
        while True:
            for char in sequence:
                send_msg(ser, char)
                time.sleep(0.1)
    except KeyboardInterrupt:
        print("\n程序被用户中断")
    finally:
        if hasattr(ser, "close"):
            ser.close()
        logging.info("TCP connection closed.")


if __name__ == "__main__":
    main()
