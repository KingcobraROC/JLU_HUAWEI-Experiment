import serial
import time
import logging

logging.basicConfig(level=logging.INFO)


def ser_init():
    ser = serial.Serial(
        port='/dev/ttyAMA0',
        baudrate=115200,
        # 8数据位
        bytesize=8,
        # 无校验
        parity='N',
        # 1停止位
        stopbits=1,
        timeout=0.5
    )
    return ser


def send_once(ser, msg: str):
    ser.write(msg.encode('ascii'))
    # logging.info(f"Sent: {msg}")


def car_movement(distance_cm, center_x, frame_width):
    """
    Returns:
        str: 指令如 "forward", "forward_right", "backward_left", "stop" 等
    """
    # 距离阈值（cm），小于该值认为足够接近目标
    DISTANCE_THRESHOLD = 15.0 
    DISTANCE_TOO_CLOSE = 10.0
    X_CENTER_OFFSET = 50

    img_center_x = frame_width // 2
    x_diff = center_x - img_center_x
    
    # 横向偏移判断
    if x_diff > X_CENTER_OFFSET:
        horizontal_direction = "right"
    elif x_diff < -X_CENTER_OFFSET:
        horizontal_direction = "left"
    else:
        horizontal_direction = ""
    
    # 距离判断
    if distance_cm < DISTANCE_TOO_CLOSE:
        distance_command = "backward"
    elif distance_cm > DISTANCE_THRESHOLD:
        distance_command = "forward"
    else:
        distance_command = "stop"
    
    # 组合指令
    if distance_command == "stop":
        if horizontal_direction:
            return f"backward_{horizontal_direction}"
        else:
            return "stop"
    else:
        if horizontal_direction:
            return f"{distance_command}_{horizontal_direction}"
        else:
            return distance_command


def cmd2msg(car_command: str) -> str:
    command_map = {
        "forward": 'A',           # 0x41
        "forward_right": 'B',     # 0x42
        # "turn_right": 'C',        # 0x43
        "backward_right": 'D',    # 0x44
        "backward": 'E',          # 0x45
        "backward_left": 'F',     # 0x46
        # "turn_left": 'G',         # 0x47
        "forward_left": 'H',      # 0x48
        "stop": 'Z',
    }
    if car_command not in command_map:
        logging.warning(f"⚠️ Unknown Command: {car_command}")
        return 'Z'
    
    return command_map[car_command]


def main():
    ser = ser_init()
    try:
        sequence = ['A', 'B', 'C', 'D', 'E']
        while True:
            for char in sequence:
                send_once(ser, char)
                time.sleep(0.1)
    except KeyboardInterrupt:
        print("\n程序被用户中断")
    finally:
        ser.close()


if __name__ == '__main__':
    main()