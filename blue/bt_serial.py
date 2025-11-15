import serial

ser = serial.Serial(port='/dev/ttyAMA0', baudrate=115200, timeout=0.5)

while True:
    ser.write(b'A')
    # print('A')

ser.close()
