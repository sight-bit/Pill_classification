import serial

# innit serial
ser = serial.Serial(

    port='/dev/ttyACM0', # 컴포트는 장치관리자에서 꼭 확인하세요.

    baudrate=115200)


print(ser.portstr) #연결된 포트 확인.
