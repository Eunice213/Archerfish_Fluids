import serial
import time
serialcomm14 = serial.Serial('COM14', 9600)
serialcomm14.timeout = 1

serialcomm10 = serial.Serial('COM10', 9600)
serialcomm10.timeout = 1
while True:
    i = input("Enter Input: ").strip()
    if i == "Done":
        print('finished')
        break
    elif i =="Port14":
        port = serialcomm14
    elif i == "Port10":
        port = serialcomm10
    elif i == "ShowPort":
        print(port.readline().decode('ascii'))
    else:
        port.write(i.encode())
        time.sleep(0.5)
        print(port.readline().decode('ascii'))
serialcomm7.close()
serialcomm10.close()
# 1['0000', '0000', '0000']1