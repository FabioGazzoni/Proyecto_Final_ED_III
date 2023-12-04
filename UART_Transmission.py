import serial

ser = serial.Serial('/dev/ttyUSB0', 9600)  

try:
    while True:
        print("Ingrese 1 para encender la bomba (depende del nivel de humedad) o 0 para apagarla: ")
        mensaje = int(input())
        ser.write(bytes([mensaje]))
except KeyboardInterrupt:
    pass

ser.close()
