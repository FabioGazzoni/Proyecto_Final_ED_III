import serial

ser = serial.Serial('/dev/ttyUSB0', 9600)  

try:
    while True:
        respuesta_bytes = ser.read(2)  # Lee 4 bytes (32 bits)
        respuesta = int.from_bytes(respuesta_bytes, byteorder='little')  # Interpreta los bytes como un número binario
        print(f'Resultado: {respuesta}')
except KeyboardInterrupt:
    pass

ser.close()

