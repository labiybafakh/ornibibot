import serial
import time


recData = "" 
buffer = "$VNYBA,+059.745,+003.857,-004.949,+00.001,-00.008,+00.150,+00.000478,-00.000896,-00.001366*5B"

ser     = serial.Serial('/dev/ttyUSB0', 115200)
ser.write("$VNWRG,6,16*6B\r\n")

while True:
    #print(ser.readline())
    #recData = ser.readline()
    buffer = "+059.745,+003.857,-004.949,+00.001,-00.008,+00.150,+00.000478,-00.000896,-00.001366*5B"
    print(buffer)
    #buffer = recData.split(",")
    #parts = [part.strip() for part in buffer.split(',')]
    #parts = list(map(int,buffer.replace('+','')))
    #print(len(parts))
    #print("Yaw:",parts[1],"  Pitch:",parts[2],"  Roll:",parts[3], "  Ax:",parts[4],"  Ay:",parts[5],"  Az:",parts[6])
    time.sleep(0.005)
 
ser.flush()
ser.close()
