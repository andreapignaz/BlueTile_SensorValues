import BLE_GATT
import os
import subprocess
import time
import numpy

bt_address = '02:80:E1:00:34:12'

envi_uuid = "0b2a5ca1-bcb9-11ec-bf84-0800200c9a66"
angu_uuid = "0b2a5ca4-bcb9-11ec-bf84-0800200c9a66"
acce_uuid = "0b2a5ca5-bcb9-11ec-bf84-0800200c9a66"

samplingTimes = input("How many seconds of sampling? ")
if int(samplingTimes) < 1:
    exit()

print("Wait for 5 seconds for the bluetooth scan to complete...")
proc = subprocess.Popen(["bluetoothctl","scan", "on"], stdout = subprocess.DEVNULL)
time.sleep(5)
proc.kill()
print("Trying to connect...")

try:
    device = BLE_GATT.Central(bt_address)
    device.connect()
except:
    print("Can't connect. Verify that:\n- The device is turned ON\n- The device is in pairing mode (Blue LED)\nthen try again.")
    exit()

print("Connected!")

try:
    f = open("sampled_data.csv","w")
    f.write("timestamp_ms,pressure,humidity,temperature,accelerationX,accelerationY,accelerationZ,gyroX,gyroY,gyroZ\n")
except:
    print("Can't create file, exiting...")
    exit()

for seconds in range(int(samplingTimes)):

    startTime = time.time()

    value_env = device.char_read(envi_uuid)
    value_ang = device.char_read(angu_uuid)
    value_acc = device.char_read(acce_uuid)

    if (value_env[0] != value_ang[0] or value_ang[0] != value_acc[0] or value_env[0] != value_acc[0]):
        print("Time synchronization lost, exiting...")
        exit()

    for sample in range(25):
        timestamp = value_env[0] * 1000 + (40)*sample
        pressure = value_env[7*sample + 1] + (value_env[7*sample + 2] << 8) + (value_env[7*sample + 3] << 16)
        humidity = value_env[7*sample + 4] + (value_env[7*sample + 5] << 8)
        temperature = value_env[7*sample + 6] + (value_env[7*sample + 7] << 8)
        accelerationX = value_acc[6*sample + 1] + (value_acc[6*sample + 2] << 8)
        accelerationX = numpy.int16(accelerationX)
        accelerationY = value_acc[6*sample + 3] + (value_acc[6*sample + 4] << 8)
        accelerationY = numpy.int16(accelerationY)
        accelerationZ = value_acc[6*sample + 5] + (value_acc[6*sample + 6] << 8)
        accelerationZ = numpy.int16(accelerationZ)
        gyroX = value_ang[6*sample + 1] + (value_ang[6*sample + 2] << 8)
        gyroX = numpy.int16(gyroX)
        gyroY = value_ang[6*sample + 3] + (value_ang[6*sample + 4] << 8)
        gyroY = numpy.int16(gyroY)
        gyroZ = value_ang[6*sample + 5] + (value_ang[6*sample + 6] << 8)
        gyroZ = numpy.int16(gyroZ)
        f.write(str(timestamp) + "," + str(pressure) + "," + str(humidity) + "," + str(temperature) + "," + str(accelerationX) + "," + str(accelerationY) + "," + str(accelerationZ) + "," + str(gyroX) + "," + str(gyroY) + "," + str(gyroZ) + "\n")

    time.sleep(1 - (time.time() - startTime))

print("Sampling completed! Disconnecting...")
device.disconnect()
