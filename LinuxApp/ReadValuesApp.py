import BLE_GATT
import os
import subprocess
import time

bt_address = '02:80:E1:00:34:12'

env_uuid = "495fb220-bb0d-11ec-bf84-0800200c9a66"
acc_uuid = "495fb222-bb0d-11ec-bf84-0800200c9a66"
gyr_uuid = "495fb223-bb0d-11ec-bf84-0800200c9a66"

print("Wait for 5 seconds for the bluetooth scan to complete...")
proc = subprocess.Popen(["bluetoothctl","scan", "on"],stdout = subprocess.DEVNULL)
time.sleep(5)
proc.kill()
print("Now we can connect!")

device = BLE_GATT.Central(bt_address)
# Should add a TRY construct around this :)
device.connect()
startTime = time.time()
for x in range(3):
    value = device.char_read(env_uuid)
    print("Random value: " + str(value[0]))

    pressure = 0.0
    pressure = pressure + value[1] + (value[2] << 8) + (value[3] << 16)
    pressure = pressure / 4096.0
    print("Pressure: " + str(pressure) + " hPa")

    humidity = 0.0
    humidity = humidity + value[4] + (value[5] << 8)
    print("Humidity: " + str(humidity))

    temperature = 0.0
    temperature = temperature + value[6] + (value[7] << 8)
    temperature = temperature/10
    print("Temperature: " + str(temperature))

    value = device.char_read(acc_uuid)
    accX = value[0] + (value[1] << 8)
    accY = value[2] + (value[3] << 8)
    accZ = value[4] + (value[5] << 8)
    print("AccX: " + str(accX) + " AccY: " + str(accY) + " AccZ: " + str(accZ))

    value = device.char_read(gyr_uuid)
    gyrX = value[0] + (value[1] << 8)
    gyrY = value[2] + (value[3] << 8)
    gyrZ = value[4] + (value[5] << 8)
    print("GyroX: " + str(gyrX) + " GyroY: " + str(gyrY) + " GyroZ: " + str(gyrZ))

    #print("current Time: ", time.time() - startTime)
    #20Hz capture:
    #time.sleep(0.05)

device.disconnect()
