import BLE_GATT
import os
import subprocess
import time

bt_address = '02:80:E1:00:34:12'
char_uuid = "00e00000-0001-11e1-ac36-0002a5d5c51b"

print("Wait for 5 seconds for the bluetooth scan to complete...")
proc = subprocess.Popen(["bluetoothctl","scan", "on"],stdout = subprocess.DEVNULL)
time.sleep(5)
proc.kill()
print("Now we can connect!")

device = BLE_GATT.Central(bt_address)
# Should add a TRY construct around this :)
device.connect()
value = device.char_read(char_uuid)
print("Random value: " + str(value[0]))
pressure = 0.0
pressure = pressure + value[2] + (value[3] << 8) + (value[4] << 16)
pressure = pressure / 4096.0
print("Pressure: " + str(pressure) + " hPa")
humidity = 0.0
humidity = humidity + value[6] + (value[7] << 8)
print("Humidity: " + str(humidity))
temperature = 0.0
temperature = temperature + value[9] + (value[10] << 8)
temperature = temperature/10
print("Temperature: " + str(temperature))
accX = value[12] + (value[13] << 8)
accY = value[14] + (value[15] << 8)
accZ = value[16] + (value[17] << 8)
print("AccX: " + str(accX) + " AccY: " + str(accY) + " AccZ: " + str(accZ))

device.disconnect()
