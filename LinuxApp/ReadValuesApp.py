import gatt
import numpy
from threading import Thread

####################################################
## GLOBAL VARIABLES                               ##
####################################################

seconds = 0

sense1_address = '00:11:22:33:44:55'
sense2_address = 'aa:bb:cc:dd:ee:ff'

service_uuid = '0b2a5ca0-bcb9-11ec-bf84-0800200c9a66'
env_uuid = '0b2a5ca1-bcb9-11ec-bf84-0800200c9a66'
ang_uuid = '0b2a5ca4-bcb9-11ec-bf84-0800200c9a66'
acc_uuid = '0b2a5ca5-bcb9-11ec-bf84-0800200c9a66'

env_tmp_1 = []
env_tmp_2 = []

ang_tmp_1 = []
ang_tmp_2 = []

acc_tmp_1 = []
acc_tmp_2 = []

toWrite_1 = []
toWrite_2 = []

current_ts_1 = 0
current_ts_2 = 0

############################################################
## DEFINITION OF THE THREAD THAT CONNECTS TO THE DEVICE   ##
############################################################

class ConnectionThread(Thread):
    def __init__(self, controller_name):
        Thread.__init__(self)
        self.controller_name = controller_name
    def run(self):
        global manager1, manager2
        if(self.controller_name=='hci0'):
            manager1 = AnyDeviceManager(adapter_name='hci0')
            manager1.start_discovery()
            manager1.run()
            manager1.remove_all_devices()
        if(self.controller_name=='hci1'):
            manager2 = AnyDeviceManager(adapter_name='hci1')
            manager2.start_discovery()
            manager2.run()
            manager2.remove_all_devices()

##############################################################
## PROCEDURE THAT WRITES TO FILE ONCE IT GATHERED ALL DATA  ##
##############################################################

def writeToFile(deviceNumber):
    global toWrite_1, toWrite_2, f

    if (deviceNumber == 1):
        value_env = toWrite_1.pop()
        value_acc = toWrite_1.pop()
        value_ang = toWrite_1.pop()
    else:
        value_env = toWrite_2.pop()
        value_acc = toWrite_2.pop()
        value_ang = toWrite_2.pop()

    for sample in range(25):
        timestamp = value_env[0] * 1000 + (40)*sample
        #print("Writing on file with timestamp " + str(timestamp))
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
        f.write(str(timestamp) + "," + str(deviceNumber) + ',' + str(pressure) + "," + str(humidity) + "," + str(temperature) + "," + str(accelerationX) + "," + str(accelerationY) + "," + str(accelerationZ) + "," + str(gyroX) + "," + str(gyroY) + "," + str(gyroZ) + "\n")

##############################################################
## PROCEDURE THAT BUFFERS DATA BEFORE WRITING TO FILE       ##
##############################################################

def putInQueue(deviceNumber, sensorType, sensorValue):
    global env_tmp_1, env_tmp_2, ang_tmp_1, ang_tmp_2, acc_tmp_1, acc_tmp_2, current_ts_1, current_ts_2, toWrite_1, toWrite_2

    if(deviceNumber == 1):
        #If I received newer data than the ones we have in the buffers, empty the buffers.
        if(sensorValue[0] > current_ts_1):
            env_tmp_1 = []
            ang_tmp_1 = []
            acc_tmp_1 = []
            current_ts_1 = sensorValue[0]
        #Fill the correct buffer with the one we're interested in
        if(sensorType == 'env'):
            env_tmp_1 = sensorValue
        if(sensorType == 'acc'):
            acc_tmp_1 = sensorValue
        if(sensorType == 'ang'):
            ang_tmp_1 = sensorValue
        #If buffers are full, write them.
        if (len(env_tmp_1) > 0 and len(acc_tmp_1) > 0 and len(ang_tmp_1) > 0):
            toWrite_1.append(ang_tmp_1)
            toWrite_1.append(acc_tmp_1)
            toWrite_1.append(env_tmp_1)
            writeToFile(1)

    if(deviceNumber == 2):
        if(sensorValue[0] > current_ts_2):
            env_tmp_2 = []
            ang_tmp_2 = []
            acc_tmp_2 = []
            current_ts_2 = sensorValue[0]
        if(sensorType == 'env'):
            env_tmp_2 = sensorValue
        if(sensorType == 'acc'):
            acc_tmp_2 = sensorValue
        if(sensorType == 'ang'):
            ang_tmp_2 = sensorValue
        if (len(env_tmp_2) > 0 and len(acc_tmp_2) > 0 and len(ang_tmp_2) > 0):
            toWrite_2.append(ang_tmp_2)
            toWrite_2.append(acc_tmp_2)
            toWrite_2.append(env_tmp_2)
            writeToFile(2)

#######################################################################
## IMPLEMENTATION OF GATT PROCEDURES (CONNECTION, NOTIFICATIONS...)  ##
#######################################################################

class AnyDevice(gatt.Device):
    def connect_succeeded(self):
        super().connect_succeeded()
        print("[%s] Connected" % (self.mac_address))

    def connect_failed(self, error):
        super().connect_failed(error)
        print("[%s] Connection failed: %s" % (self.mac_address, str(error)))

    def disconnect_succeeded(self):
        super().disconnect_succeeded()
        print("[%s] Disconnected" % (self.mac_address))
        manager.stop()

    def characteristic_value_updated(self, characteristic, value):
        #When I receive a notification from a device, I send it to the corresponding procedure
        global manager1, manager2
        if(self.mac_address == sense1_address):
            if(int(value[0]) > seconds):
                manager1.stop()
            else:
                if(characteristic.uuid == env_uuid):
                    putInQueue(1,'env',value)
                if(characteristic.uuid == ang_uuid):
                    putInQueue(1,'ang',value)
                if(characteristic.uuid == acc_uuid):
                    putInQueue(1,'acc',value)

        if(self.mac_address == sense2_address):
            if(int(value[0]) > seconds):
                manager2.stop()
            else:
                if(characteristic.uuid == env_uuid):
                    putInQueue(2,'env',value)
                if(characteristic.uuid == ang_uuid):
                    putInQueue(2,'ang',value)
                if(characteristic.uuid == acc_uuid):
                    putInQueue(2,'acc',value)

    def services_resolved(self):
        #Once I receive all the services from the peripheral, I subscribe to the ones I'm interested in
        super().services_resolved()

        sensors_service = next(
            s for s in self.services
            if s.uuid == service_uuid)

        env_char = next(
            c for c in sensors_service.characteristics
            if c.uuid == env_uuid)

        env_char.enable_notifications()

        acc_char = next(
            c for c in sensors_service.characteristics
            if c.uuid == acc_uuid)

        acc_char.enable_notifications()

        gyr_char = next(
            c for c in sensors_service.characteristics
            if c.uuid == ang_uuid)

        gyr_char.enable_notifications()

class AnyDeviceManager(gatt.DeviceManager):
    def device_discovered(self, device):
        global manager1, manager2
        if (self.adapter_name == 'hci0' and device.mac_address == sense1_address):
            device_sense1 = AnyDevice(mac_address=sense1_address, manager=manager1)
            device_sense1.connect()
            self.stop_discovery()
        if (self.adapter_name == 'hci1' and device.mac_address == sense2_address):
            device_sense2 = AnyDevice(mac_address=sense2_address, manager=manager2)
            device_sense2.connect()
            self.stop_discovery()

########################################################################
##  MAIN BODY                                                         ##
########################################################################

seconds = input("Seconds of sampling: ")
seconds = int(seconds)

try:
    f = open("sampled_data.csv","w")
    f.write("timestamp_ms,device,pressure,humidity,temperature,accelerationX,accelerationY,accelerationZ,gyroX,gyroY,gyroZ\n")
except:
    print("Can't create file, exiting...")
    exit()

thread1 = ConnectionThread('hci0')
thread2 = ConnectionThread('hci1')
thread1.start()
thread2.start()
thread1.join()
thread2.join()
f.close()
exit()
