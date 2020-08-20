
#!/usr/bin/env python

#----------------------------------Lib----------------------------------------#
import rospy
import threading
import multiprocessing
from std_msgs.msg import Float32MultiArray
from dynamixel_sdk import *

#--------------------------------Settings-------------------------------------#

PROTOCOL_VERSION           = 1.0
# Default setting
BAUDRATE                   = 1000000
DEVICENAME                 = '/dev/ttyUSB0'
LEN_MOTOR_SCAN             = 4

Mdata = [
         [0,1,2,3], #Motor number
         [100,1124,100,1124], #Motor speed
         [1,2,3,4] #Motor duration
         ]

#--------------------------------Don't touch----------------------------------#
# Motor Address
ADDR_AX_TORQUE_ENABLE      = 24
ADDR_AX_MOVING_SPEED       = 32
ADDR_AX_PRESENT_SPEED      = 38
ADDR_AX_LIGHT              = 25

COMM_SUCCESS               = 0

DXL_ENABLE                 = 1
DXL_DISABLE                = 0

connected_motor = []
portHandler = PortHandler(DEVICENAME)
packetHandler = PacketHandler(PROTOCOL_VERSION)

#-----------------------------------------------------------------------------#

def Dynamixel_Open_port():
    if portHandler.openPort():
        print("Succeeded to open the port")
    else:
        print("Failed to open the port")
        print("Press any key to terminate...")
        getch()
        quit()

def Dynamixel_Set_baudrate():
    if portHandler.setBaudRate(BAUDRATE):
        print("Succeeded to change the baudrate")
    else:
        print("Failed to change the baudrate")
        print("Press any key to terminate...")
        getch()
        quit()

def Dynamixel_Light_enable(DXL_ID):
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID, ADDR_AX_LIGHT, DXL_ENABLE)

def Dynamixel_Light_disable(DXL_ID):
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID, ADDR_AX_LIGHT, DXL_DISABLE)

def Dynamixel_Ping(DXL_ID):
    dxl_model_number, dxl_comm_result, dxl_error = packetHandler.ping(portHandler, DXL_ID)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
    else:
        print("[ID:%03d] ping Succeeded." % DXL_ID)
        connected_motor.append(DXL_ID)

class Dynamixel_Motor_control:

    def __init__(self, connected_motor, LEN_MOTOR_SCAN, Mdata):
        self.Mdata = Mdata
        self.connected_motor = connected_motor
        self.LEN_MOTOR_SCAN = LEN_MOTOR_SCAN

#----------------------------only sequential ---------------------------------#
    def Write_motor(self, DXL_ID, spd, sec):
        dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL_ID, ADDR_AX_MOVING_SPEED, spd)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))
        time.sleep(sec)
        dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL_ID, ADDR_AX_MOVING_SPEED, DXL_DISABLE)

    def Stop_motor(self, DXL_ID):
        dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL_ID, ADDR_AX_MOVING_SPEED, DXL_DISABLE)

    def Read_motor(DXL_ID):
        dxl_present_speed, dxl_comm_result, dxl_error = packetHandler.read2ByteTxRx(portHandler, DXL_ID, ADDR_AX_PRESENT_SPEED)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))

        print("[ID:%03d] PresSpd:%03d" % (DXL_ID,dxl_present_speed))
#------------------------------sync drive-------------------------------------#

    def Sync_write(self, Mdata):
        pl = [Process(target=self.Write_motor, args=(Mdata[0][q], Mdata[1][q], Mdata[2][q])) for q in self.connected_motor]
        for w in pl:
            w.start()
        for w in pl:
            w.join()
#-----------------------------------------------------------------------------#
    def Torque_enable(self, DXL_ID):
        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID, ADDR_AX_TORQUE_ENABLE, DXL_ENABLE)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))
        else:
            Dynamixel_Light_enable(DXL_ID)

    def Torque_disable(self, DXL_ID):
        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID, ADDR_AX_TORQUE_ENABLE, DXL_DISABLE)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))
        else:
            Dynamixel_Light_disable(DXL_ID)

    def Motor_enable(self):
        for i in range(self.LEN_MOTOR_SCAN):
            Dynamixel_Ping(i)
            self.Torque_enable(i)
        print("Motor [" + ','.join(map(str,self.connected_motor)) + "] successfully connected")

    def Motor_disable(self):
        for i in self.connected_motor:
            self.Torque_disable(i)

def Dynamixel_Close_port():
    portHandler.closePort()

def callback(data):
    msg = data.data
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", msg)

def listener():

    rospy.init_node('mani_test_listener', anonymous=False)

    rospy.Subscriber("Motor_Information", Float32MultiArray, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':
    Dynamixel_Open_port()
    Dynamixel_Set_baudrate()

    dmc = Dynamixel_Motor_control(connected_motor,LEN_MOTOR_SCAN, Mdata)

    dmc.Motor_enable()
#    listener()

    dmc.Sync_write(Mdata)

    dmc.Motor_disable()

    Dynamixel_Close_port()
    print("Successfully Node Closed")
