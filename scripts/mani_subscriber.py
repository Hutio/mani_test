
#!/usr/bin/env python

#-----------------------------------------------------------------------------#
import rospy
from std_msgs.msg import Float32MultiArray
from dynamixel_sdk import *

#--------------------------------Settings-------------------------------------#

PROTOCOL_VERSION           = 1.0
# Default setting
BAUDRATE                   = 1000000
DEVICENAME                 = '/dev/ttyUSB0'
LEN_MOTOR_SCAN             = 4



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

def Dynamixel_Read_motor(DXL_ID):
    dxl_present_speed, dxl_comm_result, dxl_error = packetHandler.read2ByteTxRx(portHandler, DXL_ID, ADDR_AX_PRESENT_SPEED)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))

    print("[ID:%03d] PresSpd:%03d" % (DXL_ID,dxl_present_speed))

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

    def __init__(self,connected_motor):
        A = 0

    def Write_motor(DXL_ID, spd, sec):
        dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL_ID, ADDR_AX_MOVING_SPEED, spd)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))
        time.sleep(sec)
        dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL_ID, ADDR_AX_MOVING_SPEED, DXL_DISABLE)

    def Torque_enable(DXL_ID):
        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID, ADDR_AX_TORQUE_ENABLE, DXL_ENABLE)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))
        else:
            Dynamixel_Light_enable(DXL_ID)

    def Torque_disable(DXL_ID):
        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID, ADDR_AX_TORQUE_ENABLE, DXL_DISABLE)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))
        else:
            Dynamixel_Light_disable(DXL_ID)

    def Motor_enable():
        for i in range(LEN_MOTOR_SCAN):
            Dynamixel_Ping(i)
            Torque_enable(i)
        print("Motor [" + ','.join(map(str,connected_motor)) + "] successfully connected")

    def Motor_disable():
        for i in range(len(connected_motor)):
            Torque_disable(i)

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

    dmc = Dynamixel_Motor_control(connected_motor)

    dmc.Motor_enable()
#   listener()
    dmc.Write_motor(0,500,3)
    dmc.Write_motor(1,1524,3)
    dmc.Write_motor(2,500,3)
    dmc.Write_motor(3,1524,3)

    dmc.Motor_disable()

    Dynamixel_Close_port()
    print("Successfully Node Closed")