# Comment
# Controls Dynamixel motors using SDK

import sys
import termios
import time
import tty
import numpy as np
from dynamixel_sdk import *  # Uses Dynamixel SDK library

# ### ====================================================================================
# added by Shu
# import sys
# import time
# import numpy as np
# from dynamixel_sdk import *  # Uses Dynamixel SDK library

# # OS-specific imports for keyboard input handling
# if sys.platform.startswith("linux") or sys.platform.startswith("darwin"):  # Linux/macOS
#     import termios
#     import tty

#     def getch():
#         """Get a single character from user input (Linux/macOS version)."""
#         fd = sys.stdin.fileno()
#         old_settings = termios.tcgetattr(fd)
#         try:
#             tty.setraw(sys.stdin.fileno())
#             ch = sys.stdin.read(1)
#         finally:
#             termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
#         return ch

# elif sys.platform.startswith("win"):  # Windows
#     import msvcrt

#     def getch():
#         """Get a single character from user input (Windows version)."""
#         return msvcrt.getch().decode('utf-8')
# ### ====================================================================================


fd = sys.stdin.fileno()
old_settings = termios.tcgetattr(fd)


def getch():
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch


def dyna_alert(dxl_comm_result, dxl_error, packetHandler, step):
    ''' Alert for dynamixel communication error
    Args:
        dxl_comm_result: communication result
        dxl_error: error
        packetHandler: packet handler
        step: step name
    '''
    if dxl_comm_result != COMM_SUCCESS:
        print("alarm from " + step)
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("alarm from " + step)
        print("%s" % packetHandler.getRxPacketError(dxl_error))


def dyna_write_and_read(packetHandler, groupSyncWrite, groupSyncRead, num_motor, act):    
    # ********* DYNAMIXEL Model definition *********
    MY_DXL = 'X_SERIES'  # X330 (5.0 V recommended), X430, X540, 2X430

    ADDR_PRESENT_POSITION = 132

    LEN_PRESENT_POSITION = 4

    positions = np.zeros([num_motor // 3, 3])

    dxl_comm_result = groupSyncRead.txRxPacket()
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))

    for i in range(num_motor):

        # Check if groupsyncread data of each Dynamixel is available
        dxl_getdata_result = groupSyncRead.isAvailable(i + 1, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)
        if not dxl_getdata_result:
            print("[ID:%03d] groupSyncRead getdata failed" % (i + 1))
            quit()

        positions[i // 3, i % 3] = groupSyncRead.getData(i + 1, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)

    for i in range(num_motor):

        param_goal_position = [DXL_LOBYTE(DXL_LOWORD(act[i // 3, i % 3])),
                               DXL_HIBYTE(DXL_LOWORD(act[i // 3, i % 3])),
                               DXL_LOBYTE(DXL_HIWORD(act[i // 3, i % 3])),
                               DXL_HIBYTE(DXL_HIWORD(act[i // 3, i % 3]))]

        dxl_addparam_result = groupSyncWrite.addParam(i + 1, param_goal_position)
        if not dxl_addparam_result:
            print("[ID:%03d] groupSyncWrite addparam failed" % (i + 1))
            quit()
    dxl_comm_result = groupSyncWrite.txPacket()
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))

    # Clear syncwrite parameter storage
    groupSyncWrite.clearParam()

    return positions


def all_dyna_init(num_motor):
    ''' Initialize the dynamixel sensing
    Args:
        num_motor: number of motors
    Returns:
        motor_list: num_motor x 3 numpy array
        packetHandler: packet handler
        groupSyncWrite: group sync write
        groupSyncRead: group sync read
    '''
    motor_list = np.zeros([num_motor // 3, 3])
    BAUDRATE = 1000000
    PROTOCOL_VERSION = 2.0
    DEVICENAME = '/dev/ttyUSB0'
    # DEVICENAME = 'COM9'

    portHandler = PortHandler(DEVICENAME)
    packetHandler = PacketHandler(PROTOCOL_VERSION)

    # Open port
    if portHandler.openPort():
        pass
    else:
        print("Failed to open the port")
        print("Press any key to terminate...")
        getch()
        quit()

    # Set port baudrate
    if portHandler.setBaudRate(BAUDRATE):
        pass
    else:
        print("Failed to change the baudrate")
        print("Press any key to terminate...")
        getch()
        quit()

    MY_DXL = 'X_SERIES'  # X330 (5.0 V recommended), X430, X540, 2X430

    # Control table address
    if MY_DXL == 'X_SERIES' or MY_DXL == 'MX_SERIES':
        ADDR_OPERATION_MODE = 11
        ADDR_OFFSET = 20
        ADDR_TORQUE_ENABLE = 64
        ADDR_GOAL_CURRENT = 102
        ADDR_GOAL_POSITION = 116
        ADDR_PRESENT_POSITION = 132

        LEN_GOAL_CURRENT = 2
        LEN_GOAL_POSITION = 4
        LEN_PRESENT_POSITION = 4

        CURRENT_CTRL = 0
        POSITION_CTRL = 4  # Extended Position Control Mode

    TORQUE_ENABLE = 1  # Value for enabling the torque
    TORQUE_DISABLE = 0  # Value for disabling the torque

    groupSyncWrite_goal_current = GroupSyncWrite(portHandler, packetHandler, ADDR_GOAL_CURRENT, LEN_GOAL_CURRENT)
    groupSyncRead_present_position = GroupSyncRead(portHandler, packetHandler, ADDR_PRESENT_POSITION,
                                                   LEN_PRESENT_POSITION)

    # Initialize GroupSyncWrite instance
    groupSyncWrite = GroupSyncWrite(portHandler, packetHandler, ADDR_GOAL_POSITION, LEN_GOAL_POSITION)

    # Initialize GroupSyncRead instace for Present Position
    groupSyncRead = GroupSyncRead(portHandler, packetHandler, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)

    for i in range(num_motor):
        DXL_ID = i + 1

        dxl_addparam_result = groupSyncRead.addParam(DXL_ID)
        if dxl_addparam_result != True:
            print("[ID:%03d] groupSyncRead addparam failed" % DXL_ID)
            quit()

        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID, ADDR_TORQUE_ENABLE,
                                                                  TORQUE_DISABLE)
        dyna_alert(dxl_comm_result, dxl_error, packetHandler, str(i + 1))

        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID, ADDR_OPERATION_MODE,
                                                                  CURRENT_CTRL)
        dyna_alert(dxl_comm_result, dxl_error, packetHandler, str(i + 1))

        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID, ADDR_TORQUE_ENABLE,
                                                                  TORQUE_ENABLE)
        dyna_alert(dxl_comm_result, dxl_error, packetHandler, str(i + 1))


        goal_current = 50

        param_goal_current = [DXL_LOBYTE(goal_current), DXL_HIBYTE(goal_current)]
        dxl_addparam_result = groupSyncWrite_goal_current.addParam(DXL_ID, param_goal_current)
        dxl_addparam_result = groupSyncRead_present_position.addParam(DXL_ID)

    dxl_comm_result = groupSyncWrite_goal_current.txPacket()
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))

    time.sleep(10)
    # Clear syncwrite parameter storage
    groupSyncWrite_goal_current.clearParam()

    for i in range(num_motor):
        DXL_ID = i + 1
        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID, ADDR_TORQUE_ENABLE,
                                                                  TORQUE_DISABLE)
        dyna_alert(dxl_comm_result, dxl_error, packetHandler, str(i + 1))

        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID, ADDR_OPERATION_MODE,
                                                                  POSITION_CTRL)
        dyna_alert(dxl_comm_result, dxl_error, packetHandler, str(i + 1))

        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID, ADDR_TORQUE_ENABLE,
                                                                  TORQUE_ENABLE)
        dyna_alert(dxl_comm_result, dxl_error, packetHandler, str(i + 1))

    dxl_comm_result = groupSyncRead_present_position.txRxPacket()
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))

    for i in range(num_motor):
        DXL_ID = i + 1


        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID, ADDR_TORQUE_ENABLE,
                                                                  TORQUE_DISABLE)
        dyna_alert(dxl_comm_result, dxl_error, packetHandler, str(i + 1))
        dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL_ID, ADDR_OFFSET, 0)
        dyna_alert(dxl_comm_result, dxl_error, packetHandler, str(i + 1))
        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID, ADDR_TORQUE_ENABLE,
                                                                  TORQUE_ENABLE)
        dyna_alert(dxl_comm_result, dxl_error, packetHandler, str(i + 1))

        # Check if groupsyncread data of Dynamixel is available
        dxl_present_position, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, DXL_ID,
                                                                                       ADDR_PRESENT_POSITION)
        dyna_alert(dxl_comm_result, dxl_error, packetHandler, str(i + 1))

        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID, ADDR_TORQUE_ENABLE,
                                                                  TORQUE_DISABLE)
        dyna_alert(dxl_comm_result, dxl_error, packetHandler, str(i + 1))
        dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL_ID, ADDR_OFFSET,
                                                                  10000 - dxl_present_position)
        dyna_alert(dxl_comm_result, dxl_error, packetHandler, str(i + 1))
        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID, ADDR_TORQUE_ENABLE,
                                                                  TORQUE_ENABLE)
        dyna_alert(dxl_comm_result, dxl_error, packetHandler, str(i + 1))


        dxl_present_position_, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, DXL_ID,
                                                                                        ADDR_PRESENT_POSITION)
        dyna_alert(dxl_comm_result, dxl_error, packetHandler, str(i + 1))
        motor_list[i // 3, i % 3] = dxl_present_position_

    groupSyncRead_present_position.clearParam()
    print("release")
    time.sleep(1)

    return motor_list, packetHandler, groupSyncWrite, groupSyncRead


def all_dyna_sensing(num_motor, act_list):
    ''' Initialize the dynamixel sensing
    Args:
        num_motor: number of motors
        act_list: num_motor x 3 numpy array
    Returns:
        motor_list: num_motor x 3 numpy array
    '''
    motor_list = np.zeros([num_motor])
    for i in range(num_motor):
        motor_list[i] = dyna_write_and_read(i, act_list[i])
    return motor_list