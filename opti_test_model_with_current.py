import sys
import time
import numpy as np
from dynamixel_sdk import *  # Uses Dynamixel SDK library
import torch
import joblib 
from nn import KinematicMLP
from matplotlib import pyplot as plt


from optitrack_reader import MinimalNatNetClient
import threading
import csv

# ------------------------------------------- #
# --- OptiTrack client setup  --- #
# ------------------------------------------- #
print("OptiTrack client setup...")

rigid_body_ids = {
    'SA-base': 1,
    'SA-low': 2,
    'SA-middle': 3,
    'SA-up': 4
}

optitrack_client = MinimalNatNetClient(
    # server_ip="169.254.118.69", # Qinghua's
    server_ip="169.254.118.142", 
    # server_ip="192.168.56.1", from shujiro's computer
    multicast_ip="239.255.42.99"
)

optitrack_client.rigid_bodies = rigid_body_ids

### ------------------------------------------- #
### --------- TO CHANGE -------------### 
n = 3 # simulation number


# Define Motion Parameters
num_seg = 3
num_motor = 3 * num_seg

thr_list_length = 10 # threshold list length

list_length = thr_list_length * 3
# dif_act_range = 150 # different actuation range; try later with 300; 400;
# limit_motor = 2000 # Try later with 3000
dif_act_range = 300 # different actuation range; try later with 300; 400;
limit_motor = 2000 # Try later with 3000
### ------------------------------------------- #

# Prepare to receive data
csv_filename = f"optitrack_mass_data_full_bodies_data{list_length}_actrange{dif_act_range}_date2105_night_{n}.csv"
optitrack_file = open(csv_filename, "w", newline="")
csv_writer = csv.writer(optitrack_file)

# CSV Header
header = ["Timestamp"]
for name in rigid_body_ids:
    header += [
        f"{name}-X", f"{name}-Y", f"{name}-Z",
        f"{name}-Roll", f"{name}-Pitch", f"{name}-Yaw"
    ]

csv_writer.writerow(header)

data_lock = threading.Lock()

collecting_data = True
latest_data = {name: [None]*6 for name in rigid_body_ids}

def collect_optitrack_data():
    global latest_data
    if not optitrack_client.connect():
        print("OptiTrack connection failed.")
        return
    # print("OptiTrack connected.")

    while collecting_data:
        frame_data = optitrack_client.receive_data()
        # print("SUCCESS: Received OptiTrack data:", frame_data)
        if frame_data:
            with data_lock:
                for rb_id, pos, euler in frame_data:
                    for name, id_num in rigid_body_ids.items():
                        if rb_id == id_num:
                            latest_data[name] = list(pos) + list(euler)
        else :
            print("No data received from OptiTrack.")

        time.sleep(0.005)

# -------------------------------- #
# --- INITIALIZATION AND SETUP --- #
# -------------------------------- #
# --- Dynamixel Setup ---
DEVICENAME = "COM10"  # or "/dev/ttyUSB0" on Linux
BAUDRATE = 1000000
PROTOCOL_VERSION = 2.0

# Dynamixel motor settings
num_motors = 9
ADDR_TORQUE_ENABLE = 64
ADDR_GOAL_POSITION = 116
ADDR_PRESENT_POSITION = 132
ADDR_OPERATING_MODE = 11
ADDR_GOAL_CURRENT = 102

TORQUE_ENABLE = 1
TORQUE_DISABLE = 0

# Operation Mode
CURRENT_MODE = 0
VELOCITY_MODE = 1
POSITION_MODE = 3
EXTENDED_POSITION_MODE = 4
CURRENT_BASED_POSITION_MODE = 5
PWM_MODE = 16

# Open port
portHandler = PortHandler(DEVICENAME)
packetHandler = PacketHandler(PROTOCOL_VERSION)

# Open port
if not portHandler.openPort():
    print("Failed to open the port")
    sys.exit()
if not portHandler.setBaudRate(BAUDRATE):
    print("Failed to set baudrate")
    sys.exit()
print("Port and baudrate initialized.")


# ------------------------------------------- #
# --- Set motors to Current Mode --- #
# ------------------------------------------- #
for motor_id in range(1, num_motors + 1):
    result, error = packetHandler.write1ByteTxRx(portHandler, motor_id, ADDR_OPERATING_MODE, CURRENT_MODE)
    if result != COMM_SUCCESS:
        print(f"Motor {motor_id} mode set error: {packetHandler.getTxRxResult(result)}")
    time.sleep(0.05)

print("All motors set to Current Mode.")
# ------------------------------------------- #

# ------------------------------------------- #
# --- Disable torque  --- #
# ------------------------------------------- #
for motor_id in range(1, 10):
    result, error = packetHandler.write1ByteTxRx(portHandler, motor_id, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
    if result != COMM_SUCCESS:
        print(f"Motor {motor_id} torque disable error: {packetHandler.getTxRxResult(result)}")
    time.sleep(0.05)

print("Torque disabled for all motors")
print("Press Enter to continue")
input()
# ------------------------------------------- #

# ------------------------------------------- #
# --- Enable torque  --- #
# ------------------------------------------- #
for motor_id in range(1, 10):
    packetHandler.write1ByteTxRx(portHandler, motor_id, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
    if result != COMM_SUCCESS:
        print(f"Motor {motor_id} torque enable error: {packetHandler.getTxRxResult(result)}")
    time.sleep(0.05)

print("Torque enabled for all motors.")
print("Press Enter to continue")
input()

# ---------------------------------------------- #
# Set motors to defined current values --> verticaly straight
# ---------------------------------------------- #
currents = [20,20,20,
            20,20,20,
            20,20,20]

# Set motors to the defined current values
for motor_id in range(1, 10):
    packetHandler.write2ByteTxRx(portHandler, motor_id, ADDR_GOAL_CURRENT, currents[motor_id - 1])
    if result != COMM_SUCCESS:
        print(f"Motor {motor_id} current setting error: {packetHandler.getTxRxResult(result)}")

print("Adjust --> ")
print("Press Enter to continue")
input()

# ------------------------------------------- #
# --- Measure safe baseline positions  --- #
# ------------------------------------------- #
safe_baseline_positions = []
for motor_id in range(1, 10):
    pos, _, _ = packetHandler.read4ByteTxRx(portHandler, motor_id, ADDR_PRESENT_POSITION)
    safe_baseline_positions.append(pos)

print("Actual positions for safe baseline:", safe_baseline_positions)

# ------------------------------------------- #
# ------------------------------------------- #
# SWITCH TO EXTENDED POSITION CONTROL MODE
# ------------------------------------------- #
# ------------------------------------------- #


# ------------------------------------------- #
# --- Disable torque  --- #
# ------------------------------------------- #
for motor_id in range(1, 10):
    result, error = packetHandler.write1ByteTxRx(portHandler, motor_id, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
    if result != COMM_SUCCESS:
        print(f"Motor {motor_id} torque disable error: {packetHandler.getTxRxResult(result)}")
    time.sleep(0.05)

# ------------------------------------------- #
# --- Set motors to Extented Position Control Mode --- #
# ------------------------------------------- #
for motor_id in range(1, num_motors + 1):
    result, error = packetHandler.write1ByteTxRx(portHandler, motor_id, ADDR_OPERATING_MODE, EXTENDED_POSITION_MODE)
    if result != COMM_SUCCESS:
        print(f"Motor {motor_id} mode set error: {packetHandler.getTxRxResult(result)}")
    time.sleep(0.05)

print("All motors set to Extented Position Control Mode.")

# ------------------------------------------- #
# --- Enable torque  --- #
# ------------------------------------------- #
for motor_id in range(1, 10):
    packetHandler.write1ByteTxRx(portHandler, motor_id, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
    if result != COMM_SUCCESS:
        print(f"Motor {motor_id} torque enable error: {packetHandler.getTxRxResult(result)}")
    time.sleep(0.05)


# ---------------------------------------------- #
# Set motors to defined position values
# ---------------------------------------------- #
for motor_id in range(1, 10):
    packetHandler.write4ByteTxRx(portHandler, motor_id, ADDR_GOAL_POSITION, safe_baseline_positions[motor_id - 1])
    print(f"Motor {motor_id} set to position {safe_baseline_positions[motor_id - 1]}")

# ------------------------------------------- #


# ---------------------------------------- #
# --- Actuation List  --- #
# ---------------------------------------- #

# Random actuation
act_list = np.zeros([list_length, num_seg, 3])
# Stores cable positions from Dynamixel motors
actual_position_list = np.zeros([list_length, num_seg, 3])
# Stores new motors positions
wanted_position_list = np.zeros([list_length, num_seg, 3])



# --------------------------------------------- #
# Actuation Signal Generation
# Generate random actuation signal for each segment
# Entire arm move synchronously
for i in range(1, thr_list_length):
    # Synchronous: Each segment receives the same random increment
    dif_act_1 = np.random.randint(-dif_act_range, dif_act_range + 1, [3])
    for j in range(3):
        act_list[i, j] = act_list[i - 1, j] + dif_act_1
        # Ensure the actuation is balanced (sum to zero)
        act_list[i, j, i % 3] = -(np.sum(act_list[i, j]) - act_list[i, j, i % 3])
        if np.max(np.abs(act_list[i, j])) > limit_motor:
            # if the actuation exceeds the limit, keep the previous value
            act_list[i, j] = act_list[i - 1, j]

# First two segments move synchronously, the last segment moves independently
for i in range(thr_list_length, thr_list_length * 2):
    # print("2 here: ", i)
    dif_act_2 = np.random.randint(-dif_act_range, dif_act_range + 1, [3, 3])
    dif_act_2[0] = dif_act_2[1]
    for j in range(3):
        act_list[i, j] = act_list[i - 1, j] + dif_act_2[j]
        act_list[i, j, i % 3] = -(np.sum(act_list[i, j]) - act_list[i, j, i % 3])
        if np.max(np.abs(act_list[i, j])) > limit_motor:
            act_list[i, j] = act_list[i - 1, j]

# Each segment moves independently
for i in range(thr_list_length * 2, thr_list_length * 3):
    # print("3 here: ", i)
    dif_act_3 = np.random.randint(-dif_act_range, dif_act_range + 1, [3, 3])
    for j in range(3):
        act_list[i, j] = act_list[i - 1, j] + dif_act_3[j]
        act_list[i, j, i % 3] = -(np.sum(act_list[i, j]) - act_list[i, j, i % 3])
        if np.max(np.abs(act_list[i, j])) > limit_motor:
            act_list[i, j] = act_list[i - 1, j]
# --------------------------------------------- #

# Plot the actuation data, which contains motor commands sent to the soft robotic arm
# 0-th seg act plot
# each line should represents one of the three motors controlling the first segment
# plt.plot(act_list[:, 0, 0]) # motor 1 of segment 0
# plt.plot(act_list[:, 0, 1]) # motor 2 of segment 0
# plt.plot(act_list[:, 0, 2]) # motor 3 of segment 0
# plt.legend(['motor 1', 'motor 2', 'motor 3'])
# plt.xlabel('Time')
# plt.ylabel('Actuation')
# plt.title('Actuation Signal for Segment 0')
# plt.grid()
# plt.show()

# Check if:
# random actuation signal is correctly generated
# motors stays within expected limits
# ask the user to check the plot


# compares the same motor index across all three segments
# plt.plot(act_list[:, 0, 1]) # motor 2 of segment 0
# plt.plot(act_list[:, 1, 1]) # motor 2 of segment 1
# plt.plot(act_list[:, 2, 1]) # motor 2 of segment 2
# plt.legend(['segment 0', 'segment 1', 'segment 2'])
# plt.xlabel('Time')
# plt.ylabel('Actuation')
# plt.title('Actuation Signal for Motor 2')
# plt.grid()
# plt.show()

# Check if:
# motor coordination follows the intended actuation pattern
# we can detect unexpected behavior, such as a motor moving out of sync
# ask the user to check the plot


# save the actuation list to a file based on dif_act_range
# save_path = f"data_optitrack/data{list_length}_actrange{dif_act_range}_date0904_{n}.npz"
# np.savez(save_path, act_list=act_list)
# print(f"Actuation list saved to {save_path}")


print("Start simulation ?")
print("Press Enter to continue")
input()


optitrack_thread = threading.Thread(target=collect_optitrack_data)
optitrack_thread.start()
print("OptiTrack data collection started.")

all_data_rows = []  # This will store data rows temporarily in memory
# save_interval = list_length/300 # Clearly define interval to save (3000/100 = 30)
save_interval = 1

# ----------------------------------------------- #
# ---- Main Control Loop ---- #
# ----------------------------------------------- #

for step in range(thr_list_length*3):

    # print(f"\nStep {step + 1}")

    # Flatten the actuation increments to clearly apply to each motor
    current_actuation = act_list[step].flatten()
    # print("Current actuation:", current_actuation)

    # Calculate new positions explicitly from baseline
    new_motor_positions = [int(base + increment) for base, increment in zip(safe_baseline_positions, current_actuation)]
    # print(f"Motor positions: {new_motor_positions}")
    new_motor_positions_2 = np.array(new_motor_positions).reshape(3, 3)  # Reshape to 3x3 matrix
    wanted_position_list[step] = new_motor_positions_2  # Store new positions

    # Actual positions before movement
    actual_positions = []
    for motor_id in range(1, 10):
        pos, _, _ = packetHandler.read4ByteTxRx(portHandler, motor_id, ADDR_PRESENT_POSITION)
        actual_positions.append(pos)
    # print("Actual positions before movement:", actual_positions)
    # actual_postions to 3*3 matrix
    actual_positions = np.array(actual_positions).reshape(3, 3)  # Reshape to 3x3 matrix
    # Store each motor's position in the cab_pos_list
    actual_position_list[step] = actual_positions  

    # Clearly set new positions
    for motor_id, goal_pos in enumerate(new_motor_positions, start=1):
        packetHandler.write4ByteTxRx(portHandler, motor_id, ADDR_GOAL_POSITION, goal_pos)
    
    # Wait clearly to visually observe movement
    time.sleep(2)

    # Record current OptiTrack positions clearly after motors stabilize
    row = [time.time()]
    with data_lock:
        for name in rigid_body_ids:
            data = latest_data[name]
            if None not in data:
                row += data
            else:
                row += [np.nan]*6  # If no data received yet

    all_data_rows.append(row)

    # Write data every save_interval steps clearly
    if (step + 1) % save_interval == 0:
        # save actual positions to file
        
        with open(f"data_optitrack/data_MASS{list_length}_actrange{dif_act_range}_date2105_night_{n}.npz", "wb") as f:
            print("Saving data...")
            np.savez(f, actual_position_list=actual_position_list, act_list=act_list, wanted_position_list=wanted_position_list)
        csv_writer.writerows(all_data_rows)  # clearly write all collected rows to file
        optitrack_file.flush()  # explicitly flush to disk immediately
        print(f"OptiTrack data saved clearly at step {step + 1}.")
        all_data_rows.clear()  # explicitly clear data in memory after saving


# explicitly save leftover data
if all_data_rows:
    csv_writer.writerows(all_data_rows)
    optitrack_file.flush()
    print("Remaining OptiTrack data explicitly saved at end.")

print("All movements completed safely.")

collecting_data = False
optitrack_thread.join()
print("OptiTrack data collection stopped.")

optitrack_file.close()
print(f"OptiTrack data collection completed. Data saved to {csv_filename}.")

print("Simulation over, press disable torque.")
input()

# ------------------------------------------- #
# --- Disable torque  --- #
# ------------------------------------------- #
for motor_id in range(1, 10):
    packetHandler.write1ByteTxRx(portHandler, motor_id, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
    if result != COMM_SUCCESS:
        print(f"Motor {motor_id} torque disable error: {packetHandler.getTxRxResult(result)}")
    time.sleep(0.05)

print("Torque disabled for all motors.")

# Clean up
portHandler.closePort()
print("Port closed.")