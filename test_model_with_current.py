import sys
import time
import numpy as np
from dynamixel_sdk import *  # Uses Dynamixel SDK library
import torch
import joblib 
from nn import KinematicMLP

# # ---------------------------------------- #
# # --- Neural Network Prediction --- #
# # ---------------------------------------- #
# # Load model
# model = KinematicMLP(input_size=9, output_size=9)
# model.load_state_dict(torch.load("kinematic_model.pth"))
# model.eval()

# Load actuation sample from saved dataset
# scaler_X = joblib.load("scaler_X.save")
# scaler_Y = joblib.load("scaler_Y.save")

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
    packetHandler.write1ByteTxRx(portHandler, motor_id, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
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
    packetHandler.write1ByteTxRx(portHandler, motor_id, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
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


print("Start simulation ?")
print("Press Enter to continue")
input()

# ---------------------------------------- #
# --- Test  --- #
# ---------------------------------------- #

# Define Motion Parameters
num_seg = 3
num_motor = 3 * num_seg
thr_list_length = 3 # threshold list length

list_length = thr_list_length * 3
# dif_act_range = 150 # different actuation range; try later with 300; 400;
# limit_motor = 2000 # Try later with 3000
dif_act_range = 150 # different actuation range; try later with 300; 400;
limit_motor = 3000 # Try later with 3000

# Random actuation
act_list = np.zeros([list_length, num_seg, 3])

# --------------------------------------------- #
# Actuation Signal Generation
# Generate random actuation signal for each segment
# Entire arm move synchronously
for i in range(1, thr_list_length):
    # print("1 here: ", i)
    dif_act_1 = np.random.randint(-dif_act_range, dif_act_range + 1, [3])
    for j in range(3):
        act_list[i, j] = act_list[i - 1, j] + dif_act_1
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


# initialiaze input for the model
delta_L_list = []  

n = 2

# save the actuation list to a file based on dif_act_range
# save_path = f"data/new_act_list_{dif_act_range}_{n}.npz"
# np.savez(save_path, act_list=act_list)
# print(f"Actuation list saved to {save_path}")

# Perform 30 movements explicitly with delays
for step in range(thr_list_length*3):

    print(f"\nStep {step + 1}")

    # Flatten the actuation increments to clearly apply to each motor
    current_actuation = act_list[step].flatten()
    print("Current actuation:", current_actuation)

    # Calculate new positions explicitly from baseline
    new_motor_positions = [int(base + increment) for base, increment in zip(safe_baseline_positions, current_actuation)]

    print(f"Motor positions: {new_motor_positions}")

    # print("Enter to continue")
    # input()

    # Clearly set new positions
    for motor_id, goal_pos in enumerate(new_motor_positions, start=1):
        packetHandler.write4ByteTxRx(portHandler, motor_id, ADDR_GOAL_POSITION, goal_pos)

    # Measure actual positions after each movement
    actual_positions = []
    for motor_id in range(1, 10):
        pos, _, _ = packetHandler.read4ByteTxRx(portHandler, motor_id, ADDR_PRESENT_POSITION)
        actual_positions.append(pos)

    print("Actual positions after movement:", actual_positions)

    # Compute delta L relative to safe baseline
    # delta_L_ticks = [a - b for a, b in zip(actual_positions, safe_baseline_positions)]
    # delta_L_ticks = np.array(delta_L_ticks)  # encoder ticks
    # print("Delta L (ticks):", delta_L_ticks)
    # # Convert to mm:
    # delta_L_mm = delta_L_ticks * (62.83 / 4096) # 62.83 = 20 mm * pi
    # # Show delta in shape of 3x3 format
    # print("Delta L (mm):")
    # print(delta_L_mm.reshape(3,3))

    # # Append to the list for model input
    # delta_L_list.append(delta_L_ticks)

    # Wait clearly to visually observe movement
    time.sleep(2)

print("All movements completed safely.")

# ### PREDICTIONS ###
# predicted_positions = []

# # Iterate clearly through stored delta_L values and predict
# for i, delta_L in enumerate(delta_L_list, start=1):

#     # Normalize clearly
#     X_real_norm = scaler_X.transform([delta_L])

#     # Predict clearly
#     with torch.no_grad():
#         input_tensor = torch.tensor(X_real_norm, dtype=torch.float32)
#         Y_pred_norm = model(input_tensor).numpy()

#     # De-normalize prediction
#     Y_pred_mm = scaler_Y.inverse_transform(Y_pred_norm).reshape(3,3)

#     print(f"\nStep {i} - Model Predicted positions :")
#     print(Y_pred_mm)

#     # save the predicted positions to a list
#     predicted_positions.append(Y_pred_mm)

# # save the predicted positions based on dif_act_range
# save_path = f"data/predicted_positions_{dif_act_range}_{n}.npz"
# np.savez(save_path, predicted_positions=predicted_positions)

# print(f"Predicted positions saved to {save_path}")

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



# actual_positions = []
# for motor_id in range(1, 10):
#     pos, _, _ = packetHandler.read4ByteTxRx(portHandler, motor_id, ADDR_PRESENT_POSITION)
#     actual_positions.append(pos)

# print("Set Positions:", safe_baseline_positions)
# print("Actual Positions:", actual_positions)


# for motor_id in range(1, num_motors + 1):
#     result, error = packetHandler.write1ByteTxRx(portHandler, motor_id, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
#     if result != COMM_SUCCESS:
#         print(f"Motor {motor_id} torque enable error: {packetHandler.getTxRxResult(result)}")
#     time.sleep(0.05)

# print("Torque enabled for all motors.")

# ------------------------------------------- #
# --- Set to safe baseline  --- #
# ------------------------------------------- #
# Set motors to safe baseline positions


# ----------------------------------------------------- #
# --- Define Baseline and Move to Desired Positions --- #
# ----------------------------------------------------- #
# baseline_positions = []
# for motor_id in range(1, num_motors + 1):
#     pos, result, error = packetHandler.read4ByteTxRx(portHandler, motor_id, ADDR_PRESENT_POSITION)
#     baseline_positions.append(pos)
#     time.sleep(0.05)

# print("Baseline Positions:", baseline_positions)


# # Enable torque
# for motor_id in range(1, 10):
#     packetHandler.write1ByteTxRx(portHandler, motor_id, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)

# print("Sending actuation to motors:")
# for motor_id, goal_pos in enumerate(act_sample, start=1):
#     dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(
#         portHandler, motor_id, ADDR_GOAL_POSITION, goal_pos
#     )
#     print(f"Motor {motor_id} → Position {goal_pos}")

# # --- Model Prediction ---
# # Load model
# model_sec = joblib.load("a_files/mplregressor_model.pkl")

# # model_full = your trained model
# # scaler_X_full, scaler_Y_full = your fitted scalers



# # Load actuation sample from saved dataset
# data = np.load("a_files/dataset.npz")  # <-- path to your collected dataset
# act_list = data["act_list"]  # (9000, 3, 3)

# # Choose a test index where motion is clear and safe
# test_idx = 6500
# act_sample = act_list[test_idx].flatten().astype(int)  # shape (9,)



# # Predict using the same input
# X_input = act_sample.reshape(1, -1)
# X_input_norm = scaler_X_full.transform(X_input)
# y_pred = model_full.predict(X_input_norm)
# y_pred_denorm = scaler_Y_full.inverse_transform(y_pred)
# predicted_pos = y_pred_denorm[0, 9:12]

# print("\n Predicted End-Effector Position:")
# print("X:", round(predicted_pos[0], 4), "m")
# print("Y:", round(predicted_pos[1], 4), "m")
# print("Z:", round(predicted_pos[2], 4), "m")
