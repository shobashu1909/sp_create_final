# Soft Robotic Arm - Kinematic Modeling and Evaluation

This repository contains all the necessary code, data, and analysis notebooks related to the modeling, control, and physical evaluation of a soft robotic arm. The focus lies in developing and comparing neural network-based forward and inverse kinematic models.

## 📁 Folder Structure & Key Files

### 🔍 Data Visualization & Verification

- **`data_collection_viz.ipynb`**  
  Visualizes the collected soft arm motion data to verify coverage and consistency of the workspace.

- **`data_verification.ipynb`**  
  Contains the forward kinematic model evaluation comparing predicted versus true end-effector positions on physical tests.

- **`data_verification_ik_[].ipynb`**  
  Evaluates the inverse kinematic models using physical test data. The suffix (`3inputs`, `9inputs`, `MASS`) corresponds to the input configuration used:
  - `3inputs`: Only end-effector position.
  - `9inputs`: Position of all three sections.
  - `MASS`: With external load at the end-effector.

### 🧠 Model Building

- **`inv_kinematic_[].ipynb`**  
  Trains the inverse kinematic neural network. Includes models for:
  - `3inputs`, `9inputs`, `MASS`
  - `weighted_kinematic`: Includes positional weighting schemes.

- **`kinematic_[].ipynb`**  
  Trains the forward kinematic model using actuator lengths as input and predicting the position of each section.

### 🤖 Real-time Execution

- **`opti_test_model_with_current.py`**  
  Main script to run on the soft arm hardware. Collects real-time motion data using OptiTrack while first stabilizing in **current mode**, and then operating in **extended position mode**.

### 📂 Data

- **`data_optitrack/`**  
  Contains the motion capture datasets collected using the OptiTrack system.

- **`csv_files/`**  
  Includes processed datasets used during training and evaluation.

## 🛠 Setup

Install all necessary dependencies using the provided `requirements.txt`:

```bash
pip install -r requirements.txt
