################################################################################ 
#                                 IMPORTS                                      #
################################################################################ 

# ================== STANDARD LIBRARY IMPORTS ==================
import time
import threading
import keyboard # not in req just for testing

# ================== THIRD-PARTY LIBRARY IMPORTS ==================
import board
import busio
import numpy as np
from adafruit_servokit import ServoKit
import adafruit_pca9685


# ================== LOCAL MODULE IMPORTS ==================

import sys
sys.path.append('/home/rpirobot/Desktop/Main/Sparky/src/')  

from Motors import arm 
from Motors import MotorData_Base 

################################################################################ 
#                              INITIALIZATIONS                                 #  
################################################################################ 

# ================ INITIALIZE THE I2C BUS ================
i2c = busio.I2C(board.SCL, board.SDA)

# ================ INITIALIZE THE PCA9685 ================
pca = adafruit_pca9685.PCA9685(i2c)
pca.frequency = 60

# ================ Initialize the servo controller ========
kit = ServoKit(channels=16, i2c=i2c)

################################################################################ 
#                              CALIBRATION                                     #
################################################################################ 

topLeftArm = arm(3, 4, 5, kit)
topLeftArm.calibrateArm(98, 0, 175,1)

topRightArm = arm(12, 11, 10, kit)
topRightArm.calibrateArm(12, 167, 30, -1)

bottomLeftArm = arm(0, 1, 2, kit)
bottomLeftArm.calibrateArm(130, 25, 150, 1)

bottomRightArm = arm(15, 14, 13, kit)
bottomRightArm.calibrateArm(38, 170, 25, -1)


def threadProccessing(all_threads): 
    all_threads = [item for sublist in all_threads for item in sublist]
    # Use thread pools in the future this is a dumb way to handle this 

    for thread in all_threads:
        thread.start()

    for thread in all_threads:
        thread.join()

### Init

# Engage Arms 
motor_data = {
    0: MotorData_Base(motor_speed=0.02, target_angle=8),
}

all_threads = []

all_threads.append(topLeftArm.generateSyncThreads(motor_data))
all_threads.append(topRightArm.generateSyncThreads(motor_data))
all_threads.append(bottomRightArm.generateSyncThreads(motor_data))
all_threads.append(bottomLeftArm.generateSyncThreads(motor_data))

threadProccessing(all_threads)

# Stand up
motor_data = {
    1: MotorData_Base(motor_speed=0.02, target_angle=-40),
    2: MotorData_Base(motor_speed=0.02, target_angle=180)
}

all_threads = []
all_threads.append(topLeftArm.generateSyncThreads(motor_data))
all_threads.append(topRightArm.generateSyncThreads(motor_data))
all_threads.append(bottomRightArm.generateSyncThreads(motor_data))
all_threads.append(bottomLeftArm.generateSyncThreads(motor_data))

threadProccessing(all_threads)

input("Press Enter to continue...")

## read data

# Step 1: Open the file in read mode
WalkCycle_1 = np.load('/home/rpirobot/Desktop/Main/Sparky/testData/Walk_Cycle_Sync_1.npy')
WalkCycle_2 = np.load('/home/rpirobot/Desktop/Main/Sparky/testData/Walk_Cycle_Sync_2.npy')

for i in range(0, WalkCycle_1.shape[0]-1): 
    q1 = WalkCycle_1[:, 0][i]
    q2 = WalkCycle_1[:, 1][i] 
    print(q1)
    print(q2)
    print("1 --------")
    topLeftArm.moveMotor(1, q1)
    topLeftArm.moveMotor(2, q2)
    bottomRightArm.moveMotor(1, q1)
    bottomRightArm.moveMotor(2, q2)

    q1 = WalkCycle_2[:, 0][i]
    q2 = WalkCycle_2[:, 1][i] 

    print(q1)
    print(q2)
    print("2 --------")
    topRightArm.moveMotor(1, q1)
    topRightArm.moveMotor(2, q2)
    bottomLeftArm.moveMotor(1, q1)
    bottomLeftArm.moveMotor(2, q2)

    time.sleep(.15)

input("Press Enter to continue...")

#Sit down 
motor_data = {
    1: MotorData_Base(motor_speed=0.02, target_angle=90),
    2: MotorData_Base(motor_speed=0.05, target_angle=90)
}

all_threads = []
all_threads.append(topLeftArm.generateSyncThreads(motor_data))
all_threads.append(topRightArm.generateSyncThreads(motor_data))

threadProccessing(all_threads)

time.sleep(1)
all_threads = []
all_threads.append(bottomRightArm.generateSyncThreads(motor_data))
all_threads.append(bottomLeftArm.generateSyncThreads(motor_data))

threadProccessing(all_threads)