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
sys.path.append('/home/rpirobot/Desktop/Main/Sparky/src/Motors.py')  

from src.Motors import arm 
from src.Motors import MotorData 

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
    0: MotorData(motor_speed=0.02, target_angle=10),
}

all_threads = []

all_threads.append(topLeftArm.generateSyncThreads(motor_data))
all_threads.append(topRightArm.generateSyncThreads(motor_data))
all_threads.append(bottomRightArm.generateSyncThreads(motor_data))
all_threads.append(bottomLeftArm.generateSyncThreads(motor_data))

threadProccessing(all_threads)

# Stand up
motor_data = {
    1: MotorData(motor_speed=0.02, target_angle=-40),
    2: MotorData(motor_speed=0.02, target_angle=180)
}

all_threads = []
all_threads.append(topLeftArm.generateSyncThreads(motor_data))
all_threads.append(topRightArm.generateSyncThreads(motor_data))
all_threads.append(bottomRightArm.generateSyncThreads(motor_data))
all_threads.append(bottomLeftArm.generateSyncThreads(motor_data))

threadProccessing(all_threads)

# One leg up 
input("Press Enter to continue...")

## read data

# Step 1: Open the file in read mode
with open('src/testData/output.csv', 'r') as file:
    # Step 2: Read the contents of the file
    lines = file.readlines()


values = lines[0].strip().split(',')

# Step 3: Split the contents into individual values

# Step 4: Assign each value to a variable
var1 = values[0]
var2 = values[1]
var3 = values[2]
