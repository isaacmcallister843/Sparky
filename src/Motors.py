# General Imports
import board
import busio
import adafruit_pca9685
from adafruit_servokit import ServoKit
import time 
import threading
from typing import Dict
import numpy as np 

# Control dictionary for the motors
class MotorData:
    def __init__(self, motor_speed: int, target_angle: int):
        self.motor_speed = motor_speed
        self.target_angle = target_angle


# Dictionary we will use to control multiple motors 
#motor_data = {
#    0: MotorData(motor_speed=0, target_angle=0),
#    1: MotorData(motor_speed=0, target_angle=0),
#    2: MotorData(motor_speed=0, target_angle=0),
#}


# Controls All the arms 
class arm: 
    def __init__(self, topMotor_Address: int, middleMotor_Address: int, bottomMotor_Address: int, kit: ServoKit):
        self.topMotor_Address = topMotor_Address
        self.middleMotor_Address = middleMotor_Address
        self.bottomMotor_Address = bottomMotor_Address
        self.motorKit = kit

        self.topMotor_0_deg = None
        self.middleMotor_90_deg = None 
        self.bottomMottor_90_deg = None

        self.topAngle_Corrected = 0
        self.middleAngle_Corrected = 90 
        self.bottomAngle_Corrected = 90 

        self.ccw = None

        self.motorMatrix = None; 
    
    def calibrateArm(self, topMotor_0_deg, middleMotor_90_deg, bottomMottor_90_deg, ccw):
        self.topMotor_0_deg = topMotor_0_deg
        self.middleMotor_90_deg = middleMotor_90_deg
        self.bottomMottor_90_deg = bottomMottor_90_deg
        self.ccw = ccw

        #self.motorKit.servo[self.topMotor_Address].angle = topMotor_0_deg
        #self.motorKit.servo[self.middleMotor_Address].angle = middleMotor_90_deg
        #self.motorKit.servo[self.bottomMotor_Address].angle = bottomMottor_90_deg

        self.motorMatrix = np.array([[0, 1, 2], 
                                     [0, 90, 90], 
                                     [self.topMotor_0_deg, self.middleMotor_90_deg, self.bottomMottor_90_deg],
                                     [self.topMotor_Address, self.middleMotor_Address, self.bottomMotor_Address]])
        
        time.sleep(1)
    
    def moveMotor(self, motorNumber, targetAngle, motorDelay = .02): 
        startingPos = self.motorMatrix[1, motorNumber]
        targetPos = targetAngle
        self.motorMatrix[1, motorNumber] = targetAngle

        # Check if we need to count up or down 
        if startingPos > targetAngle:
            it = 1 
        else:
            it = -1 

        startingRealAngle = self.motorMatrix[2, motorNumber]
        angleChange = startingPos - targetPos
        finalRealAngle = startingRealAngle + angleChange * self.ccw

        for i in range(startingRealAngle, finalRealAngle, it*self.ccw):
            time.sleep(motorDelay) 
            self.motorKit.servo[self.motorMatrix[3, motorNumber]].angle = i 

        self.motorKit.servo[self.motorMatrix[3, motorNumber]].angle = finalRealAngle
        time.sleep(.1)
        self.motorKit.servo[self.motorMatrix[3, motorNumber]].angle = finalRealAngle
        self.motorMatrix[2, motorNumber] = finalRealAngle
        time.sleep(.5)
        
        print(motorNumber)
        print(self.motorMatrix)
        print("-----------------")

    def generateSyncThreads(self, motor_data: Dict[int, MotorData]):
            threads = []

            for key, value in motor_data.items():
                newThread = threading.Thread(target=self.moveMotor, args=(key, value.target_angle, value.motor_speed))
                threads.append(newThread)
            
            return threads


class SparkySkeleton():
    def __init__(self, topLeftArm: arm, topRightArm:arm, bottomLeftArm: arm, bottomRightArm: arm):
        self.armList = [topLeftArm, topRightArm, bottomLeftArm, bottomRightArm]
    
