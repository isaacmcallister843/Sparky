import board
import busio
import adafruit_pca9685
from adafruit_servokit import ServoKit
import time 
import numpy as np 
import threading

# Initialize the I2C bus
i2c = busio.I2C(board.SCL, board.SDA)

# Initialize the PCA9685
pca = adafruit_pca9685.PCA9685(i2c)
pca.frequency = 60

# Initialize the servo controller
kit = ServoKit(channels=16, i2c=i2c)


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

        self.motorKit.servo[self.topMotor_Address].angle = topMotor_0_deg
        self.motorKit.servo[self.middleMotor_Address].angle = middleMotor_90_deg
        self.motorKit.servo[self.bottomMotor_Address].angle = bottomMottor_90_deg

        self.motorMatrix = np.array([[0, 1, 2], 
                                     [0, 90, 90], 
                                     [self.topMotor_0_deg, self.middleMotor_90_deg, self.bottomMottor_90_deg],
                                     [self.topMotor_Address, self.middleMotor_Address, self.bottomMotor_Address]])
        
        time.sleep(1)
    
    def moveMotor(self, motorNumber, targetAngle): 
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
            time.sleep(.02) 
            self.motorKit.servo[self.motorMatrix[3, motorNumber]].angle = i 

        print(finalRealAngle)
        self.motorKit.servo[self.motorMatrix[3, motorNumber]].angle = finalRealAngle
        time.sleep(.1)
        self.motorKit.servo[self.motorMatrix[3, motorNumber]].angle = finalRealAngle
        self.motorMatrix[2, motorNumber] = finalRealAngle
        time.sleep(.5)

    
topLeftArm = arm(3, 4, 5, kit)
topLeftArm.calibrateArm(98, 0, 175,1)

topRightArm = arm(12, 11, 10, kit)
topRightArm.calibrateArm(12, 167, 30, -1)

bottomLeftArm = arm(0, 1, 2, kit)
bottomLeftArm.calibrateArm(130, 25, 150, 1)

bottomRightArm = arm(15, 14, 13, kit)
bottomRightArm.calibrateArm(38, 170, 25, -1)


def AllMotorTest():
    bottomRightArm.moveMotor(0, 90)
    bottomRightArm.moveMotor(1, 0)
    bottomRightArm.moveMotor(2, 180)
    bottomRightArm.moveMotor(2, 90)
    bottomRightArm.moveMotor(1, 90)
    bottomRightArm.moveMotor(0, 0)

    topRightArm.moveMotor(0, 90)
    topRightArm.moveMotor(1, 0)
    topRightArm.moveMotor(2, 180)
    topRightArm.moveMotor(2, 90)
    topRightArm.moveMotor(1, 90)
    topRightArm.moveMotor(0, 0)


    topLeftArm.moveMotor(0, 90)
    topLeftArm.moveMotor(1, 0)
    topLeftArm.moveMotor(2, 180)
    topLeftArm.moveMotor(2, 90)
    topLeftArm.moveMotor(1, 90)
    topLeftArm.moveMotor(0, 0)


# Assuming topLeftArm is already defined

thread1 = threading.Thread(target=topLeftArm.moveMotor, args=(0, 10))
thread2 = threading.Thread(target=topRightArm.moveMotor, args=(0, 10))
thread3 = threading.Thread(target=bottomRightArm.moveMotor, args=(0, 10))
thread4 = threading.Thread(target=bottomLeftArm.moveMotor, args=(0, 10))

thread1.start()
thread2.start()
thread3.start()
thread4.start()

thread1.join()
thread2.join()
thread3.join()
thread4.join()

# Create threads
thread1 = threading.Thread(target=topLeftArm.moveMotor, args=(1, -40))
thread2 = threading.Thread(target=topLeftArm.moveMotor, args=(2, 180))

thread3 = threading.Thread(target=topRightArm.moveMotor, args=(1, -40))
thread4 = threading.Thread(target=topRightArm.moveMotor, args=(2, 180))

thread5 = threading.Thread(target=bottomRightArm.moveMotor, args=(1, -40))
thread6 = threading.Thread(target=bottomRightArm.moveMotor, args=(2, 180))

thread7 = threading.Thread(target=bottomLeftArm.moveMotor, args=(1, -40))
thread8 = threading.Thread(target=bottomLeftArm.moveMotor, args=(2, 180))

# Start threads

thread5.start()
thread6.start()
thread7.start()
thread8.start()
thread1.start()
thread2.start()
thread3.start()
thread4.start()



# Wait for both threads to complete
thread1.join()
thread2.join()
thread3.join()
thread4.join()
thread5.join()
thread6.join()
thread7.join()
thread8.join()

time.sleep(4)
thread1 = threading.Thread(target=topLeftArm.moveMotor, args=(1, 90))
thread2 = threading.Thread(target=topLeftArm.moveMotor, args=(2, 90))

thread3 = threading.Thread(target=topRightArm.moveMotor, args=(1, 90))
thread4 = threading.Thread(target=topRightArm.moveMotor, args=(2, 90))

thread5 = threading.Thread(target=bottomRightArm.moveMotor, args=(1, 90))
thread6 = threading.Thread(target=bottomRightArm.moveMotor, args=(2, 90))

thread7 = threading.Thread(target=bottomLeftArm.moveMotor, args=(1, 90))
thread8 = threading.Thread(target=bottomLeftArm.moveMotor, args=(2, 90))

# Start threads

thread5.start()
thread6.start()
thread7.start()
thread8.start()

time.sleep(.5)
thread1.start()
thread2.start()
thread3.start()
thread4.start()

# Wait for both threads to complete
thread1.join()
thread2.join()
thread3.join()
thread4.join()
thread5.join()
thread6.join()
thread7.join()
thread8.join()




thread1 = threading.Thread(target=topLeftArm.moveMotor, args=(0, 0))
thread2 = threading.Thread(target=topRightArm.moveMotor, args=(0, 0))
thread3 = threading.Thread(target=bottomRightArm.moveMotor, args=(0, 0))
thread4 = threading.Thread(target=bottomLeftArm.moveMotor, args=(0, 0))

thread1.start()
thread2.start()
thread3.start()
thread4.start()

thread1.join()
thread2.join()
thread3.join()
thread4.join()