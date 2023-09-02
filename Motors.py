import board
import busio
import adafruit_pca9685
from adafruit_servokit import ServoKit
import time 

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
    
    def calibrateArm(self, topMotor_0_deg, middleMotor_90_deg, bottomMottor_90_deg):
        self.topMotor_0_deg = topMotor_0_deg
        self.middleMotor_90_deg = middleMotor_90_deg
        self.bottomMottor_90_deg = bottomMottor_90_deg

        self.motorKit.servo[self.topMotor_Address].angle = topMotor_0_deg
        self.motorKit.servo[self.middleMotor_Address].angle = middleMotor_90_deg
        self.motorKit.servo[self.bottomMotor_Address].angle = bottomMottor_90_deg
    
    


topLeftArm = arm(3, 4, 5, kit)
topLeftArm.calibrateArm(100, 0, 170)

topRightArm = arm(12, 11, 10, kit)
topRightArm.calibrateArm(12, 167, 30)

bottomLeftArm = arm(0, 1, 2, kit)
bottomLeftArm.calibrateArm(65, 25, 150)

bottomRightArm = arm(15, 14, 13, kit)
bottomRightArm.calibrateArm(30, 170, 20)