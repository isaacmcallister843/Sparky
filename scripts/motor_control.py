################################################################################ 
#                                 IMPORTS                                      #
################################################################################ 

# ================== STANDARD LIBRARY IMPORTS ==================
import time
import threading

# ================== THIRD-PARTY LIBRARY IMPORTS ==================
import board
import busio
import numpy as np
from adafruit_servokit import ServoKit
import adafruit_pca9685

# ================== LOCAL MODULE IMPORTS ==================
from src.Motors import arm 


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


################################################################################ 
#                              GENERAL TESTS                                   #
################################################################################ 

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


################################################################################ 
#                              STANDING UP                                     #
################################################################################ 

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
