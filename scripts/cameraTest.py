from picamera import PiCamera
from time import sleep

camera = PiCamera()
sleep(3)
camera.capture('/home/rpirobot/Desktop/Main/test.jpg')
