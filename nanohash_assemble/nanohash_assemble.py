from pca9685 import PCA9685
from machine import I2C, Pin
from servo import Servos



sda = Pin(8)
scl = Pin(9)
id = 0
i2c = I2C(0, sda=sda, scl=scl)

pca = PCA9685(i2c=i2c)
servo = Servos(i2c=i2c)
servo.position(index=0, degrees=90) #Right Leg 1
servo.position(index=1, degrees=90) #Right Leg 2
servo.position(index=2, degrees=90) #Right Leg 3
servo.position(index=3, degrees=90) #Left Leg 1
servo.position(index=4, degrees=90) #Left Leg 2
servo.position(index=5, degrees=90) #Left Leg 3
servo.position(index=6, degrees=90) #Right Hand 1
servo.position(index=7, degrees=0) #Right Hand 2
servo.position(index=8, degrees=0) #Right Hand 3
servo.position(index=9, degrees=90) #Left Hand 1
servo.position(index=10, degrees=180) #Left Hand 2
servo.position(index=11, degrees=180) #Left Hand 3
servo.position(index=12, degrees=90) #Head


