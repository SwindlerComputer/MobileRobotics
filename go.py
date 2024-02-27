#!/usr/bin/env python3

from ev3dev2.motor import LargeMotor, OUTPUT_B, OUTPUT_D
from ev3dev2.sensor.lego import UltrasonicSensor, GyroSensor
from time import sleep

# Constants
ON = True
OFF = False

# Initialize motors, ultrasonic sensor, and gyroscope
motor = LargeMotor(OUTPUT_B)
motor1 = LargeMotor(OUTPUT_D)
sonar = UltrasonicSensor()
gyro = GyroSensor(INPUT_2)

# Print "Assignment 1" on the screen
print("Assignment 1")

# Wait for a button to be pressed
while not sonar.distance_centimeters < 25:
    motor.on(50)
    motor1.on(50)
    sleep(0.1)
motor.off()
motor1.off()

# Turn 180 degrees using the gyroscope to measure angle
gyro.mode = 'GYRO-ANG'
while gyro.angle < 180:
    motor.on(-50)
    motor1.on(50)
    sleep(0.1)
motor.off()
motor1.off()

# Move forward 20 units
motor.on_for_rotations(50, 1)
motor1.on_for_rotations(50, 1)

# Turn 90 degrees to the left
gyro.mode = 'GYRO-ANG'
while gyro.angle < 90:
    motor.on(-50)
    motor1.on(50)
    sleep(0.1)
motor.off()
motor1.off()

# Move forward until detecting a dark surface underneath
while True:
    if color.reflected_light_intensity < 30:
        motor.on(50)
        motor1.on(50)
        sleep(0.1)
    else:
        break
motor.off()
motor1.off()

# Stop
motor.off()
motor1.off()

# Rotate 90 degrees to the left
gyro.mode = 'GYRO-ANG'
while gyro.angle < 90:
    motor.on(-50)
    motor1.on(50)
    sleep(0.1)
motor.off()
motor1.off()

# Move backward until the touch sensor causes the robot to stop
while not touch.is_pressed:
    motor.on(-50)
    motor1.on(-50)
motor.off()
motor1.off()
