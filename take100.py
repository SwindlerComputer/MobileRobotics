#!/usr/bin/env python3

from ev3dev2.motor import MoveTank, OUTPUT_A, OUTPUT_B
from ev3dev2.sensor import INPUT_1, INPUT_2, INPUT_3, INPUT_4
from ev3dev2.sensor.lego import TouchSensor, GyroSensor, ColorSensor, UltrasonicSensor
from time import sleep

# Initialize sensors
touch_sensor = TouchSensor(INPUT_1)
gyro_sensor = GyroSensor(INPUT_2)
color_sensor = ColorSensor(INPUT_3)
ultrasonic_sensor = UltrasonicSensor(INPUT_4)

# Initialize motors
tank_drive = MoveTank(OUTPUT_A, OUTPUT_B)

# Print "Assignment 1" on the screen
print("Assignment 1")

# Wait for a button to be pressed
while not touch_sensor.is_pressed:
    sleep(0.1)

# Clear the screen
print("\x1B[H\x1B[2J")

# Move forward until the sonar detects an obstacle at less than 25 cm distance
tank_drive.on(50, 50)  # Adjust speed as needed
while ultrasonic_sensor.distance_centimeters > 25:
    sleep(0.1)
tank_drive.off()

# Turn 180 degrees using the gyroscope to measure angle
gyro_sensor.mode = 'GYRO-ANG'
while gyro_sensor.angle < 180:
    tank_drive.on(-50, 50)  # Adjust speed as needed
tank_drive.off()

# Move forward 20 units
tank_drive.on_for_rotations(50, 50, 1)  # Adjust distance as needed

# Turn 90 degrees to the left
gyro_sensor.mode = 'GYRO-ANG'
while gyro_sensor.angle < 90:
    tank_drive.on(-50, 50)  # Adjust speed as needed
tank_drive.off()

# Move forward until detecting a dark surface underneath
while color_sensor.reflected_light_intensity > 30:  # Adjust threshold as needed
    tank_drive.on(50, 50)  # Adjust speed as needed
tank_drive.off()

# Stop
tank_drive.off()

# Rotate 90 degrees to the left
gyro_sensor.mode = 'GYRO-ANG'
while gyro_sensor.angle < 90:
    tank_drive.on(-50, 50)  # Adjust speed as needed
tank_drive.off()

# Move backward until the touch sensor causes the robot to stop
while not touch_sensor.is_pressed:
    tank_drive.on(-50, -50)  # Adjust speed as needed
tank_drive.off()
