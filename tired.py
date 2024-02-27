#!/usr/bin/env python3

# Assignment 1 - Mobile Robotics Labs #3, #4, #5
# Group Members: [Your Names and Student Numbers]

import ev3dev2.sensor.lego as sensors
import ev3dev2.motor as motors
import time

# Initialize sensors
sonar = sensors.UltrasonicSensor()
touch = sensors.TouchSensor()
light = sensors.ColorSensor()
gyro = sensors.GyroSensor()

# Initialize motors
left_motor = motors.LargeMotor('outB')
right_motor = motors.LargeMotor('outC')

# Define constants
SONAR_THRESHOLD = 25  # Threshold distance for obstacle detection (in cm)
FORWARD_SPEED = 30    # Speed for forward movement
TURN_SPEED = 20       # Speed for turning
ROTATION_ANGLE = 180  # Angle for turning 180 degrees
MOVE_DISTANCE = 20    # Distance to move forward (in units)
TURN_ANGLE = 90       # Angle to turn left
BACKWARD_SPEED = -30  # Speed for backward movement

# Function to print "Assignment 1" on the screen
def print_assignment():
    print("Assignment 1")

# Function to wait for a button press
def wait_for_button_press():
    input("Press any key to continue...")

# Function to clear the screen
def clear_screen():
    print("\033c", end="")

# Function to move forward until obstacle detected within 25 cm
def move_forward_until_obstacle():
    left_motor.on(FORWARD_SPEED)
    right_motor.on(FORWARD_SPEED)
    while sonar.distance_centimeters <= SONAR_THRESHOLD:
        time.sleep(0.1)
    left_motor.off()
    right_motor.off()

# Function to turn 180 degrees using gyroscope
def turn_180_degrees():
    gyro.reset()
    left_motor.on(TURN_SPEED)
    right_motor.on(-TURN_SPEED)
    while abs(gyro.angle) < ROTATION_ANGLE:
        time.sleep(0.1)
    left_motor.off()
    right_motor.off()

# Function to move forward 20 units
def move_forward_20_units():
    left_motor.on_for_rotations(FORWARD_SPEED, motors.UnitRotation(MOVE_DISTANCE))

# Function to turn 90 degrees to the left
def turn_90_degrees_left():
    gyro.reset()
    left_motor.on(-TURN_SPEED)
    right_motor.on(TURN_SPEED)
    while abs(gyro.angle) < TURN_ANGLE:
        time.sleep(0.1)
    left_motor.off()
    right_motor.off()

# Function to move forward until detecting a dark surface
def move_forward_until_dark_surface():
    left_motor.on(FORWARD_SPEED)
    right_motor.on(FORWARD_SPEED)
    while light.reflected_light_intensity > 30:  # Adjust the threshold as needed
        time.sleep(0.1)
    left_motor.off()
    right_motor.off()

# Function to rotate 90 degrees to the left
def rotate_90_degrees_left():
    left_motor.on_for_rotations(BACKWARD_SPEED, motors.UnitRotation(0.5))

# Function to move backward until touch sensor triggered
def move_backward_until_touch():
    left_motor.on(BACKWARD_SPEED)
    right_motor.on(BACKWARD_SPEED)
    while not touch.is_pressed:
        time.sleep(0.1)
    left_motor.off()
    right_motor.off()

# Main function to execute the assignment tasks
def main():
    print_assignment()
    wait_for_button_press()
    clear_screen()
    
    move_forward_until_obstacle()
    turn_180_degrees()
    move_forward_20_units()
    turn_90_degrees_left()
    move_forward_until_dark_surface()
    rotate_90_degrees_left()
    move_backward_until_touch()

# Execute the main function
if __name__ == "__main__":
    main()
