#!/usr/bin/env python3
'''Hello to the world from ev3dev.org'''

import time
from ev3dev2.motor import LargeMotor, OUTPUT_B, OUTPUT_D
from ev3dev2.sensor.lego import UltrasonicSensor, GyroSensor, ColorSensor, TouchSensor
from ev3dev2.sensor import INPUT_2, INPUT_3, INPUT_1

# Initialize motors, ultrasonic sensor, and gyroscope
motor = LargeMotor(OUTPUT_B)
motor1 = LargeMotor(OUTPUT_D)
sonar = UltrasonicSensor()
gyro = GyroSensor(INPUT_2)
color_sensor = ColorSensor(INPUT_3)
touch_sensor = TouchSensor(INPUT_1)

def display_message(message):
    '''Display message on the console'''
    print(message)

def wait_for_button_press():
    '''Wait for a button press event'''
    while True:
        if touch_sensor.is_pressed:
            break
        time.sleep(0.1)

def move_forward_until_obstacle():
    '''Move forward until an obstacle is detected'''
    motor.run_forever(speed_sp=500)
    motor1.run_forever(speed_sp=500)
    while True:
        distance = sonar.distance_centimeters
        if distance < 25:
            motor.stop()
            motor1.stop()
            break
        time.sleep(0.1)

def turn_180_degrees():
    '''Turn the robot 180 degrees using gyroscope'''
    gyro.mode = 'GYRO-RATE'
    gyro.reset()
    motor.run_forever(speed_sp=-200)
    motor1.run_forever(speed_sp=200)
    running = True

    while running:
        angle = gyro.angle
        if angle <= -170:  # Adjusted angle condition
            motor.stop()
            motor1.stop()
            running = False
        time.sleep(0.01)

def move_forward_20_units():
    '''Move forward for 20 units'''
    motor.run_to_rel_pos(position_sp=200, speed_sp=500)
    motor1.run_to_rel_pos(position_sp=200, speed_sp=500)
    motor.wait_while('running')
    motor1.wait_while('running')

def turn_90_degrees_left():
    '''Turn the robot 90 degrees to the left'''
    motor.run_to_rel_pos(position_sp=-450, speed_sp=200)
    motor1.run_to_rel_pos(position_sp=450, speed_sp=200)
    motor.wait_while('running')
    motor1.wait_while('running')

def move_forward_until_dark_surface():
    '''Move forward until detecting a dark surface underneath'''
    motor.run_forever(speed_sp=200)
    motor1.run_forever(speed_sp=200)
    while True:
        if color_sensor.reflected_light_intensity < 30:  # Adjust threshold as needed
            motor.stop()
            motor1.stop()
            break

def rotate_90_degrees_left():
    '''Rotate 90 degrees to the left'''
    motor.run_to_rel_pos(position_sp=-450, speed_sp=200)
    motor1.run_to_rel_pos(position_sp=450, speed_sp=200)
    motor.wait_while('running')
    motor1.wait_while('running')

def move_backward_until_touch_sensor():
    '''Move backward until the touch sensor causes the robot to stop'''
    motor.run_forever(speed_sp=-200)
    motor1.run_forever(speed_sp=-200)
    while True:
        if touch_sensor.is_pressed:
            motor.stop()
            motor1.stop()
            break

def main():
    '''The main function of our program'''

    # Print "Assignment 1" on the screen
    display_message('Assignment 1')

    # Wait for button press
    display_message('Waiting for button press...')
    wait_for_button_press()

    # Clear the screen (optional, as it's not possible on EV3)
    # Move forward until obstacle
    display_message('Moving forward until obstacle...')
    move_forward_until_obstacle()
    display_message('Obstacle detected!')

    # Turn 180 degrees using gyroscope
    display_message('Turning 180 degrees...')
    turn_180_degrees()
    display_message('Turn completed!')

    # Move forward 20 units
    display_message('Moving forward 20 units...')
    move_forward_20_units()
    display_message('Move forward completed!')

    # Turn 90 degrees to the left
    display_message('Turning 90 degrees to the left...')
    turn_90_degrees_left()
    display_message('Turn left completed!')

    # Move forward until detecting a dark surface underneath
    display_message('Moving forward until dark surface detected...')
    move_forward_until_dark_surface()
    display_message('Dark surface detected!')

    # Stop
    motor.stop()
    motor1.stop()

    # Rotate 90 degrees to the left
    display_message('Rotating 90 degrees to the left...')
    rotate_90_degrees_left()
    display_message('Rotation completed!')

    # Move backward until the touch sensor causes the robot to stop
    display_message('Moving backward until touch sensor pressed...')
    move_backward_until_touch_sensor()
    display_message('Touch sensor pressed!')

if __name__ == '__main__':
    main()
