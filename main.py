#!/usr/bin/env python3
'''Hello to the world from ev3dev.org'''

import os
import sys
import time
import evdev
from ev3dev2.motor import LargeMotor, OUTPUT_B, OUTPUT_D
from ev3dev2.sensor.lego import UltrasonicSensor, GyroSensor
from ev3dev2.sensor import INPUT_2  # Import INPUT_2 for port 2

# state constants
ON = True
OFF = False

# Initialize motors, ultrasonic sensor, and gyroscope
motor = LargeMotor(OUTPUT_B)
motor1 = LargeMotor(OUTPUT_D)
sonar = UltrasonicSensor()
gyro = GyroSensor(INPUT_2)  # Initialize gyro sensor on port 2

def debug_print(*args, **kwargs):
    '''Print debug messages to stderr.

    This shows up in the output panel in VS Code.
    '''
    print(*args, **kwargs, file=sys.stderr)


def reset_console():
    '''Resets the console to the default state'''
    print('\x1Bc', end='')


def set_cursor(state):
    '''Turn the cursor on or off'''
    if state:
        print('\x1B[?25h', end='')
    else:
        print('\x1B[?25l', end='')


def set_font(name):
    '''Sets the console font

    A full list of fonts can be found with `ls /usr/share/consolefonts`
    '''
    os.system('setfont ' + name)


def display_message(message):
    '''Display message on the console'''
    reset_console()
    set_cursor(OFF)
    set_font('Lat15-Terminus24x12')
    print(message)


def wait_for_button_press():
    '''Wait for a button press event'''
    devices = [evdev.InputDevice(fn) for fn in evdev.list_devices()]
    for device in devices:
        if "buttons" in device.name.lower():
            button_device = device
            break
    else:
        raise IOError("No button device found")

    button_device.grab()
    for event in button_device.read_loop():
        if event.type == evdev.ecodes.EV_KEY and event.value == 1:
            return True


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
        debug_print("Current Angle:", angle)  # Print current angle for debugging
        if angle <= -170:  # Adjusted angle condition
            motor.stop()
            motor1.stop()
            running = False
        time.sleep(0.01)

def turn_90_degrees_left():
    '''Turn the robot 90 degrees to the left using gyroscope'''
    gyro.mode = 'GYRO-RATE'
    gyro.reset()
    motor.run_forever(speed_sp=200)  # Adjust speed as needed
    motor1.run_forever(speed_sp=-200)  # Adjust speed as needed
    running = True

    while running:
        angle = gyro.angle
        debug_print("Current Angle:", angle)  # Print current angle for debugging
        if angle >= 90:  # Adjusted angle condition for 90 degrees
            motor.stop()
            motor1.stop()
            running = False
        time.sleep(0.01)


def move_forward_100cm():
    '''Move forward for 100cm'''
    motor.run_to_rel_pos(position_sp=1000, speed_sp=500)
    motor1.run_to_rel_pos(position_sp=1000, speed_sp=500)
    motor.wait_while('running')
    motor1.wait_while('running')

def move_forward_20_units():
    '''Move forward for 20 units'''
    motor.run_to_rel_pos(position_sp=200, speed_sp=500)
    motor1.run_to_rel_pos(position_sp=200, speed_sp=500)
    motor.wait_while('running')
    motor1.wait_while('running')




def main():
    '''The main function of our program'''

    # print something to the screen of the device
    display_message('Assignment 1')

    # print something to the output panel in VS Code
    debug_print('Hello VS Code!')

    # wait for button press
    debug_print('Waiting for button press...')
    wait_for_button_press()

    # clear the message
    reset_console()

    # Move forward until obstacle
    debug_print('Moving forward until obstacle...')
    move_forward_until_obstacle()
    debug_print('Obstacle detected!')

    # Turn 180 degrees using gyroscope
    debug_print('Turning 180 degrees...')
    turn_180_degrees()
    debug_print('Turn completed!')

    # Move forward for 100cm
    debug_print('Moving forward for 100cm...')
    move_forward_100cm()
    debug_print('Move forward completed!')

    # Move forward for 20 units
    debug_print('Moving forward for 20 units...')
    move_forward_20_units()
    debug_print('Move forward for 20 units completed!')

     # Turn 90 degrees to the left
    debug_print('Turning 90 degrees to the left...')
    turn_90_degrees_left()
    debug_print('Turn left completed!')

    # You may add further actions here, after moving forward


if __name__ == '__main__':
    main()
