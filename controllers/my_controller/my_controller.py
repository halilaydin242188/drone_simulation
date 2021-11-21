"""my_controller controller."""

import math

from controller import Robot, Motor
from drone_controller import Drone_controller
import sensorProcess

robot = Robot()
dc = Drone_controller(robot)
dc.enable_all()

# set propeller motors to velocity mode.
for i in range(0, 4, 1):
    Motor.setPosition(dc.MOTORS[i], float("inf"))
    Motor.setVelocity(dc.MOTORS[i], 1.0)

print("Start the drone...")

# Wait one second.
while robot.step(dc.TIME_STEP) != -1:
    if robot.getTime() > 1.0:
        break

# Display manual control message.
print("You can control the drone with your computer keyboard:")
print("- 'up': move forward.")
print("- 'down': move backward.")
print("- 'right': turn right.")
print("- 'left': turn left.")
print("- 'shift + up': increase the target altitude.")
print("- 'shift + down': decrease the target altitude.")
print("- 'shift + right': strafe right.")
print("- 'shift + left': strafe left.")

# Main loop
while robot.step(dc.TIME_STEP) != -1:
    TIME = robot.getTime()  # in seconds.
    KEY = dc.KEYBOARD.getKey()
    # Retrieve robot position using the sensors.
    ROLL = dc.IMU.getRollPitchYaw()[0] + math.pi / 2.0
    PITCH = dc.IMU.getRollPitchYaw()[1]
    ALTITUDE = dc.GPS.getValues()[1]
    ROLL_ACCELERATION = dc.GYRO.getValues()[0]
    PITCH_ACCELERATION = dc.GYRO.getValues()[1]

    dc.blink_leds(TIME) # Blink the front LEDs alternatively with a 1 second rate.
    dc.keyboard_control(KEY) # Control the drone using keyboard    
    dc.motor_control(ROLL, PITCH, ALTITUDE, ROLL_ACCELERATION, PITCH_ACCELERATION)
    dc.camera_stabilize(ROLL_ACCELERATION, PITCH_ACCELERATION)
    
    if TIME % 1.0 == 0: # process image every second
        data = dc.CAMERA.getImage()
        sensorProcess.readImage(data, dc.CAMERA_WIDTH, dc.CAMERA_HEIGHT)


    if TIME % 0.5 == 0:
        """
        print("DS_FRONT value : ", str(DS_FRONT.getValue()))
        print("DS_BACK : ", str(DS_BACK.getValue()))
        print("DS_RIGHT value : ", str(DS_RIGHT.getValue()))
        print("DS_LEFT value : ", str(DS_LEFT.getValue()))
        print("DS_DOWN value : ", str(DS_DOWN.getValue()))
        """
