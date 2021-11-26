"""my_controller controller."""

import math
from controller import Robot, Motor
from drone_controller import Drone_controller

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
print("- 'up': increase the target altitude.")
print("- 'down': decrease the target altitud.")
print("- 'right': turn right.")
print("- 'left': turn left.")
print("- 'w': move forward.")
print("- 's': move backward.")
print("- 'd': strafe right.")
print("- 'a': strafe left.")
print("- 'e': move north-east.")
print("- 'q': move north-west.")

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

    FV = dc.DS_FRONT.getValue()
    BV = dc.DS_BACK.getValue()
    RV = dc.DS_RIGHT.getValue()
    LV = dc.DS_LEFT.getValue()
    DV = dc.DS_DOWN.getValue()           

    if TIME > 2.0:
        dc.blink_leds(TIME) # Blink the front LEDs alternatively with a 1 second rate.
        if KEY == ord("X"):
            if TIME % 0.5 == 0: # get image every second
                dc.getImage()
            dc.landingSupport()
        else:
            dc.checkCrash(FV, BV, RV, LV, DV)
            dc.keyboard_control(KEY) # Control the drone using keyboard    
        dc.camera_stabilize(ROLL_ACCELERATION, PITCH_ACCELERATION)

        if TIME % 1.0 == 0:
            print("----------------------------")
            print(dc.retval)
            print(dc.destX)
            print(dc.destY)
            
        
    dc.motor_control(ROLL, PITCH, ALTITUDE, ROLL_ACCELERATION, PITCH_ACCELERATION)

