"""my_controller controller."""

import math
from controller import Robot, DistanceSensor, Camera, InertialUnit, GPS as Gps, Compass, Gyro, Keyboard, Motor
import sensorProcess

def clamp(value, low, high):
    return low if value < low else high if value > high else value

robot = Robot()

# Get and enable devices.
TIME_STEP = int(robot.getBasicTimeStep())
CAMERA = robot.getDevice("camera")
FRONT_LEFT_LED = robot.getDevice("front left led")
FRONT_RIGHT_LED = robot.getDevice("front right led")
IMU = robot.getDevice("inertial unit")
GPS = robot.getDevice("gps")
COMPASS = robot.getDevice("compass")
GYRO = robot.getDevice("gyro")
KEYBOARD = robot.getKeyboard()
CAMERA_ROLL_MOTOR = robot.getDevice("camera roll")
CAMERA_PITCH_MOTOR = robot.getDevice("camera pitch")
FRONT_LEFT_MOTOR = robot.getDevice("front left propeller")
FRONT_RIGHT_MOTOR = robot.getDevice("front right propeller")
REAR_LEFT_MOTOR = robot.getDevice("rear left propeller")
REAR_RIGHT_MOTOR = robot.getDevice("rear right propeller")
# CAMERA_YAW_MOTOR = robot.getDevice("camera yaw")
DS_FRONT = robot.getDevice("ds_front")
DS_BACK = robot.getDevice("ds_back")
DS_RIGHT = robot.getDevice("ds_right")
DS_LEFT = robot.getDevice("ds_left")
DS_DOWN = robot.getDevice("ds_down")

MOTORS = [FRONT_LEFT_MOTOR, FRONT_RIGHT_MOTOR, REAR_LEFT_MOTOR, REAR_RIGHT_MOTOR]

CAMERA_WIDTH = CAMERA.getWidth()
CAMERA_HEIGHT = CAMERA.getHeight()

K_VERTICAL_THRUST = 68.5  # with this thrust, the drone lifts.
K_VERTICAL_OFFSET = 0.6;  # Vertical offset where the robot actually targets to stabilize itself.
K_VERTICAL_P = 3.0        # P constant of the vertical PID.
K_ROLL_P = 50.0           # P constant of the roll PID.
K_PITCH_P = 30.0          # P constant of the pitch PID.

target_altitude = 1.5 # The target altitude. Can be changed by the user.

# Transform the keyboard input to disturbances on the stabilization algorithm.
roll_disturbance = 0.0
pitch_disturbance = 0.0
yaw_disturbance = 0.0

Camera.enable(CAMERA, TIME_STEP)
InertialUnit.enable(IMU, TIME_STEP)
Gps.enable(GPS, TIME_STEP)
Compass.enable(COMPASS, TIME_STEP)
Gyro.enable(GYRO, TIME_STEP)
Keyboard.enable(KEYBOARD, TIME_STEP)
DistanceSensor.enable(DS_FRONT, TIME_STEP)
DistanceSensor.enable(DS_BACK, TIME_STEP)
DistanceSensor.enable(DS_RIGHT, TIME_STEP)
DistanceSensor.enable(DS_LEFT, TIME_STEP)
DistanceSensor.enable(DS_DOWN, TIME_STEP)

def camera_stabilize():
    # Stabilize the Camera by actuating the camera motors according to the gyro feedback.
    CAMERA_ROLL_MOTOR.setPosition(-0.115 * ROLL_ACCELERATION)
    CAMERA_PITCH_MOTOR.setPosition(-0.1 * PITCH_ACCELERATION)

def pitch_normalize():
    global pitch_disturbance
    if pitch_disturbance > 0:
            if pitch_disturbance <= 0.2:
                pitch_disturbance = 0
            pitch_disturbance -= 0.05
    elif pitch_disturbance < 0:
        if pitch_disturbance >= -0.2:
            pitch_disturbance = 0
        pitch_disturbance += 0.05

def roll_normalize():
    global roll_disturbance
    if roll_disturbance < 0:
            if roll_disturbance >= -1.0:
                roll_disturbance = 0
            roll_disturbance += 0.05
    elif roll_disturbance > 0:
        if roll_disturbance <= 1.0:
            roll_disturbance = 0
        roll_disturbance -= 0.05

def keyboard_control(KEY):
    global roll_disturbance, pitch_disturbance, yaw_disturbance, target_altitude

    if KEY > 0 :
        if KEY == ord('W'): # W
            if IMU.getRollPitchYaw()[1] > -0.1: 
                pitch_disturbance += 0.05

            roll_normalize()
            yaw_disturbance = 0

        elif KEY == ord('S'): # S
            if IMU.getRollPitchYaw()[1] < 0.1: 
                pitch_disturbance -= 0.05

            roll_normalize()
            yaw_disturbance = 0

        elif KEY == ord('D'): # D
            if IMU.getRollPitchYaw()[0] < -1.47:
                roll_disturbance -= 0.05

            pitch_normalize()
            yaw_disturbance = 0

        elif KEY == ord('A'): # A
            if IMU.getRollPitchYaw()[0] > -1.67:
                roll_disturbance += 0.05

            pitch_normalize()
            yaw_disturbance = 0

        elif KEY == ord('E'): # E
            if IMU.getRollPitchYaw()[1] > -0.1: 
                pitch_disturbance += 0.05

            if IMU.getRollPitchYaw()[0] < -1.47:
                roll_disturbance -= 0.05

        elif KEY == ord('Q'): # Q
            if IMU.getRollPitchYaw()[1] > -0.1: 
                pitch_disturbance += 0.05

            if IMU.getRollPitchYaw()[0] > -1.67:
                roll_disturbance += 0.05

        elif KEY == KEYBOARD.RIGHT: # RIGHT
            if yaw_disturbance < 1.3:
                yaw_disturbance += 0.05

        elif KEY == KEYBOARD.LEFT: # LEFT
            if yaw_disturbance > -1.3:
                yaw_disturbance -= 0.05

        elif KEY == KEYBOARD.UP: # UP
            target_altitude += 0.01
            yaw_disturbance = 0

        elif KEY == KEYBOARD.DOWN: # DOWN
            target_altitude -= 0.01
            yaw_disturbance = 0

        else: # other key combinations we didn't declare is being used
            pitch_normalize()
            roll_normalize()
            yaw_disturbance = 0

    else: # if no key is being pressed
        pitch_normalize()
        roll_normalize()
        yaw_disturbance = 0

def blink_leds(TIME):
    LED_STATE = int(TIME) % 2
    FRONT_LEFT_LED.set(LED_STATE)
    FRONT_RIGHT_LED.set(LED_STATE)

def motor_control():
    global roll_disturbance, pitch_disturbance, yaw_disturbance, target_altitude
    
    # Compute the roll, pitch, yaw and vertical inputs.
    ROLL_INPUT = K_ROLL_P * clamp(ROLL, -1.0, 1.0) + ROLL_ACCELERATION + roll_disturbance
    PITCH_INPUT = K_PITCH_P * ( clamp(PITCH, -1.0, 1.0) - 0.005) - PITCH_ACCELERATION + pitch_disturbance
    YAW_INPUT = yaw_disturbance
    CLAMPED_DIFFERENCE_ALTITUDE = clamp(target_altitude - ALTITUDE + K_VERTICAL_OFFSET, -1.0, 1.0)
    VERTICAL_INPUT = K_VERTICAL_P * pow(CLAMPED_DIFFERENCE_ALTITUDE, 3.0)

    # Actuate the motors taking into consideration all the computed inputs.
    FRONT_LEFT_MOTOR_INPUT = K_VERTICAL_THRUST + VERTICAL_INPUT - ROLL_INPUT - PITCH_INPUT + YAW_INPUT
    FRONT_RIGHT_MOTOR_INPUT  = K_VERTICAL_THRUST + VERTICAL_INPUT + ROLL_INPUT - PITCH_INPUT - YAW_INPUT
    REAR_LEFT_MOTOR_INPUT  = K_VERTICAL_THRUST + VERTICAL_INPUT - ROLL_INPUT + PITCH_INPUT - YAW_INPUT
    REAR_RIGHT_MOTOR_INPUT = K_VERTICAL_THRUST + VERTICAL_INPUT + ROLL_INPUT + PITCH_INPUT + YAW_INPUT

    Motor.setVelocity(FRONT_LEFT_MOTOR, FRONT_LEFT_MOTOR_INPUT)
    Motor.setVelocity(FRONT_RIGHT_MOTOR, -FRONT_RIGHT_MOTOR_INPUT)
    Motor.setVelocity(REAR_LEFT_MOTOR, -REAR_LEFT_MOTOR_INPUT)
    Motor.setVelocity(REAR_RIGHT_MOTOR, REAR_RIGHT_MOTOR_INPUT)

# set propeller motors to velocity mode.
for i in range(0, 4, 1):
    Motor.setPosition(MOTORS[i], float("inf"))
    Motor.setVelocity(MOTORS[i], 1.0)

print("Start the drone...")

# Wait one second.
while robot.step(TIME_STEP) != -1:
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
while robot.step(TIME_STEP) != -1:
    TIME = robot.getTime()  # in seconds.
    KEY = KEYBOARD.getKey()
    # Retrieve robot position using the sensors.
    ROLL = IMU.getRollPitchYaw()[0] + math.pi / 2.0
    PITCH = IMU.getRollPitchYaw()[1]
    ALTITUDE = GPS.getValues()[1]
    ROLL_ACCELERATION = GYRO.getValues()[0]
    PITCH_ACCELERATION = GYRO.getValues()[1]

    blink_leds(TIME) # Blink the front LEDs alternatively with a 1 second rate.
    keyboard_control(KEY) # Control the drone using keyboard    
    motor_control()
    camera_stabilize()
    
    if TIME % 1.0 == 0: # process image every second
        data = CAMERA.getImage()
        sensorProcess.readImage(data, CAMERA_WIDTH, CAMERA_HEIGHT)


    if TIME % 2.0 == 0:
        """
        print("DS_FRONT value : ", str(DS_FRONT.getValue()))
        print("DS_BACK : ", str(DS_BACK.getValue()))
        print("DS_RIGHT value : ", str(DS_RIGHT.getValue()))
        print("DS_LEFT value : ", str(DS_LEFT.getValue()))
        print("DS_DOWN value : ", str(DS_DOWN.getValue()))
        """
