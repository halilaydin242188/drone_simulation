"""my_controller controller."""

import math
from controller import Robot, DistanceSensor, Camera, InertialUnit, GPS as Gps, Compass, Gyro, Keyboard, Motor

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
MOTORS = [FRONT_LEFT_MOTOR, FRONT_RIGHT_MOTOR, REAR_LEFT_MOTOR, REAR_RIGHT_MOTOR]
# CAMERA_YAW_MOTOR = robot.getDevice("camera yaw") # Not used in this example.

Camera.enable(CAMERA, TIME_STEP)
InertialUnit.enable(IMU, TIME_STEP)
Gps.enable(GPS, TIME_STEP)
Compass.enable(COMPASS, TIME_STEP)
Gyro.enable(GYRO, TIME_STEP)
Keyboard.enable(KEYBOARD, TIME_STEP)

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

# Constants, empirically found.
K_VERTICAL_THRUST = 68.5  # with this thrust, the drone lifts.
K_VERTICAL_OFFSET = 0.6;  # Vertical offset where the robot actually targets to stabilize itself.
K_VERTICAL_P = 3.0        # P constant of the vertical PID.
K_ROLL_P = 50.0           # P constant of the roll PID.
K_PITCH_P = 30.0          # P constant of the pitch PID.

target_altitude = 1.0 # The target altitude. Can be changed by the user.

# Main loop
while robot.step(TIME_STEP) != -1:
    TIME = robot.getTime()  # in seconds.

    # Retrieve robot position using the sensors.
    ROLL = IMU.getRollPitchYaw()[0] + math.pi / 2.0
    PITCH = IMU.getRollPitchYaw()[1]
    ALTITUDE = GPS.getValues()[1]
    ROLL_ACCELERATION = GYRO.getValues()[0]
    PITCH_ACCELERATION = GYRO.getValues()[1]

    # Blink the front LEDs alternatively with a 1 second rate.
    LED_STATE = int(TIME) % 2
    FRONT_LEFT_LED.set(LED_STATE)
    FRONT_RIGHT_LED.set(LED_STATE)

    # Stabilize the Camera by actuating the camera motors according to the gyro feedback.
    CAMERA_ROLL_MOTOR.setPosition(-0.115 * ROLL_ACCELERATION)
    CAMERA_PITCH_MOTOR.setPosition(-0.1 * PITCH_ACCELERATION)

    # Transform the keyboard input to disturbances on the stabilization algorithm.
    roll_disturbance = 0.0
    pitch_disturbance = 0.0
    yaw_disturbance = 0.0
    
    key = KEYBOARD.getKey()
    while key > 0 :
        if key == KEYBOARD.UP:
            pitch_disturbance = 2.0
        elif key == KEYBOARD.DOWN:
            pitch_disturbance = -2.0
        elif key == KEYBOARD.RIGHT:
            yaw_disturbance = 1.3
        elif key == KEYBOARD.LEFT:
            yaw_disturbance = -1.3
        elif key == KEYBOARD.SHIFT + KEYBOARD.RIGHT:
            roll_disturbance = -1.0
        elif key == KEYBOARD.SHIFT + KEYBOARD.LEFT:
            roll_disturbance = 1.0
        elif key == KEYBOARD.SHIFT + KEYBOARD.UP:
            target_altitude += 0.05
            print("target altitude :" + str(target_altitude))
        elif key == KEYBOARD.SHIFT + KEYBOARD.DOWN:
            target_altitude -= 0.05
            print("target altitude : ", str(target_altitude))

        key = KEYBOARD.getKey()
    
    
    # Compute the roll, pitch, yaw and vertical inputs.
    ROLL_INPUT = K_ROLL_P * clamp(ROLL, -1.0, 1.0) + ROLL_ACCELERATION + roll_disturbance
    PITCH_INPUT = K_PITCH_P * clamp(PITCH, -1.0, 1.0) - PITCH_ACCELERATION + pitch_disturbance
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
    
