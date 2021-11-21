
from controller import Robot, DistanceSensor, Camera, InertialUnit, GPS as Gps, Compass, Gyro, Keyboard, Motor

class Drone_controller:
    def __init__(self, robot):
        self.robot = robot
        # Get and enable devices.
        self.TIME_STEP = int(self.robot.getBasicTimeStep())
        self.CAMERA = self.robot.getDevice("camera")
        self.FRONT_LEFT_LED = self.robot.getDevice("front left led")
        self.FRONT_RIGHT_LED = self.robot.getDevice("front right led")
        self.IMU = self.robot.getDevice("inertial unit")
        self.GPS = self.robot.getDevice("gps")
        self.COMPASS = self.robot.getDevice("compass")
        self.GYRO = self.robot.getDevice("gyro")
        self.KEYBOARD = self.robot.getKeyboard()
        self.CAMERA_ROLL_MOTOR = self.robot.getDevice("camera roll")
        self.CAMERA_PITCH_MOTOR = self.robot.getDevice("camera pitch")
        self.FRONT_LEFT_MOTOR = self.robot.getDevice("front left propeller")
        self.FRONT_RIGHT_MOTOR = self.robot.getDevice("front right propeller")
        self.REAR_LEFT_MOTOR = self.robot.getDevice("rear left propeller")
        self.REAR_RIGHT_MOTOR = self.robot.getDevice("rear right propeller")
        # CAMERA_YAW_MOTOR = robot.getDevice("camera yaw")
        self.DS_FRONT = self.robot.getDevice("ds_front")
        self.DS_BACK = self.robot.getDevice("ds_back")
        self.DS_RIGHT = self.robot.getDevice("ds_right")
        self.DS_LEFT = self.robot.getDevice("ds_left")
        self.DS_DOWN = self.robot.getDevice("ds_down")

        self.MOTORS = [self.FRONT_LEFT_MOTOR, self.FRONT_RIGHT_MOTOR, self.REAR_LEFT_MOTOR, self.REAR_RIGHT_MOTOR]

        self.CAMERA_WIDTH = self.CAMERA.getWidth()
        self.CAMERA_HEIGHT = self.CAMERA.getHeight()

        self.K_VERTICAL_THRUST = 68.5  # with this thrust, the drone lifts.
        self.K_VERTICAL_OFFSET = 0.6;  # Vertical offset where the robot actually targets to stabilize itself.
        self.K_VERTICAL_P = 3.0        # P constant of the vertical PID.
        self.K_ROLL_P = 50.0           # P constant of the roll PID.
        self.K_PITCH_P = 30.0          # P constant of the pitch PID.

        self.target_altitude = 1.5 # The target altitude. Can be changed by the user.

        # Transform the keyboard input to disturbances on the stabilization algorithm.
        self.roll_disturbance = 0.0
        self.pitch_disturbance = 0.0
        self.yaw_disturbance = 0.0
    
    def clamp(self, value, low, high):
        return low if value < low else high if value > high else value

    def camera_stabilize(self, ROLL_ACCELERATION, PITCH_ACCELERATION):
        # Stabilize the Camera by actuating the camera motors according to the gyro feedback.
        self.CAMERA_ROLL_MOTOR.setPosition(-0.115 * ROLL_ACCELERATION)
        self.CAMERA_PITCH_MOTOR.setPosition(-0.1 * PITCH_ACCELERATION)

    def pitch_normalize(self):
        if self.pitch_disturbance > 0:
                if self.pitch_disturbance <= 0.2:
                    self.pitch_disturbance = 0
                self.pitch_disturbance -= 0.05
        elif self.pitch_disturbance < 0:
            if self.pitch_disturbance >= -0.2:
                self.pitch_disturbance = 0
            self.pitch_disturbance += 0.05

    def roll_normalize(self):
        if self.roll_disturbance < 0:
                if self.roll_disturbance >= -1.0:
                    self.roll_disturbance = 0
                self.roll_disturbance += 0.05
        elif self.roll_disturbance > 0:
            if self.roll_disturbance <= 1.0:
                self.roll_disturbance = 0
            self.roll_disturbance -= 0.05

    def key_w(self):
        if self.IMU.getRollPitchYaw()[1] > -0.1: 
            self.pitch_disturbance += 0.05

        self.roll_normalize()
        self.yaw_disturbance = 0

    def key_s(self):
        if self.IMU.getRollPitchYaw()[1] < 0.1: 
            self.pitch_disturbance -= 0.05

        self.roll_normalize()
        self.yaw_disturbance = 0

    def key_d(self):
        if self.IMU.getRollPitchYaw()[0] < -1.47:
            self.roll_disturbance -= 0.05

        self.pitch_normalize()
        self.yaw_disturbance = 0

    def key_a(self):
        if self.IMU.getRollPitchYaw()[0] > -1.67:
            self.roll_disturbance += 0.05

        self.pitch_normalize()
        self.yaw_disturbance = 0

    def key_e(self):
        if self.IMU.getRollPitchYaw()[1] > -0.1: 
            self.pitch_disturbance += 0.05

        if self.IMU.getRollPitchYaw()[0] < -1.47:
            self.roll_disturbance -= 0.05

        self.yaw_disturbance = 0

    def key_q(self):
        if self.IMU.getRollPitchYaw()[1] > -0.1: 
            self.pitch_disturbance += 0.05

        if self.IMU.getRollPitchYaw()[0] > -1.67:
            self.roll_disturbance += 0.05

        self.yaw_disturbance = 0

    def key_right(self):
        if self.yaw_disturbance < 1.3:
            self.yaw_disturbance += 0.05

    def key_left(self):
        if self.yaw_disturbance > -1.3:
            self.yaw_disturbance -= 0.05

    def key_up(self):
        self.target_altitude += 0.01
        self.yaw_disturbance = 0

    def key_down(self):
        self.target_altitude -= 0.01
        self.yaw_disturbance = 0

    def keyboard_control(self, KEY):

        if KEY > 0 :
            if KEY == ord('W'): # W
                self.key_w()

            elif KEY == ord('S'): # S
                self.key_s()

            elif KEY == ord('D'): # D
                self.key_d()

            elif KEY == ord('A'): # A
                self.key_a()

            elif KEY == ord('E'): # E
                self.key_e()

            elif KEY == ord('Q'): # Q
                self.key_q()

            elif KEY == self.KEYBOARD.RIGHT: # RIGHT
                self.key_right()

            elif KEY == self.KEYBOARD.LEFT: # LEFT
                self.key_left()

            elif KEY == self.KEYBOARD.UP: # UP
                self.key_up()

            elif KEY == self.KEYBOARD.DOWN: # DOWN
                self.key_down()

            else: # other key combinations we didn't declare is being used
                self.pitch_normalize()
                self.roll_normalize()
                self.yaw_disturbance = 0

        else: # if no key is being pressed
            self.pitch_normalize()
            self.roll_normalize()
            self.yaw_disturbance = 0

    def blink_leds(self, TIME):
        self.LED_STATE = int(TIME) % 2
        self.FRONT_LEFT_LED.set(self.LED_STATE)
        self.FRONT_RIGHT_LED.set(self.LED_STATE)

    def motor_control(self, ROLL, PITCH, ALTITUDE, ROLL_ACCELERATION, PITCH_ACCELERATION):
        
        # Compute the roll, pitch, yaw and vertical inputs.
        self.ROLL_INPUT = self.K_ROLL_P * self.clamp(ROLL, -1.0, 1.0) + ROLL_ACCELERATION + self.roll_disturbance
        self.PITCH_INPUT = self.K_PITCH_P * ( self.clamp(PITCH, -1.0, 1.0) - 0.005) - PITCH_ACCELERATION + self.pitch_disturbance
        self.YAW_INPUT = self.yaw_disturbance
        self.CLAMPED_DIFFERENCE_ALTITUDE = self.clamp(self.target_altitude - ALTITUDE + self.K_VERTICAL_OFFSET, -1.0, 1.0)
        self.VERTICAL_INPUT = self.K_VERTICAL_P * pow(self.CLAMPED_DIFFERENCE_ALTITUDE, 3.0)

        # Actuate the motors taking into consideration all the computed inputs.
        self.FRONT_LEFT_MOTOR_INPUT = self.K_VERTICAL_THRUST + self.VERTICAL_INPUT - self.ROLL_INPUT - self.PITCH_INPUT + self.YAW_INPUT
        self.FRONT_RIGHT_MOTOR_INPUT  = self.K_VERTICAL_THRUST + self.VERTICAL_INPUT + self.ROLL_INPUT - self.PITCH_INPUT - self.YAW_INPUT
        self.REAR_LEFT_MOTOR_INPUT  = self.K_VERTICAL_THRUST + self.VERTICAL_INPUT - self.ROLL_INPUT + self.PITCH_INPUT - self.YAW_INPUT
        self.REAR_RIGHT_MOTOR_INPUT = self.K_VERTICAL_THRUST + self.VERTICAL_INPUT + self.ROLL_INPUT + self.PITCH_INPUT + self.YAW_INPUT

        Motor.setVelocity(self.FRONT_LEFT_MOTOR, self.FRONT_LEFT_MOTOR_INPUT)
        Motor.setVelocity(self.FRONT_RIGHT_MOTOR, -self.FRONT_RIGHT_MOTOR_INPUT)
        Motor.setVelocity(self.REAR_LEFT_MOTOR, -self.REAR_LEFT_MOTOR_INPUT)
        Motor.setVelocity(self.REAR_RIGHT_MOTOR, self.REAR_RIGHT_MOTOR_INPUT)

    def enable_all(self):
        Camera.enable(self.CAMERA, self.TIME_STEP)
        InertialUnit.enable(self.IMU, self.TIME_STEP)
        Gps.enable(self.GPS, self.TIME_STEP)
        Compass.enable(self.COMPASS, self.TIME_STEP)
        Gyro.enable(self.GYRO, self.TIME_STEP)
        Keyboard.enable(self.KEYBOARD, self.TIME_STEP)
        DistanceSensor.enable(self.DS_FRONT, self.TIME_STEP)
        DistanceSensor.enable(self.DS_BACK, self.TIME_STEP)
        DistanceSensor.enable(self.DS_RIGHT, self.TIME_STEP)
        DistanceSensor.enable(self.DS_LEFT, self.TIME_STEP)
        DistanceSensor.enable(self.DS_DOWN, self.TIME_STEP)