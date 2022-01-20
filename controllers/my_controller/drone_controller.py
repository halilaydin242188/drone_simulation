
from controller import Robot, DistanceSensor, Camera, InertialUnit, GPS as Gps, Compass, Gyro, Keyboard, Motor, Accelerometer
import cv2 as cv
import numpy as np

qrDetector = cv.QRCodeDetector() # create a opencv qr detector object

class Drone_controller:
    def __init__(self, robot): # declare necessary variables when object is created 
        self.robot = robot
        # Get devices.
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
        self.CAMERA_YAW_MOTOR = robot.getDevice("camera yaw")
        self.DS_FRONT = self.robot.getDevice("ds_front")
        self.DS_BACK = self.robot.getDevice("ds_back")
        self.DS_RIGHT = self.robot.getDevice("ds_right")
        self.DS_LEFT = self.robot.getDevice("ds_left")
        self.DS_DOWN = self.robot.getDevice("ds_down")
        self.ACCELEROMETER = self.robot.getDevice("accelerometer")

        self.MOTORS = [self.FRONT_LEFT_MOTOR, self.FRONT_RIGHT_MOTOR, self.REAR_LEFT_MOTOR, self.REAR_RIGHT_MOTOR]

        self.CAMERA_WIDTH = self.CAMERA.getWidth()
        self.CAMERA_HEIGHT = self.CAMERA.getHeight()

        self.K_VERTICAL_THRUST = 69.14  # with this thrust, the drone lifts.
        self.K_VERTICAL_OFFSET = 0.6;  # Vertical offset where the robot actually targets to stabilize itself.
        self.K_VERTICAL_P = 3.0        # P constant of the vertical PID.
        self.K_ROLL_P = 50.0           # P constant of the roll PID.
        self.K_PITCH_P = 30.0          # P constant of the pitch PID.

        #self.target_altitude = 1.5 # The target altitude. Can be changed by the user.
        self.vertical_input = 0.0

        # Transform the keyboard input to disturbances on the stabilization algorithm.
        self.roll_disturbance = 0.0
        self.pitch_disturbance = 0.0
        self.yaw_disturbance = 0.0

        self.camera_pitch_value = 0.0
        self.camera_yaw_value = 0.0

        self.willCrash = 0
        self.shouldUp = 0
        self.shouldForward = 5
        self.shouldBackward = 5
        self.shouldRight = 0
        self.shouldLeft = 0

        self.w_factor = 5
        self.s_factor = 5
        self.d_factor = 5
        self.a_factor = 5

        self.posX = self.CAMERA_WIDTH / 2
        self.posY = self.CAMERA_HEIGHT / 2
        self.destX = 0
        self.destY = 0

        self.img = None # image
        self.retval = False # image is read from drone's camera
        self.points = 0 # 4 points of the qr code roi found in image

        self.normalizeTime = 0
        self.timeCalculated = False

        self.imuControl = False # When True limit the velocity according to imu sensor
        self.dsControl = False # When True activate distance sensor algorithms

        self.lastChanged = { # to prevent holding down
            "key_z" : 0,
            "key_c" : 0
        }
    
    def clamp(self, value, low, high):
        return low if value < low else high if value > high else value

    def camera_stabilize(self, ROLL_ACCELERATION, PITCH_ACCELERATION): # Stabilize the Camera by actuating the camera motors according to the gyro feedback.
        self.CAMERA_ROLL_MOTOR.setPosition(-0.115 * ROLL_ACCELERATION)
        self.CAMERA_PITCH_MOTOR.setPosition(-0.1 * PITCH_ACCELERATION)

    def pitch_normalize(self):
        if self.pitch_disturbance > 0:
                if self.pitch_disturbance <= 0.2:
                    self.pitch_disturbance = 0
                else:
                    self.pitch_disturbance -= 0.05
        elif self.pitch_disturbance < 0:
            if self.pitch_disturbance >= -0.2:
                self.pitch_disturbance = 0
            else:
                self.pitch_disturbance += 0.05

    def roll_normalize(self):
        if self.roll_disturbance < 0:
                if self.roll_disturbance >= -1.0:
                    self.roll_disturbance = 0
                else:
                    self.roll_disturbance += 0.05
        elif self.roll_disturbance > 0:
            if self.roll_disturbance <= 1.0:
                self.roll_disturbance = 0
            else:
                self.roll_disturbance -= 0.05

    def vertical_normalize(self):
        if self.vertical_input > 0:
            if self.vertical_input <= 0.2:
                self.vertical_input = 0.0
            else:
                self.vertical_input -= 0.05
        elif self.vertical_input < 0.0:
            if self.vertical_input >= -0.2:
                self.vertical_input = 0.0
            else:
                self.vertical_input += 0.05

    def key_w(self, w_factor=5, p_factor=-0.1):
        if self.imuControl:
            if self.IMU.getRollPitchYaw()[1] > p_factor: 
                self.pitch_disturbance += 0.01 * w_factor
        else:
            self.pitch_disturbance += 0.01 * w_factor

        self.roll_normalize()
        self.vertical_normalize()
        self.yaw_disturbance = 0

    def key_s(self, s_factor=5, p_factor=0.1):
        if self.imuControl:
            if self.IMU.getRollPitchYaw()[1] < p_factor: 
                self.pitch_disturbance -= 0.01 * s_factor
        else:
            self.pitch_disturbance -= 0.01 * s_factor

        self.roll_normalize()
        self.vertical_normalize()
        self.yaw_disturbance = 0

    def key_d(self, d_factor=5, r_factor=-1.47):
        if self.imuControl:
            if self.IMU.getRollPitchYaw()[0] < r_factor:
                self.roll_disturbance -= 0.01 * d_factor
        else:
            self.roll_disturbance -= 0.01 * d_factor

        self.pitch_normalize()
        self.vertical_normalize()
        self.yaw_disturbance = 0

    def key_a(self, a_factor=5, r_factor=-1.67):
        if self.imuControl:
            if self.IMU.getRollPitchYaw()[0] > r_factor:
                self.roll_disturbance += 0.01 * a_factor
        else:
            self.roll_disturbance += 0.01 * a_factor

        self.pitch_normalize()
        self.vertical_normalize()
        self.yaw_disturbance = 0

    def key_e(self):
        if self.imuControl:
            if self.IMU.getRollPitchYaw()[1] > -0.1: 
                self.pitch_disturbance += 0.05

            if self.IMU.getRollPitchYaw()[0] < -1.47:
                self.roll_disturbance -= 0.05
        else:
            self.pitch_disturbance += 0.05
            self.roll_disturbance -= 0.05

        self.vertical_normalize()
        self.yaw_disturbance = 0

    def key_q(self):
        if self.imuControl:
            if self.IMU.getRollPitchYaw()[1] > -0.1: 
                self.pitch_disturbance += 0.05

            if self.IMU.getRollPitchYaw()[0] > -1.67:
                self.roll_disturbance += 0.05
        else:
            self.pitch_disturbance += 0.05
            self.roll_disturbance += 0.05

        self.vertical_normalize()
        self.yaw_disturbance = 0

    def key_right(self):
        if self.yaw_disturbance < 1.3:
            self.yaw_disturbance += 0.05
        self.vertical_normalize()

    def key_left(self):
        if self.yaw_disturbance > -1.3:
            self.yaw_disturbance -= 0.05
        self.vertical_normalize()

    def key_up(self, up_factor=5):
        self.vertical_input += 0.005 * up_factor
        self.yaw_disturbance = 0

    def key_down(self, d_factor=5):
        self.vertical_input -= 0.005 * d_factor
        self.yaw_disturbance = 0

    def key_u(self):
        if self.camera_pitch_value < 1.65:
            self.camera_pitch_value += 0.025

        self.vertical_normalize()
        self.yaw_disturbance = 0

    def key_j(self):
        if self.camera_pitch_value > -0.45:
            self.camera_pitch_value -= 0.025

        self.vertical_normalize()
        self.yaw_disturbance = 0

    def key_k(self):
        if self.camera_yaw_value < 1.65:
            self.camera_yaw_value += 0.025

        self.vertical_normalize()
        self.yaw_disturbance = 0

    def key_h(self):
        if self.camera_yaw_value > -0.4:
            self.camera_yaw_value -= 0.025

        self.vertical_normalize()
        self.yaw_disturbance = 0

    def key_z(self, TIME):
        if TIME - self.lastChanged["key_z"] > 0.20:
            self.imuControl = not self.imuControl
            self.lastChanged["key_z"] = TIME
            if self.imuControl:
                print("- - - - - -\nImu Control Has Been Activated...")
            else:
                print("- - - - - -\nImu Control Has Been Deactivated...")
  
    def key_c(self, TIME):
        if TIME - self.lastChanged["key_c"] > 0.20:
            self.dsControl = not self.dsControl
            self.lastChanged["key_c"] = TIME
            if self.dsControl:
                print("- - - - - -\nDistance Sensors Has Been Activated...")
            else:
                print("- - - - - -\nDistance Sensors Has Been Deactivated...")

    def keyboard_control(self, KEY, TIME):
        if self.willCrash == 0:
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

                elif KEY == ord('U'): # U
                    self.key_u()
                
                elif KEY == ord('J'): # J
                    self.key_j()

                elif KEY == ord('K'): # K
                    self.key_k()
                
                elif KEY == ord('H'): # H
                    self.key_h()

                elif KEY == ord('Z'): # Z
                    self.key_z(TIME)
                
                elif KEY == ord('C'): # X
                    self.key_c(TIME)

                else: # other key combinations we didn't declare is being used
                    self.pitch_normalize()
                    self.roll_normalize()
                    self.vertical_normalize()
                    self.yaw_disturbance = 0

            else: # if no key is being pressed
                self.pitch_normalize()
                self.roll_normalize()
                self.vertical_normalize()
                self.yaw_disturbance = 0
        else :
            if self.shouldForward:
                self.key_w(self.w_factor)
            if self.shouldBackward:
                self.key_s(self.s_factor)
            if self.shouldRight:
                self.key_d(self.d_factor)
            if self.shouldLeft:
                self.key_a(self.a_factor)
            if self.shouldUp:
                self.key_up(self.up_factor)

    def blink_leds(self, TIME):
        self.LED_STATE = int(TIME) % 2
        self.FRONT_LEFT_LED.set(self.LED_STATE)
        self.FRONT_RIGHT_LED.set(self.LED_STATE)

    def motor_control(self, ROLL, PITCH, ALTITUDE, ROLL_ACCELERATION, PITCH_ACCELERATION):
        
        # Compute the roll, pitch, yaw and vertical inputs.
        self.ROLL_INPUT = self.K_ROLL_P * self.clamp(ROLL, -1.0, 1.0) + ROLL_ACCELERATION + self.roll_disturbance
        self.PITCH_INPUT = self.K_PITCH_P * ( self.clamp(PITCH, -1.0, 1.0) - 0.005) - PITCH_ACCELERATION + self.pitch_disturbance
        self.YAW_INPUT = self.yaw_disturbance
        #self.CLAMPED_DIFFERENCE_ALTITUDE = self.clamp(self.target_altitude - ALTITUDE + self.K_VERTICAL_OFFSET, -1.0, 1.0)
        self.VERTICAL_INPUT = self.vertical_input
        
        # Actuate the motors taking into consideration all the computed inputs.
        self.FRONT_LEFT_MOTOR_INPUT = self.K_VERTICAL_THRUST + self.VERTICAL_INPUT - self.ROLL_INPUT - self.PITCH_INPUT + self.YAW_INPUT
        self.FRONT_RIGHT_MOTOR_INPUT  = self.K_VERTICAL_THRUST + self.VERTICAL_INPUT + self.ROLL_INPUT - self.PITCH_INPUT - self.YAW_INPUT
        self.REAR_LEFT_MOTOR_INPUT  = self.K_VERTICAL_THRUST + self.VERTICAL_INPUT - self.ROLL_INPUT + self.PITCH_INPUT - self.YAW_INPUT
        self.REAR_RIGHT_MOTOR_INPUT = self.K_VERTICAL_THRUST + self.VERTICAL_INPUT + self.ROLL_INPUT + self.PITCH_INPUT + self.YAW_INPUT

        Motor.setVelocity(self.FRONT_LEFT_MOTOR, self.FRONT_LEFT_MOTOR_INPUT)
        Motor.setVelocity(self.FRONT_RIGHT_MOTOR, -self.FRONT_RIGHT_MOTOR_INPUT)
        Motor.setVelocity(self.REAR_LEFT_MOTOR, -self.REAR_LEFT_MOTOR_INPUT)
        Motor.setVelocity(self.REAR_RIGHT_MOTOR, self.REAR_RIGHT_MOTOR_INPUT)

        self.CAMERA_PITCH_MOTOR.setPosition(self.camera_pitch_value)
        self.CAMERA_YAW_MOTOR.setPosition(self.camera_yaw_value)

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
        Accelerometer.enable(self.ACCELEROMETER, self.TIME_STEP)

    def checkCrash(self, FV, BV, RV, LV, DV):
        if FV < 1000.0 or  BV < 1000.0 or RV < 1000.0 or LV < 1000.0 or DV < 1000.0 :
            if self.DS_FRONT.getValue() < 1000.0 :
                self.willCrash = 1
                self.shouldBackward = 1

                if self.DS_FRONT.getValue() < 300.0 :
                    self.s_factor = 13
                elif self.DS_FRONT.getValue() < 500.0 :
                    self.s_factor = 11
                elif self.DS_FRONT.getValue() < 750.0 : 
                    self.s_factor = 9

                self.s_factor = 7
            else:
                self.shouldBackward = 0
                self.s_factor = 5

            if self.DS_BACK.getValue() < 1000.0 :
                self.willCrash = 1
                self.shouldForward = 1

                if self.DS_BACK.getValue() < 300.0 :
                    self.w_factor = 13
                if self.DS_BACK.getValue() < 500.0 :
                    self.w_factor = 11
                if self.DS_BACK.getValue() < 750.0 : 
                    self.w_factor = 9

                self.w_factor = 7
            else:
                self.shouldForward = 0
                self.w_factor = 5

            if self.DS_LEFT.getValue() < 1000.0 :
                self.willCrash = 1
                self.shouldRight = 1

                if self.DS_LEFT.getValue() < 300.0 :
                    self.d_factor = 13
                if self.DS_LEFT.getValue() < 500.0 :
                    self.d_factor = 11
                if self.DS_LEFT.getValue() < 750.0 : 
                    self.d_factor = 9

                self.d_factor = 7
            else:
                self.shouldRight = 0
                self.d_factor = 5

            if self.DS_RIGHT.getValue() < 1000.0 :
                self.willCrash = 1
                self.shouldLeft = 1

                if self.DS_RIGHT.getValue() < 300.0 :
                    self.a_factor = 13
                if self.DS_RIGHT.getValue() < 500.0 :
                    self.a_factor = 11
                if self.DS_RIGHT.getValue() < 750.0 : 
                    self.a_factor = 9

                self.d_factor = 7 
            else:
                self.shouldLeft = 0
                self.a_factor = 5

            if self.DS_DOWN.getValue() < 1000.0 :
                self.willCrash = 1
                self.shouldUp = 1

                if self.DS_DOWN.getValue() < 300.0 :
                    self.up_factor = 13
                if self.DS_DOWN.getValue() < 500.0 :
                    self.up_factor = 11
                if self.DS_DOWN.getValue() < 750.0 : 
                    self.up_factor = 9

                self.up_factor = 7 
            else:
                self.shouldUp = 0
                self.up_factor = 5

        else:
            self.willCrash = 0
            self.shouldBackward = 0
            self.shouldForward = 0
            self.shouldRight = 0
            self.shouldLeft = 0
            self.shouldUp = 0

    def getImage(self):
        data = self.CAMERA.getImage()
        img = np.fromstring( data, dtype='uint8').reshape((self.CAMERA_HEIGHT, self.CAMERA_WIDTH, 4)) # covert image buffer to uint8 array
        #image = np.frombuffer(imgData, np.uint8).reshape((camHeight, camWidth, 4))
        self.img = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
        self.retval, self.points = qrDetector.detect(self.img)

    def landingSupport(self, TIME): # otonom landing using kamera and opencv qr, finds the center of qr in img and tries to centerize the qr
        if self.retval:
            x1 = (self.points[0, 0, 0] + self.points[0, 1, 0]) / 2
            y1 = (self.points[0, 0, 1] + self.points[0, 1, 1]) / 2
            x2 = (self.points[0, 2, 0] + self.points[0, 3, 0]) / 2
            y2 = (self.points[0, 2, 1] + self.points[0, 3, 1]) / 2

            self.destX = (x1 + x2) / 2
            self.destY = (y1 + y2) / 2

            if (self.destX > self.posX - 35 and self.destX < self.posX + 35) and (self.destY > self.posY - 35 and self.destY < self.posY + 35 ):
                """if self.pitch_disturbance != 0 :
                    self.pitch_normalize()
                elif self.roll_disturbance != 0 :
                    self.roll_normalize()
                else:
                    self.key_down(d_factor=1)"""
                
                if self.accelerationNormalize(TIME):
                    if self.ACCELEROMETER.getValues()[2] > 9.7:
                        self.key_down(d_factor=1)
        
            elif self.destX < self.posX - 50:
                self.key_a(a_factor=1, r_factor=-1.6)

            elif self.destX > self.posX + 50:
                self.key_d(d_factor=1, r_factor=-1.54)

            elif self.destY < self.posY - 40:
                self.key_w(w_factor=1, p_factor=-0.05)

            elif self.destY > self.posY + 40:
                self.key_s(s_factor=1, p_factor=0.05)

    def accelerationNormalize(self, TIME):
        accelarationValues = self.ACCELEROMETER.getValues()
        if not self.timeCalculated:
            self.normalizeTime = TIME + 0.1
            self.timeCalculated = True

        if accelarationValues[0] < -0.2:
            # stop forward acceleration
            if self.normalizeTime > TIME:
                self.pitch_disturbance = -0.2
            else:
                self.timeCalculated = False
                self.pitch_disturbance = 0
            return 0
        elif accelarationValues[0] > 0.2:
            # stop backward acceleration
            if self.normalizeTime > TIME:
                self.pitch_disturbance = 0.2
            else:
                self.timeCalculated = False
                self.pitch_disturbance = 0
            return 0
        elif accelarationValues[1] < -0.2:
            # stop left acceleration
            if self.normalizeTime > TIME:
                self.roll_disturbance = -0.1
            else:
                self.timeCalculated = False
                self.roll_disturbance = 0
            return 0
        elif accelarationValues[1] > 0.2:
            # stop right acceleration
            if self.normalizeTime > TIME:
                self.roll_disturbance = 0.1
            else:
                self.timeCalculated = False
                self.roll_disturbance = 0
            return 0
        else:
            return 1

    def landingSupport2(self, TIME): # otonomous landing using kamera and opencv qr, finds the center of qr in img and tries to centerize the qr
        if self.retval:
            x = [self.points[0, 0, 0], self.points[0, 1, 0], self.points[0, 2, 0], self.points[0, 3, 0]]
            y = [self.points[0, 0, 1], self.points[0, 1, 1], self.points[0, 2, 1], self.points[0, 3, 1]]

            minx = min(x)
            maxx = max(x)
            miny = min(y)
            maxy = max(y)

            if  (miny < 55 and maxy > 185):# top and bottom parts of qr place is near the camera heights top and bottom
                if ( minx > 85 ) : # qr place is in the middle enough, land
                    if self.accelerationNormalize(TIME):
                        if self.ACCELEROMETER.getValues()[2] > 9.7:
                            self.key_down(d_factor=1)
                else:
                    self.key_d(d_factor=1, r_factor=-1.54)

            else:
                if ( (self.CAMERA_HEIGHT - maxy) - miny )  > 10: # qr is closer to top
                    self.key_w(w_factor=1, p_factor=-0.05)
                
                elif ( (self.CAMERA_HEIGHT - maxy) - miny )  < -10: # qr is closer to bottom
                    self.key_s(s_factor=1, p_factor=0.05)
                
                else:
                    if self.accelerationNormalize(TIME):
                        if self.ACCELEROMETER.getValues()[2] > 9.7:
                            self.key_down(d_factor=1)
