import argparse
import Queue
import subprocess
import sys
import threading
import time
import traceback

import numpy as np
import odrive
import odrive.enums
from fibre.utils import TimeoutError
from fibre.protocol import ChannelBrokenException




possibleErrorCodes = {
    "axis": odrive.enums.errors.axis.__dict__,
    "motor": odrive.enums.errors.motor.__dict__,
    "encoder": odrive.enums.errors.encoder.__dict__,
    "controller": odrive.enums.errors.controller.__dict__
}

if "europa" == getHarvestRobotName():
    sideName = ""
elif "ceres1" == getHarvestRobotName():
    sideName = "PORT"
elif "ceres2" == getHarvestRobotName():
    sideName = "STBD"


class MotorController(object):
    """
    Interfaces with the BLDC motor controller hardware. Runs as a background process
    and handles commands from MotorControllerClient (gripper.py).
    """
    retractTimeoutSeconds = 0.05
    extrudeTimeoutSeconds = 0.05
    def __init__(self, shouldLogEncoderCounts=False):
        self.shouldLogEncoderCounts = shouldLogEncoderCounts
        self.isAlive = True
        self.retractHalfwaySteps = 7800.0
        self.extrudeSteps = -3*self.retractHalfwaySteps
        self.rotationThreshold = 0.5 # 0.1 makes the belts turn too little, was originally 0.9
        self.lock = threading.Lock()

        self.servoDriver = None
        self.isServoDriverCalibrated = False
        self.isApplyingHoldingCurrent = False
        self.lastHoldingCurrentTimeStamp = time.time()
        self.connectionManagementLoopThread = threading.Thread(target=self.connectionManagementLoop)
        self.connectionManagementLoopThread.daemon = True
        self.connectionManagementLoopThread.start()

        while not self.isServoDriverReady():
            time.sleep(0.1)

        gripperBeltServiceName = "gripperbeltservice"
        print("{} is ready.".format(gripperBeltServiceName))

    def connectionManagementLoop(self):
        holdingLimitSeconds = 5.0
        numFailedConnectionAttempts = 0
        try:
            while self.isAlive:
                if not self.isServoDriverReady():
                    print("BLDC motor controller trying to connect, {} consecutive "
                                  "failed attempts.".format(numFailedConnectionAttempts))
                    self.servoDriver = None
                    didMotorControllerConnect, failReason = self.connectMotorController()
                    if didMotorControllerConnect:
                        print("BLDC motor controller is connected.")
                        numFailedConnectionAttempts = 0
                        self.isServoDriverCalibrated = True
                    else:
                        print(15, failReason)
                        numFailedConnectionAttempts += 1
                else:
                    if (self.isApplyingHoldingCurrent
                            and time.time() - self.lastHoldingCurrentTimeStamp > holdingLimitSeconds):
                        self.idleBelts()
                time.sleep(0.1)
        except:
            e = traceback.format_exc()
            print("Motor controller connection management thread caught error and exiting: {}".format(e))

    def connectMotorController(self):


        deviceID = "1209:0d32"
        try:
            usbBusDevice = subprocess.check_output("lsusb -d {}".format(deviceID), shell=True)
        except subprocess.CalledProcessError as e:
            return False, "USB device \'{}\' not found in query".format(deviceID)

        busIndex = usbBusDevice.index("Bus")
        deviceIndex = usbBusDevice.index("Device")
        if usbBusDevice == "" or busIndex < 0 or deviceIndex < 0:
            return False, "USB device had unexpected bus or device index \'{}\'".format(usbBusDevice)
        busNumber = usbBusDevice[busIndex+4 : busIndex+7]
        deviceNumber = usbBusDevice[deviceIndex+7 : deviceIndex+10]
        try:
            self.servoDriver = odrive.find_any(path="usb:{}:{}".format(busNumber, deviceNumber), timeout=30)
        except TimeoutError:
            return False, "Gripper could not connect to motor controller"

        self.servoDriver.config.brake_resistance = 0.5
        for servo in [self.servoDriver.axis0, self.servoDriver.axis1]:
            servo.motor.config.current_lim = 13.0
            servo.motor.config.calibration_current = 13.0
            servo.motor.config.resistance_calib_max_voltage = 4.0
            servo.motor.config.pole_pairs = 7
            servo.motor.config.motor_type = 0
            servo.encoder.config.cpr = 8192
            servo.controller.config.pos_gain = 100.0
            servo.controller.config.vel_gain = 0.000125
            servo.controller.config.vel_integrator_gain = 0.00001875
            servo.controller.config.vel_limit = 901120.0
            servo.trap_traj.config.vel_limit = 819200.0
            servo.trap_traj.config.accel_limit = 8192000.0
            servo.trap_traj.config.decel_limit = 8192000.0

        try:
            if False in [self.servoDriver.axis0.motor.is_calibrated, self.servoDriver.axis0.encoder.is_ready,
                         self.servoDriver.axis1.motor.is_calibrated, self.servoDriver.axis1.encoder.is_ready]:
                with self.lock:
                    self.servoDriver.axis0.requested_state = odrive.enums.AXIS_STATE_FULL_CALIBRATION_SEQUENCE
                    self.servoDriver.axis1.requested_state = odrive.enums.AXIS_STATE_FULL_CALIBRATION_SEQUENCE
                startCalibrationTime = time.time()
                while ((self.servoDriver.axis0.current_state != odrive.enums.AXIS_STATE_IDLE or
                        self.servoDriver.axis1.current_state != odrive.enums.AXIS_STATE_IDLE) and time.time()-startCalibrationTime <= 20 and self.isAlive):
                    time.sleep(0.1)

                if False in [self.servoDriver.axis0.motor.is_calibrated, self.servoDriver.axis0.encoder.is_ready,
                             self.servoDriver.axis1.motor.is_calibrated, self.servoDriver.axis1.encoder.is_ready]:
                    self.isReady()
                    return False, "Gripper failed motor+encoder calibration"

            if (abs(self.servoDriver.axis0.controller.pos_setpoint) == float("inf") or
                    abs(self.servoDriver.axis1.controller.pos_setpoint) == float("inf")):
                return False, "Gripper encoder position set-point stuck at infinity"

        except AttributeError as e:
            if "'RemoteObject' object has no attribute 'axis" in str(e):
                print("BLDC controller: RemoteObject error, disconnected while connecting")
                return False, "Gripper failed motor+encoder calibration"
        except:
            e = traceback.format_exc()
            print("BLDC controller: unknown exception in connectMotorController: {}".format(e))
            return False, str(e)
        return True, ""

    def protectFromDisconnect(commandFunction):
        def protectedCommandFunction(self):
            didSucceed = False
            try:
                didSucceed = commandFunction(self)
            except ChannelBrokenException as e:
                print("BLDC controller: ChannelBrokenException")
                self.isServoDriverCalibrated = False
                self.servoDriver = None
            except AttributeError as e:
                if "'RemoteObject' object has no attribute 'axis" in str(e):
                    print("BLDC controller: RemoteObject error, disconnected while calling "
                                 "\'{}()\': {}".format(commandFunction.__name__, e))
                else:
                    print("BLDC controller: AttributeError, disconnected while calling: "
                                 "{}".format(commandFunction.__name__, e))
                self.isServoDriverCalibrated = False
                self.servoDriver = None
            return didSucceed
        return protectedCommandFunction

    @protectFromDisconnect
    def retractHalfway(self):
        self._enableControl()

        axis0StartingCounts = self.servoDriver.axis0.encoder.shadow_count
        axis1StartingCounts = self.servoDriver.axis1.encoder.shadow_count

        with self.lock:
            self.isApplyingHoldingCurrent = True
            self.lastHoldingCurrentTimeStamp = time.time()
            self.servoDriver.axis0.controller.move_incremental(self.retractHalfwaySteps, False)
            self.servoDriver.axis1.controller.move_incremental(self.retractHalfwaySteps, False)

        didRetract = True
        timeStart = time.time()
        while ((self.servoDriver.axis0.encoder.shadow_count - axis0StartingCounts < self.retractHalfwaySteps * self.rotationThreshold or
                self.servoDriver.axis1.encoder.shadow_count - axis1StartingCounts < self.retractHalfwaySteps * self.rotationThreshold) and self.isAlive):
            elapsedTime = time.time() - timeStart
            if self.shouldLogEncoderCounts:
                print("BLDC retract counts: {}, {}, {}".format(
                    self.servoDriver.axis0.encoder.shadow_count - axis0StartingCounts,
                    self.servoDriver.axis1.encoder.shadow_count - axis1StartingCounts,
                    self.retractHalfwaySteps * self.rotationThreshold))
            if elapsedTime >= self.retractTimeoutSeconds:
                if self.shouldLogEncoderCounts:
                    print("BLDC retract break")
                didRetract = False
                break
            time.sleep(0.005)

        return didRetract

    @protectFromDisconnect
    def extrude(self):
        self._enableControl()

        axis0StartingCounts = self.servoDriver.axis0.encoder.shadow_count
        axis1StartingCounts = self.servoDriver.axis1.encoder.shadow_count

        with self.lock:
            self.isApplyingHoldingCurrent = True
            self.lastHoldingCurrentTimeStamp = time.time()
            self.servoDriver.axis0.controller.move_incremental(self.extrudeSteps, False)
            self.servoDriver.axis1.controller.move_incremental(self.extrudeSteps, False)

        didExtrude = True
        timeStart = time.time()
        while ((self.servoDriver.axis0.encoder.shadow_count - axis0StartingCounts > self.extrudeSteps * self.rotationThreshold or
                self.servoDriver.axis1.encoder.shadow_count - axis1StartingCounts > self.extrudeSteps * self.rotationThreshold) and self.isAlive):
            elapsedTime = time.time() - timeStart
            if self.shouldLogEncoderCounts:
                print("BLDC extrude counts: {}, {}, {}".format(
                    self.servoDriver.axis0.encoder.shadow_count - axis0StartingCounts,
                    self.servoDriver.axis1.encoder.shadow_count - axis1StartingCounts,
                    self.extrudeSteps * self.rotationThreshold))
            if elapsedTime >= self.extrudeTimeoutSeconds:
                if self.shouldLogEncoderCounts:
                    print("BLDC extrude break")
                didExtrude = False
                break
            time.sleep(0.005)

        self.idleBelts()
        return didExtrude

    @protectFromDisconnect
    def _enableControl(self):
        with self.lock:
            self.servoDriver.axis0.controller.pos_setpoint = self.servoDriver.axis0.encoder.shadow_count
            self.servoDriver.axis1.controller.pos_setpoint = self.servoDriver.axis1.encoder.shadow_count
            self.servoDriver.axis0.requested_state = odrive.enums.AXIS_STATE_CLOSED_LOOP_CONTROL
            self.servoDriver.axis1.requested_state = odrive.enums.AXIS_STATE_CLOSED_LOOP_CONTROL
        return True

    @protectFromDisconnect
    def idleBelts(self):
        with self.lock:
            self.servoDriver.axis0.requested_state = odrive.enums.AXIS_STATE_IDLE
            self.servoDriver.axis1.requested_state = odrive.enums.AXIS_STATE_IDLE
            self.isApplyingHoldingCurrent = False
        return True

    def isReady(self):
        try:
            controllerReportedErrors = {
                "axis0": {
                    "axis": self.servoDriver.axis0.error,
                    "motor": self.servoDriver.axis0.motor.error,
                    "encoder": self.servoDriver.axis0.encoder.error,
                    "controller": self.servoDriver.axis0.controller.error},
                "axis1": {
                    "axis": self.servoDriver.axis1.error,
                    "motor": self.servoDriver.axis1.motor.error,
                    "encoder": self.servoDriver.axis1.encoder.error,
                    "controller": self.servoDriver.axis1.controller.error}
            }
        except ChannelBrokenException as e:
            print("BLDC controller: ChannelBrokenException")
            return False
        except:
            e = traceback.format_exc()
            print("BLDC controller: unknown exception in isReady: {}".format(e))
            return False

        sumAxis0ErrorCodes = sum([errorValue for errorKey, errorValue in controllerReportedErrors["axis0"].iteritems()])
        sumAxis1ErrorCodes = sum([errorValue for errorKey, errorValue in controllerReportedErrors["axis1"].iteritems()])
        if sumAxis0ErrorCodes == 0 and sumAxis1ErrorCodes == 0:
            return True

        for axisName in controllerReportedErrors.keys():
            for errorSource, errorValue in controllerReportedErrors[axisName].iteritems():
                if errorValue == 0:
                    continue
                listOfErrorNames = []
                for errorCodeName, errorCodeValue in possibleErrorCodes[errorSource].iteritems():
                    if type(errorCodeValue) is not int or errorCodeValue == 0:
                        continue
                    if 1 & (errorValue >> int(np.log2(errorCodeValue))):
                        listOfErrorNames.append(errorCodeName)
                print("Gripper error in {}, {}: {}".format(axisName, errorSource, listOfErrorNames))
        return False

    def teardown(self):
        self.idleBelts()
        self.isAlive = False
        self.connectionManagementLoopThread.join(0.5)

    def handleRemoteCommand(self, request):
        if not self.isServoDriverReady():
            print("BLDC motor controller is not ready, command {} failed.".format(request.commandName))
            return gripperbeltResponse(False, "BLDC controller not ready")
        if request.commandName == "retractHalfway":
            return gripperbeltResponse(self.retractHalfway(), "")
        elif request.commandName == "extrude":
            return gripperbeltResponse(self.extrude(), "")
        elif request.commandName == "idleBelts":
            return gripperbeltResponse(self.idleBelts(), "")
        elif request.commandName == "isReady":
            return gripperbeltResponse(self.isReady(), "")
        elif request.commandName == "teardown":
            return gripperbeltResponse(self.teardown(), "")

    def isServoDriverReady(self):
        if self.servoDriver is None:
            return False
        if not self.isServoDriverCalibrated:
            return False
        if not hasattr(self.servoDriver, "axis0") or not hasattr(self.servoDriver, "axis1"):
            return False
        if not self.isReady():
            return False
        return True


def main(args):
    try:
        motorController = MotorController(args.shouldLogEncoderCounts)
    except IOError:
        errorMessage = "Gripper failed to init motor controller"
        print(errorMessage)
        raise IOError(errorMessage)

    motorController.retractHalfway()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=("Background process which controls "
            "gripper BLDC motors."))
    parser.add_argument("-l",
        "--logencodercounts",
        dest="shouldLogEncoderCounts",
        help="Set flag to log encoder counts of BLDC during retract and extrude.",
        action="store_true")
    args = parser.parse_args()
    main(args)