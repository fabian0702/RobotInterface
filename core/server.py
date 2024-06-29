from nicegui import ui, app
import rowan, time, os
import numpy as np

from multiprocessing import Process

import python_main.ch.bfh.roboticsLab.robot.RobotSimulator as RobotSimulator
from python_main.ch.bfh.roboticsLab.util.Logger import Logger
import logging

from core.model import RobotModel
from core.constants import SERVER_ADDRESS, START_SERVER, LOG_LEVEL
logger = Logger(os.path.basename(__file__), LOG_LEVEL).getInstance()

from sys import path
path.append('./python_main/')

from python_main.ch.bfh.roboticsLab.robot.RobotClient import RobotClient
from python_main.ch.bfh.roboticsLab import Base_pb2 as pbBase
from python_main.ch.bfh.roboticsLab.robot import RobotControl_pb2 as pbRobotControl
from python_main.ch.bfh.roboticsLab.util.TransformationMatrix import TransformationMatix

class RobotServer:
    """Class to handle communication with the server"""
    def __init__(self, robotModel:RobotModel) -> None:
        self.robotModel = robotModel
        self.jointsValue = None
        self.robotPose = None
        self.moving = None
        self.freeDrive = False
        self.gripper = 0.0
        self.on_gripper_change = None
        self.on_freedrive_change = None
        self.on_power_change = None
        self.on_initialized = None
        self.initializedAxis = False
        self.currentTime = 0.0
        self.Cartesian = None
        self.wrongRobot = False
        self.client = None
        self.precision = False
        self.speed = 0
        self.max_speed = pbBase.LinearAngularPair(linear = robotModel.maxLinearSpeed, angular=robotModel.maxAngualarSpeed)
        self.max_tolerance = pbBase.LinearAngularPair(linear = robotModel.maxLinearTolerance, angular=robotModel.maxAngularTolerance)
        self.callbacks = []

        def startServer(axis_count, logger:Logger):
            pass

        try:
            self.local_server = None
            if START_SERVER:
                self.local_server = RobotSimulator.Robot(self.robotModel.axisCount)
                #self.local_server.start()
        except RuntimeError as e:
            with ui.dialog() as self.invalideMoveDialog, ui.card().classes('w-1/4'):          # Dialog to display errormessage when freedrive is enabled but the robot is commanded to move
                ui.label("Unable to start a simulation server. This is most likely because there is already another server running.")
                ui.button('OK', on_click=self.invalideMoveDialog.close)
            logger.error(e)
            self.local_server = None

        self.connectRobot()
        app.on_shutdown(self.shutdown)
        app.on_disconnect(self.shutdown)
        with ui.dialog() as self.invalideMoveDialog, ui.card().style('width:25%'):          # Dialog to display errormessage when freedrive is enabled but the robot is commanded to move
            ui.label("The robot can't be moved by the buttons while in freedrive.")
            ui.button('OK', on_click=self.invalideMoveDialog.close)
        with ui.dialog() as self.robotNotConnectedDialog, ui.card().style('width:25%'):     # Dialog to display errormessage when robot isn't is connected but the robot is commanded
            ui.label("The robot can't be commanded while it's disconnected.")
            ui.button('OK', on_click=self.robotNotConnectedDialog.close)

    def connectRobot(self):         
        """Connects the robot to the grpc server and changes the status button"""
        if self.on_power_change is not None:
            self.on_power_change(True, suppress=True)
        if self.client is None:
            self.client = RobotClient(SERVER_ADDRESS, self.myProcessSubscription)      # connect robot
    def disconnectRobot(self):      
        """Disconnects the robot and change the status"""
        if self.on_power_change is not None:
            self.on_power_change(False, suppress=True)       # Update Btn
        if not self.client is None:
            self.client.shutdown()      # disconnect robot
            self.client = None
    def btnMoveJoints(self, axisName:str, distance:float, absolute:bool = False):   
        """Function to move one joint by the distance, distance can either be relative or absolute, joint is selected by name"""
        if self.freeDrive:
            self.invalideMoveDialog.open()
            return
        if self.client is None:
            self.robotNotConnectedDialog.open()
            return
        axisIndex = self.robotModel.getAxisIndex(axisName)
        pose = [0] * self.robotModel.axisCount
        pose[axisIndex%self.robotModel.axisCount] = distance * (0.1 if self.precision else 1) * (self.robotModel.AxisSteps[axisIndex] if not absolute else 1) / self.robotModel.AxisGain[axisIndex]
        return self.moveJoints(pose)
    def btnMoveCartesian(self, axisName:str, distance:float, absolute:bool = False):
        """Function to move one axis by the distance, distance can either be relative or absolute, axis is selected by name"""
        if self.freeDrive:           # Show error when in freedrive and attempt to move
            self.invalideMoveDialog.open()
            return
        if self.client is None:           # Show error when robot not connected and attempt to move
            self.robotNotConnectedDialog.open()
            return
        axisIndex = self.robotModel.getAxisIndex(axisName)
        pose = [0] * self.robotModel.axisCount
        pose[axisIndex%self.robotModel.axisCount] = distance * (0.1 if self.precision else 1) * (self.robotModel.AxisSteps[axisIndex] if not absolute else 1) / self.robotModel.AxisGain[axisIndex]
        return self.moveCartesian(pose)
    
    def changeGripperState(self, state:bool):
        """Function to change the gripper state indicator button and changes the status button"""
        if self.client is None:
            self.robotNotConnectedDialog.open()
            return
        self.client.gripper(1.0 if state else 0.0)      # Send command

    def changeFreedrive(self, state:bool):
        """Function to change the freedrive state indicator button and changes the status button"""
        if self.client is None:
            self.robotNotConnectedDialog.open()
            return
        self.client.requester.freedrive(pbRobotControl.Freedrive(state=state))      # Send command

    def moveJoints(self, pose:list[int]):             
        """Function to moves the Joints to a absolute position specified in the pose list as radians"""
        if self.jointsValue is not None:
            actualJoints = np.array(self.jointsValue)
            logger.info(actualJoints)
            newJoints = actualJoints + np.array(pose)
            self.client.moveJoints(jointsOrPose=pbBase.ArrayDouble(value=list(newJoints)),override=self.speed)
    def moveCartesian(self, pose:list[int]):
        """Moves the Axis to a absolute position specified in the pose list as radians"""
        if self.robotPose is not None:
            actualPose = TransformationMatix.fromPose(self.robotPose)       # Initial Pose of the robot
            newRotation = rowan.from_euler(pose[self.robotModel.getAxisIndex('RZ')-self.robotModel.axisCount] if 'RZ' in self.robotModel.AxisNames else 0.0,   # new Transformation to apply
                                           pose[self.robotModel.getAxisIndex('RY')-self.robotModel.axisCount] if 'RY' in self.robotModel.AxisNames else 0.0, 
                                           pose[self.robotModel.getAxisIndex('RX')-self.robotModel.axisCount] if 'RX' in self.robotModel.AxisNames else 0.0,'zyx')
            offset = TransformationMatix.compose(pose[:3],[1,0,0,0])
            offset2 = TransformationMatix.compose([0,0,0],newRotation)
            newPose = offset*actualPose*offset2            
            self.client.moveCartesian(pose=newPose.pose(), override=self.speed)        # Move the robot to the new pose
    def registerUpdateCallback(self, callback:lambda:None):     
        """Function to register a callback for the process subscription"""
        self.callbacks.append(callback)
    def myProcessSubscription(self, message):                   
        """Function which gets call when a new message from the grpc server has been received"""
        try:
            self.published = message
            self.jointsValue = np.array(message.jointValues.value) if message.HasField("jointValues") else self.jointsValue     # Extract Values from message
            self.robotPose = message.robotPose if message.HasField("robotPose") else self.robotPose
            self.moving = message.state.moving if message.HasField("state") else self.moving
            freeDriveAct = message.freedrive.state if message.HasField("freedrive") else False
            gripperAct = message.gripper.position if message.HasField("gripper") else 0.0
            if self.robotPose is None or self.jointsValue is None or self.moving is None:       # Return if the server hasn't send any values yet
                return
            if not len(self.jointsValue) == self.robotModel.axisCount:                       # Calls the dialog when a robot has been selected which doesnt match the information from the grpc server
                if self.wrongRobot:
                    return
                self.wrongRobot = True
                self.wrongRobotSelected()
            if message.HasField("freedrive") and (not self.freeDrive == freeDriveAct) and (not self.on_freedrive_change is None):         # Updates the freedrive button if the state has changed
                self.freeDrive = freeDriveAct
                self.on_freedrive_change(state=freeDriveAct, suppress=True)
            if message.HasField("gripper") and (not (abs(self.gripper - gripperAct) < 0.01)) and (not self.on_gripper_change is None):    # Updates the gripper button if the state has changed
                self.gripper = gripperAct
                self.on_gripper_change(state=gripperAct>0.5, suppress=True)
            if not self.initializedAxis and message.HasField("robotPose") and message.HasField("jointValues"):          # Sets the initial Values for the axis after initializing in the Interface
                self.initializedAxis = True
                if self.on_initialized is not None:
                    self.on_initialized()
            newPosition, newOrientation = TransformationMatix.fromPose(self.robotPose).decomposeNumpy()
            newEuler = rowan.to_euler(newOrientation)
            self.Cartesian = np.concatenate((newPosition, newEuler[::-1]))
            self.currentTime = time.time()
            for callback in self.callbacks:
                callback()
        except Exception as e:
            logger.error(e)

    def shutdown(self):                 
        """Function to shut the requester down"""
        self.client.shutdown()
        if self.local_server is not None:
            self.local_server.shutdown()

    def wrongRobotSelected(self):                       
        """Is called when a robot with different amount of axis is selected then the grpc server says"""
        with ui.dialog() as wrongSelectionDialog:
            with ui.card().classes('w-1/4'):
                ui.label('Please select the same robot as you are connected to.')
                ui.button('Done', on_click=lambda: ui.open('/'))
        wrongSelectionDialog.open()
