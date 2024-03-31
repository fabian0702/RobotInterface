import time, rowan, base64, cv2, json, sys, argparse

from math import log10
from os.path import isfile

from urllib.parse import quote

from fastapi.responses import RedirectResponse

from nicegui import ui, app, events
from nicegui.element import Element
from nicegui.elements.scene import Scene
from nicegui.elements.scene_object3d import Object3D

import numpy as np

from matplotlib import pyplot as plt
plt.switch_backend('agg')               # change backend to optimise charts

sys.path.append('./python_main/')

from python_main.ch.bfh.roboticsLab.robot.RobotClient import RobotClient
from python_main.ch.bfh.roboticsLab import Base_pb2 as pbBase
from python_main.ch.bfh.roboticsLab.robot import RobotControl_pb2 as pbRobotControl
from python_main.ch.bfh.roboticsLab.util.Logger import Logger
from python_main.ch.bfh.roboticsLab.util.TransformationMatrix import TransformationMatix

import resources.environment.environment as environment

import CameraServer

parser = argparse.ArgumentParser(
    description='Run the BFH Robotic Interface')

parser.add_argument('--serverhost', 
                    default='localhost', 
                    type=str,
                    help='the ip address of the grpc robot server')

SERVER_ADDRESS = parser.parse_args().serverhost

logger = Logger('main').getInstance()        # Setup Logger

class direction: 
    """Data class to store direction"""   
    up = 1
    down = -1

speed = 0.1                     # global speed

CHART_LOG_TIME = 10              # Total timespan on chart
CHART_UPDATE_INTERVAL = 0.4       # Interval between data refreshs

JSON_PATH = './resources/models/'        # Path to where the configuration files for the robots are saved
ENVIRON_PATH = './resources/environment/'        # Path to where the environments files are saved

with open(JSON_PATH+'robotSelect.json') as f:        # Parses all robots from the robotSelect.json
    robotSelectionData = json.loads(f.read())

class toggleButton:
    """Togglebutton class"""
    def __init__(self, text:str, icon:str = None, disable=False, on_change=lambda a:None, tooltip:str=None):
        self.text = text
        self.color = 'rgb(15 23 42)'
        self.pressedColor = 'rgb(187 247 208)'
        self.pressed = False
        self.icon = icon        
        self.disable = disable
        self.tooltip = tooltip
        self.toggle()
        self.onchange = on_change
        
    @ui.refreshable
    def toggle(self):
        """Function to setup the toggle element itself"""
        with ui.button(text=self.text, icon=self.icon, on_click=self.handlePress, color=self.color if not self.pressed else self.pressedColor) as btn:
            btn.classes('px-5 m-[-0.2em] text-white')
            if not self.tooltip is None:
                btn.tooltip(self.tooltip)
    def handlePress(self, state=None, suppress = False):
        """Function to handle the press of the button or to silently change the state of the button with the suppress argument"""
        if state is None and not self.disable:
            self.pressed = not self.pressed
        if state == self.pressed:
            return
        elif not state is None:
            self.pressed = state
        if not suppress:
            self.onchange(self.pressed)
        self.toggle.refresh()

class RobotModel:
    """Class to parse and hold all important parameters of the robot"""
    maxLinearSpeed = 0.1
    maxAngualarSpeed = 0.1
    maxLinearTolerance = 0
    maxAngularTolerance = 0
    isInitalized = False
    jointLookupMatrix = [np.array([])]
    offsets:list[list[float]] = [()]
    files:list[str] = []

    def __init__(self, id:str=None, path:str=None):
        if path is None or path == '' or not path.endswith('.json') or id is None:      # Redirect to selection page if robot hasn't been chosen
            self.name = ''
            self.axisCount = 0
            self.AxisNames = []
            self.isCompliant = False
            self.has3DModel = False
            self.rotationAxisCount = 0
            ui.open('/')
            return

        with open(path, 'r', encoding='utf-8') as f:        # Open the json file of the robot
            self.isInitalized = True
            jsonData = json.loads(f.read())
            self.name = jsonData['name']
            self.has3DModel = jsonData['has3DModel']
            jsonModelPath = f'{JSON_PATH}{id}/model.json'
            if isfile(jsonModelPath) and self.has3DModel:    # check if file exists and if the robot has a 3d model
                with open(jsonModelPath, 'r') as f:      # open the json file for the simulation
                    modelJson = json.loads(f.read())
                    self.jointLookupMatrix = [np.array(x) for x in modelJson['jointLookupMatrix']]      # Matrix to lookup the vector of the joint
                    self.offsets = modelJson['offsets']                                                 # List to get the offset of the link relative to the origin
                    self.files = modelJson['files']                                                     # List of all 3D files of the robot
                    self.globalSimulationRotation = [np.array(x) for x in modelJson['globalRotation']]  # Calibration offsets for the robot
                app.add_static_files(f'/static/{id}/', f'{JSON_PATH}{id}/')                    # Configure the path for the 3D models
            self.hasJoints = 'joints' in jsonData.keys()
            cartesian = jsonData['cartesian']
            if self.hasJoints:
                joints = jsonData['joints']
            self.axisCount = len(joints) if self.hasJoints else len(cartesian)
            self.AxisNames = [str(i) for i in range(self.axisCount) if self.hasJoints] + [axis['name'] for axis in cartesian]
            self.jointType = [joint['type'] for joint in (joints if self.hasJoints else [])]
            self.AxisUnits = [joint['properties']['unit'] for joint in ((joints+cartesian) if self.hasJoints else cartesian)]
            self.AxisGain  = [joint['properties']['gain'] for joint in ((joints+cartesian) if self.hasJoints else cartesian)]
            self.AxisSteps = [joint['properties']['step'] for joint in ((joints+cartesian) if self.hasJoints else cartesian)]
            jointsOffset = (self.axisCount if self.hasJoints else 0)
            self.rotationAxisCount = len([name for name in self.AxisNames[jointsOffset:] if 'R' in name.upper()])
            self.isCompliant = jsonData['freedrive'] if 'freedrive' in jsonData.keys() else False
            self.id = id
    def getAxisIndex(self, name):
        """Function which return the internal index of a axis by name"""
        return self.AxisNames.index(name)
    
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
        self.initializedAxis = False
        self.currentTime = 0.0
        self.Cartesian = None
        self.wrongRobot = False
        self.client = None
        self.precision = False
        self.speed = 0
        self.connectRobot()
        self.max_speed = pbBase.LinearAngularPair(linear = robotModel.maxLinearSpeed, angular=robotModel.maxAngualarSpeed)
        self.max_tolerance = pbBase.LinearAngularPair(linear = robotModel.maxLinearTolerance, angular=robotModel.maxAngularTolerance)
        self.callbacks = []
        app.on_shutdown(self.shutdown)
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
            self.client.moveJoints(jointsOrPose=pbBase.ArrayDouble(value=list(newJoints)),override=speed)
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
            self.client.moveCartesian(pose=newPose.pose(),override=speed)        # Move the robot to the new pose
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
                renderRobot.refresh()
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

    def wrongRobotSelected(self):                       
        """Is called when a robot with different amount of axis is selected then the grpc server says"""
        with ui.dialog() as wrongSelectionDialog:
            with ui.card().classes('w-1/4'):
                ui.label('Please select the same robot as you are connected to.')
                ui.button('Done', on_click=lambda: ui.open('/'))
        wrongSelectionDialog.open()

class Chart:
    """Class to render a chart for a configurable amount of axies"""
    def __init__(self, robotModel:RobotModel, robotServer:RobotServer, title:str, xaxisName:str, yaxisName:str, graphNames:list[str], visible = False):
        self.title = title
        self.robotServer = robotServer
        self.index = 0
        self.graphNames = graphNames
        self.maxIndex = int(CHART_LOG_TIME / CHART_UPDATE_INTERVAL)
        self.data:list[list[float]] = [[0 for _ in range(self.maxIndex)] for i in range(len(self.graphNames) + 1)]
        self.graphNames = graphNames
        self.xaxisName = xaxisName
        self.yaxisName = yaxisName
        self.robotModel = robotModel
        self.chart()
        self.plot.set_visibility(visible)       # Hide chart per default
        self.startTime = time.time()
        self.gains = np.array([gain if robotModel.AxisNames[i] in self.graphNames else 0.0 for i, gain in enumerate(robotModel.AxisGain)])
        axisIndecies = list(map(robotModel.getAxisIndex, self.graphNames))
        self.startIndex = min(axisIndecies)
        self.endIndex = max(axisIndecies)
        
    @ui.refreshable
    def chart(self):
        """Function to render the chart"""
        self.chartUpdateTimer = ui.timer(CHART_UPDATE_INTERVAL, self.updateData, active=False)        # setup refresh timer
        self.plot:ui.line_plot = ui.line_plot(n=len(self.graphNames), limit=self.maxIndex-1, figsize=(6, 2)).with_legend(self.graphNames, loc='upper right').classes('w-full')        # plot
        self.plot.fig.text(0.5, 0.03, self.xaxisName, ha='center', va='center')
        self.plot.fig.text(0.1, 0.95, self.yaxisName, ha='center', va='center')
        plt.grid(axis = 'x')        # grid
    
    def updateData(self,):
        """Function to update the data from the server, and display it on the chart"""
        if not self.robotServer.client is None and self.plot.visible and len(self.graphNames) > 0:       # Sanity check
            self.index = int((self.robotServer.currentTime - self.startTime) / CHART_UPDATE_INTERVAL) % self.maxIndex      # Calculate index in ring buffer
            self.data[0][self.index] = (self.robotServer.currentTime - self.startTime) % CHART_LOG_TIME                   # write time in ringbuffer
            for i, val in enumerate((self.gains * (np.concatenate((self.robotServer.jointsValue, self.robotServer.Cartesian)) if self.robotModel.hasJoints else self.robotServer.Cartesian))[self.startIndex:self.endIndex+1]):
                self.data[i+1][self.index] = val      # write the values neccesary in the ring buffer
            self.plot.push(self.data[0], self.data[1:])     # update the chart

    def changeVisibility(self, visible:bool):
        """Function to change the visibility of the charts"""
        if visible:
            self.startTime = time.time()        # keep track of the start time
            self.chartUpdateTimer.activate()    # activate the refresh timer
            self.plot.clear()                   # reset the plot
            if self.robotServer.Cartesian is None or ((not self.robotModel.hasJoints) and self.robotServer.jointsValue is None):
                return
            self.data = [[i * CHART_UPDATE_INTERVAL for i in range(self.maxIndex)]] + [[(np.concatenate((self.robotServer.jointsValue, self.robotServer.Cartesian)) if self.robotModel.hasJoints else self.robotServer.Cartesian)[i +  self.startIndex] for n in range(self.maxIndex)] for i in range(len(self.graphNames))]       # prepare the data
        else:
            self.chartUpdateTimer.deactivate()      # deactivate refresh timer
        
        self.plot.set_visibility(visible)           # hide / show chart

class Axis:
    """Class to render the controll for one axis"""
    def __init__(self, robotModel:RobotModel, robotServer:RobotServer, axisname:str, unit='°', step=0.001, on_move=lambda a, d: None, position = 0.0):
        self.axisname = axisname
        self.robotModel = robotModel
        self.robotServer = robotServer
        self.axisIndex = self.robotModel.getAxisIndex(self.axisname)
        self.position = position*self.robotModel.AxisGain[self.axisIndex]
        self.places = int(abs(log10(self.robotModel.AxisSteps[self.axisIndex])) + 1)
        self.unit = unit
        self.elements = []
        self.fastSpeed = 10
        self.slowSpeed = 1
        self.on_move = on_move
        self.lastPosition = -1
        self.minDelta = 0.0001
        self.editing = False
        self.render()

    def updatePosition(self):
        """Function to update the internal position from the grpc server"""
        if self.editing or self.robotServer.jointsValue is None or self.robotServer.Cartesian is None:
            return
        newPosition = (self.robotServer.Cartesian[self.axisIndex%self.robotModel.axisCount] if self.axisIndex >= self.robotModel.axisCount else list(self.robotServer.jointsValue)[self.axisIndex%self.robotModel.axisCount])*self.robotModel.AxisGain[self.axisIndex]
        if abs(newPosition - self.lastPosition) > self.minDelta:
            self.lastPosition = self.position = newPosition

    def move(self, direction:int, distance:float, absolute:bool = False):
        """Function to move the axis either a distance in the direction or to the absolute position specified in the position field"""
        self.editing = False
        if not self.robotServer.moving:
            logger.info(f'Moving to {distance * direction}')
            self.on_move(self.axisname, distance * direction if not absolute else self.position-self.lastPosition, absolute)

    def changeEditing(self):
        """A Funtion to change the Axis position not to refresh"""
        self.editing = True

    def render(self):
        """Function to render the Axis btns"""
        buttonClasses = 'm-[-0.2em] mb-[-0.3em] mt-[-0.3em]'
        with ui.column().classes('text-center items-center items-stretch mb-[0em]'):
            ui.colors(primary='rgb(15 23 42)')      # setup style
            ui.label(f'{self.axisname} [{self.unit}]').classes('mb-[-0.5em] text-white')        # Axisname
            ui.button('',icon='keyboard_double_arrow_up', on_click=lambda e: self.move(direction.up, self.fastSpeed)).classes(buttonClasses)    # Btn up fast
            ui.button('',icon='keyboard_arrow_up', on_click=lambda e: self.move(direction.up, self.slowSpeed)).classes(buttonClasses)           # Btn up slow
            self.input = ui.number(value=self.position, format=f'%.{self.places}f', step=self.robotModel.AxisSteps[self.axisIndex]/10).on('blur', lambda e: self.move(0, 0, True)).on('focus', self.changeEditing).props('dense borderless color=red-1').classes('bg-slate-700 border-solid border rounded-md my-[-0.4em]').style('width: 5em').bind_value(self, 'position')  # Current position of the Axis
            ui.button('',icon='keyboard_arrow_down', on_click=lambda e: self.move(direction.down, self.slowSpeed)).classes(buttonClasses)       # Btn down slow
            ui.button('',icon='keyboard_double_arrow_down', on_click=lambda e: self.move(direction.down, self.fastSpeed)).classes(buttonClasses)# Btn down fast

class Simulation(Element, component='simulation.js',):
    """Class for the simulation of the robot"""
    def __init__(self, robotModel:RobotModel, robotServer:RobotServer) -> None:
        super().__init__()
        app.on_shutdown(self.shutdown)
        app.add_static_files('/static/environment/', './resources/environment/')
        self.robotModel = robotModel
        self.robotServer = robotServer
        self.graspableObjects:list[Object3D] = []
        self.staticObjects:list[Object3D]
        self.environmentData:dict = {}
        self.grippedObjects:list[Object3D] = []
        self.grippedObjectProperties:list[Object3D] = []
        self.grippedObjectsMatrix:list[tuple[float]] = [[]]
        self.color:list[list[str]] = [[]]
        self.movingClones:list[Object3D] = []
        self.staticClones:list[Object3D] = []
        self.staticObjects:list[Object3D] = []
        self.grippercloseColor:str = '#0088ff'
        self.grippedColor:str = '#ff0000'
        self.gripThreshold:float = 0.004
        self.capture:bool = False
        self.updateClones = True
        self.cameraHeight:float = 0.2
        self.cameras:list[Object3D] = []
        self.environmentCameras = []
        self.environmentCameraClones = []
        self.cameraProperties:list = []
        self.gripped = None
        self.minGripperChange = 0.001
        self.lastGripperPosition = 0
        self.isInitialized = False
        self.updateSimulationTimer:ui.timer = ui.timer(0.1, callback=self.update, active=True)
        self.publisherPair:list[tuple[CameraServer.Publisher, CameraServer.grpc.Server]] = []  # = [CameraServer.serve(i) for i in range(self.cameraID)]

    async def postCameraStream(self, cameraID:int=0):
        """Function to parse a post request to the api, decode get the individual images and send the to the grpc server"""
        base64Img:str = await self.run_method('captureImage', self.cameras[cameraID].id)
        print(len(base64Img))
        if not len(base64Img):
            return
        imgData = base64.decodebytes(base64Img[base64Img.index(','):].encode())     # decode base64 to openCV image
        nparr = np.frombuffer(imgData, np.uint8)
        img = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
        logger.info(f'received capture of camera {cameraID}')
        if cameraID >= len(self.publisherPair):
            try:
                cameraServerPair = CameraServer.serve(cameraID)
                self.publisherPair.append(cameraServerPair)
            except RuntimeError:
                with ui.dialog(value=True) as dialog:
                    with ui.card().classes('w-1/4'):
                        ui.label(f'Please connect a camera client to port 4083{cameraID*2+1} and then refresh the page.')
                        ui.button('Close', on_click=lambda x: dialog.close())
        self.publisherPair[cameraID][0].setImg(img)      # send image to the server

    def changeVisibility(self, visible:bool):
        """changes the visibility of the simulation"""
        self.simulationContainer.set_visibility(visible)

    def changeCapture(self, state):
        """Starts or stops capturing a video stream"""
        logger.info(f'Changing state of video capture to {state}')
        self.capture = state
        
    @ui.refreshable
    def renderSimulation(self):  
        """Funtion for setting up the simulation"""
        if self.robotModel is None or not self.robotModel.isInitalized or self.robotServer.wrongRobot or not self.robotModel.has3DModel or not self.robotModel.hasJoints:
            return
        self.isInitialized = False
        with ui.column(wrap=False).classes('h-screen w-full bg-slate-300 h-full flex flex-column flex-nowrap p-0') as self.simulationContainer:
            with ui.scene().classes('w-full flex-1 m-0 overflow-hidden') as self.scene:
                with self.scene.group() as group:
                    environment.initialize(self, self.scene)    # Initializing of the scene
                                    
                    for obj in self.graspableObjects:       # preparation for highlightes objects
                        self.staticClones.append(Object3D(obj.type, *obj.args[:-1], True if obj.args[-1] == False else obj.args[-1]).material(color=self.grippercloseColor))     # Copy object but change wireframe option if awailable
                        
                    lastOffset = [0.0, 0.0, 0.0]
                    self.links:list[ui.scene.group] = []
                    group.scale(3.5)
                    for i, file in enumerate(self.robotModel.files):         # Adds all links to the robot
                        self.scene.stack.append(self.scene.group()) 
                        self.scene.stack[-1].rotate(*self.robotModel.globalSimulationRotation[i])             
                        self.scene.stack[-1].move(*(-np.array(self.robotModel.offsets[i]) + np.array(lastOffset)))
                        lastOffset = self.robotModel.offsets[i]
                        self.links.append(self.scene.group())  
                        self.scene.stack.append(self.links[-1])   
                        self.scene.gltf(f'/static/{self.robotModel.id}/{file}').scale(0.001).move(*self.robotModel.offsets[i])
                    with self.scene.group() as self.environmentGroupMoving:
                        self.gripperTip = self.scene.sphere(0, 0, 0)
                        environment.initializeGripper(self, self.scene)         # Initializing gripper if present

                        for envCam in self.environmentCameras:
                            clone = self.scene.box(0.01, 0.01, 0.01, True).visible(False)
                            self.environmentCameraClones.append(clone)

                        for obj in self.graspableObjects:
                            with self.scene.group() as clone:
                                Object3D(obj.type, *obj.args).material(obj.color, obj.opacity, obj.side_).with_name(f'obj {self.graspableObjects.index(obj)}')
                                Object3D(obj.type, *obj.args[:-1], True if obj.args[-1] == False else obj.args[-1]).material(color=self.grippedColor).with_name(f'obj {self.graspableObjects.index(obj)}')     # Copy object but change wireframe option if awailable
                            self.movingClones.append(clone)
                    for _ in range(len(self.robotModel.files)*2):        # Cleaning up
                        self.scene.stack.pop() 

            time.sleep(1)

            if len(self.cameraProperties):
                with ui.row().classes('flex flex-row flex-nowrap w-full bg-slate-700 h-1/4 flex-none m-0') as self.additionalCamerasContainer:
                    pass

            self.isInitialized = True        
                    
    def shutdown(self):
        """Function to shut the capture Process down"""
        for publisher, publishServer in self.publisherPair:
            publisher.shutdown()
            publishServer.stop(2)
        
    def addEnvironmentCamera(self,
                             x: float = None,
                             y: float = None,
                             z: float = None,
                             look_at_x: float = None,
                             look_at_y: float = None,
                             look_at_z: float = None,
                             up_x: float = None,
                             up_y: float = None,
                             up_z: float = None) -> None:
        """A Function to add additional cameras to your scene whose parameter can be adjusted and their image is shown in a configurable bottom porch

        :param x: camera x position
        :param y: camera y position
        :param z: camera z position
        :param look_at_x: camera look-at x position
        :param look_at_y: camera look-at y position
        :param look_at_z: camera look-at z position
        :param up_x: x component of the camera up vector
        :param up_y: y component of the camera up vector
        :param up_z: z component of the camera up vector
        """
        logger.info(f'adding camera {len(self.cameraProperties)}')
        self.cameraProperties.append((x, y, z, look_at_x, look_at_y, look_at_z, up_x, up_y, up_z, 0, False))

    def addGripperCamera(self,
                         x: float = None,
                         y: float = None,
                         z: float = None,
                         look_at_x: float = None,
                         look_at_y: float = None,
                         look_at_z: float = None,
                         up_x: float = None,
                         up_y: float = None,
                         up_z: float = None) -> None:
        """A Function to additional cameras to your scene whose parameter can be adjusted and their image is shown in a configurable bottom porch

        :param x: camera x position
        :param y: camera y position
        :param z: camera z position
        :param look_at_x: camera look-at x position
        :param look_at_y: camera look-at y position
        :param look_at_z: camera look-at z position
        :param up_x: x component of the camera up vector
        :param up_y: y component of the camera up vector
        :param up_z: z component of the camera up vector
        """
        logger.info(f'adding camera {len(self.cameraProperties)}')
        self.cameraProperties.append((x, y, z, look_at_x, look_at_y, look_at_z, up_x, up_y, up_z, 0, True))

    def _attachCamera(self, camera:Scene, group:Object3D):
        self.run_method('attachCamera', camera.id, group.id)
 
    def resize(self):
        for camera in self.cameras:
            camera.run_method('resize')
        self.scene.run_method('resize')

    def _applyProperties(self, object:Object3D, properties:list[dict[str, float], list[float]]):
        position, rotation = properties
        rotationMatrix = rowan.to_matrix(np.array(rotation))
        object.R = rotationMatrix
        object.x = position['x']
        object.y = position['y']
        object.z = position['z']
    
    async def _match(self, staticObject:Object3D, referenceObject:Object3D, dynamicObject:Object3D):
        properties = await self.run_method('match', staticObject.id, referenceObject.id, dynamicObject.id, self.scene.id)
        self._applyProperties(dynamicObject, properties)
    async def _follow(self, staticObject:Object3D, dynamicObject:Object3D):
        properties = await self.run_method('follow', staticObject.id, dynamicObject.id, self.scene.id)
        self._applyProperties(dynamicObject, properties)
    
    async def _worldDistance(self, a:Object3D, b:Object3D):
        return await self.run_method('calcDistance', a.id, b.id, self.scene.id)

    async def update(self) -> None:             
        """Funtion for updating the simulation"""    
        if self.robotServer.client is None or self.robotServer.jointsValue is None or not self.robotModel.has3DModel or not self.robotModel.hasJoints or not self.isInitialized: # check if all components have been initialized
            return
        if len(self.cameraProperties):
            self.cameras = []
            with self.additionalCamerasContainer:
                for *cameraProperties, attachToGripper in self.cameraProperties:
                    with ui.scene(parent_scene=self.scene).classes('flex-1 flex-nowrap h-full overflow-x-hidden') as camera:
                        camera.move_camera(*cameraProperties)
                    if attachToGripper:
                        self._attachCamera(camera, self.environmentGroupMoving)
                    self.cameras.append(camera)

        if abs(self.robotServer.gripper-self.lastGripperPosition) > self.minGripperChange:
            environment.gripperMove(self.robotServer.gripper)
            self.lastGripperPosition = self.robotServer.gripper
        self.jointRotations = [0.0] + list(self.robotServer.jointsValue)
        jointTypes = ['REVOLUTE'] + self.robotModel.jointType
        for i, link in enumerate(self.links):       # Exclude Base
            if jointTypes[i] == 'REVOLUTE':         # Update robot joints
                link.rotate(*list(self.robotModel.jointLookupMatrix[i] * self.jointRotations[i]))
            else:
                link.move(*list(self.robotModel.jointLookupMatrix[i] * self.jointRotations[i] + self.robotModel.globalSimulationRotation[i]))
        
        environment.update(self, self.robotServer, self.robotModel)       # Update Environment

        for obj, movingClone, staticClone in zip(self.graspableObjects, self.movingClones, self.staticClones):
            distance = await self._worldDistance(self.gripperTip, obj)
            print(distance)
            if self.updateClones:
                await self._follow(obj, staticClone)
            if distance < self.gripThreshold or self.gripped == obj.id:   # Check if object if close enough to be gripped
                if self.robotServer.gripper > 0.5 and self.gripped is None:       # Grip objects if gripper has just been turned on
                    self.gripped = obj.id
                    await self._match(obj, self.gripperTip, movingClone)
                    obj.visible(False)
                    movingClone.visible(True)

                elif self.robotServer.gripper < 0.5 and self.gripped == obj.id:
                    self.gripped = None
                    await self._follow(movingClone, obj)
                    await self._follow(obj, staticClone)
                    obj.visible(True)
                    movingClone.visible(False)
                
                staticClone.visible(not self.gripped)
            else:
                staticClone.visible(False)
        if self.capture:
            for cameraID in range(len(self.cameras)):
                await self.postCameraStream(cameraID)
        if len(self.cameraProperties):
            self.cameraProperties = []
            self.resize()

@ui.refreshable
def renderRobot(robotModel:RobotModel, robotServer:RobotServer):
    """renders the controlls of the robot"""
    robotSimulation = Simulation(robotModel, robotServer)
    ui.html('''<style>input[type=number]{-moz-appearance: textfield;color:white;text-align: center;}input::-webkit-outer-spin-button,input::-webkit-inner-spin-button {-webkit-appearance: none;margin: 0;}</style>''')
    with ui.column().classes('m-0 p-0 flex flex-column w-full h-full mx-[-1rem] my-[-2rem] overflow-x-hidden'):
        with ui.row().classes('w-screen h-full', remove='wrap'):            
            with ui.column().classes('bg-slate-400 p-4 h-full'):
                with ui.row().classes(remove='wrap'):
                    ui.button('', icon='settings', on_click=lambda e: ui.open('/')).classes('mt-4').tooltip('Displays the robot selection dialog')      # Returns to the selection page
                    ui.label(robotModel.name).classes('text-white text-4xl mt-4')
                with ui.row().classes(remove='wrap'):
                    with ui.card().classes('filter-none bg-slate-500 text-center items-center my-1'):
                        ui.label('Status').classes('mb-[-0.6em] mt-[-0.4em] text-white')
                        with ui.row().classes(remove='wrap'):
                            power = toggleButton('', icon='power', tooltip='displays if the robot is connected')        # Button to connect the grpc server
                            power.handlePress(True)
                            robotServer.on_power_change = power.handlePress
                            def onpowerchange(state):
                                """connects and disconnects the grpc server"""
                                if power.pressed:
                                    robotServer.connectRobot()
                                else:
                                    robotServer.disconnectRobot()
                            power.onchange = onpowerchange
                            moving = toggleButton('', icon='open_in_full', disable=True, tooltip='displays if the robot is moving')     # Button which displays whether the robot is moving
                            def updateMoving():
                                """updates the moving button to the current state of the robot"""
                                moving.handlePress(robotServer.moving if not robotServer.moving is None else False)
                            robotServer.registerUpdateCallback(updateMoving)        # Register a callback to update the moving button
                    with ui.card().classes('filter-none bg-slate-500 text-center items-center my-1'):
                        ui.label('Control').classes('mb-[-0.6em] mt-[-0.4em] text-white')       # Controlls elements for the robot
                        with ui.row().classes(remove='wrap'):
                            def changePrecision(pressed):
                                """changes the precision mode of the interface"""
                                robotServer.precision = pressed
                            toggleButton('', icon='biotech', on_change=changePrecision, tooltip='Puts the interface in precision mode where the robot only moves 1/10th the usual distance')    # Button to put the Interface in precision mode
                            gripperChangeBtn = toggleButton('', icon='precision_manufacturing', on_change=robotServer.changeGripperState, tooltip='Changes the state of the gripper')           # Button to change the state of the gripper
                            robotServer.on_gripper_change = gripperChangeBtn.handlePress
                            with ui.dialog() as speedChangeDialog, ui.card().style('width:25%'):        # dialog to change the speed on certain robots
                                ui.label('Change speed:').classes('text-2xl')
                                ui.slider(min=0, max=1, step=0.01, value=robotServer.speed, on_change=lambda e: speedLabel.set_text(f'Speed: {robotServer.speed*100:.1f}%')).bind_value(robotServer, 'speed')
                                speedLabel = ui.label(f'Speed: {speed*100:.1f}%').classes('mt-[-1em]')
                                ui.button('Close', on_click=speedChangeDialog.close)
                            ui.button('', icon='speed', on_click=speedChangeDialog.open).tooltip("Opens a dialog to set the robot's speed").classes('px-5 m-[-0.2em] text-white')   # Button to speed change dialog
                            if robotModel.isCompliant:          # If the robot has freedrive show button for it
                                freedriveChangeBtn = toggleButton('', icon='pan_tool', on_change=robotServer.changeFreedrive, tooltip='Puts the robot into freedrive')
                                freedriveChangeBtn.handlePress(robotServer.freeDrive)
                                robotServer.on_freedrive_change = freedriveChangeBtn.handlePress
                if robotModel.hasJoints:
                    with ui.card().classes('filter-none bg-slate-500 text-center items-center w-120 my-1'):  # The joints axis controlls
                        with ui.row().classes(remove='wrap'):
                            ui.label('Joints').classes(f'mb-[-1em] mt-[-0.4em] text-white')
                            async def copyToClipboardJoints():
                                """copies the joints position of the robot to the clipboard"""
                                content = f'[{", ".join([f"{x:.4f}" for x in robotServer.jointsValue])}]'
                                logger.info(f'copied {content} to clipboard')
                                await ui.run_javascript(f'navigator.clipboard.writeText("{content}")')
                            ui.button('', on_click=copyToClipboardJoints, icon='content_copy').props('dense').style('font-size:0.85em;justify-content:right;').classes(f'my-[-1em]').tooltip('Copies the joint values as radians to clipboard')     # Copy to clipboard button
                        with ui.row().classes(remove='wrap'):
                            for axis, pos in zip(robotModel.AxisNames[:robotModel.axisCount], robotServer.jointsValue if not robotServer.jointsValue is None else [0] * robotModel.axisCount):
                                a = Axis(robotModel, robotServer, axis, unit=robotModel.AxisUnits[robotModel.getAxisIndex(axis)],on_move=robotServer.btnMoveJoints, position = pos)
                                robotServer.registerUpdateCallback(a.updatePosition)        # Registering the nessecary callbacks for updating the position values
                with ui.card().classes('filter-none bg-slate-500 text-center items-center my-1'):   # The cartesian axis controlls
                    with ui.row().classes(remove='wrap'):  
                        ui.label('Cartesian').classes('mb-[-1em] mt-[-0.4em] text-white')
                        async def copyToClipboardCartesian():
                            """copies the cartesian position of the robot to the clipboard"""
                            content = f'[{", ".join([f"{x:.4f}" for x in np.concatenate(TransformationMatix.fromPose(robotServer.robotPose).decomposeNumpy())])}]'
                            logger.info(f'copied {content} to clipboard')
                            ui.run_javascript(f'navigator.clipboard.writeText("{content}")')
                        ui.button('', on_click=copyToClipboardCartesian, icon='content_copy').props('dense').style('font-size:0.85em;justify-content:right;').classes(f'my-[-1em]').tooltip('Copies the cartesian values as x,y,z and a Quaternion to clipboard')     # Copy to clipboard button
                    with ui.row().classes(remove='wrap'):
                        for axis, pos in zip(robotModel.AxisNames[robotModel.axisCount if robotModel.hasJoints else 0:], robotServer.Cartesian if not robotServer.Cartesian is None else [0] * robotModel.axisCount):
                            a = Axis(robotModel, robotServer, axis, unit=robotModel.AxisUnits[robotModel.getAxisIndex(axis)], on_move=robotServer.btnMoveCartesian, position=pos)
                            robotServer.registerUpdateCallback(a.updatePosition)        # Registering the nessecary callbacks for updating the position values
            
                with ui.card().classes('filter-none bg-slate-500 text-center items-center my-1'):       # Buttons to show the charts or hide the simulation
                    ui.label('Charts and Simulation').classes('mb-[-0.6em] mt-[-0.4em] text-white')
                    with ui.row().classes(remove='wrap'):
                        if robotModel.hasJoints:
                            jChartBtn = toggleButton('J', tooltip='Displays a chart with all the joints')
                        XChartBtn = toggleButton('X', tooltip='Displays a chart with all the linear axis')
                        RChartBtn = toggleButton('R', tooltip='Displays a chart with all the rotation axis')
                        AllChartsBtn = toggleButton('', icon='done_all', tooltip='Displays a chart with all joints and axies')
                        SimulationBtn = toggleButton('', icon='view_in_ar', tooltip='Hides the 3D simulation', on_change=robotSimulation.changeVisibility)
                        CaptureBtn = toggleButton('', icon='videocam', tooltip='Starts capturing a video stream', on_change=robotSimulation.changeCapture)
                        SimulationBtn.handlePress(state=True, suppress=True)
            with ui.column().classes('h-screen'):
                if robotModel.hasJoints:
                    jChart = Chart(robotModel, robotServer,  '', 'Time / s', 'Rotation / °', robotModel.AxisNames[:robotModel.axisCount])
                XChart = Chart(robotModel, robotServer, '', 'Time / s', 'Position / m', robotModel.AxisNames[robotModel.axisCount if robotModel.hasJoints else 0:-robotModel.rotationAxisCount])
                RChart = Chart(robotModel, robotServer, '', 'Time / s', 'Rotation / °', robotModel.AxisNames[-robotModel.rotationAxisCount:])

            with ui.column().classes('min-h-screen flex-1 flex-nowrap overflow-x-auto'):
                robotSimulation.renderSimulation()

            def allChartsVisible(visible):
                """makes all charts visible"""
                if robotModel.hasJoints:
                    jChartBtn.handlePress(visible)
                XChartBtn.handlePress(visible)
                RChartBtn.handlePress(visible)

            def changeVisibility(chart:Chart, *args):
                robotSimulation.resize()
                chart.changeVisibility(*args)
            XChartBtn.onchange = lambda *args: changeVisibility(XChart, *args)        # Setting up the buttons for the charts
            if robotModel.hasJoints:
                jChartBtn.onchange = lambda *args: changeVisibility(jChart, *args)
            RChartBtn.onchange = lambda *args: changeVisibility(RChart, *args)
            AllChartsBtn.onchange = allChartsVisible
        
@ui.page('/robot')
def returnToOriginalPage():
    return RedirectResponse('/')

@ui.page('/robot/{id}')
def robotPage(id):   
    files = {file['ID']:file['file'] for file in robotSelectionData}     
    print(id, JSON_PATH+files[id])
    """Main Interface page"""
    robotModel = RobotModel(id, JSON_PATH+files[id])      # Setup of the parameters for the robot
    robotServer = RobotServer(robotModel)                                                            # Grpc requester and subscriber setup
    renderRobot(robotModel, robotServer)  

@ui.page('/')   
def select():
    """Page for selecting a robot"""
    def returnToFirstDialog():              # Return to selection dialog after selection of robot
        wrongSelectionDialog.close()
        choseRobotDialog.open()
    def handleChoice():                     # processes the choice and configures the interface
        if robotSelector.value is None:
            wrongSelectionDialog.open()
            return
        logger.info(f'Selected robot {robotSelector.value}')
        ui.open(f'/robot/{quote(robotSelector.value, safe="")}')

    robots = [robot['ID'] for robot in robotSelectionData]
    with ui.dialog(value=True).props("no-backdrop-dismiss no-esc-dismiss") as choseRobotDialog:
        with ui.card().classes('w-1/4'):
            ui.label('Please choose a robot:')
            robotSelector = ui.select(robots).props("persistent")
            ui.button('Done', on_click=handleChoice)

    with ui.dialog(value=False) as wrongSelectionDialog:
        with ui.card():
            ui.label('Please select a robot from the dropdown before continuing.')
            ui.button('Close', on_click=returnToFirstDialog)

ui.run(show=False, title='Robot Interface', reload=False)