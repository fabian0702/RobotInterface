import time
import rowan
import math
import base64
import cv2
import uvicorn
import multiprocessing
from fastapi import Request, FastAPI
from fastapi.middleware.cors import CORSMiddleware
import json
from fastapi.responses import RedirectResponse
from nicegui import ui, app
from nicegui.elements.scene_object3d import Object3D
import numpy as np
from matplotlib import pyplot as plt
plt.switch_backend('agg')               # change backend to optimise charts
from sys import path
import os
path.append('./Python-main/')           # Additional Paths for the BFH Module and the environments script
path.append('./resources/environment/')
from ch.bfh.roboticsLab.robot.RobotClient import RobotClient
from ch.bfh.roboticsLab import Base_pb2 as pbBase
from ch.bfh.roboticsLab.robot import RobotControl_pb2 as pbRobotControl
from ch.bfh.roboticsLab.util.Logger import Logger
from ch.bfh.roboticsLab.util.TransformationMatrix import TransformationMatix
import environment
import CameraServer

logger = Logger('main').getInstance()        # Setup Logger

class direction: 
    """Data class to store direction"""   
    up = 1
    down = -1

clientAdress = '192.168.0.100'  # The address of the grpc server

precision = False               # if in precision mode or not
speed = 0.1                     # global speed

chartsLogTime = 10              # Total timespan on chart
chartUpdateInterval = 0.4       # Interval between data refreshs

JSONPath = './resources/models/'        # Path to where the configuration files for the robots are saved
EnvironmentPath = './resources/environment/'        # Path to where the environments files are saved
apiHost = '127.0.0.1'
apiPort = 8000

class chart:
    """Class to render a chart for a configurable amount of axies"""
    # @param data A Dictionary with graphnames as keys a list of np.ndarray with the 0th element as the x axis and the 1st element as the y axis
    def __init__(self, title:str, xaxisName:str, yaxisName:str, graphNames:list[str], visible = False):
        self.title = title
        self.index = 0
        self.graphNames = graphNames
        self.maxIndex = int(chartsLogTime / chartUpdateInterval)
        self.data:list[list[float]] = [[0 for i in range(self.maxIndex)] for i in range(len(self.graphNames) + 1)]
        self.graphNames = graphNames
        self.xaxisName = xaxisName
        self.yaxisName = yaxisName
        self.chart()
        self.plot.set_visibility(visible)       # Hide chart per default
        self.startTime = time.time()
        self.gains = np.array([gain if robotModel.AxisNames[i] in self.graphNames else 0.0 for i, gain in enumerate(robotModel.AxisGain)])
        self.startIndex = min(map(robotModel.getAxisIndex, self.graphNames))
        self.endIndex = max(map(robotModel.getAxisIndex, self.graphNames))
        
    @ui.refreshable
    def chart(self):
        """Function to render the chart"""
        self.chartUpdateTimer = ui.timer(chartUpdateInterval, self.updateData, active=False)        # setup refresh timer
        self.plot:ui.line_plot = ui.line_plot(n=len(self.graphNames), limit=self.maxIndex-1, figsize=(6, 2)).with_legend(self.graphNames, loc='upper right')        # plot
        #self.plot.fig.gca().set_xticklabels([])
        plt.grid(axis = 'x')        # grid
    
    def updateData(self,):
        """Function to update the data from the server, and display it on the chart"""
        if not robotServer.client is None and self.plot.visible and len(self.graphNames) > 0:       # Sanity check
            self.index = int((robotServer.currentTime - self.startTime) / chartUpdateInterval) % self.maxIndex      # Calculate index in ring buffer
            self.data[0][self.index] = (robotServer.currentTime - self.startTime) % chartsLogTime                   # write time in ringbuffer
            for i, val in enumerate((self.gains * (np.concatenate((robotServer.jointsValue, robotServer.Cartesian)) if robotModel.hasJoints else robotServer.Cartesian))[self.startIndex:self.endIndex+1]):
                self.data[i+1][self.index]=val      # write the values neccesary in the ring buffer
            self.plot.push(self.data[0], self.data[1:])     # update the chart

    def changeVisibility(self, visible:bool):
        """Function to change the visibility of the charts"""
        if visible:
            self.startTime = time.time()        # keep track of the start time
            self.chartUpdateTimer.activate()    # activate the refresh timer
            self.plot.clear()                   # reset the plot
            if robotServer.Cartesian is None or ((not robotModel.hasJoints) and robotServer.jointsValues is None):
                return
            self.data = [[i * chartUpdateInterval for i in range(self.maxIndex)]] + [[(np.concatenate((robotServer.jointsValue, robotServer.Cartesian)) if robotModel.hasJoints else robotServer.Cartesian)[i +  self.startIndex] for n in range(self.maxIndex)] for i in range(len(self.graphNames))]       # prepare the data
        else:
            self.chartUpdateTimer.deactivate()      # deactivate refresh timer
        self.plot.set_visibility(visible)           # hide / show chart

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


class Axis:
    """Class to render the controll for one axis"""
    def __init__(self, axisname:str, unit='°', step=0.001, on_move=lambda a, d: None, position = 0.0):
        self.axisname = axisname
        self.axisIndex = robotModel.getAxisIndex(self.axisname)
        self.position = position*robotModel.AxisGain[self.axisIndex]
        self.places = int(abs(math.log10(robotModel.AxisSteps[self.axisIndex])) + 1)
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
        if self.editing or robotServer.jointsValue is None or robotServer.Cartesian is None:
            return
        newPosition = (robotServer.Cartesian[self.axisIndex%robotModel.axisCount] if self.axisIndex >= robotModel.axisCount else list(robotServer.jointsValue)[self.axisIndex%robotModel.axisCount])*robotModel.AxisGain[self.axisIndex]
        if abs(newPosition - self.lastPosition) > self.minDelta:
            self.lastPosition = self.position = newPosition

    def move(self, direction:int, distance:float, absolute:bool = False):
        """Function to move the axis either a distance in the direction or to the absolute position specified in the position field"""
        self.editing = False
        if not robotServer.moving:
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
            self.input = ui.number(value=self.position, format=f'%.{self.places}f', step=robotModel.AxisSteps[self.axisIndex]/10).on('blur', lambda e: self.move(0, 0, True)).on('focus', self.changeEditing).props('dense borderless color=red-1').classes('bg-slate-700 border-solid border rounded-md my-[-0.4em]').style('width: 5em').bind_value(self, 'position')  # Current position of the Axis
            ui.button('',icon='keyboard_arrow_down', on_click=lambda e: self.move(direction.down, self.slowSpeed)).classes(buttonClasses)       # Btn down slow
            ui.button('',icon='keyboard_double_arrow_down', on_click=lambda e: self.move(direction.down, self.fastSpeed)).classes(buttonClasses)# Btn down fast

class CaptureApi(FastAPI):
    """Class to receive a camera capture and send it over grpc"""
    def __init__(self, cameraCount, cameraHeight) -> None:
        super().__init__()
        self.add_middleware(    # Important to allow the api to have a different port
            CORSMiddleware,
            allow_origins=["*"],
            allow_credentials=True,
            allow_methods=["*"],
            allow_headers=["*"],
        )
        self.cameraCount = cameraCount      # Setup Variables for image processing
        self.cameraHeight = cameraHeight
        # Setup Servers
        self.publisherPair:list[tuple[CameraServer.Publisher, CameraServer.grpc.Server]] = [CameraServer.serve(i) for i in range(cameraCount)]
        self.publishers:list[CameraServer.Publisher] = [publisher for publisher, _ in self.publisherPair]
        logger.info(f'started {cameraCount} publishers')

        @self.post('/capture/')
        async def capturePost(request:Request):
            """Function to parse a post request to the api, decode get the individual images and send the to the grpc server"""
            base64Img = await request.body()    # Get image data
            imgData = base64.decodebytes(base64Img[base64Img.index(b','):])     # decode base64 to openCV image
            nparr = np.frombuffer(imgData, np.uint8)
            img = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
            logger.info('received capture')
            for i in range(min(self.cameraCount, len(self.publishers))):        # Crop individual images and send them to the servers
                height, width, _ = img.shape
                x, y, w, h = map(int, ((1.0/self.cameraCount) * width * i, (1.0-self.cameraHeight)*height, 1.0/self.cameraCount*width, height*self.cameraHeight))
                cameraImage = img[y:y+h, x:x+w]
                self.publishers[i].setImg(cameraImage)      # send image to the server
            return 'Ack'
    def shutdown(self):
        """Function to shut down the grpc servers"""
        logger.info('shutting captureApi and its servers down')
        for publisher, publishServer in self.publisherPair:
            publisher.shutdown()
            publishServer.stop(2)
        

class robotDescription:
    """Class to parse and hold all important parameters of the robot"""
    maxLinearSpeed = 0.1
    maxAngualarSpeed = 0.1
    maxLinearTolerance = 0
    maxAngularTolerance = 0
    isInitalized = False
    jointLookupMatrix = [np.array([])]
    offsets:list[list[float]] = [()]
    files:list[str] = []

    def __init__(self, path:str=None, id:str=None):
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
            if os.path.isfile(f'{JSONPath}{id}/model.json') and self.has3DModel:    # check if file exists and if the robot has a 3d model
                with open(f'{JSONPath}{id}/model.json', 'r') as f:      # open the json file for the simulation
                    modelJson = json.loads(f.read())
                    self.jointLookupMatrix = [np.array(x) for x in modelJson['jointLookupMatrix']]      # Matrix to lookup the vector of the joint
                    self.offsets = modelJson['offsets']                                                 # List to get the offset of the link relative to the origin
                    self.files = modelJson['files']                                                     # List of all 3D files of the robot
                    self.globalSimulationRotation = [np.array(x) for x in modelJson['globalRotation']]  # Calibration offsets for the robot
                app.add_static_files(f'/static/{id}/', f'{JSONPath}{id}/')                    # Configure the path for the 3D models
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
    
freedriveChangeBtn = None
power = None
@ui.refreshable
def renderRobot(robot:robotDescription):
    """renders the controlls of the robot"""
    global freedriveChangeBtn, gripperChangeBtn, power
    ui.html('''<style>input[type=number]{-moz-appearance: textfield;color:white;text-align: center;}input::-webkit-outer-spin-button,input::-webkit-inner-spin-button {-webkit-appearance: none;margin: 0;}</style>''')
    with ui.left_drawer().classes('filter-none bg-slate-400 text-center items-center items-stretch py-0').props(f':width="{max(robotModel.axisCount - 1, 4)}80"'):
        with ui.row():
            ui.button('', icon='settings', on_click=lambda e: ui.open('/')).classes('mt-4').tooltip('Displays the robot selection dialog')      # Returns to the selection page
            ui.label(robot.name).classes('text-white text-4xl mt-4')
        with ui.card().classes('filter-none bg-slate-500 text-center items-center my-3'):
            ui.label('Status').classes('mb-[-0.6em] mt-[-0.4em] text-white')
            with ui.row():
                power = toggleButton('', icon='power', tooltip='displays if the robot is connected')        # Button to connect the grpc server
                power.handlePress(True)
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
        with ui.card().classes('filter-none bg-slate-500 text-center items-center my-3'):
            ui.label('Control').classes('mb-[-0.6em] mt-[-0.4em] text-white')       # Controlls elements for the robot
            with ui.row():
                def changePrecision(pressed):
                    """changes the precision mode of the interface"""
                    global precision
                    precision = pressed
                toggleButton('', icon='biotech', on_change=changePrecision, tooltip='Puts the interface in precision mode where the robot only moves 1/10th the usual distance')    # Button to put the Interface in precision mode
                gripperChangeBtn = toggleButton('', icon='precision_manufacturing', on_change=robotServer.changeGripperState, tooltip='Changes the state of the gripper')           # Button to change the state of the gripper
                with ui.dialog() as speedChangeDialog, ui.card().style('width:25%'):        # dialog to change the speed on certain robots
                    ui.label('Change speed:').classes('text-2xl')
                    ui.slider(min=0, max=1, step=0.01, value=speed, on_change=lambda e: speedLabel.set_text(f'Speed: {speed*100:.1f}%')).bind_value(globals(), 'speed')
                    speedLabel = ui.label(f'Speed: {speed*100:.1f}%').classes('mt-[-1em]')
                    ui.button('Close', on_click=speedChangeDialog.close)
                ui.button('', icon='speed', on_click=speedChangeDialog.open).tooltip("Opens a dialog to set the robot's speed").classes('px-5 m-[-0.2em] text-white')   # Button to speed change dialog

                if robotModel.isCompliant:          # If the robot has freedrive show button for it
                    freedriveChangeBtn = toggleButton('', icon='pan_tool', on_change=robotServer.changeFreedrive, tooltip='Puts the robot into freedrive')
                    freedriveChangeBtn.handlePress(robotModel.isCompliant and robotServer.freeDrive)
        if robotModel.hasJoints:
            with ui.card().classes('filter-none bg-slate-500 text-center items-center my-3'):  # The joints axis controlls
                with ui.row():
                    ui.label('Joints').classes(f'mb-[-1em] mt-[-0.4em] text-white')
                    async def copyToClipboardJoints():
                        """copies the joints position of the robot to the clipboard"""
                        content = f'[{", ".join([f"{x:.4f}" for x in robotServer.jointsValue])}]'
                        logger.info(f'copied {content} to clipboard')
                        await ui.run_javascript(f'navigator.clipboard.writeText("{content}")', respond=False)
                    ui.button('', on_click=copyToClipboardJoints, icon='content_copy').props('dense').style('font-size:0.85em;justify-content:right;').classes(f'my-[-1em]').tooltip('Copies the joint values as radians to clipboard')     # Copy to clipboard button
                with ui.row():
                    for axis, pos in zip(robot.AxisNames[:robotModel.axisCount], robotServer.jointsValue if not robotServer.jointsValue is None else [0] * robotModel.axisCount):
                        a = Axis(axis, unit=robotModel.AxisUnits[robotModel.getAxisIndex(axis)],on_move=robotServer.btnMoveJoints, position = pos)
                        robotServer.registerUpdateCallback(a.updatePosition)        # Registering the nessecary callbacks for updating the position values

        with ui.card().classes('filter-none bg-slate-500 text-center items-center my-3'):   # The cartesian axis controlls
            with ui.row():  
                ui.label('Cartesian').classes('mb-[-1em] mt-[-0.4em] text-white')
                async def copyToClipboardCartesian():
                    """copies the cartesian position of the robot to the clipboard"""
                    content = f'[{", ".join([f"{x:.4f}" for x in np.concatenate(TransformationMatix.fromPose(robotServer.robotPose).decomposeNumpy())])}]'
                    logger.info(f'copied {content} to clipboard')
                    await ui.run_javascript(f'navigator.clipboard.writeText("{content}")', respond=False)
                ui.button('', on_click=copyToClipboardCartesian, icon='content_copy').props('dense').style('font-size:0.85em;justify-content:right;').classes(f'my-[-1em]').tooltip('Copies the cartesian values as x,y,z and a Quaternion to clipboard')     # Copy to clipboard button
            with ui.row():
                for axis, pos in zip(robot.AxisNames[robotModel.axisCount if robotModel.hasJoints else 0:], robotServer.Cartesian if not robotServer.Cartesian is None else [0] * robotModel.axisCount):
                    a = Axis(axis, unit=robotModel.AxisUnits[robotModel.getAxisIndex(axis)], on_move=robotServer.btnMoveCartesian, position=pos)
                    robotServer.registerUpdateCallback(a.updatePosition)        # Registering the nessecary callbacks for updating the position values
    
        with ui.card().classes('filter-none bg-slate-500 text-center items-center my-3'):       # Buttons to show the charts or hide the simulation
            ui.label('Charts').classes('mb-[-0.6em] mt-[-0.4em] text-white')
            with ui.row():
                if robotModel.hasJoints:
                    jChartBtn = toggleButton('J', tooltip='Displays a chart with all the joints')
                XChartBtn = toggleButton('X', tooltip='Displays a chart with all the linear axis')
                RChartBtn = toggleButton('R', tooltip='Displays a chart with all the rotation axis')
                AllChartsBtn = toggleButton('', icon='done_all', tooltip='Displays a chart with all joints and axies')
                SimulationBtn = toggleButton('', icon='view_in_ar', tooltip='Hides the 3D simulation', on_change=robotSim.changeVisibility)
                CaptureBtn = toggleButton('', icon='videocam', tooltip='Starts capturing a video stream', on_change=robotSim.changeCapture)
                SimulationBtn.handlePress(state=True, suppress=True)
                
    with ui.column():       # All the charts
        if robotModel.hasJoints:
            jChart = chart('', 'Time / s', '', robotModel.AxisNames[:robotModel.axisCount])
        XChart = chart('', 'Time / s', '', robotModel.AxisNames[robotModel.axisCount if robotModel.hasJoints else 0:-robotModel.rotationAxisCount])
        RChart = chart('', 'Time / s', '', robotModel.AxisNames[-robotModel.rotationAxisCount:])
        
    def allChartsVisible(visible):
        """makes all charts visible"""
        if robotModel.hasJoints:
            jChartBtn.handlePress(visible)
        XChartBtn.handlePress(visible)
        RChartBtn.handlePress(visible)
    XChartBtn.onchange = XChart.changeVisibility        # Setting up the buttons for the charts
    if robotModel.hasJoints:
        jChartBtn.onchange = jChart.changeVisibility
    RChartBtn.onchange = RChart.changeVisibility
    AllChartsBtn.onchange = allChartsVisible

class Simulation:
    """Class for the simulation of the robot"""
    def __init__(self) -> None:
        app.add_static_files('/static/environment/', './resources/environment/')
        self.graspableObjects:list[Object3D] = []
        self.staticObjects:list[Object3D]
        self.environmentData:dict = {}
        self.grippedObjects:list[Object3D] = []
        self.grippedObjectProperties:list[Object3D] = []
        self.color:list[list[str]] = [[]]
        self.highlightedObjects:list[Object3D] = []
        self.staticObjects:list[Object3D] = []
        self.grippercloseColor:str = '#0088ff'
        self.grippedColor:str = '#ff0000'
        self.gripThreshold:float = 0.004
        self.capture:bool = False
        self.cameraIndex:int = 0
        self.cameraHight:float = 0.2
        self.cameras:list[Object3D] = []
        self.addCameraHelper = True
        self.renderSimulation()
        self.updateSimulationTimer:ui.timer = ui.timer(0.1, callback=self.update, active=True)  

    def changeVisibility(self, visible:bool):
        """changes the visibility of the simulation"""
        self.simulationDrawer.set_visibility(visible)

    def changeCapture(self, state):
        """Starts or stops capturing a video stream"""
        logger.info(f'Changing state of video capture to {state}')
        self.capture = state
        
    @ui.refreshable
    def renderSimulation(self):  
        global captureApi     
        """Funtion for setting up the simulation"""
        if robotModel is None or not robotModel.isInitalized or robotServer.wrongRobot or not robotModel.has3DModel or not robotModel.hasJoints:
            return
        with ui.right_drawer().props('width=auto') as self.simulationDrawer:
            with ui.scene(width=700, height=950).classes('m-[-1em]') as self.scene:
                with self.scene.group() as group:
                    group.rotate(0,0,0.685)
                    environment.initialize(self, self.scene)    # Initializing of the scene

                    for obj in self.graspableObjects:       # preparation for highlightes objects
                        self.highlightedObjects.append(Object3D(obj.type, *obj.args[:-1], True).material(color='#0088ff', opacity=1.0))

                    lastOffset = [0.0, 0.0, 0.0]
                    self.links:list[ui.scene.group] = []
                    group.scale(-3.5)
                    for i, file in enumerate(robotModel.files):         # Adds all links to the robot
                        self.scene.stack.append(self.scene.group())              
                        self.scene.stack[-1].move(*(-np.array(robotModel.offsets[i]) + np.array(lastOffset)))
                        lastOffset = robotModel.offsets[i]
                        self.links.append(self.scene.group())  
                        self.scene.stack.append(self.links[-1])   
                        self.scene.gltf(f'/static/{robotModel.id}/'+file, scale=0.001, offset=robotModel.offsets[i])
                    
                    with self.scene.group() as self.environmentGroupMoving:
                        environment.initializeGripper(self, self.scene)         # Initializing gripper if present
                    for _ in range(len(robotModel.files)*2):        # Cleaning up
                        self.scene.stack.pop() 
                if self.addCameraHelper:        # Visualizing of cameras
                    for camera in self.cameras: 
                        self.scene.cameraHelper(camera)
        
        CameraCount = self.cameraIndex      # Setup parameters for capture
        CameraHeight = self.cameraHight
        self.captureProcess = multiprocessing.Process(target=startApi, args=[CameraCount, CameraHeight])     # Create new Process for api
        self.captureProcess.start()      # start new process

    def shutdown(self):
        """Function to shut the capture Process down"""
        self.captureProcess.kill()
        
    def addEnvironmentCamera(self, position:list[float], look_at:list[float], fov:float = 75, focus:float = 10, far:float=1000, near:float=0.1) -> None:
        """A Function to add up to three additional cameras to your scene whose parameter can be adjusted and their image is shown in a configurable bottom porch"""
        self.cameraIndex += 1
        if self.cameraIndex > 3:        # Check if more than three cameras has been added
            return
        with self.scene.group() as g:
            g.scale(1, 1, -1)
            self.cameras.append(self.scene.subCamera(left=1.0/self.cameraIndex if self.cameraIndex > 1 else 0.0, bottom=0.0, width=1.0/self.cameraIndex, height=self.cameraHight, lookat=look_at, position=position, fov=fov, focus=focus, far=far, near=near)) # Initialize camera with given parameters

    def addGripperCamera(self, fov:float = 75, focus:float = 10, far:float=1000, near:float=0.1) -> None:
        """A Function to add up to three additional cameras to your scene whose parameter can be adjusted and their image is shown in a configurable bottom porch"""
        self.cameraIndex += 1
        if self.cameraIndex > 3:        # Check if more than three cameras has been added
            return
        with self.scene.group() as g:
            g.scale(-1, 1, 1)
            self.cameras.append(self.scene.subCamera(left=1.0/self.cameraIndex if self.cameraIndex > 1 else 0.0,
                                                     bottom=0.0,
                                                     width=1.0/self.cameraIndex, 
                                                     height=self.cameraHight, 
                                                     lookat=[0, -1, 0], 
                                                     position=[0, 0, 0], 
                                                     fov=fov, 
                                                     focus=focus, 
                                                     far=far, 
                                                     near=near, 
                                                     up=[0, -1, 0])) # Initialize camera with given parameters
    def update(self) -> None:             
        """Funtion for updating the simulation""" 
        if robotServer.client is None or robotServer.jointsValue is None or not robotModel.has3DModel or not robotModel.hasJoints: # check if all components have been initialized
            return
        self.jointRotations = [0.0] + list(robotServer.jointsValue)
        jointTypes = ['REVOLUTE'] + robotModel.jointType
        for i, link in enumerate(self.links):       # Exclude Base
            if jointTypes[i] == 'REVOLUTE':         # Update robot joints
                link.rotate(*list(robotModel.jointLookupMatrix[i] * self.jointRotations[i] + robotModel.globalSimulationRotation[i]))
            else:
                link.move(*list(robotModel.jointLookupMatrix[i] * self.jointRotations[i] + robotModel.globalSimulationRotation[i]))
        environment.update(self, robotServer, robotModel)       # Update Environment
        if robotServer.gripper > 0.5:                
            for property, obj in zip(self.grippedObjectProperties, self.grippedObjects):        # Handle gripped objects
                obj.move(robotServer.Cartesian[0]+property[0][0]-property[2][0], robotServer.Cartesian[1]+property[0][1]-property[2][1], -robotServer.Cartesian[2]+property[0][2]+property[2][2])
                matRot = Object3D.rotation_matrix_from_euler((robotServer.Cartesian[3] - property[2][3]), (robotServer.Cartesian[4] - property[2][4]), (robotServer.Cartesian[5] - property[2][5]))
                matNew = np.matmul(np.array(matRot), np.array(property[1]))
                obj.R = list(map(list, matNew))
                obj._rotate()
        for i, obj, clone in zip(range(len(self.graspableObjects)), self.graspableObjects, self.highlightedObjects):
            if sum([(obj.x-robotServer.Cartesian[0])**2, (obj.y-robotServer.Cartesian[1])**2, (obj.z+robotServer.Cartesian[2])**2]) < self.gripThreshold:   # Check if object if close enough to be gripped
                if robotServer.gripper > 0.5:
                    if len(self.grippedObjects) == 0:       # Grip objects if gripper has just been turned on
                        self.grippedObjects.append(obj)
                        self.grippedObjectProperties.append([[obj.x, obj.y, obj.z], obj.R, robotServer.Cartesian])
                clone.move(obj.x, obj.y, obj.z)             # Setup highlight
                clone.scale(obj.sx, obj.sy, obj.sz)
                clone.rotate_R(obj.R)
                clone.material(self.grippedColor if robotServer.gripper > 0.5 else self.grippercloseColor)
                clone.visible(True)
            else:
                self.grippedObjects = []                # reset highlight
                self.grippedObjectProperties = []
                clone.visible(False)
        if self.capture:
            self.scene.img(f'http://{apiHost}:{apiPort}/capture/')     # Request a Frame

class Robot:                        
    """Class to handle communication with the server"""
    def __init__(self) -> None:
        self.jointsValue = None
        self.robotPose = None
        self.moving = None
        self.freeDrive = False
        self.gripper = 0.0
        self.initializedAxis = False
        self.currentTime = 0.0
        self.Cartesian = None
        self.wrongRobot = False
        self.client = None
        self.connectRobot()
        self.max_speed = pbBase.LinearAngularPair(linear = robotModel.maxLinearSpeed, angular=robotModel.maxAngualarSpeed)
        self.max_tolerance = pbBase.LinearAngularPair(linear = robotModel.maxLinearTolerance, angular=robotModel.maxAngularTolerance)
        self.callbacks = []
        with ui.dialog() as self.invalideMoveDialog, ui.card().style('width:25%'):          # Dialog to display errormessage when freedrive is enabled but the robot is commanded to move
            ui.label("The robot can't be moved by the buttons while in freedrive.")
            ui.button('OK', on_click=self.invalideMoveDialog.close)
        with ui.dialog() as self.robotNotConnectedDialog, ui.card().style('width:25%'):     # Dialog to display errormessage when robot isn't is connected but the robot is commanded
            ui.label("The robot can't be commanded while it's disconnected.")
            ui.button('OK', on_click=self.robotNotConnectedDialog.close)
    def connectRobot(self):         
        """Connects the robot to the grpc server and changes the status button"""
        if power is not None:
            power.handlePress(True, suppress=True)
        if self.client is None:
            self.client = RobotClient(clientAdress, self.myProcessSubscription)      # connect robot
    def disconnectRobot(self):      
        """Disconnects the robot and change the status"""
        power.handlePress(False, suppress=True)       # Update Btn
        if not self.client is None:
            self.client.shutdown()      # disconnect robot
            self.client = None
    def btnMoveJoints(self, axisName:str, distance:float, absolute:bool = False):   
        """Function to move one joint by the distance, distance can either be relative or absolute, joint is selected by name"""
        if robotServer.freeDrive:
            self.invalideMoveDialog.open()
            return
        if robotServer.client is None:
            self.robotNotConnectedDialog.open()
            return
        axisIndex = robotModel.getAxisIndex(axisName)
        pose = [0] * robotModel.axisCount
        pose[axisIndex%robotModel.axisCount] = distance * (0.1 if precision else 1) * (robotModel.AxisSteps[axisIndex] if not absolute else 1) / robotModel.AxisGain[axisIndex]
        return self.moveJoints(pose)
    def btnMoveCartesian(self, axisName:str, distance:float, absolute:bool = False):
        """Function to move one axis by the distance, distance can either be relative or absolute, axis is selected by name"""
        if robotServer.freeDrive:           # Show error when in freedrive and attempt to move
            self.invalideMoveDialog.open()
            return
        if robotServer.client is None:           # Show error when robot not connected and attempt to move
            self.robotNotConnectedDialog.open()
            return
        axisIndex = robotModel.getAxisIndex(axisName)
        pose = [0] * robotModel.axisCount
        pose[axisIndex%robotModel.axisCount] = distance * -(0.1 if precision else 1) * (robotModel.AxisSteps[axisIndex] if not absolute else 1) / robotModel.AxisGain[axisIndex]
        return self.moveCartesian(pose)
    
    def changeGripperState(self, state:bool):
        """Function to change the gripper state indicator button and changes the status button"""
        if robotServer.client is None:
            self.robotNotConnectedDialog.open()
            gripperChangeBtn.handlePress(not gripperChangeBtn.pressed, suppress=True)       # Update Btn
            return
        self.client.gripper(1.0 if state else 0.0)      # Send command

    def changeFreedrive(self, state:bool):
        """Function to change the freedrive state indicator button and changes the status button"""
        if robotServer.client is None:
            freedriveChangeBtn.handlePress(not freedriveChangeBtn.pressed, suppress=True)   # Update Btn
            self.robotNotConnectedDialog.open()
            return
        self.client.requester.freedrive(pbRobotControl.Freedrive(state=state))      # Send command

    def moveJoints(self, pose:list[int]):             
        """Function to moves the Joints to a absolute position specified in the pose list as radians"""
        if self.jointsValue is not None:
            actualJoints = np.array(robotServer.jointsValue)
            logger.info(actualJoints)
            newJoints = actualJoints + np.array(pose)
            robotServer.client.moveJoints(jointsOrPose=pbBase.ArrayDouble(value=list(newJoints)),override=speed)
    def moveCartesian(self, pose:list[int]):
        """Moves the Axis to a absolute position specified in the pose list as radians"""
        if self.robotPose is not None:
            actualPose = TransformationMatix.fromPose(self.robotPose)       # Initial Pose of the robot
            offset = TransformationMatix.compose(pose[:3],rowan.from_euler(pose[robotModel.getAxisIndex('RZ')-robotModel.axisCount] if 'RZ' in robotModel.AxisNames else 0.0,   # new Transformation to apply
                                                                           pose[robotModel.getAxisIndex('RY')-robotModel.axisCount] if 'RY' in robotModel.AxisNames else 0.0, 
                                                                           pose[robotModel.getAxisIndex('RX')-robotModel.axisCount] if 'RX' in robotModel.AxisNames else 0.0,'zyx'))
            newPose = actualPose*offset            
            robotServer.client.moveCartesian(pose=newPose.pose(),override=speed)        # Move the robot to the new pose
    def registerUpdateCallback(self, callback:lambda:None):     
        """Function to register a callback for the process subscription"""
        self.callbacks.append(callback)
    def myProcessSubscription(self, message):                   
        """Function which gets call when a new message from the grpc server has been received"""
        global freedriveChangeBtn
        try:
            self.published = message
            self.jointsValue = np.array(message.jointValues.value) if message.HasField("jointValues") else self.jointsValue     # Extract Values from message
            self.robotPose = message.robotPose if message.HasField("robotPose") else self.robotPose
            self.moving = message.state.moving if message.HasField("state") else self.moving
            freeDriveAct = message.freedrive.state if message.HasField("freedrive") else False
            gripperAct = message.gripper.position if message.HasField("gripper") else 0.0
            if self.robotPose is None or self.jointsValue is None or self.moving is None:       # Return if the server hasn't send any values yet
                return
            if not len(self.jointsValue) == robotModel.axisCount:                       # Calls the dialog when a robot has been selected which doesnt match the information from the grpc server
                if self.wrongRobot:
                    return
                self.wrongRobot = True
                wrongRobotSelected()
            if message.HasField("freedrive") and (not self.freeDrive == freeDriveAct) and (not freedriveChangeBtn is None):         # Updates the freedrive button if the state has changed
                self.freeDrive = freeDriveAct
                freedriveChangeBtn.handlePress(state=freeDriveAct, suppress=True)
            if message.HasField("gripper") and (not (abs(self.gripper - gripperAct) < 0.01)) and (not gripperChangeBtn is None):    # Updates the gripper button if the state has changed
                self.gripper = gripperAct
                gripperChangeBtn.handlePress(state=gripperAct>0.5, suppress=True)
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

class robotManager:                     
    """Class for managing all robots available in the interface"""
    def __init__(self):
        with open(JSONPath+'robotSelect.json') as f:        # Parses all robots from the robotSelect.json
            jsonData = json.loads(f.read())
        self.ids = [robot['ID'] for robot in jsonData]
        self.files = [robot['file'] for robot in jsonData]
        with ui.dialog() as self.choseRobotDialog, ui.card().style('width:25%'):
            ui.label('Please choose a robot:')
            robotSelector = ui.select(self.ids)
            ui.button('Done', on_click=self.handleChoice)
            self.choseRobotDialog.props("no-backdrop-dismiss no-esc-dismiss")
        self.choseRobotDialog.open()
        self.selectedRobot = 0
        robotSelector.bind_value(self, 'selectedRobot').props("persistent")
        with ui.dialog() as self.wrongSelectionDialog, ui.card():
            ui.label('Please select a robot from the dropdown before continuing.')
            ui.button('Close', on_click=self.returnToFirstDialog)
        self.wrongSelectionDialog.close()
    def returnToFirstDialog(self):              # Return to selection dialog after selection of robot
        self.wrongSelectionDialog.close()
        self.choseRobotDialog.open()
    def handleChoice(self, e):                  # processes the choice and configures the interface
        global robotPath
        if self.selectedRobot == 0:
            self.wrongSelectionDialog.open()            
        else:
            robotPath = (self.files[self.ids.index(self.selectedRobot)], self.selectedRobot)
            ui.open('/robot')

def wrongRobotSelected():                       
    """Is called when a robot with different amount of axis is selected then the grpc server says"""
    with ui.dialog() as wrongSelectionDialog, ui.card().style('width:25%'):
        ui.label('Please select the same robot as you are connected to.')
        ui.button('Done', on_click=lambda: ui.open('/'))
    wrongSelectionDialog.open()

@ui.page('/robot')
def robotPage():            
    """Main Interface page"""
    global robotModel, robotServer, robotSim
    robotPath = ('robotModel-UR3.json', 'UR3')
    if robotPath[0] == '' or robotPath[1] == '':
        return RedirectResponse('/')
    robotModel = robotDescription(path=JSONPath+robotPath[0], id=robotPath[1])      # Setup of the parameters for the robot
    robotServer = Robot()                                                           # Grpc requester and subscriber setup
    robotSim = Simulation()                                                         # Setup of the simulation                                   
    renderRobot(robotModel)                                                         # Renders the Interface
    app.on_shutdown(shutdown)

robotPath = ('', '')

@ui.page('/')               
def select():
    """Page for selecting a robot"""
    mgr = robotManager()
    mgr.choseRobotDialog.open()

def startApi(cameraCount, cameraHeight):
    captureApi = CaptureApi(cameraCount, cameraHeight)
    logger.info('Done setting up api, now starting it')
    uvicorn.run(app=captureApi, port=apiPort, host=apiHost, log_level="info")

def shutdown():
    robotSim.shutdown()
    robotServer.client.shutdown()
    pass


ui.run(show=False, title='Robot Interface')

# TODO:
# - add labels to charts
# - add labels to charts
