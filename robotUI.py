import time
from typing import Any
from nicegui import ui, app
from nicegui.elements.scene_object3d import Object3D
import numpy as np
from matplotlib import pyplot as plt
plt.switch_backend('agg')
from sys import path
import os
path.append('./Python-main/')
path.append('./resources/environment/')
from ch.bfh.roboticsLab.robot.RobotClient import RobotClient
from ch.bfh.roboticsLab import Base_pb2 as pbBase
from ch.bfh.roboticsLab.robot import RobotControl_pb2 as pbRobotControl
from ch.bfh.roboticsLab.util.Logger import Logger
from ch.bfh.roboticsLab.util.TransformationMatrix import TransformationMatix
import rowan
import math
import json
from fastapi.responses import RedirectResponse
import environment

logger = Logger('robotUI').getInstance()

class direction:
    up = 1
    down = -1

precision = False
speed = 0.1

chartsLogTime = 30
chartUpdateInterval = 0.2

JSONPath = './resources/models/'
EnvironmentPath = './resources/environment/'

def rgb_to_hsv(r, g, b):
    maxc = max(r, g, b)
    minc = min(r, g, b)
    rangec = (maxc-minc)
    v = maxc
    if minc == maxc:
        return 0.0, 0.0, v
    s = rangec / maxc
    rc = (maxc-r) / rangec
    gc = (maxc-g) / rangec
    bc = (maxc-b) / rangec
    if r == maxc:
        h = bc-gc
    elif g == maxc:
        h = 2.0+rc-bc
    else:
        h = 4.0+gc-rc
    h = (h/6.0) % 1.0
    return h, s, v

def hsv_to_rgb(h, s, v):
    if s == 0.0:
        return v, v, v
    i = int(h*6.0) # XXX assume int() truncates!
    f = (h*6.0) - i
    p = v*(1.0 - s)
    q = v*(1.0 - s*f)
    t = v*(1.0 - s*(1.0-f))
    i = i%6
    if i == 0:
        return v, t, p
    if i == 1:
        return q, v, p
    if i == 2:
        return p, v, t
    if i == 3:
        return p, q, v
    if i == 4:
        return t, p, v
    if i == 5:
        return v, p, q

class axisData:
    def __init__(self, name:str, dataX:np.ndarray, dataY:np.ndarray) -> None:
        self.name = name
        self.dataX = dataX
        self.dataY = dataY

class chart:
    # @param data A Dictionary with graphnames as keys a list of np.ndarray with the 0th element as the x axis and the 1st element as the y axis
    def __init__(self, title:str, xaxisName:str, yaxisName:str, graphNames:list[str], visible = False):
        self.title = title
        self.data = []
        self.graphNames = graphNames
        self.xaxisName = xaxisName
        self.yaxisName = yaxisName
        self.chart()
        self.plot.set_visibility(visible)
        self.startTime = 0
        self.gains = np.array([gain if robotModel.AxisNames[i] in self.graphNames else 0.0 for i, gain in enumerate(robotModel.AxisGain)])
        self.startIndex = min(map(robotModel.getAxisIndex, self.graphNames))
        self.endIndex = max(map(robotModel.getAxisIndex, self.graphNames))
    @ui.refreshable
    def chart(self):
        self.chartUpdateTimer = ui.timer(chartUpdateInterval, self.updateData, active=False)
        self.plot:ui.line_plot = ui.line_plot(n=len(self.graphNames), figsize=(6, 2)).with_legend(self.graphNames, loc='upper right')
        self.plot.fig.gca().set_xticklabels([])
    
    def updateData(self,):
        if self.plot.visible and len(self.graphNames) > 0:
            data = [[val] for val in self.gains * np.concatenate((robotServer.jointsValue, robotServer.Cartesian))][self.startIndex:]      # Optimise Line
            self.plot.push([robotServer.currentTime], data)

    def changeVisibility(self, visible:bool):
        if visible:
            self.startTime = time.time()
            self.chartUpdateTimer.activate()
            self.plot.clear()
        else:
            self.chartUpdateTimer.deactivate()
        self.plot.set_visibility(visible)

class toggleButton:
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
        with ui.button(text=self.text, icon=self.icon, on_click=self.handlePress, color=self.color if not self.pressed else self.pressedColor) as btn:
            btn.classes('px-5 m-[-0.2em] text-white')
            # if self.disable:
            #     btn.disable()
            if not self.tooltip is None:
                #print(self.tooltip)
                btn.tooltip(self.tooltip)
    def handlePress(self, state=None, suppress = False):
        if state is None:
            self.pressed = not self.pressed
        elif state == self.pressed or self.disable:
            return
        else:
            self.pressed = state
        if not suppress:
            self.onchange(self.pressed)
        self.toggle.refresh()


class Axis:
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
        if self.editing or robotServer.jointsValue is None or robotServer.Cartesian is None:
            return
        newPosition = (robotServer.Cartesian[self.axisIndex%robotModel.axisCount] if self.axisIndex >= robotModel.axisCount else list(robotServer.jointsValue)[self.axisIndex%robotModel.axisCount])*robotModel.AxisGain[self.axisIndex]
        if abs(newPosition - self.lastPosition) > self.minDelta:
            self.lastPosition = self.position = newPosition

    def move(self, direction:int, distance:float, absolute:bool = False):
        self.editing = False
        if not robotServer.moving:
            logger.info(f'Moving to {distance * direction}')
            self.on_move(self.axisname, distance * direction if not absolute else self.position-self.lastPosition, absolute)

    def changeEditing(self):
        self.editing = True

    def render(self):
        buttonClasses = 'm-[-0.2em] mb-[-0.3em] mt-[-0.3em]'
        with ui.column().classes('text-center items-center items-stretch mb-[0em]'):
            ui.colors(primary='rgb(15 23 42)')
            ui.label(f'{self.axisname} [{self.unit}]').classes('mb-[-0.5em] text-white')
            ui.button('',icon='keyboard_double_arrow_up', on_click=lambda e: self.move(direction.up, self.fastSpeed)).classes(buttonClasses)
            ui.button('',icon='keyboard_arrow_up', on_click=lambda e: self.move(direction.up, self.slowSpeed)).classes(buttonClasses)
            self.input = ui.number(value=self.position, format=f'%.{self.places}f', step=robotModel.AxisSteps[self.axisIndex]/10).on('blur', lambda e: self.move(0, 0, True)).on('focus', self.changeEditing).props('dense borderless color=red-1').classes('bg-slate-700 border-solid border rounded-md my-[-0.4em]').style('width: 5em').bind_value(self, 'position')
            ui.button('',icon='keyboard_arrow_down', on_click=lambda e: self.move(direction.down, self.slowSpeed)).classes(buttonClasses)
            ui.button('',icon='keyboard_double_arrow_down', on_click=lambda e: self.move(direction.down, self.fastSpeed)).classes(buttonClasses)

class robotDescription:
    maxLinearSpeed = 0.1
    maxAngualarSpeed = 0.1
    maxLinearTolerance = 0
    maxAngularTolerance = 0
    isInitalized = False
    jointLookupMatrix = [np.array([])]
    offsets:list[list[float]] = [()]
    files:list[str] = []

    def __init__(self, path:str=None, id:str=None):
        if path is None or path == '' or not path.endswith('.json') or id is None:
            self.name = ''
            self.axisCount = 0
            self.AxisNames = []
            self.isCompliant = False
            self.has3DModel = False
            self.rotationAxisCount = 0
            ui.open('/')
            return
        with open(path, 'r', encoding='utf-8') as f:
            self.isInitalized = True
            jsonData = json.loads(f.read())
            self.name = jsonData['name']
            self.has3DModel = jsonData['has3DModel']
            if os.path.isfile(f'{JSONPath}{id}/model.json') and self.has3DModel:
                with open(f'{JSONPath}{id}/model.json', 'r') as f:
                    modelJson = json.loads(f.read())
                    self.jointLookupMatrix = [np.array(x) for x in modelJson['jointLookupMatrix']]
                    self.offsets = modelJson['offsets']
                    self.files = modelJson['files']
                    self.globalSimulationRotation = [np.array(x) for x in modelJson['globalRotation']]
                app.add_static_files(f'/static/{id}/', f'{JSONPath}{id}/')
            joints = jsonData['joints']
            cartesian = jsonData['cartesian']
            self.axisCount = len(joints)
            self.AxisNames = [str(i) for i in range(self.axisCount)] + [axis['name'] for axis in cartesian]
            self.jointType = [joint['type'] for joint in joints]
            self.AxisUnits = [joint['properties']['unit'] for joint in joints+cartesian]
            self.AxisGain  = [joint['properties']['gain'] for joint in joints+cartesian]
            self.AxisSteps = [joint['properties']['step'] for joint in joints+cartesian]
            self.rotationAxisCount = len([name for name in self.AxisNames[self.axisCount:] if 'R' in name])
            self.isCompliant = jsonData['freedrive'] if 'freedrive' in jsonData.keys() else False
            self.id = id
    def getAxisIndex(self, name):
        return self.AxisNames.index(name)
    
freedriveChangeBtn = None
@ui.refreshable
def renderRobot(robot:robotDescription):
    global freedriveChangeBtn, gripperChangeBtn
    ui.html('''<style>input[type=number]{-moz-appearance: textfield;color:white;text-align: center;}input::-webkit-outer-spin-button,input::-webkit-inner-spin-button {-webkit-appearance: none;margin: 0;}</style>''')
    with ui.left_drawer().classes('filter-none bg-slate-400 text-center items-center items-stretch py-0').props(f':width="{max(robotModel.axisCount - 1, 4)}80"'):
        with ui.row():
            ui.button('', icon='settings', on_click=lambda e: ui.open('/')).classes('mt-4').tooltip('Displays the robot selection dialog')
            ui.label(robot.name).classes('text-white text-4xl mt-4')
        with ui.card().classes('filter-none bg-slate-500 text-center items-center my-3'):
            ui.label('Status').classes('mb-[-0.6em] mt-[-0.4em] text-white')
            with ui.row():
                power = toggleButton('', icon='power', disable=True, tooltip='displays if the robot is connected')
                power.handlePress(True)
                moving = toggleButton('', icon='open_in_full', disable=True, tooltip='displays if the robot is moving')
                def updateMoving():
                    moving.handlePress(robotServer.moving if not robotServer.moving is None else False)
                robotServer.registerUpdateCallback(updateMoving)
        with ui.card().classes('filter-none bg-slate-500 text-center items-center my-3'):
            ui.label('Control').classes('mb-[-0.6em] mt-[-0.4em] text-white')
            with ui.row():
                def changePrecision(pressed):
                    global precision
                    precision = pressed
                toggleButton('', icon='biotech', on_change=changePrecision, tooltip='Puts the interface in precision mode where the robot only moves 1/10th the usual distance')
                gripperChangeBtn = toggleButton('', icon='precision_manufacturing', on_change=robotServer.changeGripperState, tooltip='Changes the state of the gripper')
                with ui.dialog() as speedChangeDialog, ui.card().style('width:25%'):
                    ui.label('Change speed:').classes('text-2xl')
                    ui.slider(min=0, max=1, step=0.01, value=speed, on_change=lambda e: speedLabel.set_text(f'Speed: {speed*100:.1f}%')).bind_value(globals(), 'speed')
                    speedLabel = ui.label(f'Speed: {speed*100:.1f}%').classes('mt-[-1em]')
                    ui.button('Close', on_click=speedChangeDialog.close)
                ui.button('', icon='speed', on_click=speedChangeDialog.open).tooltip("Opens a dialog to set the robot's speed").classes('px-5 m-[-0.2em] text-white')

                if robotModel.isCompliant:
                    freedriveChangeBtn = toggleButton('', icon='pan_tool', on_change=robotServer.changeFreedrive, tooltip='Puts the robot into freedrive')
                    freedriveChangeBtn.handlePress(robotModel.isCompliant and robotServer.freeDrive)
        with ui.card().classes('filter-none bg-slate-500 text-center items-center my-3'):
            with ui.row():
                ui.label('Joints').classes(f'mb-[-1em] mt-[-0.4em] text-white')
                async def copyToClipboardJoints():
                    content = f'[{", ".join([f"{x:.4f}" for x in robotServer.jointsValue])}]'
                    logger.info(f'copied {content} to clipboard')
                    await ui.run_javascript(f'navigator.clipboard.writeText("{content}")', respond=False)
                ui.button('', on_click=copyToClipboardJoints, icon='content_copy').props('dense').style('font-size:0.85em;justify-content:right;').classes(f'my-[-1em]').tooltip('Copies the joint values as radians to clipboard')
            with ui.row():
                for axis, pos in zip(robot.AxisNames[:robotModel.axisCount], robotServer.jointsValue if not robotServer.jointsValue is None else [0] * robotModel.axisCount):
                    a = Axis(axis, unit=robotModel.AxisUnits[robotModel.getAxisIndex(axis)],on_move=robotServer.btnMoveJoints, position = pos)
                    robotServer.registerUpdateCallback(a.updatePosition)
        with ui.card().classes('filter-none bg-slate-500 text-center items-center my-3'):
            with ui.row():
                ui.label('Cartesian').classes('mb-[-1em] mt-[-0.4em] text-white')
                async def copyToClipboardCartesian():
                    content = f'[{", ".join([f"{x:.4f}" for x in np.concatenate(TransformationMatix.fromPose(robotServer.robotPose).decomposeNumpy())])}]'
                    logger.info(f'copied {content} to clipboard')
                    await ui.run_javascript(f'navigator.clipboard.writeText("{content}")', respond=False)
                ui.button('', on_click=copyToClipboardCartesian, icon='content_copy').props('dense').style('font-size:0.85em;justify-content:right;').classes(f'my-[-1em]').tooltip('Copies the cartesian values as x,y,z and a Quaternion to clipboard')
            with ui.row():
                for axis, pos in zip(robot.AxisNames[robotModel.axisCount:], robotServer.Cartesian if not robotServer.Cartesian is None else [0] * robotModel.axisCount):
                    a = Axis(axis, unit=robotModel.AxisUnits[robotModel.getAxisIndex(axis)], on_move=robotServer.btnMoveCartesian, position=pos)
                    robotServer.registerUpdateCallback(a.updatePosition)
        with ui.card().classes('filter-none bg-slate-500 text-center items-center my-3'):
            ui.label('Charts').classes('mb-[-0.6em] mt-[-0.4em] text-white')
            with ui.row():
                jChartBtn = toggleButton('J', tooltip='Displays a chart with all the joints')
                XChartBtn = toggleButton('X', tooltip='Displays a chart with all the linear axis')
                RChartBtn = toggleButton('R', tooltip='Displays a chart with all the rotation axis')
                AllChartsBtn = toggleButton('', icon='done_all', tooltip='Displays a chart with all joints and axies')
                SimulationBtn = toggleButton('', icon='view_in_ar', tooltip='Hides the 3D simulation', on_change=robotSim.changeVisibility)
                SimulationBtn.handlePress(state=True, suppress=True)
                
    with ui.column():
        jChart = chart('', 'Time / s', '', robotModel.AxisNames[:robotModel.axisCount])
        XChart = chart('', 'Time / s', '', robotModel.AxisNames[robotModel.axisCount:robotModel.axisCount+robotModel.rotationAxisCount])
        RChart = chart('', 'Time / s', '', robotModel.AxisNames[robotModel.axisCount+robotModel.rotationAxisCount:])
        
    def allChartsVisible(visible):
        jChartBtn.handlePress(visible)
        XChartBtn.handlePress(visible)
        RChartBtn.handlePress(visible)
    XChartBtn.onchange = XChart.changeVisibility
    jChartBtn.onchange = jChart.changeVisibility
    RChartBtn.onchange = RChart.changeVisibility
    AllChartsBtn.onchange = allChartsVisible

class Simulation:
    def __init__(self) -> None:
        app.add_static_files('/static/environment/', './resources/environment/')
        self.environment:list[Object3D] = []
        self.environmentData:dict = {}
        self.grippedObjects:list[Object3D] = []
        self.grippedObjectProperties:list[Object3D] = []
        self.color:list[list[str]] = [[]]
        self.highlightedObjects:list[Object3D] = []
        self.staticObjects:list[Object3D] = []
        self.highlightColor = '#0088ff'
        self.gripThreshold = 0.004
        self.renderSimulation()
        self.updateSimulationTimer = ui.timer(0.1, callback=self.update, active=True)  

    def forwardKin(self):
        rotations = robotServer.jointsValue
        matrix = TransformationMatix.compose(np.array([0.0, 0.0, 0.0]), rowan.from_euler(0.0, 0.0, 0.0, 'zyx'))
        last_offset = np.array([0.0, 0.0, 0.0])
        for offset, jointmatrix, rotation in zip(robotModel.offsets, robotModel.jointLookupMatrix, rotations):
            matrix = matrix * TransformationMatix.compose(np.array(offset) - last_offset, rowan.from_euler(*jointmatrix[::-1] * rotation, 'zyx'))
            last_offset = np.array(offset)
        position, rotation = matrix.decomposeNumpy()
        return position

    def changeVisibility(self, visible:bool):
        self.simulationDrawer.set_visibility(visible)
        
    @ui.refreshable
    def renderSimulation(self):
        if robotModel is None or not robotModel.isInitalized or robotServer.wrongRobot or not robotModel.has3DModel:
            return
        with ui.right_drawer().props('width=auto') as self.simulationDrawer:
            with ui.scene(width=700, height=950).classes('m-[-1em]') as self.scene, self.scene.group() as group:
                group.rotate(0,0,0.685)

                environment.initialize(self, self.scene)

                lastOffset = [0.0, 0.0, 0.0]
                self.links:list[ui.scene.group] = []
                group.scale(-3.5)
                for i, file in enumerate(robotModel.files):
                    self.scene.stack.append(self.scene.group())              
                    self.scene.stack[-1].move(*(-np.array(robotModel.offsets[i]) + np.array(lastOffset)))
                    lastOffset = robotModel.offsets[i]
                    self.links.append(self.scene.group())  
                    self.scene.stack.append(self.links[-1])   
                    self.scene.gltf(f'/static/{robotModel.id}/'+file, scale=0.001, offset=robotModel.offsets[i])
                
                with self.scene.group() as self.environmentGroupMoving:
                    environment.initializeGripper(self, self.scene)
                for _ in range(len(robotModel.files)*2):
                    self.scene.stack.pop()

    def update(self):
        if robotServer.jointsValue is None:
            return
        self.jointRotations = [0.0] + list(robotServer.jointsValue)
        jointTypes = ['REVOLUTE'] + robotModel.jointType
        for i, link in enumerate(self.links):       # Exclude Base
            if jointTypes[i] == 'REVOLUTE':
                link.rotate(*list(robotModel.jointLookupMatrix[i] * self.jointRotations[i] + robotModel.globalSimulationRotation[i]))
            else:
                link.move(*list(robotModel.jointLookupMatrix[i] * self.jointRotations[i] + robotModel.globalSimulationRotation[i]))
        environment.update(self, self.environmentGroupStatic, robotServer)
        if robotServer.gripper > 0.5:                
            for property, obj in zip(self.grippedObjectProperties, self.grippedObjects):
                obj.move(robotServer.Cartesian[0]+property[0][0]-property[2][0], robotServer.Cartesian[1]+property[0][1]-property[2][1], -robotServer.Cartesian[2]+property[0][2]+property[2][2])
                matRot = Object3D.rotation_matrix_from_euler((robotServer.Cartesian[3] - property[2][3]), (robotServer.Cartesian[4] - property[2][4]), (robotServer.Cartesian[5] - property[2][5]))
                matNew = np.matmul(np.array(matRot), np.array(property[1]))
                obj.R = list(map(list, matNew))
                obj._rotate()
        for i, obj, clone in zip(range(len(self.environment)), self.environment, self.highlightedObjects):
            print(sum([(obj.x-robotServer.Cartesian[0])**2, (obj.y-robotServer.Cartesian[1])**2, (obj.z+robotServer.Cartesian[2])**2])**0.5)
            if sum([(obj.x-robotServer.Cartesian[0])**2, (obj.y-robotServer.Cartesian[1])**2, (obj.z+robotServer.Cartesian[2])**2]) < self.gripThreshold:
                if robotServer.gripper > 0.5:
                    if len(self.grippedObjects) == 0:
                        self.grippedObjects.append(obj)
                        self.grippedObjectProperties.append([[obj.x, obj.y, obj.z], obj.R, robotServer.Cartesian])
                clone.move(obj.x, obj.y, obj.z)
                clone.scale(obj.sx, obj.sy, obj.sz)
                clone.rotate_R(obj.R)
                clone.visible(True)
            else:
                self.grippedObjects = []
                self.grippedObjectProperties = []
                clone.visible(False)

class Robot:
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
        self.client = RobotClient('localhost', self.myProcessSubscription)
        self.max_speed = pbBase.LinearAngularPair(linear = robotModel.maxLinearSpeed, angular=robotModel.maxAngualarSpeed)
        self.max_tolerance = pbBase.LinearAngularPair(linear = robotModel.maxLinearTolerance, angular=robotModel.maxAngularTolerance)
        self.callbacks = []
        with ui.dialog() as self.invalideMoveDialog, ui.card().style('width:25%'):
            ui.label("The robot can't be moved by the buttons while in freedrive.")
            ui.button('OK', on_click=self.invalideMoveDialog.close)

    def btnMoveJoints(self, axisName:str, distance:float, absolute:bool = False):
        if robotServer.freeDrive:
            self.invalideMoveDialog.open()
            return
        axisIndex = robotModel.getAxisIndex(axisName)
        pose = [0] * robotModel.axisCount
        pose[axisIndex%robotModel.axisCount] = distance * (0.1 if precision else 1) * (robotModel.AxisSteps[axisIndex] if not absolute else 1) / robotModel.AxisGain[axisIndex]
        return self.moveJoints(pose)
    def btnMoveCartesian(self, axisName:str, distance:float, absolute:bool = False):
        if robotServer.freeDrive:
            self.invalideMoveDialog.open()
            return
        axisIndex = robotModel.getAxisIndex(axisName)
        pose = [0] * robotModel.axisCount
        pose[axisIndex%robotModel.axisCount] = distance * -(0.1 if precision else 1) * (robotModel.AxisSteps[axisIndex] if not absolute else 1) / robotModel.AxisGain[axisIndex]
        return self.moveCartesian(pose)
    
    def changeGripperState(self, state:bool):
        self.client.gripper(1.0 if state else 0.0)

    def changeFreedrive(self, state:bool):
        self.client.requester.freedrive(pbRobotControl.Freedrive(state=state))

    def moveJoints(self, pose:list[int]):
        if self.jointsValue is not None:
            actualJoints = np.array(robotServer.jointsValue)
            logger.info(actualJoints)
            newJoints = actualJoints + np.array(pose)
            robotServer.client.moveJoints(jointsOrPose=pbBase.ArrayDouble(value=list(newJoints)),override=speed)
    def moveCartesian(self, pose:list[int]):
        if self.robotPose is not None:
            actualPose = TransformationMatix.fromPose(self.robotPose)
            robotModel.AxisNames
            offset = TransformationMatix.compose(pose[:3],rowan.from_euler(pose[robotModel.getAxisIndex('RZ')-robotModel.axisCount] if 'RZ' in robotModel.AxisNames else 0.0,
                                                                           pose[robotModel.getAxisIndex('RY')-robotModel.axisCount] if 'RY' in robotModel.AxisNames else 0.0, 
                                                                           pose[robotModel.getAxisIndex('RX')-robotModel.axisCount] if 'RX' in robotModel.AxisNames else 0.0,'zyx'))
            newPose = actualPose*offset            
            robotServer.client.moveCartesian(pose=newPose.pose(),override=speed)
    def registerUpdateCallback(self, callback:lambda:None):
        self.callbacks.append(callback)
    def myProcessSubscription(self, message):
        global freedriveChangeBtn
        try:
            self.published = message
            self.jointsValue = np.array(message.jointValues.value) if message.HasField("jointValues") else self.jointsValue
            self.robotPose = message.robotPose if message.HasField("robotPose") else self.robotPose
            self.moving = message.state.moving if message.HasField("state") else self.moving
            freeDriveAct = message.freedrive.state if message.HasField("freedrive") else False
            gripperAct = message.gripper.position if message.HasField("gripper") else 0.0
            if self.robotPose is None or self.jointsValue is None or self.moving is None:
                return
            if not len(self.jointsValue) == robotModel.axisCount:
                if self.wrongRobot:
                    return
                self.wrongRobot = True
                wrongRobotSelected()
            if message.HasField("freedrive") and (not self.freeDrive == freeDriveAct) and (not freedriveChangeBtn is None):
                self.freeDrive = freeDriveAct
                freedriveChangeBtn.handlePress(state=freeDriveAct, suppress=True)
            if message.HasField("gripper") and (not (abs(self.gripper - gripperAct) < 0.01)) and (not gripperChangeBtn is None):
                self.gripper = gripperAct
                gripperChangeBtn.handlePress(state=gripperAct>0.5, suppress=True)
            if not self.initializedAxis and message.HasField("robotPose") and message.HasField("jointValues"):
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
        self.client.shutdown()

class robotManager:
    def __init__(self):
        with open(JSONPath+'robotSelect.json') as f:
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
    def returnToFirstDialog(self):
        self.wrongSelectionDialog.close()
        self.choseRobotDialog.open()
    def handleChoice(self, e):
        global robotPath
        if self.selectedRobot == 0:
            self.wrongSelectionDialog.open()            
        else:
            robotPath = (self.files[self.ids.index(self.selectedRobot)], self.selectedRobot)
            ui.open('/robot')

def wrongRobotSelected():
    with ui.dialog() as wrongSelectionDialog, ui.card().style('width:25%'):
        ui.label('Please select the same robot as you are connected to.')
        ui.button('Done', on_click=lambda: ui.open('/'))
    wrongSelectionDialog.open()

@ui.page('/robot')
def robotPage():
    global robotModel, robotServer, robotSim
    if robotPath[0] == '' or robotPath[1] == '':
        return RedirectResponse('/')
    robotModel = robotDescription(path=JSONPath+robotPath[0], id=robotPath[1])
    robotServer = Robot()
    robotSim = Simulation()
    app.on_shutdown(robotServer.client.shutdown)
    renderRobot(robotModel)

robotPath = ('', '')

@ui.page('/')
def select():
    mgr = robotManager()
    mgr.choseRobotDialog.open()

ui.run(show=False, title='Robot Interface')

# TODO:
# - charts grid