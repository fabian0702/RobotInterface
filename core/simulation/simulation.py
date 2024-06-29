import time, rowan, base64, cv2, os, asyncio
import numpy as np

from threading import Thread

from dataclasses import dataclass
# from math import sqrt

from nicegui import ui, core, app, background_tasks
from nicegui.element import Element
from nicegui.elements.scene_object3d import Object3D
from nicegui.elements.scene_objects import Group

from core.simulation import CameraServer
from core.constants import LOG_LEVEL, \
                           GRIPPER_FAR_COLOR, \
                           GRIPPER_FAR_DISTANCE, \
                           GRIPPER_CLOSE_COLOR, \
                           GRIPPER_CLOSE_DISTANCE, \
                           GRIP_THRESHOLD, \
                           GRIPPER_THRESHOLD_COLOR, \
                           GRIPPER_GRIPPED_COLOR, \
                           UPDATE_CLONES, \
                           CAMERA_HEIGHT, \
                           MIN_GRIPPER_CHANGE, \
                           SIMULATION_REFRESH_RATE
from core.model import RobotModel
from core.server import RobotServer

from resources.environment import environment

from python_main.ch.bfh.roboticsLab.util.Logger import Logger

@dataclass
class CameraProperties:
    x: float = None,
    y: float = None,
    z: float = None,
    look_at_x: float = None,
    look_at_y: float = None,
    look_at_z: float = None,
    up_x: float = None,
    up_y: float = None,
    up_z: float = None
    duration:float=0
    helper:bool = False
    attached:bool = False

    def to_args(self):
        return (self.x, 
                self.y, 
                self.z, 
                self.look_at_x, 
                self.look_at_y, 
                self.look_at_z, 
                self.up_x,
                self.up_y,
                self.up_z)
    
@dataclass
class ObjectTransform:
    x:float
    y:float
    z:float
    q:list[float]

logger = Logger(os.path.basename(__file__), LOG_LEVEL).getInstance()

class Simulation(Element, component='simulation.js',):
    """Class for the simulation of the robot"""
    def __init__(self, robotModel:RobotModel, robotServer:RobotServer) -> None:
        super().__init__()
        app.on_shutdown(self.shutdown)
        self.robotModel = robotModel
        self.robotServer = robotServer
        self.graspableObjects:list[Object3D] = []
        self.environmentData:dict = {}
        self.movingClones:list[Object3D] = []
        self.staticClones:list[Object3D] = []
        self.staticObjects:list[Object3D] = []
        self.capture:bool = False
        self.cameras:list[ui.scene_view] = []
        self.cameraProperties:list[CameraProperties] = []
        self.gripped:bool = None
        self.lastGripperPosition:float = 0
        self.isInitialized:bool = False
        self.displayedError:bool = False
        self.gripperGroup:Group = None
        self.updateSimulationTimer:ui.timer = ui.timer(SIMULATION_REFRESH_RATE, callback=self.update, active=True)
        self.publisherPair:list[tuple[CameraServer.Publisher, CameraServer.grpc.Server]] = []

    async def postCameraStream(self, cameraID:int=0):
        """Function to parse a post request to the api, decode get the individual images and send the to the grpc server"""
        base64Img:str = await self.run_method('captureImage', self.cameras[cameraID].id)
        # print(len(base64Img))
        if not len(base64Img):
            return
        imgData = base64.decodebytes(base64Img[base64Img.index(','):].encode())     # decode base64 to openCV image
        nparr = np.frombuffer(imgData, np.uint8)
        img = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
        logger.debug(f'received capture of camera {cameraID}')
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
    async def renderSimulation(self):  
        """Funtion for setting up the simulation"""
        if self.robotModel is None or not self.robotModel.isInitalized or self.robotServer.wrongRobot or not self.robotModel.has3DModel or not self.robotModel.hasJoints:
            return
        self.isInitialized = False
        with ui.column(wrap=False).classes('h-screen w-full bg-slate-300 h-full flex flex-column flex-nowrap p-0') as self.simulationContainer:
            with ui.scene().classes('w-full flex-1 m-0 overflow-hidden') as self.scene:
                with self.scene.group() as group:
                    environment.initialize(self)    # Initializing of the scene
                                    
                    for obj in self.graspableObjects:       # preparation for highlightes objects
                        self.staticClones.append(Object3D(obj.type, *obj.args[:-1], True if obj.args[-1] == False else obj.args[-1]).material(color=GRIPPER_CLOSE_COLOR))     # Copy object but change wireframe option if awailable
                        
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
                    with self.scene.group() as self.gripperGroup:
                        self.gripperTip = self.scene.sphere(0, 0, 0)
                        environment.initializeGripper(self)         # Initializing gripper if present
                        for obj in self.graspableObjects:
                            with self.scene.group() as clone:
                                Object3D(obj.type, *obj.args).material(obj.color, obj.opacity, obj.side_).with_name(f'obj {self.graspableObjects.index(obj)}')
                                Object3D(obj.type, *obj.args[:-1], True if obj.args[-1] == False else obj.args[-1]).material(color=GRIPPER_GRIPPED_COLOR).with_name(f'obj {self.graspableObjects.index(obj)}')     # Copy object but change wireframe option if awailable
                            self.movingClones.append(clone)
                    for _ in range(len(self.robotModel.files)*2):        # Cleaning up
                        self.scene.stack.pop() 

            await self.scene.initialized()

            if len(self.cameraProperties):
                self.additionalCamerasContainer = ui.row().classes(f'flex flex-row flex-nowrap w-full bg-slate-700 h-[{int(CAMERA_HEIGHT*100)}%] flex-none m-0')

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
                             up_z: float = None,
                             helper:bool=False) -> None:
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
        :param helper: wether the camera has a camera helper associated
        """
        logger.info(f'adding camera {len(self.cameraProperties)}')
        self.cameraProperties.append(CameraProperties(x, y, z, look_at_x, look_at_y, look_at_z, up_x, up_y, up_z, duration=0, helper=helper, attached=False))

    def addGripperCamera(self,
                         x: float = None,
                         y: float = None,
                         z: float = None,
                         look_at_x: float = None,
                         look_at_y: float = None,
                         look_at_z: float = None,
                         up_x: float = None,
                         up_y: float = None,
                         up_z: float = None,
                         helper:bool = False) -> None:
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
        :param helper: wether the camera has a camera helper associated
        """
        logger.info(f'adding camera {len(self.cameraProperties)}')
        self.cameraProperties.append(CameraProperties(x, y, z, look_at_x, look_at_y, look_at_z, up_x, up_y, up_z, duration=0, helper=helper, attached=True))

    def _attachCamera(self, camera:ui.scene_view, group:Object3D):
        while not self.run_method('attachCamera', self.scene.id, camera.id, group.id):
            time.sleep(0.1)     # retry after sleeping for 0.1 seconds

    def _cameraHelper(self, camera:ui.scene_view):
        self.run_method('cameraHelper', camera.id, timeout=None)
 
    def resize(self):
        for camera in self.cameras:
            camera.run_method('resize',timeout=None)
        self.scene.run_method('resize',timeout=None)

    def _applyProperties(self, object:Object3D, properties:ObjectTransform):
        object.R = rowan.to_matrix(properties.q)
        object.x = properties.x
        object.y = properties.y
        object.z = properties.z
    
    async def _match(self, staticObject:Object3D, referenceObject:Object3D, dynamicObject:Object3D):
        position, rotation = await self.run_method('match', staticObject.id, referenceObject.id, dynamicObject.id, self.scene.id, timeout=None)
        self._applyProperties(dynamicObject, ObjectTransform(**position, q=rotation))
        
    async def _follow(self, staticObject:Object3D, dynamicObject:Object3D):
        position, rotation = await self.run_method('follow', staticObject.id, dynamicObject.id, self.scene.id, timeout=None)
        self._applyProperties(dynamicObject, ObjectTransform(**position, q=rotation))

    async def _getWorldTransform(self, object:Object3D) -> ObjectTransform:
        position, rotation = await self.run_method('getWorldTransform', object.id, self.scene.id, timeout=None)
        return ObjectTransform(**position, q=rotation)
    
    async def _worldDistance(self, a:Object3D, b:Object3D):
        return await self.run_method('calcDistance', a.id, b.id, self.scene.id, timeout=None)

    async def update(self) -> None:             
        """Funtion for updating the simulation""" 

        def errorDialog(message):
            if self.displayedError:
                return
            self.displayedError = True
            with ui.dialog(value=True) as dialog, ui.card().classes('w-1/4'):
                ui.label(message)
                ui.button('OK', on_click=dialog.close)

        if self.robotServer.jointsValue is None:
            logger.error('Robot server has not sent any messages')   
            self.updateSimulationTimer.interval = 1
            errorDialog('The robot server has not sent any messages. This is likely because no server is connected to the interface.')
            return
        if self.robotServer.client is None:
            logger.error('Robot server has not been connected')   
            errorDialog('No robot server is connected to the Interface. This is likely because no server was connected or the server was disconected.')
            return
        if not self.robotModel.has3DModel:
            logger.debug('Robot does not have any 3D model associated')
            errorDialog('The selected robot does not have any 3D model associated.')
            self.updateSimulationTimer.interval = 1
            self.simulationContainer.visible = False
            return
        if not self.robotModel.hasJoints:
            logger.debug('Robot does not have any joints')
            self.updateSimulationTimer.interval = 1
            self.simulationContainer.visible = False
            return
        if not self.isInitialized:
            logger.debug('Simulation is not yet initilized')
            self.updateSimulationTimer.interval = 1
            return
        elif self.updateSimulationTimer.interval == 1:
            self.updateSimulationTimer.interval = 0.1
            self.displayedError = False
        
        if self.cameraProperties:
            self.cameras = []
            with self.additionalCamerasContainer:
                for cameraProperties in self.cameraProperties:
                    with ui.scene_view(self.scene).classes('flex-1 flex-nowrap h-full overflow-x-hidden') as camera:
                        camera.move_camera(*cameraProperties.to_args())
                    if cameraProperties.attached:
                        self._attachCamera(camera, self.gripperGroup)
                    if cameraProperties.helper:
                        self._cameraHelper(camera)
                    self.cameras.append(camera)

        if abs(self.robotServer.gripper-self.lastGripperPosition) > MIN_GRIPPER_CHANGE:
            environment.gripperMove(self, self.robotServer.gripper)
            self.lastGripperPosition = self.robotServer.gripper
        self.jointRotations = [0.0] + list(self.robotServer.jointsValue)
        jointTypes = ['REVOLUTE'] + self.robotModel.jointType
        for i, link in enumerate(self.links):       # Exclude Base
            if jointTypes[i] == 'REVOLUTE':         # Update robot joints
                link.rotate(*list(self.robotModel.jointLookupMatrix[i] * self.jointRotations[i]))
            else:
                link.move(*list(self.robotModel.jointLookupMatrix[i] * self.jointRotations[i] + self.robotModel.globalSimulationRotation[i]))
        
        environment.update(self, self.robotServer, self.robotModel)       # Update Environment

        def lerpInt(v1:int, v2:int, t:float) -> int:
            return int((1-t) * v1 + t * v2)

        def lerpColor(v1:str, v2:str, t:float) -> str:
            rt, gt, bt = (lerpInt(int(v1[1:3], 16), int(v2[1:3], 16), t),
                          lerpInt(int(v1[3:5], 16), int(v2[3:5], 16), t),
                          lerpInt(int(v1[5:7], 16), int(v2[5:7], 16), t))
            return '#'+ hex(rt)[2:].rjust(2, '0') + \
                        hex(gt)[2:].rjust(2, '0') + \
                        hex(bt)[2:].rjust(2, '0')
        
        def limit(v:float, maxValue:float, minValue:float) -> float:
            return min(max(v, minValue), maxValue)

        for obj, movingClone, staticClone in zip(self.graspableObjects, self.movingClones, self.staticClones):
            distance = await self._worldDistance(self.gripperTip, obj)
            if distance < GRIPPER_FAR_DISTANCE and self.gripped is None:
                t = limit(distance, GRIPPER_FAR_DISTANCE, GRIPPER_CLOSE_DISTANCE)
                tNorm = t / (GRIPPER_FAR_DISTANCE - GRIPPER_CLOSE_DISTANCE)
                color = lerpColor(GRIPPER_CLOSE_COLOR, GRIPPER_FAR_COLOR, tNorm)
                staticClone.material(color if distance > GRIP_THRESHOLD else GRIPPER_THRESHOLD_COLOR)
                staticClone.visible(not self.gripped)
            else:
                staticClone.visible(False)

            if UPDATE_CLONES:
                await self._follow(obj, staticClone)

            if distance < GRIP_THRESHOLD or self.gripped == obj.id:   # Check if object if close enough to be gripped
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

        if self.capture:
            for cameraID in range(len(self.cameras)):
                await self.postCameraStream(cameraID)
        if len(self.cameraProperties):
            self.cameraProperties = []
            self.resize()