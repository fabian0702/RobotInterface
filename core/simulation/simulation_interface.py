from nicegui import ui
from nicegui.element import Element
from nicegui.elements.scene_object3d import Object3D
from nicegui.elements.scene_objects import Group

from core.model import RobotModel
from core.server import RobotServer

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
    attached:bool = False

    def to_kwargs(self):
        ...

class Simulation(Element, component='simulation.js',):
    """Class for the simulation of the robot"""

    robotModel:RobotModel
    robotServer:RobotServer
    graspableObjects:list[Object3D]
    environmentData:dict
    movingClones:list[Object3D]
    staticClones:list[Object3D]
    staticObjects:list[Object3D]
    capture:bool
    cameras:list[ui.scene_view]
    cameraProperties:list[CameraProperties]
    gripped:bool
    lastGripperPosition:float
    isInitialized:bool
    updateSimulationTimer:ui.timer
    scene:ui.scene
    gripperGroup:Group

    def __init__(self, robotModel:RobotModel, robotServer:RobotServer) -> None:
        ...

    def changeVisibility(self, visible:bool):
        """changes the visibility of the simulation"""
        ...

    def changeCapture(self, state:bool):
        """Starts or stops capturing a video stream"""
        ...

    def requestWorldTransform(self, object:Object3D) -> Object3D:
        """Request the world transform of the object."""
        ...
    

        
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
        ...

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
        ...