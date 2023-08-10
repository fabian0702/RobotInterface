from nicegui.elements.scene_object3d import Object3D
from nicegui import ui
import time, math

class sim:
    graspableObjects:list[Object3D]
    staticObjects:list[Object3D]
    environmentData:dict
    gripThreshold:float = 0.004
    highlightColor:str = '#0088ff'
    addCameraHelper:bool = True

    def addEnvironmentCamera(self, position:list[float], look_at:list[float], fov:float = 75, focus:float = 10, far:float=1000, near:float=0.1) -> None:
        pass
    def addGripperCamera(self, fov:float = 75, focus:float = 10, far:float=1000, near:float=0.1) -> None:
        pass

def initialize(self:sim, scene:ui.scene):
    """
        This Function will initialize the environment of the simulation.
        When you create an object which can be grasp please append it to the ```self.graspableObjects``` list to be able to access it in the update method.
        Static objects can be added to the ```self.staticObjects``` list. Those are not graspable by the robot.
        If you want variables you can also access in the update funcion use global variabels or append the data you want to store to the ```self.environmentData```` dict.
        To access a local 3D file please put it in the same directory as the environments.py script and use ```/static/environment/youFileName``` to access it.
        If you want to change the color for the highlight the change ```self.highlightColor``` to your desired color as a rgb hex string (default='#0088ff').
        If you want up to three additional cameras, call ```self.addEnvironmentCamera``` with the desired position and look_at as the point for the camera to face.
        There are also some optional parameters of the camera such as fov or focus.
        Please adjust ```self.gripThreshold``` to your desired Value, usually 0.004 works well but may need to be adjusted to suit your needs.
    """
    self.addCameraHelper = False
    self.environmentData |= {'startTime':time.time()}

    self.addEnvironmentCamera([0, 0, 5], [0, 0, 0])

    box1 = scene.box(0.1, 0.1, 0.1)
    box1.move(0.5, 0.0, 0.0)
    self.graspableObjects.append(box1)

    box1 = scene.box(0.1, 0.1, 0.1)
    box1.move(0.0, 0.5, 0.0)
    self.staticObjects.append(box1)
    pass

def initializeGripper(self:sim, scene:ui.scene):
    """
        In this function you can add your own gripper to robot.
        If you want variables you can also access in the update funcion use global variabels or append the data you want to store to the ```self.environmentData``` dict.
        To access a local 3D file please put it in the same directory as the environments.py script and use ```/static/environment/youFileName``` to access it.
        If you want to add a moving camera, call ```self.addGripperCamera``` with optional parameters of the camera such as fov or focus. 
        Keep in mind that the total limit of additional cameras is three in the entire scen.
        Please dont add any objects to ```self.graspableObjects``` list, as this can lead to weird behaviour of the simulation.
    """
    with scene.group() as g:
        g.move(0, 0, 0)
        self.addGripperCamera()
    pass
def update(self:sim, robotServer, robotModel):
    """ 
        This Function updates the environment for the simulation and will be called after every renderingcycle of the robot.
        To access data about the robot, use ```robotServer.JointsValue``` to get the rotation of the Joints or use ```robotServer.Cartesian``` to get the Cartesian pose from the robot.
        To access the parameters the robot has use ```robotModel.yourDesiredParameter```. To find all possible parameters please look at the parameter of the ```robotDescription``` class in main.py
        If you want a camera to move, you can either manipulate it here, or if its a gripperCamera it will move automatically with the robot
        Please do not change the robotServer or robotModel object as this can lead to inconsistencies or bugs in the interface.
    """   
    #self.c.move((time.time()-self.environmentData['startTime'])/40, 0.0, 0.0)
    for obj in self.staticObjects:
        obj.rotate(time.time(), time.time(), time.time())
    pass