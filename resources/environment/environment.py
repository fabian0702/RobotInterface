from nicegui.elements.scene_object3d import Object3D
from nicegui import ui
import time, math

class Simulation:
    pass


UPDATE_CLONES = True # wether to update the clones once every simulation cycle or just update them when a object is moved by a robot.
                     # updating constantly may impact performace negativly especially with a lot of graspable objects.

GRIPPER_LENGTH = 0.10
# Wafer diameter in inches (try to use only standard sizes)
WAFER_DIAMETER = 2
WAFER_RADIUS_M = WAFER_DIAMETER * 25.4 / 2.0/1000.0
RACK_WIDTH = math.floor((2*WAFER_RADIUS_M + 0.02)*1000)/1000
# Converyor belt length
CONVEYOR_LENGTH = 0.38
# Conveyor speed [m/s] (nominal 0.1, accelerated here 15% to compensate for animation period sleep)
CONVEYOR_SPEED = 0.115
# Conveyor limit (symmetric)
CONVEYOR_LIMIT = 0.18

L5Z = -0.0225

blueMaterial = '#2222DD'
redMaterial = '#DD2222'
metalMaterial = '#C0C0C0'
silverMaterial = '#C0C0C0'
blackMaterial = '#222222'
orangeMaterial = '#f1c242'
orangePlasticMaterial = '#f1c242'
blackSpringMaterial = '#1e1e1e'
grayMaterial = '#97a0b4'


def createRack(scene:ui.scene, color:str=redMaterial):
    with scene.group() as rack:
        leftLeg = scene.box(RACK_WIDTH, 0.005, 0.7).material(color=color)
        rightLeg = scene.box(RACK_WIDTH, 0.005, 0.7).material(color=color)
        rightLeg.move(rightLeg.x, RACK_WIDTH-0.002, rightLeg.z).material(color=color)
        boardTop = scene.box(RACK_WIDTH, RACK_WIDTH, 0.002).material(color=color)
        boardTop.move(boardTop.x, RACK_WIDTH/2, 0.349).material(color=color)
        boardBottom = scene.box(RACK_WIDTH, RACK_WIDTH, 0.002).material(color=color)
        boardBottom.move(boardBottom.x, RACK_WIDTH/2, 0.25).material(color=color)
        for y in [0.01, RACK_WIDTH - 0.01]:
            for z in [0.27, 0.29, 0.31, 0.33]:
                board = scene.box(RACK_WIDTH, 0.02, 0.002).material(color=color)
                board.move(board.x, y, z)
    return rack

def gripperMove(position: float):
    print(f'gripper moved to {position}!!!')

def createWaver(scene:ui.scene, color:str=blackMaterial):
    return scene.cylinder(WAFER_RADIUS_M, WAFER_RADIUS_M, 0.001, 32).material(color)

def initialize(self:Simulation, scene:ui.scene):
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
        If you have moving objects which can be picked up by a robot, you might consider turing updateClones on as this ensures the highlights follow accuratly even on movin object.
        In scenes with a lot of graspable object you might experience some performace penalty by using this setting.
    """
    self.updateClones = True
    self.gripThreshold = 0.01
    # Blue -> Output

    #self.addCameraHelper = True
    self.environmentData |= {'startTime':time.time()}

    # camera = scene.gltf('/static/environment/camera.glb', scale=1, offset=[0.22, 0.0, 0.0])

    self.addEnvironmentCamera(0, 1.8, 3)
    self.addGripperCamera(0, -1.8, 3)

    rackLeft = createRack(scene, blueMaterial)
    rackLeft.move(0.20, -0.20, 0.0)
    rackLeft.rotate(0, 0, 3 * math.pi / 4.0)
    rackRight = createRack(scene, redMaterial)
    rackRight.move(0.20, 0.20, 0.0)
    rackRight.rotate(0, 0, math.pi / 4.0)

    for z in [0.27, 0.29]:#, 0.31, 0.33]:
        waver = createWaver(scene, blackMaterial)
        waver.move(0.18, 0.19+RACK_WIDTH/2, z+0.002)
        waver.rotate(math.pi / 2.0, 0.0, 0.0)
        self.graspableObjects.append(waver)

    pass

def initializeGripper(self:Simulation, scene:ui.scene):
    """
        In this function you can add your own gripper to robot.
        If you want variables you can also access in the update funcion use global variabels or append the data you want to store to the ```self.environmentData``` dict.
        To access a local 3D file please put it in the same directory as the environments.py script and use ```/static/environment/youFileName``` to access it.
        If you want to add a moving camera, call ```self.addGripperCamera``` with optional parameters of the camera such as fov or focus. 
        Keep in mind that the total limit of additional cameras is three in the entire scen.
        Please dont add any objects to ```self.graspableObjects``` list, as this can lead to weird behaviour of the simulation.
    """

    with scene.group() as gripper:
        gripper.move(0, 0.25, 0.0)
        gripper.rotate(1.57, 0, 0)
        #self.addGripperCamera()
        b1 = scene.box(GRIPPER_LENGTH, 0.005, 0.005)
        b1.move(GRIPPER_LENGTH/2, -0.0125, 0.0)
        b2 = scene.box(GRIPPER_LENGTH, 0.005, 0.005)
        b2.move(GRIPPER_LENGTH/2, 0.0125, 0.0)
        base = scene.cylinder(0.02, 0.02, 0.025, 32)
        base.rotate(math.pi / 2.0, 0.0, 0.0)
        base.move(0, 0, -0.01)
    pass
def update(self:Simulation, robotServer, robotModel):
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