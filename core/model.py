from .constants import JSON_PATH
import numpy as np
import json
from nicegui import ui, app
from os.path import isfile


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
                app.add_static_files(f'/static/{id}', f'{JSON_PATH}{id}/')                    # Configure the path for the 3D models
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
    