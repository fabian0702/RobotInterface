# Robotinterface
## Setup
To install all necessary python packages run 
```
python3 -m pip install -r requirements.txt
```
## Starting the interface
To start the interface run:
```
python3 robotUI.py
``` 
which will give you a link in the console to open the ui

## Configuration of the robots
### Resources
All the resources for the simulation can be found in the equaly names folder resources
### Json files
The robots in available in the Interface can be configured in the ```robotSelect.json``` file which contains all the robots currently available. For every robot there is an id which is used as the name of the robot in the selection dropdown. The file attibute references a json file in the same directory which contains additional information about the robot like the configurations for the axis of the robot.
### Models Folder
In this folder there are subfolder with for the robot simulation with each of the foldernames corresponding to the id field for the robot in the ```robotSelect.json``` file. Inside this folder there is a ```model.json``` file which contains the configuration for the simulation and also a field called *files* list of GLTF file which are in the same directory as the ```model.json``` file. These 3D models will be loaded in the order of the list, meaning the first file in the list will be the base of the robot and the last will be the outermost link of the robot. The field *globalRotation* is al list of 3D rotations to apply to the links of the robot including the base. The first entry corresponds to the first entry in the *files* field. The field *jointLookupMatrix* contains a matrix where the rows correspond to the links of the robot and the row gets multiplyed by the rotation of the corresponding link to get the euler rotations. The field *offsets* define the the offsets from the joints to the origin.
### Environment folder
