import json

from core.args import args

import logging

class direction: 
    """Data class to store direction"""   
    up = 1
    down = -1

SERVER_ADDRESS = 'localhost' if 'local_server' in args else args.server_host   # Hostname of the robot server
START_SERVER = 'local_server' in args or True      # Wether to start a local server
CLIENT_PORT = args.client_port             # Port for the nicegui application

LOG_LEVEL = logging.INFO

CHART_LOG_TIME = 10               # Total timespan on chart
CHART_UPDATE_INTERVAL = 0.4       # Interval between data refreshs

JSON_PATH = './resources/models/'        # Path to where the configuration files for the robots are saved
ENVIRON_PATH = './resources/environment/'        # Path to where the environments files are saved

with open(JSON_PATH+'robotSelect.json') as f:        # Parses all robots from the robotSelect.json
    ROBOT_SELECTION_DATA = json.loads(f.read())

# Simulation

GRIP_THRESHOLD = 0.04

GRIPPER_FAR_DISTANCE = 1
GRIPPER_CLOSE_DISTANCE = GRIP_THRESHOLD

GRIPPER_FAR_COLOR = '#00ff00'
GRIPPER_CLOSE_COLOR = '#0088ff'
GRIPPER_THRESHOLD_COLOR = '#0044ff'
GRIPPER_GRIPPED_COLOR = '#ff0000'

UPDATE_CLONES = True
CAMERA_HEIGHT = 0.25
MIN_GRIPPER_CHANGE = 0.001
SIMULATION_REFRESH_RATE = 0.1