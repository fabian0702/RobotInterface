from urllib.parse import quote

from fastapi.responses import RedirectResponse

from nicegui import ui, app

from core.constants import LOG_LEVEL, JSON_PATH, ROBOT_SELECTION_DATA, CLIENT_PORT
from core.model import RobotModel
from core.server import RobotServer
from core.controls import Controls

from python_main.ch.bfh.roboticsLab.util.Logger import Logger
logger = Logger('RobotInterface', LOG_LEVEL).getInstance()

@ui.page('/robot')
def returnToOriginalPage():
    return RedirectResponse('/')

@ui.page('/robot/{id}')
async def robotPage(id):   
    """Main Interface page"""
    files = {file['ID']:file['file'] for file in ROBOT_SELECTION_DATA}     
    robotModel = RobotModel(id, JSON_PATH+files[id])      # Setup of the parameters for the robot
    robotServer = RobotServer(robotModel)                                                            # Grpc requester and subscriber setup
    controls = Controls(robotModel, robotServer)
    def shutdown():
        logger.warning('Cleanup has been initialized')
        robotServer.shutdown()
        controls.simulation.shutdown()
        logger.info('Cleanup done.')
    ui.context.client.on_disconnect(shutdown)
    disconection_timer = ui.timer(0.05, lambda : None)
    disconection_timer._cleanup = shutdown

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
        ui.navigate.to(f'/robot/{quote(robotSelector.value, safe="")}')

    robots = [robot['ID'] for robot in ROBOT_SELECTION_DATA]
    with ui.dialog(value=True).props("no-backdrop-dismiss no-esc-dismiss") as choseRobotDialog:
        with ui.card().classes('w-1/4'):
            ui.label('Please choose a robot:')
            robotSelector = ui.select(robots).props("persistent")
            ui.button('Done', on_click=handleChoice)

    with ui.dialog(value=False) as wrongSelectionDialog:
        with ui.card():
            ui.label('Please select a robot from the dropdown before continuing.')
            ui.button('Close', on_click=returnToFirstDialog)

ui.run(show=False, title='Robot Interface', reload=False, port=CLIENT_PORT)