from math import log10
from nicegui import ui
import os

from core.constants import direction, LOG_LEVEL
from core.server import RobotServer
from core.model import RobotModel

from python_main.ch.bfh.roboticsLab.util.Logger import Logger

logger = Logger(os.path.basename(__file__), LOG_LEVEL).getInstance()

class Axis:
    """Class to render the controll for one axis"""
    def __init__(self, robotModel:RobotModel, robotServer:RobotServer, axisname:str, unit='°', on_move=lambda a, d: None, position = 0.0):
        self.axisname = axisname
        self.robotModel = robotModel
        self.robotServer = robotServer
        self.axisIndex = self.robotModel.getAxisIndex(self.axisname)
        self.position = position*self.robotModel.AxisGain[self.axisIndex]
        self.places = int(abs(log10(self.robotModel.AxisSteps[self.axisIndex])) + 1)
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
        if self.editing or self.robotServer.jointsValue is None or self.robotServer.Cartesian is None:
            return
        newPosition = (self.robotServer.Cartesian[self.axisIndex%self.robotModel.axisCount] if self.axisIndex >= self.robotModel.axisCount else list(self.robotServer.jointsValue)[self.axisIndex%self.robotModel.axisCount])*self.robotModel.AxisGain[self.axisIndex]
        if abs(newPosition - self.lastPosition) > self.minDelta:
            self.lastPosition = self.position = newPosition

    def move(self, direction:int, distance:float, absolute:bool = False):
        """Function to move the axis either a distance in the direction or to the absolute position specified in the position field"""
        self.editing = False
        if not self.robotServer.moving:
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
            ui.label(f'{self.axisname} [{self.unit}]') \
                .classes('mb-[-0.5em] text-white')        # Axisname
            ui.button('',icon='keyboard_double_arrow_up', on_click=lambda e: self.move(direction.up, self.fastSpeed)) \
                .classes(buttonClasses)    # Btn up fast
            ui.button('',icon='keyboard_arrow_up', on_click=lambda e: self.move(direction.up, self.slowSpeed)) \
                .classes(buttonClasses)           # Btn up slow
            self.input = ui.number(value=self.position, format=f'%.{self.places}f', step=self.robotModel.AxisSteps[self.axisIndex]/10) \
                .on('blur', lambda e: self.move(0, 0, True)) \
                .on('focus', self.changeEditing) \
                .props('dense borderless color=red-1') \
                .classes('bg-slate-700 border-solid border rounded-md my-[-0.4em]') \
                .style('width: 5em') \
                .bind_value(self, 'position')  # Current position of the Axis
            ui.button('',icon='keyboard_arrow_down', on_click=lambda e: self.move(direction.down, self.slowSpeed)) \
                .classes(buttonClasses)       # Btn down slow
            ui.button('',icon='keyboard_double_arrow_down', on_click=lambda e: self.move(direction.down, self.fastSpeed)) \
                .classes(buttonClasses)# Btn down fast
