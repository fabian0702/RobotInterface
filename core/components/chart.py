import time
import numpy as np

from nicegui import ui

from matplotlib import pyplot as plt
plt.switch_backend('agg')               # change backend to optimise charts

from core.constants import CHART_LOG_TIME, CHART_UPDATE_INTERVAL
from core.model import RobotModel
from core.server import RobotServer

class Chart:
    """Class to render a chart for a configurable amount of axies"""
    def __init__(self, robotModel:RobotModel, robotServer:RobotServer, title:str, xaxisName:str, yaxisName:str, graphNames:list[str], visible = False):
        self.title = title
        self.robotServer = robotServer
        self.index = 0
        self.graphNames = graphNames
        self.maxIndex = int(CHART_LOG_TIME / CHART_UPDATE_INTERVAL)
        self.data:list[list[float]] = [[0 for _ in range(self.maxIndex)] for i in range(len(self.graphNames) + 1)]
        self.graphNames = graphNames
        self.xaxisName = xaxisName
        self.yaxisName = yaxisName
        self.robotModel = robotModel
        self.chart()
        self.plot.set_visibility(visible)       # Hide chart per default
        self.startTime = time.time()
        self.gains = np.array([gain if robotModel.AxisNames[i] in self.graphNames else 0.0 for i, gain in enumerate(robotModel.AxisGain)])
        axisIndecies = list(map(robotModel.getAxisIndex, self.graphNames))
        self.startIndex = min(axisIndecies)
        self.endIndex = max(axisIndecies)
        
    @ui.refreshable
    def chart(self):
        """Function to render the chart"""
        self.chartUpdateTimer = ui.timer(CHART_UPDATE_INTERVAL, self.updateData, active=False)        # setup refresh timer
        self.plot:ui.line_plot = ui.line_plot(n=len(self.graphNames), limit=self.maxIndex-1, figsize=(6, 2)).with_legend(self.graphNames, loc='upper right').classes('w-full')        # plot
        self.plot.fig.text(0.5, 0.03, self.xaxisName, ha='center', va='center')
        self.plot.fig.text(0.1, 0.95, self.yaxisName, ha='center', va='center')
        plt.grid(axis = 'x')        # grid
    
    def updateData(self,):
        """Function to update the data from the server, and display it on the chart"""
        if not self.robotServer.client is None and self.plot.visible and len(self.graphNames) > 0:       # Sanity check
            self.index = int((self.robotServer.currentTime - self.startTime) / CHART_UPDATE_INTERVAL) % self.maxIndex      # Calculate index in ring buffer
            self.data[0][self.index] = (self.robotServer.currentTime - self.startTime) % CHART_LOG_TIME                   # write time in ringbuffer
            for i, val in enumerate((self.gains * (np.concatenate((self.robotServer.jointsValue, self.robotServer.Cartesian)) if self.robotModel.hasJoints else self.robotServer.Cartesian))[self.startIndex:self.endIndex+1]):
                self.data[i+1][self.index] = val      # write the values neccesary in the ring buffer
            self.plot.push(self.data[0], self.data[1:])     # update the chart

    def changeVisibility(self, visible:bool):
        """Function to change the visibility of the charts"""
        if visible:
            self.startTime = time.time()        # keep track of the start time
            self.chartUpdateTimer.activate()    # activate the refresh timer
            self.plot.clear()                   # reset the plot
            if self.robotServer.Cartesian is None or ((not self.robotModel.hasJoints) and self.robotServer.jointsValue is None):
                return
            self.data = [[i * CHART_UPDATE_INTERVAL for i in range(self.maxIndex)]] + [[(np.concatenate((self.robotServer.jointsValue, self.robotServer.Cartesian)) if self.robotModel.hasJoints else self.robotServer.Cartesian)[i +  self.startIndex] for n in range(self.maxIndex)] for i in range(len(self.graphNames))]       # prepare the data
        else:
            self.chartUpdateTimer.deactivate()      # deactivate refresh timer
        
        self.plot.set_visibility(visible)           # hide / show chart
