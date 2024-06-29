from nicegui import ui, background_tasks, core
import numpy as np
import os, asyncio

from sys import path
path.append('./python_main/')

from python_main.ch.bfh.roboticsLab.util.TransformationMatrix import TransformationMatix
from python_main.ch.bfh.roboticsLab.util.Logger import Logger

from core.model import RobotModel
from core.server import RobotServer
from core.simulation.simulation import Simulation
from core.components.components import toggle_button, chart, axis
from core.constants import LOG_LEVEL

logger = Logger(os.path.basename(__file__), LOG_LEVEL).getInstance()

class Controls:
    def __init__(self, model:RobotModel, server:RobotServer) -> None:
        self.model = model
        self.server = server
        self.simulation = Simulation(model, server)
        self.render()
        server.on_initialized = self.render

    @ui.refreshable
    def render(self):
        """renders the controlls of the robot"""
        ui.html('''<style>input[type=number]{-moz-appearance: textfield;color:white;text-align: center;}input::-webkit-outer-spin-button,input::-webkit-inner-spin-button {-webkit-appearance: none;margin: 0;}</style>''')
        with ui.column().classes('m-0 p-0 flex flex-column w-full h-full mx-[-1rem] my-[-2rem] overflow-x-hidden'):
            with ui.row().classes('w-screen h-full', remove='wrap'):            
                with ui.column().classes('bg-slate-400 p-4 h-full'):
                    with ui.row().classes(remove='wrap'):
                        ui.button('', icon='settings', on_click=lambda e: ui.open('/')).classes('mt-4').tooltip('Displays the robot selection dialog')      # Returns to the selection page
                        ui.label(self.model.name).classes('text-white text-4xl mt-4')
                    with ui.row().classes(remove='wrap'):
                        with ui.card().classes('filter-none bg-slate-500 text-center items-center my-1'):
                            ui.label('Status').classes('mb-[-0.6em] mt-[-0.4em] text-white')
                            with ui.row().classes(remove='wrap'):
                                power = toggle_button('', icon='power', tooltip='displays if the robot is connected')        # Button to connect the grpc server
                                power.handlePress(True)
                                self.server.on_power_change = power.handlePress
                                def onpowerchange(state):
                                    """connects and disconnects the grpc server"""
                                    if power.pressed:
                                        self.server.connectRobot()
                                    else:
                                        self.server.disconnectRobot()
                                power.onchange = onpowerchange
                                moving = toggle_button('', icon='open_in_full', disable=True, tooltip='displays if the robot is moving')     # Button which displays whether the robot is moving
                                def updateMoving():
                                    """updates the moving button to the current state of the robot"""
                                    moving.handlePress(self.server.moving if not self.server.moving is None else False)
                                self.server.registerUpdateCallback(updateMoving)        # Register a callback to update the moving button
                        with ui.card().classes('filter-none bg-slate-500 text-center items-center my-1'):
                            ui.label('Control').classes('mb-[-0.6em] mt-[-0.4em] text-white')       # Controlls elements for the robot
                            with ui.row().classes(remove='wrap'):
                                def changePrecision(pressed):
                                    """changes the precision mode of the interface"""
                                    self.server.precision = pressed
                                toggle_button('', icon='biotech', on_change=changePrecision, tooltip='Puts the interface in precision mode where the robot only moves 1/10th the usual distance')    # Button to put the Interface in precision mode
                                gripperChangeBtn = toggle_button('', icon='precision_manufacturing', on_change=self.server.changeGripperState, tooltip='Changes the state of the gripper')           # Button to change the state of the gripper
                                self.server.on_gripper_change = gripperChangeBtn.handlePress
                                with ui.dialog() as speedChangeDialog, ui.card().style('width:25%'):        # dialog to change the speed on certain robots
                                    ui.label('Change speed:').classes('text-2xl')
                                    ui.slider(min=0, max=1, step=0.01, value=self.server.speed, on_change=lambda e: speedLabel.set_text(f'Speed: {self.server.speed*100:.1f}%')).bind_value(self.server, 'speed')
                                    speedLabel = ui.label(f'Speed: {self.server.speed*100:.1f}%').classes('mt-[-1em]')
                                    ui.button('Close', on_click=speedChangeDialog.close)
                                ui.button('', icon='speed', on_click=speedChangeDialog.open).tooltip("Opens a dialog to set the robot's speed").classes('px-5 m-[-0.2em] text-white')   # Button to speed change dialog
                                if self.model.isCompliant:          # If the robot has freedrive show button for it
                                    freedriveChangeBtn = toggle_button('', icon='pan_tool', on_change=self.server.changeFreedrive, tooltip='Puts the robot into freedrive')
                                    freedriveChangeBtn.handlePress(self.server.freeDrive)
                                    self.server.on_freedrive_change = freedriveChangeBtn.handlePress
                    if self.model.hasJoints:
                        with ui.card().classes('filter-none bg-slate-500 text-center items-center w-120 my-1'):  # The joints axis controlls
                            with ui.row().classes(remove='wrap'):
                                ui.label('Joints').classes(f'mb-[-1em] mt-[-0.4em] text-white')
                                async def copyToClipboardJoints():
                                    """copies the joints position of the robot to the clipboard"""
                                    content = f'[{", ".join([f"{x:.4f}" for x in self.server.jointsValue])}]'
                                    logger.info(f'copied {content} to clipboard')
                                    await ui.run_javascript(f'navigator.clipboard.writeText("{content}")')
                                ui.button('', on_click=copyToClipboardJoints, icon='content_copy').props('dense').style('font-size:0.85em;justify-content:right;').classes(f'my-[-1em]').tooltip('Copies the joint values as radians to clipboard')     # Copy to clipboard button
                            with ui.row().classes(remove='wrap'):
                                for axis_name, pos in zip(self.model.AxisNames[:self.model.axisCount], self.server.jointsValue if not self.server.jointsValue is None else [0] * self.model.axisCount):
                                    a = axis(self.model, self.server, axis_name, unit=self.model.AxisUnits[self.model.getAxisIndex(axis_name)],on_move=self.server.btnMoveJoints, position = pos)
                                    self.server.registerUpdateCallback(a.updatePosition)        # Registering the nessecary callbacks for updating the position values
                    with ui.card().classes('filter-none bg-slate-500 text-center items-center my-1'):   # The cartesian axis controlls
                        with ui.row().classes(remove='wrap'):  
                            ui.label('Cartesian').classes('mb-[-1em] mt-[-0.4em] text-white')
                            async def copyToClipboardCartesian():
                                """copies the cartesian position of the robot to the clipboard"""
                                content = f'[{", ".join([f"{x:.4f}" for x in np.concatenate(TransformationMatix.fromPose(self.server.robotPose).decomposeNumpy())])}]'
                                logger.info(f'copied {content} to clipboard')
                                ui.clipboard.write(content)
                            ui.button('', on_click=copyToClipboardCartesian, icon='content_copy').props('dense').style('font-size:0.85em;justify-content:right;').classes(f'my-[-1em]').tooltip('Copies the cartesian values as x,y,z and a Quaternion to clipboard')     # Copy to clipboard button
                        with ui.row().classes(remove='wrap'):
                            for axis_name, pos in zip(self.model.AxisNames[self.model.axisCount if self.model.hasJoints else 0:], self.server.Cartesian if not self.server.Cartesian is None else [0] * self.model.axisCount):
                                a = axis(self.model, self.server, axis_name, unit=self.model.AxisUnits[self.model.getAxisIndex(axis_name)], on_move=self.server.btnMoveCartesian, position=pos)
                                self.server.registerUpdateCallback(a.updatePosition)        # Registering the nessecary callbacks for updating the position values
                
                    with ui.card().classes('filter-none bg-slate-500 text-center items-center my-1'):       # Buttons to show the charts or hide the simulation
                        ui.label('Charts and Simulation').classes('mb-[-0.6em] mt-[-0.4em] text-white')
                        with ui.row().classes(remove='wrap'):
                            if self.model.hasJoints:
                                jChartBtn = toggle_button('J', tooltip='Displays a chart with all the joints')
                            XChartBtn = toggle_button('X', tooltip='Displays a chart with all the linear axis')
                            RChartBtn = toggle_button('R', tooltip='Displays a chart with all the rotation axis')
                            AllChartsBtn = toggle_button('', icon='done_all', tooltip='Displays a chart with all joints and axies')
                            SimulationBtn = toggle_button('', icon='view_in_ar', tooltip='Hides the 3D simulation', on_change=self.simulation.changeVisibility)
                            CaptureBtn = toggle_button('', icon='videocam', tooltip='Starts capturing a video stream', on_change=self.simulation.changeCapture)
                            SimulationBtn.handlePress(state=True, suppress=True)
                with ui.column().classes('h-screen'):
                    if self.model.hasJoints:
                        jChart = chart(self.model, self.server,  '', 'Time / s', 'Rotation / °', self.model.AxisNames[:self.model.axisCount])
                    XChart = chart(self.model, self.server, '', 'Time / s', 'Position / m', self.model.AxisNames[self.model.axisCount if self.model.hasJoints else 0:-self.model.rotationAxisCount])
                    RChart = chart(self.model, self.server, '', 'Time / s', 'Rotation / °', self.model.AxisNames[-self.model.rotationAxisCount:])

                with ui.column().classes('min-h-screen flex-1 flex-nowrap overflow-x-auto'):
                    task = self.simulation.renderSimulation()
                    if core.loop and core.loop.is_running():
                        background_tasks.create(task)
                    else:
                        core.app.on_startup(task)

                def allChartsVisible(visible):
                    """makes all charts visible"""
                    if self.model.hasJoints:
                        jChartBtn.handlePress(visible)
                    XChartBtn.handlePress(visible)
                    RChartBtn.handlePress(visible)

                def changeVisibility(chart:chart, *args):
                    self.simulation.resize()
                    chart.changeVisibility(*args)
                XChartBtn.onchange = lambda *args: changeVisibility(XChart, *args)        # Setting up the buttons for the charts
                if self.model.hasJoints:
                    jChartBtn.onchange = lambda *args: changeVisibility(jChart, *args)
                RChartBtn.onchange = lambda *args: changeVisibility(RChart, *args)
                AllChartsBtn.onchange = allChartsVisible
        