'''
Copyright (C) BFH roboticsLab
All rights reserved.
'''

import threading
import time
import numpy as np
# import logging
import signal
import grpc
from concurrent import futures
from sys import argv, path
import os
import logging
path.append('../../../../')

from ch.bfh.roboticsLab import Base_pb2 as pbBase
from ch.bfh.roboticsLab.robot import RobotControl_pb2 as pbRobotControl
from ch.bfh.roboticsLab.robot import RobotControl_pb2_grpc as gpbRobotControl
from ch.bfh.roboticsLab.task import TaskList_pb2  as pbTaskList
from ch.bfh.roboticsLab.util.Logger import Logger


# Robot configuration, modify these values for your own application
NUMBER_OF_DIGITAL_INPUTS = 10
NUMBER_OF_ANALOG_INPUTS = 10
NUMBER_OF_DIGITAL_OUTPUTS = 10
NUMBER_OF_ANALOG_OUTPUTS = 10

# The delay between two publisher messages
PUBLISH_INTERVAL_SECONDS = 0.1
# The duration of a simulation step, in seconds
SIMULATION_STEP_DURATION = 0.01
DEFAULT_SPEED_LINEAR = 0.1
DEFAULT_SPEED_ANGULAR = np.deg2rad(90)

# Default publisher port
PUBLISHER_PORT = 40811
# Default responder port
RESPONDER_PORT = 40812


class Robot(gpbRobotControl.ResponderServicer, gpbRobotControl.PublisherServicer):
  '''! Class managing the simulation robot state. '''

  def __init__(self, numberOfJoints:int):
    '''! Create a new Robot object.
    @param numberOfJoints Number of joints that this robot shall simulate.
    '''
    self.logger : logging.Logger = Logger(os.path.basename(__file__)).getInstance()
    self.logger.info(f'Starting RobotSimulator with {numberOfJoints} joints')

    # Initial robot state. Values are stored as regular Python types or
    # as Protobuf objects, whichever is more convenient.
    # TODO Consolidate.

    # Flag to indicate whether the simulation should be stopped
    self.stoppingSimulation = False
    self.commandReady = threading.Event()
    self.commandInterrupt = threading.Event()
    self.command = pbTaskList.Task(stop=pbRobotControl.Stop())
    # Robot simulation thread
    self.thread = None
    # The current simulation time, in seconds
    self.time = 0.0

    # Whether the robot is moving. True while a long command is being processed.
    self.moving = False
    # Position of each joint
    self.jointValues = np.array([0.0 for i in range(numberOfJoints)])
    # Robot pose (position (x,y,z) and orientation (quaternion x,y,z,w))
    self.pose = pbBase.Pose(position=pbBase.Position(x=0, y=0, z=0), orientation=pbBase.Quaternion(qw=1, qx=0, qy=0, qz=0))
    # Reference frame for each joint
    self.jointFrames = [pbBase.Pose() for i in range(numberOfJoints)]
    # Gripper position (between 0 and 1)
    self.gripperPosition = 0.0
    # Whether free drive is enabled
    self.inFreeDrive = False
    # Digital inputs
    self.digitalInputs = [False for i in range(NUMBER_OF_DIGITAL_OUTPUTS)]
    # Analog inputs
    self.analogInputs = [0.0 for i in range(NUMBER_OF_ANALOG_INPUTS)]
    # Digital outputs
    self.digitalOutputs = [False for i in range(NUMBER_OF_DIGITAL_OUTPUTS)]
    # Analog outputs
    self.analogOutputs = [0.0 for i in range(NUMBER_OF_ANALOG_OUTPUTS)]

    self.startThread()

    # Start gRPC servers
    self.publisherServer = grpc.server(futures.ThreadPoolExecutor(max_workers=10))
    gpbRobotControl.add_PublisherServicer_to_server(self, self.publisherServer)
    self.publisherServer.add_insecure_port(f'[::]:{PUBLISHER_PORT}')
    self.publisherServer.start()

    self.responderServer = grpc.server(futures.ThreadPoolExecutor(max_workers=10))
    gpbRobotControl.add_ResponderServicer_to_server(self, self.responderServer)
    self.responderServer.add_insecure_port(f'[::]:{RESPONDER_PORT}')
    self.responderServer.start()

    self.logger.info('Ready')


  def startThread(self):
    '''! Start the robot simulation thread'''
    self.stoppingSimulation = False
    self.thread = threading.Thread(target=self._executionLoop)
    self.thread.start()

  def stopThread(self):
    '''! Stop the robot simulation thread'''
    if self.thread:
      self.stoppingSimulation = True
      self._interruptCommandAndWait()
      self.thread.join()
      self.thread = None
      self.logger.debug('Thread finished')

  def shutdown(self):
    '''! Shut down the robot simulation'''
    self.logger.debug('shutdown')
    self.command = pbTaskList.Task(stop=pbRobotControl.Stop())
    self.commandReady.set()
    self.stopThread()

  def subscribe(self, request, context):
    '''! Implements service `subscribe` on the gRPC publisher service.'''
    self.logger.info('Subscription received, starting streaming')
    while context.is_active() and not self.stoppingSimulation:
      try:
        time.sleep(PUBLISH_INTERVAL_SECONDS)
        published = pbRobotControl.Published(
          state = pbRobotControl.State(connected=True, moving=self.moving),
          jointValues = pbBase.ArrayDouble(value=self.jointValues),
          robotPose = self.pose,
          gripper = pbRobotControl.Gripper(position=self.gripperPosition)
        )
        yield published
      except Exception as e:
        self.logger.warning(f'Failed to send message: {e}')
        break
    self.logger.info('Streaming end')

  def _executionLoop(self):
    '''! Simulate commands that take time to complete'''
    while True:
      if self.stoppingSimulation:
        break
      # Block until a command is ready for execution
      self.commandReady.wait()

      self.logger.debug(f'_executionLoop: executing command:\n{self.command}')
      if self.command.HasField('moveJoints'):
        if self.command.moveJoints.HasField('joints'):
          goal = np.array(self.command.moveJoints.joints.value)
        else:
          self.logger.warning('_executionLoop: moveJoints pose goal not yet implemented')
        self.commandInterrupt.clear()
        self.logger.debug(f'_executionLoop: goal: {goal}')
        self.logger.debug(f'_executionLoop: self.jointValues: {self.jointValues}')
        motionDelta = goal - self.jointValues
        self.logger.debug(f'_executionLoop: motionDelta: {motionDelta}')
        angularSpeed = self.command.moveJoints.speed.angular
        if angularSpeed == 0.0:
          angularSpeed = DEFAULT_SPEED_ANGULAR
        self.logger.debug(f'_executionLoop: angularSpeed: {angularSpeed}')
        largestStep = angularSpeed * SIMULATION_STEP_DURATION
        self.logger.debug(f'_executionLoop: largestStep: {largestStep}')
        maxDelta = np.max(np.abs(motionDelta))
        self.logger.debug(f'_executionLoop: maxDelta: {maxDelta}')
        nSteps = abs(int(maxDelta/largestStep))
        self.logger.debug(f'_executionLoop: nSteps: {nSteps}')
        if nSteps > 0:
          stepSize = motionDelta/nSteps
          self.logger.debug(f'_executionLoop: stepSize: {stepSize}')
          motionTolerance = self.command.moveJoints.tolerance.angular
          if motionTolerance == 0.0:
            motionTolerance = np.max(np.abs(stepSize) - 1e-6)
          self.logger.debug(f'_executionLoop: motionTolerance: {motionTolerance}')
          self.moving = True
          while not self.commandInterrupt.is_set():
            if np.all(np.abs(goal - self.jointValues) < motionTolerance):
              break
            self.jointValues += stepSize
            time.sleep(SIMULATION_STEP_DURATION)
          self.moving = False
        self.logger.debug(f'_executionLoop: MoveJoint done: {self.jointValues}')
      elif self.command.HasField('moveCartesian'):
        # TODO Deal with orientations
        frame = self.command.moveCartesian.pose.frame
        p = self.command.moveCartesian.pose.position
        currentPosition = np.array([self.pose.position.x, self.pose.position.y, self.pose.position.z])
        if frame is pbBase.Pose.Frame.ROBOT_FLANGE:
          # Is pose is in robot flange frame assume a relative motion, but x, y, z match the base frame.
          goalPosition = currentPosition + np.array([p.x, p.y, p.z])
        else:
          goalPosition = np.array([p.x, p.y, p.z])
        self.commandInterrupt.clear()
        self.logger.debug(f'_executionLoop: goalPosition: {goalPosition}')
        self.logger.debug(f'_executionLoop: currentPosition: {currentPosition}')
        motionDelta = goalPosition - currentPosition
        self.logger.debug(f'_executionLoop: motionDelta: {motionDelta}')
        linearSpeed = self.command.moveCartesian.speed.linear
        if linearSpeed == 0.0:
          linearSpeed = DEFAULT_SPEED_LINEAR
        self.logger.debug(f'_executionLoop: linearSpeed: {linearSpeed}')
        largestStep = linearSpeed * SIMULATION_STEP_DURATION
        self.logger.debug(f'_executionLoop: largestStep: {largestStep}')
        maxDelta = np.max(np.abs(motionDelta))
        self.logger.debug(f'_executionLoop: maxDelta: {maxDelta}')
        nSteps = abs(int(maxDelta/largestStep))
        self.logger.debug(f'_executionLoop: nSteps: {nSteps}')
        if nSteps > 0:
          stepSize = motionDelta/nSteps
          self.logger.debug(f'_executionLoop: stepSize: {stepSize}')
          motionTolerance = self.command.moveCartesian.tolerance.linear
          if motionTolerance == 0.0:
            motionTolerance = np.max(np.abs(stepSize) - 1e-6)
          self.logger.debug(f'_executionLoop: motionTolerance: {motionTolerance}')
          self.moving = True
          while not self.commandInterrupt.is_set():
            currentPosition = np.array([self.pose.position.x, self.pose.position.y, self.pose.position.z])
            delta = goalPosition - currentPosition
            self.logger.debug(f'_executionLoop: delta: {delta}')
            if np.all(np.abs(goalPosition - currentPosition) < motionTolerance):
              break
            currentPosition += stepSize
            self.pose.position.x = currentPosition[0]
            self.pose.position.y = currentPosition[1]
            self.pose.position.z = currentPosition[2]
            time.sleep(SIMULATION_STEP_DURATION)
          self.moving = False
        self.logger.debug(f'_executionLoop: MoveCartesian done: {self.pose}')
      elif self.command.HasField('grip'):
        self.gripperPosition = self.command.grip.position
      elif self.command.HasField('free'):
        self.logger.debug(f'Freedrive: {self.command.digitalOutput}')
      elif self.command.HasField('digitalOutput'):
        self.logger.debug(f'DigitalOutputs: {self.command.digitalOutput}')

      # Allow a new command to be set
      self.commandReady.clear()

  def _waitForCommandToFinish(self):
    '''! Waits for a command to complete.'''
    while self.commandReady.is_set():
      time.sleep(SIMULATION_STEP_DURATION)
  
  def _interruptCommandAndWait(self):
    '''! Interrupt a running command.'''
    if self.commandReady.is_set():
      self.commandInterrupt.set()
    self._waitForCommandToFinish()

  ### Implement Responder services

  def setRobotConnection(self, request, context) -> pbBase.Acknowledge:
    self.logger.debug(f'setRobotConnection:\n{request}')
    return pbBase.Acknowledge()

  def stop(self, request, context) -> pbBase.Acknowledge:
    self.logger.debug('stop')
    # Note blocking
    self._interruptCommandAndWait()
    return pbBase.Acknowledge()
  
  def wait(self, request, context) -> pbBase.Acknowledge:
    self.logger.debug(f'wait:\n{request}')
    # Note blocking
    self._interruptCommandAndWait()
    time.sleep(request.duration)
    return pbBase.Acknowledge()

  def moveJoints(self, request, context) -> pbBase.Acknowledge:
    self.logger.debug(f'moveJoints:\n{request}')
    self._interruptCommandAndWait()
    self.command = pbTaskList.Task(moveJoints=request)
    self.commandReady.set()
    return pbBase.Acknowledge()

  def moveCartesian(self, request, context) -> pbBase.Acknowledge:
    self.logger.debug(f'moveCartesian:\n{request}')
    self._interruptCommandAndWait()
    self.command = pbTaskList.Task(moveCartesian=request)
    self.commandReady.set()
    return pbBase.Acknowledge()

  def gripper(self, request, context) -> pbBase.Acknowledge:
    self.logger.debug(f'gripper:\n{request}')
    self._interruptCommandAndWait()
    self.command = pbTaskList.Task(grip=request)
    self.commandReady.set()
    return pbBase.Acknowledge()

  def digitalOut(self, request, context) -> pbBase.Acknowledge:
    self.logger.debug(f'digitalOutput:\n{request}')
    self._interruptCommandAndWait()
    self.command = pbTaskList.Task(digitalOutput=request)
    self.commandReady.set()
    return pbBase.Acknowledge()

  # TODO Implement the rest of the services

  # def setFreedrive(self, value):
  #   self.inFreeDrive = value
  #   self.logger.debug('Freedrive = {}'.format(value))

  # def setAnalogOutput(self, index, value):
  #   self.analogOutputs[index] = value
  #   self.logger.debug('AnalogOutputs[{}] = {:.2f}'.format(index, value))

if __name__ == "__main__":
  signal.signal(signal.SIGTERM, lambda signum, frame: exit())
  
  # Start the robot server
  numberOfJoints = 6 if len(argv) == 1 else int(argv[1])
  server = Robot(numberOfJoints)

  try:
    while True:
      time.sleep(10000)
  except (KeyboardInterrupt, SystemExit):
    print('Interrupted.')

  server.shutdown()
  exit(0)
