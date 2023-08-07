'''
Copyright (C) BFH roboticsLab
All rights reserved.
'''

import grpc
from concurrent import futures
import threading
import signal
import numpy as np
import time
import logging
import sys
import glob

import google.protobuf.json_format as jsonFormat

from ch.bfh.roboticsLab.robot.RobotClient import RobotClient

from ch.bfh.roboticsLab import Base_pb2 as pbBase
from ch.bfh.roboticsLab.robot import RobotControl_pb2 as pbRobotControl
from ch.bfh.roboticsLab.task import TaskList_pb2 as pbTaskList
from ch.bfh.roboticsLab.task import TaskList_pb2_grpc as gpbTaskList
from ch.bfh.roboticsLab.util.Logger import Logger

# /** Default publisher port */
PUBLISHER_PORT = 40821
# /** Default responder port */
RESPONDER_PORT = 40822

LOG_FORMAT = '%(asctime)-15s %(name)-8s [%(levelname)-6s] %(message)s'

## Utility task to create a pretty string of a task list
# @param task The TaskList object to convert to string
def taskToString(task:pbTaskList.TaskList) -> str:
  s = "%s" % (task)
  return ' '.join(s.split())


## This class implements a TaskList executor.
class TaskListExecutor():
  ## Create a new TaskListExecutor.
  # @param robotClient A RobotClient object. If not given a new RobotClient that connects to localhost will be created.
  def __init__(self, robotClient=None) -> None:
    self.logger = Logger("TaskListExecutor").getInstance()
    if robotClient is None:
      self.robotClient = RobotClient(host='localhost', processSubscription=self.processRobotSubscription)
    else:
      self.robotClient = robotClient
    self.keepRunning = False
    self.motionStartEvent = threading.Event()
    self.motionStopEvent = threading.Event()
    self.shutdownRequested = False
    self.taskList = pbTaskList.TaskList()
    self.count = 0
    self.pointer = 0
    self.detailPointer = 0
    self.currentTask = None
    self.thread = None
    self.logger.info('Ready')

  def start(self) -> None:
    self.logger.info('Start')
    if not self.isRunning():
      self.keepRunning = True
      self.thread = threading.Thread(target=self.execute)
      self.thread.start()

  ## Stop execution of tasks.
  def stop(self) -> None:
    self.logger.info('Stop')
    self.robotClient.requester.stop(pbRobotControl.Stop())
    if self.isRunning():
      self.keepRunning = False
      self.thread.join()
      self.thread = None

  ## Retrieves the running state of the executor.
  # @return True if a task list is currently being executed.
  def isRunning(self) -> bool:
    return self.currentTask is not None and self.keepRunning is True

  ## Shutdown the executor service.
  def shutdown(self) -> None:
    self.shutdownRequested = True
    self.robotClient.shutdown()

  ## Set the task list to execute.
  # @param taskList a protobuf TaskList.
  def set(self, taskList: pbTaskList.TaskList) -> None:
    self.taskList = taskList
    self.count = len(self.taskList.task)
    self.logger.debug(f'set: list "{taskList.id.id}" with {self.count} tasks')
  
  ## Retrieve the execution state of the executor.
  # @return a protobuf Execution message.
  def getExecutionState(self) -> pbTaskList.Execution:
    state = pbTaskList.Execution.State.IDLE
    if self.isRunning():
      state = pbTaskList.Execution.State.RUNNING
    # TODO Set state = pbTaskList.TaskList.ERROR when needed
    execution = pbTaskList.Execution(
      state = state,
      count = self.count,
      pointer = self.pointer,
      detailPointer = self.detailPointer,
      task = self.currentTask
    )
    return execution

  ## Execute the current task list.
  # Runs on a separate thread
  # @see #set.
  def execute(self) -> None:
    self.logger.debug(f'execute: Executing task list "{self.taskList.id.id}" with {self.count} tasks')
    self.keepRunning = True
    self.pointer = 0
    self.detailPointer = 0
    robot = self.robotClient.getRequester()
    for task in self.taskList.task:
      if not self.keepRunning:
        self.logger.debug(f'execute: Stop requested...')
        break
      self.logger.debug(f'execute: Executing task {taskToString(task)}')
      self.currentTask = task
      self.pointer += 1
      if task.HasField("stop"):
        ack = robot.stop(task.stop)
      elif task.HasField("wait"):
        time.sleep(task.wait.duration)
      elif task.HasField("grip"):
        ack = robot.gripper(task.grip)
      elif task.HasField("moveJoints"):
        self.motionStartEvent.clear()
        ack = robot.moveJoints(task.moveJoints)
        if ack.code is pbBase.Acknowledge.Code.MOTION_COMMAND_REFUSED_ALREADY_AT_GOAL:
          continue
        self.motionStartEvent.wait()
        self.motionStopEvent.wait()
      elif task.HasField("moveCartesian"):
        self.motionStartEvent.clear()
        ack = robot.moveCartesian(task.moveCartesian)
        self.motionStartEvent.wait()
        self.motionStopEvent.wait()
      elif task.HasField("digitalOutput"):
        ack = robot.digitalOut(task.digitalOutput)
        time.sleep(0.05)

    self.keepRunning = False
    self.currentTask = None
    self.logger.info('Done executing')

  ## Process incoming published data from robot server.
  # @param subscription Stream of published messages.
  def processRobotSubscription(self, published) -> None:
    if not self.motionStartEvent.is_set() and published.state.moving:
      self.logger.debug('motionStartEvent not set and robot moving: setting motionStartEvent')
      self.motionStartEvent.set()
      self.motionStopEvent.clear()
    if not self.motionStopEvent.is_set() and not published.state.moving:
      self.logger.debug('motionStopEvent not set and robot not moving: setting motionStopEvent')
      self.motionStopEvent.set()


## This class implements the Task List Publisher
class TaskListPublisher(gpbTaskList.PublisherServicer):
  ## Construcs a new Publisher
  def __init__(self, executor: TaskListExecutor) -> None:
    super().__init__()
    self.logger = Logger("TaskListPublisher").getInstance()
    self.executor = executor
    self.stoppingPublisher = False
    self.logger.info('Ready')

  ## Shutdown the publisher
  def shutdown(self) -> None:
    self.logger.info('Shutdown')
    self.stoppingPublisher = True

  ## Implements service `subscribe` on the gRPC publisher service.
  def subscribe(self, request, context):
    self.logger.info('Subscription received, starting streaming')
    while context.is_active() and not self.stoppingPublisher:
      try:
        time.sleep(0.5)
        execution = self.executor.getExecutionState()
        yield execution
      except Exception as e:
        self.logger.warning(f'Failed to send message: {e}')
        break
    self.logger.info('Streaming end')


## This class implements the TaskList responder services
class TaskListResponder(gpbTaskList.ResponderServicer):

  # Path where the task list files are stored
  TASK_LIST_FILE_PATH = "./resources/";
  # Extension of the task list files
  TASK_LIST_FILENAME_EXT = ".json";

  ## Construcs a new ResponderTaskList.
  # @param executor A TaskListExecutor object.
  def __init__(self, executor: TaskListExecutor) -> None:
    super().__init__()
    self.logger = Logger("TaskListResponder").getInstance()
    self.executor = executor
    self.taskList = pbTaskList.TaskList()
    self.logger.info('Ready')

  # Shut down the service.
  def shutdown(self) -> None:
    self.logger.info('Shutdown')

  ## Retrieve the names of all existing task list files.
  # @return List of filenames
  def _getTaskListFileNames(self):
    res = glob.glob(f'{self.TASK_LIST_FILE_PATH}/*{self.TASK_LIST_FILENAME_EXT}')
    return res

  ## Sets the task list ready for execution
  # @param taskList TaskList to set
  # @throws Exception if a task list is currently executing.
  def _setTaskList(self, taskList: pbTaskList.TaskList) -> None:
    if self.executor.isRunning():
      raise Exception(f'_setTaskList: A TaskList is executing, stop it first: {self.executor.taskList.id.id} ({self.executor.pointer}/{self.executor.count})')
    self.taskList = taskList
    self.logger.debug(f'_setTaskList: list "{taskList.id.id}"')
    self.executor.set(taskList=taskList)

  ### Implements TaskList Responder gRPC services ###

  def newList(self, request, context) -> pbBase.Acknowledge:
    self.taskList = pbTaskList.TaskList(id=pbTaskList.TaskList.Id(id='noname'))
    return pbBase.Acknowledge()

  def save(self, request, context) -> pbBase.Acknowledge:
    newID = request.id.id
    if newID != "":
      self.taskList.id.id = newID
    else:
      self.logger.info(f'save: No Task ID given, using default id: "{self.taskList.id.id}"')
    # TODO Add error handling
    json = jsonFormat.MessageToJson(self.taskList, float_precision=3)
    filename = f'{self.TASK_LIST_FILE_PATH}/{self.taskList.id.id}{self.TASK_LIST_FILENAME_EXT}'
    with open(filename, 'w+') as f:
      f.write(json)
    return pbBase.Acknowledge()

  def load(self, request, context) -> pbBase.Acknowledge:
    res = self._getTaskListFileNames()
    for file in res:
      with open(file, 'r') as f:
        json = f.read()
        taskList = jsonFormat.Parse(json, pbTaskList.TaskList())
        if taskList.id == request.id:
          try:
            self.logger.debug(f'Loading task list "{request.id.id}"')
            self._setTaskList(taskList)
          except Exception as e:
            return pbBase.Acknowledge(code=pbBase.Acknowledge.Code.ERROR, message=f'Failed to set task with ID {request.id}: {e}')
          return pbBase.Acknowledge()
    return pbBase.Acknowledge(code=pbBase.Acknowledge.Code.ERROR, message=f'TaskList ID "{request.id}" not found.')

  def set(self, request, context) -> pbBase.Acknowledge:
    self.logger.debug(f'set: list "{request.id.id}"')
    try:
      self._setTaskList(request)
    except Exception as e:
      return pbBase.Acknowledge(code=pbBase.Acknowledge.Code.ERROR, message=f'Failed to set task list: {e}')
    return pbBase.Acknowledge()

  def getAvailableTaskLists(self, request, context) -> pbTaskList.AvailableTaskLists:
    res = self._getTaskListFileNames()
    availableTaskLists = pbTaskList.AvailableTaskLists()
    # Get TaskList ID per file
    for file in res:
      with open(file, 'r') as f:
        json = f.read()
        taskList = jsonFormat.Parse(json, pbTaskList.TaskList())
        availableTaskLists.ids.append(taskList.id)
    return availableTaskLists

  def execute(self, request, context) -> pbBase.Acknowledge:
    # TODO check error handling
    if request.run:
      self.logger.info(f'Executing task list "{self.taskList.id.id}"')
      self.executor.start()
    else:
      self.logger.info("Stopping execution")
      self.executor.stop()
    time.sleep(0.1) # Short delay to allow task to start
    return pbBase.Acknowledge()

  def getTaskList(self, request, context) -> pbTaskList.TaskList:
    return self.taskList

## This class defines a task list manager starting responder and subscriber services.
class TaskListManager(gpbTaskList.ResponderServicer):
  ## Create a new TaskListManager.
  def __init__(self, robotClient=None) -> None:
    self.logger = Logger("TaskListManager").getInstance()

    self.executor = TaskListExecutor(robotClient)

    self.publisher = TaskListPublisher(executor=self.executor)
    self.publisherServer = grpc.server(futures.ThreadPoolExecutor(max_workers=10))
    gpbTaskList.add_PublisherServicer_to_server(self.publisher, self.publisherServer)
    self.publisherServer.add_insecure_port(f'[::]:{PUBLISHER_PORT}')
    self.publisherServer.start()

    self.responder = TaskListResponder(executor=self.executor)
    self.responderServer = grpc.server(futures.ThreadPoolExecutor(max_workers=10))
    gpbTaskList.add_ResponderServicer_to_server(self.responder, self.responderServer)
    self.responderServer.add_insecure_port(f'[::]:{RESPONDER_PORT}')
    self.responderServer.start()
    self.logger.info('Ready')

  ## Shutdown the services.
  def shutdown(self) -> None:
    self.logger.info('Stopping')
    self.responder.shutdown()
    self.publisher.shutdown()
    self.responderServer.stop(1)
    self.publisherServer.stop(1)
    self.executor.shutdown()

#
### Test section ###
#

clientKeepWaiting = True
## Process the subscription to the publisher
def processSubscription(subscription) -> None:
  global clientKeepWaiting
  try:
    for state in subscription:
      logger.info(f'Got execution message: {pbTaskList.Execution.State.Name(state.state)} ({state.pointer}/{state.count})')
      if state.state is pbTaskList.Execution.State.IDLE:
        clientKeepWaiting = False
        break
  except grpc.RpcError as e:
    if e.code() == grpc.StatusCode.CANCELLED:
      logger.error('Task list subscription cancelled')
    else:
      raise
  logger.info('Task list subscription exit')

## Check an acknowledge response and logger.info information to console
def checkAcknowledge(ack: pbBase.Acknowledge) -> None:
  if ack.code:
    logger.info(f'Got acknowledge error:\n{ack}')
  else:
    logger.info('Acknowledge OK')

## Start the Task list client
# @param host The IP address of the robot server
def client(host) -> None:
  global clientKeepWaiting
  signal.signal(signal.SIGTERM, lambda signum, frame: exit())

  channelOptions = [('grpc.max_receive_message_length', 10_000_000)]

  # Connect to the task list responder and publisher
  with grpc.insecure_channel(f'{host}:{RESPONDER_PORT}') as responderChannel, \
          grpc.insecure_channel(f'{host}:{PUBLISHER_PORT}', options=channelOptions) as publisherChannel:

    subscriber = gpbTaskList.PublisherStub(publisherChannel)
    subscription = subscriber.subscribe(pbBase.Subscribe(), wait_for_ready=True)
    subscriptionThread = threading.Thread(target=processSubscription, name='TaskListManagerSubscriptionThread', args=[subscription])
    subscriptionThread.start()

    requester = gpbTaskList.ResponderStub(responderChannel)
    logger.info('Connected to task list responder, testing...')

    RAD = np.pi / 180.
    joints1 = [180 * RAD, 90 * RAD, 80 * RAD, 20 * RAD, 190 * RAD, 300 * RAD, 50 * RAD]
    joints2 = [x + 1 * RAD for x in joints1]

    try:
      tasks = [
          pbTaskList.Task(moveJoints=pbRobotControl.MoveJoints(joints=pbBase.ArrayDouble(value=joints1))),
          pbTaskList.Task(wait=pbRobotControl.Wait(duration=1.0)),
          pbTaskList.Task(moveJoints=pbRobotControl.MoveJoints(joints=pbBase.ArrayDouble(value=joints2))),
      ]

      taskList = pbTaskList.TaskList(id=pbTaskList.TaskList.Id(id="test"), task=tasks)
      logger.info(f'Sending task list: {taskList}')
      try:
        ack = requester.set(taskList)
        checkAcknowledge(ack)
      except Exception as e:
        logger.error(f'Failed to set task: {e}')

      logger.info('Executing task...')
      try:
        ack = requester.execute(pbTaskList.Execute(run=True))
        logger.info(f'Got task:\n{ack}')
      except Exception as e:
        logger.error(f'Failed to execute task: {e}')

      logger.info('Saving task list...')
      try:
        ack = requester.save(pbTaskList.Save(id=pbTaskList.TaskList.Id(id='myTaskList')))
        if ack.code:
          logger.error(f'Error saving task list: {ack.message}')
      except Exception as e:
        logger.error(f'Failed to save task list: {e}')

      while clientKeepWaiting:
        time.sleep(0.1)

    except (KeyboardInterrupt, SystemExit):
      logger.info('Interrupted.')

    logger.info('Client exiting...')
    responderChannel.close()
    publisherChannel.close()
    subscriptionThread.join()


if __name__ == "__main__":
  default_host = 'localhost'
  logging.basicConfig(level=logging.DEBUG, format=LOG_FORMAT)
  logger = Logger("Task List Client").getInstance()

  # Start the Task List server
  server = TaskListManager()

  # Start the Task List client
  client(default_host if len(sys.argv) == 1 else sys.argv[1])

  server.shutdown()
  exit(0)
