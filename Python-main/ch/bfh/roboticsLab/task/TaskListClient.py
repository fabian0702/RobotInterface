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
import sys

from ch.bfh.roboticsLab.task.TaskListManager import TaskListManager

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

class TaskListClient():
  ## Create a new TaskListClient.
  # @param host The host where the robot server is running.
  # @param processSubscription the callback function that will process incoming task list manager publisher messages.
  # It must take one parameter being the subscription stream of Published messages. 
  def __init__(self, host:str, processSubscription) -> None:
    channelOptions = [('grpc.max_receive_message_length', 10_000_000)]
    self.subscriberChannel = grpc.insecure_channel(f'{host}:{PUBLISHER_PORT}', options=channelOptions)
    subscriber = gpbTaskList.PublisherStub(self.subscriberChannel)
    subscription = subscriber.subscribe(pbBase.Subscribe(), wait_for_ready=True)
    self.subscriptionThread = threading.Thread(target=processSubscription, name='TaskListClientSubscriptionThread', args=[subscription])
    self.subscriptionThread.start()

    self.requesterChannel = grpc.insecure_channel(f'{host}:{RESPONDER_PORT}')
    self.requester = gpbTaskList.ResponderStub(self.requesterChannel)

  ## Stop the robot client, closes requester and subscriber channels.
  def stop(self) -> None:
    self.requesterChannel.close()
    self.subscriberChannel.close()
    self.subscriptionThread.join()

  ## Retrieve the requester object to make requests directly.
  # @return requester.
  def getRequester(self) -> gpbTaskList.ResponderStub:
    return self.requester

#
### Tester functions
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

  # Start the client
  logger.info("Creating TaskListClient")
  taskListClient = TaskListClient(host, processSubscription=processSubscription)
  logger.info("Retrieving requester")
  requester = taskListClient.getRequester()
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
      logger.error('Failed to set task:', e)

    logger.info('Executing task...')
    try:
      ack = requester.execute(pbTaskList.Execute(run=True))
      logger.info(f'Got task:\n{ack}')
    except Exception as e:
      logger.error(f'Failed to execute task: {e}')

    while clientKeepWaiting:
      time.sleep(0.1)

  except (KeyboardInterrupt, SystemExit):
    logger.info('Interrupted.')

  logger.info('Client exiting...')


if __name__ == "__main__":
  default_host = 'localhost'
  logger =  Logger("Task List Client Tester").getInstance()

  # Start the Task List server
  server = TaskListManager()

  # Start the Task List client
  client(default_host if len(sys.argv) == 1 else sys.argv[1])

  server.shutdown()
  exit(0)
