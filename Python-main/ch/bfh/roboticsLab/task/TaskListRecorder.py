'''
Copyright (C) BFH roboticsLab
All rights reserved.
'''

import grpc
import os
import sys
import threading
import google.protobuf.json_format as jsonFormat

from ch.bfh.roboticsLab.robot.RobotClient import RobotClient
from ch.bfh.roboticsLab.robot import RobotControl_pb2 as pbRobotControl
from ch.bfh.roboticsLab.task import TaskList_pb2 as pbTaskList
from ch.bfh.roboticsLab.util.Logger import Logger


# Not BUG:  pbRobotControl.MoveCartesian(pose=pose) and pbRobotControl.Gripper(position=position) return empty pb messages
#           => Messages with default values (e.g. 0) are sent empty. This is a protobuf convention.
# Not our BUG:  The floating point precision of jsonFormat.MessageToJson() does not work.
#               => This is a bug in the protobuf json_format library.
# Not BUG:  Gripper message has strange behaviour (access with gripper.position needed to construct pbRobotControl.Gripper)
#           => I'm not sure I understand the problem, but the code below for gripper looks perfectly normal for protobuf.

# TODO: Create proper Teaching Manager with playback
# TODO: Maybe inherit from TaskListManager/TaskListResponder?


class TaskListRecorder(RobotClient):

  # Path where the task list files are stored
  TASK_LIST_FILE_PATH = "./resources/"
  # Extension of the task list files
  TASK_LIST_FILENAME_EXT = ".json"

  def __init__(self, host: str) -> None:
    self.logger = Logger("Recorder").getInstance()
    self.receivedDataLock = threading.Lock()
    super().__init__(host, self._processSubscription)
    self.receivedData = pbRobotControl.Published()
    self._run()
    self.logger.info(f'Stopping recording client')
    self.shutdown()

  def _processSubscription(self, subscription) -> None:
    self.logger.info('Processing subscription')
    try:
      for published in subscription:
        with self.receivedDataLock:
          self.receivedData = published
    except grpc.RpcError as e:
      if e.code() == grpc.StatusCode.CANCELLED:
        self.logger.error('Subscription cancelled')
      else:
        raise
    self.logger.info('Subscription exit')

  def _run(self) -> None:
    self.quit = False
    while not self.quit:
      self.readyToSave = False
      self.tasks = []
      self.taskId = ""
      while not (self.readyToSave or self.quit):
        self._prompt()
      if self.readyToSave:
        taskList = pbTaskList.TaskList(id=pbTaskList.TaskList.Id(id=self.taskId), task=self.tasks)
        self._saveTaskList(taskList)
        print("\nTask list saved")

  def _saveTaskList(self, taskList: pbTaskList.TaskList) -> None:
    json = jsonFormat.MessageToJson(taskList, float_precision=3)
    filename = f'{self.TASK_LIST_FILE_PATH}/{taskList.id.id}{self.TASK_LIST_FILENAME_EXT}'
    if not os.path.exists(self.TASK_LIST_FILE_PATH):
      os.makedirs(self.TASK_LIST_FILE_PATH)
    with open(filename, 'w+') as f:
      f.write(json)

  def _prompt(self) -> None:
    # Get ID for a new task
    if (not self.tasks):
      print("\n####################################")
      print("A new task list will be created")
      print("####################################")
      inputStr = input("Enter task list ID or quit with q:\n")
      if inputStr == 'q':
        self.quit = True
        return
      else:
        self.taskId = inputStr

    # Prompt task
    task = None
    print("\nSelect new task:")
    print("  j: Create P2P movement in joint space to current pose")
    print("  x: Create P2P movement in Cartesian space to current pose")
    print("  g: Record current gripper position")
    print("  w: Add waiting task")
    print("  s: Save task list to JSON file")
    print("  q: Quit recording application")
    inputStr = input()

    # Joint space movements to current joint positions
    if inputStr == 'j':
      with self.receivedDataLock:
        joints = self.receivedData.jointValues
      self.logger.debug(f'j: joints = \n{joints}')
      moveJoints = pbRobotControl.MoveJoints(joints=joints)
      task = pbTaskList.Task(moveJoints=moveJoints)

    # Cartesian space movements to current pose
    elif inputStr == 'x':
      with self.receivedDataLock:
        pose = self.receivedData.robotPose
      self.logger.debug(f'x: pose = \n{pose}')
      moveCartesian = pbRobotControl.MoveCartesian(pose=pose)
      task = pbTaskList.Task(moveCartesian=moveCartesian)

    # Get current gripper position
    elif inputStr == 'g':
      with self.receivedDataLock:
        position = self.receivedData.gripper.position
      self.logger.debug(f'g: gripper position = {position}')
      grip = pbRobotControl.Gripper(position=position)
      task = pbTaskList.Task(grip=grip)

    # Wait
    elif inputStr == 'w':
      duration = float(input("-> Enter duration (s):\n"))
      task = pbTaskList.Task(wait=pbRobotControl.Wait(duration=duration))

    # Save the tasklist
    elif inputStr == 's':
      self.readyToSave = True

    # Quit
    elif inputStr == 'q':
      self.quit = True

    # Invalid input
    else:
      print("Invalid input")

    # Append to task list if valid task was generated
    if task is not None:
      self.tasks.append(task)
      print("Task appended")


if __name__ == '__main__':
  LOG_FORMAT = '%(asctime)-15s %(name)-8s [%(levelname)-6s] %(message)s'
  default_host = 'localhost'
  TaskListRecorder(default_host if len(sys.argv) == 1 else sys.argv[1])
