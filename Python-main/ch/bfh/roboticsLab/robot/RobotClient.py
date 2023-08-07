'''
Copyright (C) BFH roboticsLab
All rights reserved.
'''

import grpc
import threading
import numpy as np
import os
from typing import Callable
import logging

from ch.bfh.roboticsLab import Base_pb2 as pbBase
from ch.bfh.roboticsLab.robot import RobotControl_pb2 as pbRobotControl
from ch.bfh.roboticsLab.robot import RobotControl_pb2_grpc as gpbRobotControl
from ch.bfh.roboticsLab.util.Logger import Logger


# Default publisher port
PUBLISHER_PORT = 40811
# Default responder port
RESPONDER_PORT = 40812



def processSubscription(published:pbRobotControl.Published) -> None:
  '''! Empty, default process subscription callback.
  You may write your own subscription callback function with the same signarture as this method.
  @see Subscriber
  '''
  pass

class RobotClient():
  '''! This class implements a Robot Client.
  Usage example:
  @code{python}
    # Define your own subscription processing callback.
    def myProcessSubscription(published:pbRobotControl.Published) -> None:
      print(f'myProcessSubscription: Got: {published}')
      # Here you could process for example if the robot is moving or not,
      # so you do not need to poll like in the `waitForRobotMotion` function below.
    
    def waitForRobotMotion(client: RobotClient) -> None:
      # Poll the client to wait for the robot to stop moving.
      # This method is not recommended, better use the subscription callback above.
      time.sleep(0.5)
      while client.isMoving():
        time.sleep(0.5)

    print("Create RobotClient running on localhost (probably a simulator)")
    client = RobotClient('localhost', myProcessSubscription)

    print("moveJoints to joint-space goal")
    client.moveJoints(jointsOrPose=pbBase.ArrayDouble(value=[0,0,0,0,0,0]))
    waitForRobotMotion(client=client)

    print("moveJoints to Cartesian pose")
    client.moveJoints(jointsOrPose=pbBase.Pose(position=pbBase.Position(x=0.1,y=0.2,z=0.3), orientation=pbBase.Quaternion(qw=1,qx=0,qy=0,qz=0)))
    waitForRobotMotion(client=client)

    print("moveCartesian")
    client.moveCartesian(pose=pbBase.Pose(position=pbBase.Position(x=1,y=2,z=3), orientation=pbBase.Quaternion(qw=1,qx=0,qy=0,qz=0)))
    waitForRobotMotion(client=client)

    print("close gripper)
    client.gripper(position=1.0)

    client.shutdown()
    exit(0)
  @endcode
  '''

  def __init__(self, host: str, processSubscription:Callable[[pbRobotControl.Published], None]=processSubscription):
    '''! Create a new RobotClient.
      @param host The host where the robot server is running.
      @param processSubscription An optional callback function to process incoming robot publisher messages.
    '''
    self.logger: logging.Logger = Logger(os.path.basename(__file__)).getInstance()
    self.ipAddress: str = host

    self.callback = processSubscription
    channelOptions = [('grpc.max_receive_message_length', 10_000_000)]
    self.logger.info(f'Connecting to robot publisher on {host}:{PUBLISHER_PORT}')
    self.subscriberChannel = grpc.insecure_channel(f'{host}:{PUBLISHER_PORT}', options=channelOptions)
    self.subscriber = gpbRobotControl.PublisherStub(self.subscriberChannel)
    subscription = self._subscribe()

    # Variables to save incoming subscription data
    self.published = pbRobotControl.Published()
    self.publishedLock = threading.Lock()
    self.jointsValue: pbBase.ArrayDouble = pbBase.ArrayDouble()
    self.jointsLock = threading.Lock()
    self.robotPose: pbBase.Pose = pbBase.Pose()
    self.robotPoseLock = threading.Lock()
    self.moving: bool = False
    self.movingLock = threading.Lock()

    self.subscriptionThread = threading.Thread(target=self._processSubscription, name='RobotClientSubscriptionThread', args=[subscription])
    self.subscriptionThread.start()


    self.logger.info(f'Connecting to robot responder on {host}:{RESPONDER_PORT}')
    self.requesterChannel = grpc.insecure_channel(f'{host}:{RESPONDER_PORT}')
    self.requester = gpbRobotControl.ResponderStub(self.requesterChannel)

  def shutdown(self) -> None:
    '''! Stop the robot client, closes requester and subscriber channels.'''
    self.subscriberChannel.close()
    self.subscriptionThread.join()
    self.requesterChannel.close()

  def stop(self) -> None:
    '''! Request stop robot motion.
    @throw Exception If an error happens while talking to the responder.
    '''
    try:
      self._checkResult(self.requester.stop(pbRobotControl.Stop()))
    except Exception as e:
      raise Exception('stop: ', e)

  def moveJoints(self, jointsOrPose:pbBase.ArrayDouble|pbBase.Pose, speed:pbBase.LinearAngularPair=pbBase.LinearAngularPair(), tolerance:pbBase.LinearAngularPair=pbBase.LinearAngularPair(), override:float=1.0) -> None:
    '''! Request motion in joint space to joint-value goal.
    @param joints Desired joints values [m|rad].
    @param speed Linear and angular desired speeds for the motion [m/s, rad/s]. Defaults to the robot's default speed.
    @param tolerance Linear and angular tolerance values [m, rad]. Depending on the robot, these values may be interpreted as blending percentage and should then be defined between 0 and 1. Defaults to the robot's default motion tolerance.
    @param override Speed override value. Slows down the motion's speed [0..1]. Defaults to 1.
    @throw Exception If an error happens while talking to the responder.
    '''
    try:
      match type(jointsOrPose):
        case pbBase.ArrayDouble:
          self._checkResult(self.requester.moveJoints(pbRobotControl.MoveJoints(joints=jointsOrPose, speed=speed, override=override, tolerance=tolerance)))
        case pbBase.Pose:
          self._checkResult(self.requester.moveJoints(pbRobotControl.MoveJoints(pose=jointsOrPose, speed=speed, override=override, tolerance=tolerance)))
    except Exception as e:
      raise Exception('Requester.moveJoints: ', e)

  def moveJointsToCartesianGoal(self, pose:pbBase.Pose, speed:pbBase.LinearAngularPair=pbBase.LinearAngularPair(), tolerance:pbBase.LinearAngularPair=pbBase.LinearAngularPair(), override:float=1.0) -> None:
    '''! Request motion in joint space to a Cartesian goal.
    @param pose Desired Cartesian pose [m, rad].
    @param speed Linear and angular desired speeds for the motion [m/s, rad/s]. Defaults to the robot's default speed.
    @param tolerance Linear and angular tolerance values [m, rad]. Depending on the robot, these values may be interpreted as blending percentage and should then be defined between 0 and 1. Defaults to the robot's default motion tolerance.
    @param override Speed override value. Slows down the motion's speed [0..1]. Defaults to 1.
    @throw Exception If an error happens while talking to the responder.
    '''
    try:
      self._checkResult(self.requester.moveJoints(pbRobotControl.MoveJoints(pose=pose, speed=speed, override=override, tolerance=tolerance)))
    except Exception as e:
      raise Exception('Requester.moveJoints: ', e)

  def moveCartesian(self, pose:pbBase.Pose, speed:pbBase.LinearAngularPair=pbBase.LinearAngularPair(), tolerance:pbBase.LinearAngularPair=pbBase.LinearAngularPair(), override:float=1.0) -> None:
    '''! Request motion in Cartesian space.
    @param pose Desired Cartesian pose [m, rad].
    @param speed Linear and angular desired speeds for the motion [m/s, rad/s]. Defaults to the robot's default speed.
    @param tolerance Linear and angular tolerance values [m, rad]. Depending on the robot, these values may be interpreted as blending percentage and should then be defined between 0 and 1. Defaults to the robot's default motion tolerance.
    @param override Speed override value. Slows down the motion's speed [0..1]. Defaults to 1.
    @throw Exception If an error happens while talking to the responder.
    '''
    try:
      self._checkResult(self.requester.moveCartesian(pbRobotControl.MoveCartesian(pose=pose, speed=speed, override=override, tolerance=tolerance)))
    except Exception as e:
      raise Exception('Requester.moveCartesian: ', e)

  def gripper(self, position: float) -> None:
    '''
    Request gripper motion.
    @param position Desired gripper position [0..1]. 0 is fully open, 1 is fully closed.
    @throw Exception If an error happens while talking to the responder.
    '''
    try:
      position = np.clip(position, 0.0, 1.0)
      self._checkResult(self.requester.gripper(pbRobotControl.Gripper(position=position)))
    except Exception as e:
      raise Exception('Requester.gripper: ', e)

  def _checkResult(self, result:pbBase.Acknowledge) -> None:
    '''
    Private method to check the reply to a request. Raises an exception if an error is detected.
    @result Incoming reply to a request.
    @throw Exception If the response contains an error. The message of the Exception will describe the error details.
    '''
    if result.code is not pbBase.Acknowledge.Code.OK:
      raise Exception('Error processing request: ', result.message)

  def getPublished(self) -> pbRobotControl.Published:
    '''! Retrieve the last message received.
    Note that this message may only hold partial information.
    Refer to the other getter methods to retrieve particular data.
    @return The last incoming message.
    '''
    with self.publishedLock:
      return self.published

  def getJointValues(self) -> pbBase.ArrayDouble:
    '''! Retrieves the last incoming joint values.
    @return The last incoming joint values as proto.
    '''
    with self.jointsLock:
      return self.jointsValue

  def getRobotPose(self) -> pbBase.Pose:
    '''! Retrieves the last incoming robot pose.
    @return The last incoming robot pose as proto.
    '''
    with self.robotPoseLock:
      return self.robotPose

  def isMoving(self) -> bool:
    '''! Retrieves the last incoming robot moving state flag.
    @return True if the robot is moving, False otherwise.
    '''
    with self.movingLock:
      return self.moving

  def _processSubscription(self, subscription) -> None:
    '''! Processes incoming messages after subscription to a publisher server.
    If a callback method `processSubscription` was registered during object creation, it will be called every time a new message is received.
    This method blocks until the connection to the server is closed.
    Therefore, this method should run on its own thread.
    '''
    try:
      for message in subscription:
        with self.publishedLock:
          self.published = message
        with self.jointsLock:
          self.jointsValue = message.jointValues if message.HasField("jointValues") else self.jointsValue
        with self.robotPoseLock:
          self.robotPose = message.robotPose if message.HasField("robotPose") else self.robotPose
        with self.movingLock:
          self.moving = message.state.moving if message.HasField("state") else self.moving
        self.callback(message)
    except grpc.RpcError as e:
      if e.code() == grpc.StatusCode.CANCELLED:
        self.logger.warning('Subscription cancelled')
      else:
        raise
    self.logger.info('Subscription exit')

  def _subscribe(self, waitForReady=True, timeout=None):
    '''! Subscribes to the configured publisher.
    @param waitForReady If True this call will block until the publisher responds or timeout is exceeded. Defaults to True.
    @param timeout Amount of time to block if wait_for_ready is True in [s]. Defaults to None, implying wait for ever.
    '''
    return self.subscriber.subscribe(pbBase.Subscribe(), wait_for_ready=waitForReady, timeout=timeout)

  def getRequester(self) -> gpbRobotControl.ResponderStub:
    """! Return the grpc requester
    TODO: verify if it's really needed
    used in tasklistManager
    """
    return self.requester
