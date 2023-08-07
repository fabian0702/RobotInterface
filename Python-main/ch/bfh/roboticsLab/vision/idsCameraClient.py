'''
BFH roboticsLab python robot control interface.
@author gabriel.gruener@bfh.ch

Installation requirements:
> python3 -m pip install grpcio grpcio-tools numpy rowan
'''
import cv2
import threading
import grpc
import numpy as np
from typing import Callable
import logging
import os
import time

from ch.bfh.roboticsLab import Base_pb2 as pbBase
from ch.bfh.roboticsLab.vision import Vision_pb2 as pbVision
from ch.bfh.roboticsLab.vision import Vision_pb2_grpc as gpbVision
from ch.bfh.roboticsLab.util.Logger import Logger

# Default publisher port
PUBLISHER_PORT = 40831

def processSubscription(published:pbVision.Image) -> None:
  '''! Empty, default process subscription callback.
  You may write your own subscription callback function with the same signarture as this method.
  @see Subscriber
  '''
  pass

class IdsCameraClient():
    '''
    This class subscribes to the robot's gRPC publisher and stores incoming
    messages. Retrieve the last incoming message with `getPublished()` or use the
    individual getter methods.

    Example:
    client = IdsCameraClient('localhost')
    image = client.getImage()
    '''
    def __init__(self, host:str, port:int=PUBLISHER_PORT, processSubscription:Callable[[pbVision.Image], None]=processSubscription):
        '''
        Creates a new Subscriber object
        @param host The IP address to subscribe to.
        '''
        self.host : str = host
        self.callback = processSubscription
        self.logger: logging.Logger = Logger(os.path.basename(__file__)).getInstance()
        channelOptions = [('grpc.max_receive_message_length', 60_000_000)]
        self.logger.info(f'Connecting to ids camera publisher on {host}:{PUBLISHER_PORT}')
        self.subscriberChannel = grpc.insecure_channel(f'{host}:{port}', options=channelOptions)
        self.subscriber = gpbVision.ImagePublisherStub(self.subscriberChannel)
        subscription = self._subscribe()

        # Variables to save incoming subscription data
        self.published : pbVision.Image = pbVision.Image()
        self.publishedLock = threading.Lock()
        self.image = []
        self.format : str = 'unknown'
        self.height : int = -1
        self.width : int = -1
        self.imageLock = threading.Lock()
        

        self.subscriptionThread = threading.Thread(target=self._processSubscription, name='IdsCameraClientSubscriptionThread', args=[subscription])
        self.subscriptionThread.start()

    def shutdown(self):
        '''
        Shutdown the subscriber by stopping the background read thread.
        '''
        print('Subscriber shutdown')
        self.stoppingSubscriber = True
        self.subscriberChannel.close()
        self.subscriptionThread.join()

    def getImage(self):
        if(np.size(self.image)>0):
            image = cv2.imdecode(np.frombuffer(np.array(self.image), dtype=np.uint8), cv2.IMREAD_COLOR)
        else:
            image = []
        return image
    
    def getDim(self):
        return [self.width, self.height]
        


    def _processSubscription(self, subscription):
        '''
        Processes incoming messages after subscription to a publisher server.
        This method blocks until the connection to the server is closed.
        Therefore, this method should run on its own thread.
        '''
        try:
            for message in subscription:
                with self.publishedLock:
                  self.published = message
                with self.imageLock:
                  if message.format is not pbVision.Image.Format.UNKNOWN and len(message.imageData)>0:
                    self.image = message.imageData 
                    self.format = message.format 
                    self.height = message.height
                    self.width = message.width
                self.callback(message)
        except grpc.RpcError as e:
            if e.code() == grpc.StatusCode.CANCELLED:
                print('Subscription cancelled')
            else:
                raise
        print('Subscription exit')

    def _subscribe(self, waitForReady=True, timeout=None):
      '''! Subscribes to the configured publisher.
      @param waitForReady If True this call will block until the publisher responds or timeout is exceeded. Defaults to True.
      @param timeout Amount of time to block if wait_for_ready is True in [s]. Defaults to None, implying wait for ever.
      '''
      return self.subscriber.subscribe(pbBase.Subscribe(), wait_for_ready=waitForReady, timeout=timeout)

#########################################
# Test & Example
#########################################    
if __name__ == '__main__':
  camera = IdsCameraClient('192.168.0.100')
  while(camera.getDim()[0]<0):
    time.sleep(0.1)
  image = camera.getImage()
  cv2.imshow('Test',image)
  cv2.waitKey(0)
  camera.shutdown()