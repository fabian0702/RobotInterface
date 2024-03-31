#!/usr/bin/python

'''
BFH roboticsLab python IDS camera interface.
@author gionata.quadri@bfh.ch

Installation requirements:
> python3 -m pip install pyueye grpcio grpcio-tools numpy
'''

#Libraries
import cv2
import signal
import numpy as np
import time
from sys import path
import grpc
from concurrent import futures


path.append('./python_main/')
from ch.bfh.roboticsLab.vision import Vision_pb2 as pb
from ch.bfh.roboticsLab.vision import Vision_pb2_grpc as gpb
from ch.bfh.roboticsLab.util.Logger import Logger

logger = Logger('CameraServer').getInstance()

# The delay between two publisher messages
FRAME_RATE = 20
PUBLISH_INTERVAL_SECONDS = 1/FRAME_RATE

publisher = None

class Publisher(gpb.ImagePublisherServicer):
    image = None
    def __init__(self):
        self.stoppingPublisher = False
        logger.info('Server started')
    def setImg(self, img):
        self.image = img
        logger.info('image cached on server')
    def shutdown(self):
        self.stoppingPublisher = True

    def subscribe(self, request, context):
        logger.info('Publisher:  streaming start')
        while context.is_active() and not self.stoppingPublisher:
            time.sleep(PUBLISH_INTERVAL_SECONDS)
            if self.image is None:
                continue
            logger.info(f'Image found. Now publishing image')
            img = self.image
            self.image = None
            
            data = cv2.imencode('.jpg', img,[cv2.IMWRITE_JPEG_QUALITY, 95,cv2.IMWRITE_JPEG_OPTIMIZE,1])[1].tobytes()
            height, width, channels = img.shape
            robMsg = pb.Image(format = pb.Image.Format.RGB8, width = int(width), height = int(height), imageData = data)
            try:
                yield robMsg
            except Exception as e:
                logger.error(f'Publisher:  failed to send message {e}')
                break
            time.sleep(PUBLISH_INTERVAL_SECONDS)
        logger.warn('Publisher:  streaming end')

def serve(i):
    global publisher, publisherServer
    signal.signal(signal.SIGTERM, lambda signum, frame: exit())
    publisher = Publisher()
    publisherServer = grpc.server(futures.ThreadPoolExecutor(max_workers=10))
    gpb.add_ImagePublisherServicer_to_server(publisher, publisherServer)
    publisherServer.add_insecure_port(f'[::]:4083{i*2+1}')
    publisherServer.start()
    return publisher, publisherServer

def shutdown():
    logger.warn('Server shutdown')
    publisher.shutdown()
    publisherServer.stop(2)
