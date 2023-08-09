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


path.append('./Python-main/')
from ch.bfh.roboticsLab.vision import Vision_pb2 as pb
from ch.bfh.roboticsLab.vision import Vision_pb2_grpc as gpb

# The delay between two publisher messages
FRAME_RATE = 20
PUBLISH_INTERVAL_SECONDS = 1/FRAME_RATE

class cameraIDS:
    def __init__(self):
        self.channels = 3                    #3: channels for color mode(RGB); take 1 channel for monochrome
        self.width = 0
        self.height = 0
        self.imageData = []

    # Starts the driver and establishes the connection to the camera



class Publisher(gpb.ImagePublisherServicer):
    def __init__(self):
        self.imageCache = []
        self.stoppingPublisher = False
        print('Server started')

    def appendCache(self, img):
        self.imageCache.append(img)

    def shutdown(self):
        self.stoppingPublisher = True

    def subscribe(self, request, context):
        print('Publisher:  streaming start')
        while context.is_active() and not self.stoppingPublisher:
            if not len(self.imageCache) > 0:
                continue
            img = self.imageCache.pop(0)
            
            data = cv2.imencode('.jpg', img,[cv2.IMWRITE_JPEG_QUALITY, 95,cv2.IMWRITE_JPEG_OPTIMIZE,1])[1].tobytes()
            height, width, channels = img.shape
            robMsg = pb.Image(format = pb.Image.Format.RGB8, width = int(width), height = int(height), imageData = data)
            try:
                yield robMsg
            except Exception as e:
                print('Publisher:  failed to send message', e)
                break
            time.sleep(PUBLISH_INTERVAL_SECONDS)
        print('Publisher:  streaming end')

def serve():
    global publisher, publisherServer
    signal.signal(signal.SIGTERM, lambda signum, frame: exit())
    publisher = Publisher()
    publisherServer = grpc.server(futures.ThreadPoolExecutor(max_workers=10))
    gpb.add_ImagePublisherServicer_to_server(publisher, publisherServer)
    publisherServer.add_insecure_port('[::]:40831')
    publisherServer.start()

def appendCache(img):
    publisher.appendCache(img)

def shutdown():
    print('Server shutdown')
    publisher.shutdown()
    publisherServer.stop(2)
