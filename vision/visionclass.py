#!/usr/bin/env python3
import cv2
import logging
import loggingclass
from ColorVision import *


class imgproc:
    framedetected = ()

    def __init__(self, bLogging):
        self.bLogging = bLogging
        self.log = loggingclass.log(bLogging=bLogging)


    def do_the_work(self, image, camera):
        self.log.save_image(image, camera)


    





class colorVictims(imgproc):
    def find_color(self, image, camera):
        find_color_and_square(image)
        


class visualVictims(imgproc):
    def __init__():
        pass 
    def find_visual(self, image, camera):
        pass