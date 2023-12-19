#!/usr/bin/env python3
import cv2
import logging
import loggingclass



class imgproc:


    def __init__(self, bLogging):
        self.bLogging = bLogging
        self.log = loggingclass.log(bLogging=bLogging)
        pass


    def do_the_work(self, image0, image1, image2):
        self.log.save_image(image0, "pi")
        self.log.save_image(image1, "1")
        self.log.save_image(image2, "2")


    


    def find_visual(self):
        pass


class colorVictims(imgproc):
    pass


class visualVictims(imgproc):
    pass