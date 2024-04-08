#!/usr/bin/env python3
import cv2
import logging
import loggingclass
#from ColorVision import *
#import tensorflow as tf
import numpy as np
import socket 
import os

import tensorflow.lite as tflite
from letterRecognition import letters


class comms:
    port = 4242
    host = socket.gethostbyname(socket.gethostname())
    s = socket.socket(socket.AF_INET,socket.SOCK_STREAM)

    def __init__(self) -> None:
        self.s.connect(self.host, self.port)

    def send(self,messageType,victimType,camera,position,timeStamp):

        message = f"!{messageType},{victimType},{camera},{position},{timeStamp}"

        self.s.send(message)








class imgproc:
    
    framedetected = []

    def __init__(self, bLogging, dir_path):
        print("initiating visionclass")
        self.dir_path = dir_path
        self.bLogging = bLogging
        self.log = loggingclass.log(base_dir=dir_path, bLogging = bLogging)
        self.loadModel()


    def do_the_work(self, image, camera):
        print("working")
        self.imagecopy = image.copy()
        self.log.save_image(image, camera)
        self.find_visual(image)


    def preprocessing(self,image):
        print("preprocessing")
        gray = cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)
        binary = cv2.adaptiveThreshold(gray,255,cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY,21,10) 
        binary = np.invert(binary)

        return binary

    def find_visual(self, image):
        binary = self.preprocessing(image)
        binary2 = binary.copy()
        contours, hierarchy = cv2.findContours(binary,cv2.RETR_LIST,cv2.CHAIN_APPROX_NONE)
        cv2.drawContours(binary2, contours, 1, (255,0,0),3)
        print(len(contours))
        for contour in contours:
            area = cv2.contourArea(contour)
            if area >420: 
                rect = cv2.minAreaRect(contour)
                box = cv2.boxPoints(rect)
                box = np.int0(box)
                cv2.drawContours(self.imagecopy, [box], 0, (255, 0, 0), 3)
                x,y,w,h = cv2.boundingRect(contour)


                size = (25,25)
                potentialVictim = binary[y:y+h, x:x+w]
                potentialVictimRS = cv2.resize(potentialVictim, size)


        cv2.imshow("contours", binary2)
        cv2.imshow("imagecopy",self.imagecopy)
        cv2.waitKey(0)

                #return potentialVictimCS





    def loadModel(self):
        self.model = letters()

    def identify_victim(self, section):
        victim = self.model.recogniseSection(section)
        print(victim) 
        self.framedetected.append(victim)

        







