#!/usr/bin/env python3
import cv2
import logging
import loggingclass
#from ColorVision import *
#import tensorflow as tf
import numpy as np
import socket 



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
        #self.load_model()


    def do_the_work(self, image, camera):
        print("working")
        self.log.save_image(image, camera)
        #self.find_visual(image)

    def preprocessing(self,image):
        print("preprocessing")
        gray = cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)
        binary = cv2.adaptiveThreshold(gray,255,cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY,21,10) 

        return binary

    def find_visual(self, image):
        binary = self.preprocessing(image)
        contours, hierarchy = cv2.findContours(binary,cv2.RETR_LIST,cv2.CHAIN_APPROX_NONE)
        for contour in contours:
            area = cv2.contourArea(contour)
            if area >420: 
                x,y,w,h = cv2.boundingRect(contour)

                size = (25,25)
                potentialVictim = binary[y:y+h, x:x+w]
                potentialVictimCS = cv2.resize(potentialVictim, size)
                return potentialVictimCS







    def identify_victim(self, section):
        victim = self.model(section)
        print(victim) 
        self.framedetected.append(victim)

        





    def load_model(self):
        num_classes = 4
        self.model = tf.keras.Sequential([
        tf.keras.layers.Rescaling(1./255),
        tf.keras.layers.Conv2D(32, 3, activation='relu'),
        tf.keras.layers.MaxPooling2D(),
        tf.keras.layers.Flatten(),
        tf.keras.layers.Dense(128, activation='relu'),
        tf.keras.layers.Dense(num_classes)
    ])
        self.model.compile(
        optimizer='adam',
        loss=tf.keras.losses.SparseCategoricalCrossentropy(from_logits=True),
        metrics=['accuracy'])
        
        self.model.load_weights('./checkpoints/my_checkpoint').expect_partial()



    


