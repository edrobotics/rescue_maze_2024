#!/usr/bin/env python3
import cv2
import logging
#if using picamera 
#from picamera2 import PiCamera2, Preview
#import time, libcamera 
import visionclass






if __name__ == "__main__":

    #picam = PiCamera2()
    #picam.start()

    cap = cv2.VideoCapture("/dev/video0")
    cap2 = cv2.VideoCapture("/dev/video2")

    imgProc= visionclass.imgproc(bLogging=True)

    while True:
        try:
            print("new frame")
            #piImg = picam.capture_array()
            ret, image1 = cap.read()
            ret2, image2 = cap2.read()
            imgProc.do_the_work(image1,image1,image2)


        except Exception as ex:  
            print("Exception: ", ex)
            break


    print("releasing cameras")
    cap.release()
    cap2.release()
   # picam.stop()



        
