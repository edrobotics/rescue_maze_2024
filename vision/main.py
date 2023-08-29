#!/usr/bin/env python3
import cv2
import logging
#if using picamera 
from picamera.array import PiRGBArray
from picamera import PiCamera


if __name__ == "__main__":

    #camera starts here
    camera = PiCamera()
    camera.resolution = (640, 480)
    camera.framerate = 10
    camera.awb_mode = 'off'
    rg, bg = (1.8, 1.42)
    camera.awb_gains = (rg, bg)
    rawCapture = PiRGBArray(camera, size=(640, 480))
    for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
        image = frame.array
        
