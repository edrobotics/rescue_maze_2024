#!/usr/bin/env python3
import cv2
import logging
import visionclass
import argparse
from configparser import ConfigParser
import os.path
print("finished imports")



def get_intensity(action,x, y, flags, *userdata):
    global position
    if action == cv2.EVENT_LBUTTONDOWN:
        position = (x,y)
        print(f"position: {position}")
      #  print(f"bgr: {image[y,x]}")
    

def openCams(cPi = True, c1 = True, c2 = True):
    global piCam
    global cap
    global cap2
    print("opening cameras")
    if cPi:
        from picamera2 import PiCamera2, Preview
        import time, libcamera 
        print("opening picam")
        piCam = PiCamera2()
        piCam.start()

    if c1: 
        print("opening camera 1")
        cap = cv2.VideoCapture("/dev/video0")
    if c2: 
        print("opening camera 2")
        cap2 = cv2.VideoCapture("/dev/video2")

def closeCams():
    global piCam
    global cap
    global cap2
    print("releasing cameras")
    cap.release()
    cap2.release()
    piCam.stop()





def showSource():
    try:
        windows = ("cam0","cam1","cam2","window", "binary", "red", "yellow","green")
        for window in windows:
            cv2.namedWindow(window)
            cv2.setMouseCallback(window, get_intensity)
    except Exception as ex:
        print("couldn't create windows")
        showsource = False


if __name__ == "__main__":
    logging.basicConfig(filename='log/vision.log', encoding='utf-8', level=logging.DEBUG)
    logging.info("started")
    dir_path = os.path.dirname(os.path.realpath(__file__))
    print(dir_path)
    config_filepath = dir_path+"/config.ini"

    exists = os.path.exists(config_filepath)
    config = None
    if exists:
        print("--------config.ini file found at ", config_filepath)
        config = ConfigParser()
        config.read(config_filepath)
    else:
        print("---------config.ini file not found at ", config_filepath)

    parser = argparse.ArgumentParser()
    parser.add_argument("-l", "--logging", type=bool)
    parser.add_argument("-t", "--testing", type=bool)
    parser.add_argument("-s", "--showsource", type=bool)


    args = parser.parse_args()
    bLogging = args.logging
    showsource = args.showsource
    if bLogging:
        testing = False
    elif args.testing == True:
        testing = True
    else: testing = False

    paths_config = config["PATHS"]
    #base_folder = paths_config["basefolder"]
   #print(f"basefolder{base_folder}")
    imgProc= visionclass.imgproc(bLogging,dir_path)

    if args.testing:
        print("running validation")
        import validation as val
        valC = val.validation(dir_path)
        sorted_images_local = "log/previous"
        sorted_images = os.path.join(dir_path, sorted_images_local)
        valC.runValidation()
    else: 
        if showsource: 
            showSource()
        
                    


        openCams(cPi=False)
        while True:
            try:
                print("new frame")
                #piImg = picam.capture_array()
                ret, image1 = cap.read()
                ret2, image2 = cap2.read()
                #imgProc.do_the_work(piImg, "cam0")
                imgProc.do_the_work(image1, "cam1")
                imgProc.do_the_work(image2, "cam2")

                if showsource:
                    try:
                        #cv2.imshow("cam0",piImg)
                        cv2.imshow("cam1",image1)
                        cv2.imshow("cam2",image2)
                    except Exception as ex: 
                        print("couldn't show images")
                        showSource = False
            except Exception as ex:
                print("exception")

                
                closeCams()






    
