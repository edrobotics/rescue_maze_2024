#!/usr/bin/env python3
import cv2
import logging
import visionclass
import argparse
from configparser import ConfigParser
import os.path
import time
import threading
import queue
print("finished imports")



def get_intensity(action,x, y, flags, *userdata):
    global position
    if action == cv2.EVENT_LBUTTONDOWN:
        position = (x,y)
        print(f"position: {position}")
      #  print(f"bgr: {image[y,x]}")
    

def openCams(cPi = True, c1 = True, c2 = True):
    global piCam
    global cap1
    global cap2
    print("opening cameras")
    if cPi:
        from picamera2 import PiCamera2, Preview
        import libcamera 
        print("opening picam")
        piCam = PiCamera2()
        piCam.start()

    if c1: 
        print("opening camera 1")
        cap1 = cv2.VideoCapture("/dev/video0")

        cap1.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        cap1.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

        time.sleep(2)
        #cap.set(cv2.CAP_PROP_EXPOSURE, -8.0)

    if c2: 
        print("opening camera 2")
        cap2 = cv2.VideoCapture("/dev/video2")
        cap2.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        cap2.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        time.sleep(2)
        #cap2.set(cv2.CAP_PROP_EXPOSURE, -8.0)



def closeCams():
    global piCam
    global cap1
    global cap2
    print("releasing cameras")
    cap1.release()
    cap2.release()
    try:
        piCam.stop()
    except: 
        print("couldn't close pi cam, probably not used")





def openWindows():
    try:
        windows = ("cam0","cam1","cam2","window", "binary", "red", "yellow","green")
        for window in windows:
            cv2.namedWindow(window)
            cv2.setMouseCallback(window, get_intensity)
    except Exception as ex:
        print("couldn't create windows")
        showSource = False
def imageProcessor(queue, cam,imgProc):

    while True:
        image = queue.get()
        imgProc.do_the_work(image, cam)
        queue.task_done()


def imageCapture(queue, cap):
    lastFrame = 0
    while True:
        timeTillNext = lastFrame + 0.2 -time.time()
        if timeTillNext <=0:
            ret, image = cap.read()
            queue.put(image)
        else:
            time.sleep(timeTillNext)




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
    parser.add_argument("-l", "--logging", type=bool, default= True)
    parser.add_argument("-t", "--testing", type=bool,default=False)
    parser.add_argument("-s", "--showSource", type=bool, default=False)
    parser.add_argument("-w", "--showWrong", type=bool, default=False)
    parser.add_argument("-c", "--comms", type=bool, default=False)
    parser.add_argument("-r", "--train", type=bool, default=False)
    parser.add_argument("-p", "--pause", type=bool, default=False)


    args = parser.parse_args()
    bLogging = args.logging
    showSource = args.showSource
    testing = args.testing
    bComms = args.comms
    pause = args.pause
    showWrong = args.showWrong
    print("l" , bLogging)
    print("s" , showSource)
    print("t", testing)
    print("c", bComms)
    print("p", pause)
    if testing:
        bLogging = False
       # showSource = True
        bComms = False
    else:
        pass


    paths_config = config["PATHS"]
    width = config["WIDTH"]
    height = config["HEIGHT"]
    #base_folder = paths_config["basefolder"]
   #print(f"basefolder{base_folder}")

    if testing:
        print("running validation")
        import validation as val
        valC = val.validation(dir_path, showsource=showSource,training=args.train,pause=showWrong)
        sorted_images_local = "log/previous"
        sorted_images = os.path.join(dir_path, sorted_images_local)
        valC.runValidation()
    else: 
        if showSource: 
            openWindows()
        openCams(cPi=False)
        if True:
            imgProc1= visionclass.imgproc(bLogging,dir_path,bComms=bComms)
            imgProc2= visionclass.imgproc(bLogging,dir_path,bComms=bComms)
            imageQueue1 = queue.Queue()
            imageQueue2 = queue.Queue()


            imageProcessorThread2 = threading.Thread(target=imageProcessor, args=(imageQueue2,"cam2", imgProc2))
            imageProcessorThread1 = threading.Thread(target=imageProcessor, args=(imageQueue1,"cam1",imgProc1))
            imageProcessorThread1.start()
            imageProcessorThread2.start()

            imageCaptureThread1 = threading.Thread(target=imageCapture, args=(imageQueue1, cap1))
            imageCaptureThread2 = threading.Thread(target=imageCapture, args=(imageQueue2, cap2))
            imageCaptureThread1.start()
            imageCaptureThread2.start()

        
        else:  
            imgProc= visionclass.imgproc(bLogging,dir_path,bComms=bComms)

            while True:
                try:
                    print("new frame")
                    #piImg = picam.capture_array()
                    ret, image1 = cap1.read()
                    ret2, image2 = cap2.read()
                    #imgProc.do_the_work(piImg, "cam0")
                    imgProc.do_the_work(image1, "cam1")
                    imgProc.do_the_work(image2, "cam2")

                    if showSource:
                        try:
                            #cv2.imshow("cam0",piImg)
                            cv2.imshow("cam1",image1)
                            cv2.imshow("cam2",image2)
                        except Exception as ex: 
                            print("couldn't show images")
                            showSource = False
                except KeyboardInterrupt as ex:
                    print("exception")
                    closeCams()
                
                except Exception as ex:
                    logging.exception()







        
