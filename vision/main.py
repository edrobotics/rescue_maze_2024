#!/usr/bin/env python3
import cv2
import logging
#if using picamera 
#from picamera2 import PiCamera2, Preview
#import time, libcamera 
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
    




if __name__ == "__main__":

    dir_path = os.path.dirname(os.path.realpath(__file__))
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
    testing = args.testing
    showsource = args.showsource

    paths_config = config["PATHS"]
    #base_folder = paths_config["basefolder"]
   #print(f"basefolder{base_folder}")

    if not args.testing:
        print("opening cameras")
        #picam = PiCamera2()
        #picam.start()

        cap = cv2.VideoCapture("/dev/video0")
        cap2 = cv2.VideoCapture("/dev/video2")
    else: 

        import validation as val
        valC = val.validation()
        sorted_images_local = "log/previous"
        sorted_images = os.path.join(dir_path, sorted_images_local)

    imgProc= visionclass.imgproc(bLogging,dir_path)
    if showsource: 
        cv2.namedWindow("cam0")
        cv2.namedWindow("cam1")
        cv2.namedWindow("cam2")
        cv2.setMouseCallback("cam0", get_intensity)
        cv2.setMouseCallback("cam1", get_intensity)
        cv2.setMouseCallback("cam2", get_intensity)
        windows = ("image","window", "binary", "red", "yellow","green")
        for window in windows:
            cv2.namedWindow(window)
            cv2.setMouseCallback(window, get_intensity)
    
    try :
        if args.testing:
            print("testing")
            victims = ("U","H","S", "red","yellow","green","none")

            victim = input("victim to evaluate: ")


            if victim == "all":
                for victim in victims:
                    valC.evaluatefolder(victim)
                    valC.clearstat()
                   # if stop:
                   #     break

            else:
                if victim in victims:
                    valC.evaluatefolder(victim)
                else:
                    print("invalid victim")
            cv2.destroyAllWindows()


                    


        else:
            while True:

                print("new frame")
                #piImg = picam.capture_array()
                ret, image1 = cap.read()
                ret2, image2 = cap2.read()
                #imgProc.do_the_work(piImg, "cam0")
                imgProc.do_the_work(image1, "cam1")
                imgProc.do_the_work(image2, "cam2")

                if showsource:
                    #cv2.imshow("cam0",piImg)
                    cv2.imshow("cam1",image1)
                    cv2.imshow("cam2",image2)

    except Exception as ex:  
        print("Exception: ", ex)


    print("releasing cameras")
    cap.release()
    cap2.release()
   # picam.stop()



        
