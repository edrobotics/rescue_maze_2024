import logging
import cv2
import os
import shutil
import traceback
import numpy as np
import visionclass as vc 
from configparser import ConfigParser


def nothing(x):
	pass



class Trackbars:

    def run(self,image):
        self.create()
        self.showimage(image)
        self.clean_up()
        

    def create(self):

        cv2.namedWindow("HSV Value")
        cv2.createTrackbar("H MIN", "HSV Value", 0, 180, nothing)
        cv2.createTrackbar("S MIN", "HSV Value", 0, 255, nothing)
        cv2.createTrackbar("V MIN", "HSV Value", 0, 255, nothing)
        cv2.createTrackbar("H MAX", "HSV Value", 179, 180, nothing)
        cv2.createTrackbar("S MAX", "HSV Value", 255, 255, nothing)
        cv2.createTrackbar("V MAX", "HSV Value", 255, 255, nothing)

    def showimage(self,image):
        while(1):
            hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
            h_min = cv2.getTrackbarPos("H MIN", "HSV Value")
            s_min = cv2.getTrackbarPos("S MIN", "HSV Value")
            v_min = cv2.getTrackbarPos("V MIN", "HSV Value")
            h_max = cv2.getTrackbarPos("H MAX", "HSV Value")
            s_max = cv2.getTrackbarPos("S MAX", "HSV Value")
            v_max = cv2.getTrackbarPos("V MAX", "HSV Value")
            lower_blue = np.array([h_min, s_min, v_min])
            upper_blue = np.array([h_max, s_max, v_max])
            mask = cv2.inRange(hsv, lower_blue, upper_blue)
            result = cv2.bitwise_and(image, image, mask=mask)
    #        cv2.imshow("HSV Value", image)
            cv2.imshow("HSV Value", result)
        #    cv2.imshow("Frame Mask", result)
            key = cv2.waitKey(1)
            if key == ord("q"): break

    
    def clean_up(self):
        cv2.destroyWindow("HSV Value")
#        cv2.destroyWindow("Frame Mask")




class validation:
    TB = Trackbars() #trackbar object
    base_folder =  vc.imgproc.path
    config_path = os.path.join(base_folder, "config.ini")

    def __init__(self) -> None:
        pass


    def keystrocks(self):

        folders = {
            "u":"U",
            "h":"H",
            "s":"S",
            "r":"red",
            "y":"yellow",
            "g":"green",
            "n":"none",
            "o":"other"
        }
        moved = False
        key = 0
        while key != 27:
            key = 0
            #cv2.imshow("window",testing.image_clone)
            key = cv2.waitKey(1)
            for folder in folders:
                if key == ord(folder):
                    self.move(self.file_name,folders[folder])
                    moved = True
            if key == 32 or key == ord("q"):
                break
    #        elif key == ord("p"): print()
            elif moved:
                break
            elif key == ord("t"):
                self.TB.run(self.image)
                    
        if key == 27:
            cv2.destroyAllWindows()
            stop = True
            print("stoping")


    def get_intensity(self,action,x, y, flags, *userdata):
        if action == cv2.EVENT_LBUTTONDOWN:
            position = (x,y)
            print(f"position: {position}")
            print(f"bgr: {self.image[y,x]}")
            print(f"hsv: {self.hsv[y,x]} ")



    def move(self, file_name, type, source = None, base = None):
        pass 
        destination_path = os.path.join(self.source_path,type,file_name)
        print(f"source: {self.source_path}")
        print(f"destination: {destination_path}")
        shutil.move(self.source_path, destination_path)


    def showimage(self, victim):
        if True or len(vc.framedetected[0]) == 1: #true for now

            cv2.imshow("image",vc.image)
            #colours = ("red","green","yellow")

            self.keystrocks()            


    def evaluatefolder(self,victim):
        print("evaluating", victim)
        cams = ("picam", "cam1", "cam2")
        for cam in cams:
            source_folder = os.path.join(self.base_folder,victim,cam)
            file_list = os.listdir(source_folder)
            ct = 0
            for file_name in file_list:
                self.file_name = file_name
                if self.stop:
                    break
                ct += 1
        #        print(file_name)
                source_path = os.path.join(source_folder, file_name)

                try:
                    image = cv2.imread(source_path) 
                    vc.do_the_work(image, file_name)
                
                    if len(vc.framedetected) == 1:
                        if victim != vc.framedetected[0]:
                            self.showimage(victim, vc.image)
                    elif len(vc.framedetected) == 0 and victim == "none":
                        continue

                    else:
                        self.showimage(victim, vc.image)

        #        except IndexError:
        #            print(file_name)
        #            move(file_name,"problem")

                except Exception as e:
                    print(file_name)
                    logging.exception("exception in evaluate folder")
                    
            print(f"{victim} was evaluated")
            tot = len(file_list)
            if ct <len(file_list):
                print(f"finished {ct} out of {tot} images")
            else:
                print(f"finished {tot} images")


    def clearstat(self):
        ct = 0
        tot = 0

