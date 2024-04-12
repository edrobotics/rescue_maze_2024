import logging
import cv2
import os
import shutil
import traceback
import numpy as np
import visionclass as vc 
from configparser import ConfigParser


print("imported validation")

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
    stop = False
    detected = {"H":0,"S":0,"U":0,"none":0}

    def __init__(self,dir_path) -> None:
        self.TB = Trackbars() #trackbar object
        self.base_folder =  dir_path
        self.config_path = os.path.join(self.base_folder, "config.ini")
        self.imgproc = vc.imgproc(bLogging=False, dir_path=dir_path,bComms=False)



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
            self.stop = True
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


    def showimage(self, image):
        if True or len(vc.framedetected[0]) == 1: #true for now

            try:
                cv2.imshow("image",image)
            #colours = ("red","green","yellow")
            except:
                cv2.imshow(self.imgproc.image)

            self.keystrocks()            


    def evaluatefolder(self,victim):
        print("evaluating", victim)
        cams = ("picam", "cam1", "cam2")
        #for cam in cams:
        if True:
            source_folder = os.path.join(self.base_folder,"log/previous",victim)
            file_list = os.listdir(source_folder)
            ct = 0
            for file_name in file_list:
                self.file_name = file_name
                if self.stop:
                    break
                ct += 1
                source_path = os.path.join(source_folder, file_name)
                print(source_path)

                try:
                    image = cv2.imread(source_path) 
                    self.imgproc.do_the_work(image, file_name)
                
                    if len(self.imgproc.framedetected) == 1:
                        if victim != self.imgproc.framedetected[0]:
                            self.showimage(victim, self.imgproc.image)
                    elif len(self.imgproc.framedetected) == 0 and victim == "none":
                        continue

                    else:
                        self.showimage(self.imgproc.image)

        #        except IndexError:
        #            print(file_name)
        #            move(file_name,"problem")

                except Exception as e:
                    print(file_name)
                    print(e)
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

    def runValidation(self):
        print("testing")
        victims = ("U","H","S", "red","yellow","green","none")

        victim = input("victim to evaluate: ")


        if victim == "all":
            for victim in victims:
                self.evaluatefolder(victim)
                self.clearstat()
                if self.stop:
                    break

        elif victim in victims:
            self.evaluatefolder(victim)
        else:
            print("invalid victim")
        cv2.destroyAllWindows()


                     
