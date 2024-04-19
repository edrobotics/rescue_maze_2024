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
        print("trackbar")
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
    detected = {"H":0,
                "S":0,
                "U":0,
                "red": 0,
                "green": 0,
                "yellow": 0}

    def __init__(self,dir_path,showsource = False,training = False,pause = False) -> None:
        self.TB = Trackbars() #trackbar object
        self.base_folder =  dir_path
        self.pause = pause
        if pause:
            self.showWrong = True
            self.showsource = False

        else:
            self.showsource = showsource
            self.showWrong = False
        self.config_path = os.path.join(self.base_folder, "config.ini")
        self.imgproc = vc.imgproc(bLogging=False, dir_path=dir_path,bComms=False,training=training)



    def keystrocks(self):

        folders = {
            "u":"U",
            "h":"H",
            "s":"S",
            "r":"red",
            "y":"yellow",
            "g":"green",
            "n":"none",
            "o":"other",
            "d":"double"
        }
        moved = False
        key = 0
        while key != 27:
            key = 0
            #cv2.imshow("window",testing.image_clone)
            key = cv2.waitKey(100)
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
                print("trackbar")
                self.TB.run(self.imgproc.colorDetection.adjustedImage)
                    
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
        destinationFolder = os.path.join(self.base_folder,"log/sorted",type)
        dnum = len(os.listdir(destinationFolder))
        destinationPath = os.path.join(destinationFolder, str(dnum) + file_name)
        #type,file_name)
        print(f"source: {self.source_path}")
        print(f"destination: {destinationPath}")
        shutil.move(self.source_path, destinationPath)


    def showimage(self, image):

        if self.showsource or self.showWrong:# or len(vc.framedetected[0]) == 1: #true for now

            try:
                cv2.imshow("image",image)
                #cv2.imshow("color", self.imgproc.detector.image)
            #colours = ("red","green","yellow")
            except:
                cv2.imshow(self.imgproc.image)

            self.keystrocks()            


    def evaluatefolder(self,victim):
        self.detected = {"H":0,"S":0,"U":0, "red": 0, "green": 0, "yellow": 0}
        print("evaluating", victim)
        cams = ("picam", "cam1", "cam2")
        #for cam in cams:
        if True:
            #source_folder = os.path.join(self.base_folder,"log/previous",victim)
            source_folder = os.path.join(self.base_folder,"log/sorted",victim)
            identifiedVictims =self.iterateFolder(source_folder, expectedVictim = victim)

                    
 #               if len(self.imgproc.framedetected) == 1:
 #                   if victim != self.imgproc.framedetected[0]:
 #                       self.showimage(victim, self.imgproc.image)
 #               elif len(self.imgproc.framedetected) == 0 and victim == "none":
 #                   continue

 #               else:
 #                   self.showimage(self.imgproc.image)


    def runLog(self,folder):

        source_folder = os.path.join(self.base_folder,"log/unsorted/log" + folder)

        identifiedVictims =self.iterateFolder(source_folder)
        


    def iterateFolder(self,folder, expectedVictim = None):
        file_list = os.listdir(folder)
        ct = 0
        for file_name in file_list:
            self.file_name = file_name
            if self.stop:
                break
            ct += 1
            self.source_path = os.path.join(folder, file_name)

            #print(self.source_path)

            try:
                image = cv2.imread(self.source_path) 
                self.imgproc.do_the_work(image, file_name)
                if self.showWrong:
                    if len(self.imgproc.framedetected) == 0:
                        if expectedVictim != "none":
                            print("victim not detected")
                            self.showimage(self.imgproc.imagecopy)
                            print(expectedVictim)
                    elif self.imgproc.framedetected[0] != expectedVictim:
                        print(f"detected {self.imgproc.framedetected[0]} instead of {expectedVictim}")
                        
                        self.showimage(self.imgproc.imagecopy)
                elif self.showsource:
                    self.showimage(self.imgproc.imagecopy)
            

    #        except IndexError:
    #            print(file_name)
    #            move(file_name,"problem")

            except Exception as e:
                print(file_name)
                print(e)
                logging.exception("exception in evaluate folder")
            
            for victim in self.imgproc.framedetected:
                self.detected[victim] += 1 
            
            self.imgproc.cleanUp()
            #print(self.detected)
        if ct < len(file_list):
            print(f"run on {ct} out of {len(file_list)} images")
        else:
            print(f"finished all {ct} images")
        print(f"detected {self.detected}")



    def clearstat(self):
        self.ct = 0
        self.tot = 0
        #currently not used

    def runValidation(self):
        stop = False
        print("testing")
        victims = ("U","H","S", "red","yellow","green","none")

        victim = input("victim to evaluate: ")


        if victim == "all":
            for victim in victims:
                self.evaluatefolder(victim)
                self.clearstat()
                if self.stop:
                    break

        elif victim == "q":
            stop = True    

        elif victim in victims:
            self.evaluatefolder(victim)
        elif victim == "l":
            logfolder = input("enter a number(log folder): ")
            self.runLog(logfolder)
    
        else:
            print("invalid victim")


        if not stop: self.runValidation()
    
        cv2.destroyAllWindows()


                     
