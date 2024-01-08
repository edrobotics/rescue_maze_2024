import os 
import numpy as np
import pathlib
import cv2
import logging
#import visionclass as vc 


class log:
    fnum = 0
    def __init__(self,base_dir, bLogging = True) -> None:
        self.log_folder = os.path.join(base_dir, "log/unsorted/")
        self.bLogging = bLogging
        if bLogging: self.createfolder()

    def createfolder(self):  
        dnum = len(os.listdir(self.log_folder))

        self.current_log_folder = f"{self.log_folder}log{dnum}"
        os.mkdir(self.current_log_folder)

    
    def save_image(self, image, camera):
        self.fnum +=1
        try:

            if self.bLogging:
                path = f'{self.current_log_folder}/{camera}{self.fnum}.png'
                print(f"saving image {path}")
                cv2.imwrite(path,image)
        except Exception as ex:
            logging.exception("couldn't log")