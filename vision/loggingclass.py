import os 
import numpy as np
import pathlib
import cv2
import logging
#import visionclass as vc 


class log:
    fnum = 0
    def __init__(self,base_dir, bLogging = True,folder="log/unsorted") -> None:
        self.log_folder = os.path.join(base_dir, folder)
        self.bLogging = bLogging
        if folder == "training/autogenerated":
            self.createfolder(folderName="training")
        elif bLogging: self.createfolder()

    def createfolder(self, folderName = "log"):  
        dnum = len(os.listdir(self.log_folder))

        self.current_log_folder = f"{self.log_folder}/{folderName}{dnum}"
        os.mkdir(self.current_log_folder)

    
    def save_image(self, image, filename):
        self.fnum +=1
        try:

            if self.bLogging:
                path = f'{self.current_log_folder}/{filename}{self.fnum}.png'
                print(f"saving image {path}")
                cv2.imwrite(path,image)
        except Exception as ex:
            logging.exception("couldn't log")