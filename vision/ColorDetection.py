import cv2
import numpy as np
import logging



class ColorDetection:
    frameDetected = []

    def __init__(self) -> None:
        pass


    def doTheWork(self, image):
        self.image = image
        self.adjustedImage = self.adjust_white_balance(image)
        self.frameDetected = []

        self.findColor()
        if len(self.frameDetected)==0:

            returnValue = None
        else:
            returnValue = self.frameDetected[0]
        return returnValue 


    def adjust_white_balance(self, image_s, percent_red=0, percent_blue=0):
        image = image_s.copy()

        adjusted_image = np.zeros((480,640,3), np.uint8)
        b, g, r = cv2.split(image)
        avg_b = np.mean(b)
        avg_g = np.mean(g)
        avg_r = np.mean(r)
        
        adj_b = b * (avg_g / avg_b) * (100 - percent_blue) / 100
        adj_r = r * (avg_g / avg_r) * (100 - percent_red) / 100
        
        adj_b = np.clip(adj_b, 0, 255).astype(np.uint8)
        adj_r = np.clip(adj_r, 0, 255).astype(np.uint8)

        adjusted_image = cv2.merge([adj_b, g, adj_r])
        
        #if self.showsource:
        cv2.imshow("adjusted",adjusted_image)
        return adjusted_image       

    def findColor(self):
        image = self.adjustedImage
        #    image = image.copy
        status = 0
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        self.hsv = hsv.copy()
        self.masks = {}

        lower_range = {
            "yellow": np.array([18,100,100]),#decrese Saturation? 
            "red" : np.array([130,69,100]), #increase saturation >100
            "green": np.array([30,70,30]) #decrese V 50 
            }
        upper_range = {
            "yellow" : np.array([35,255,255]),
            "red" : np.array([180,255,255]),
            "green" : np.array([95,255,255])
            }
        for color in lower_range:
            lower = lower_range[color]
            upper = upper_range[color]
            mask = cv2.inRange(hsv,lower,upper)
            if color == "red":
                red2_lower =np.array([0,120,60]) #0,100,100?
                red2_upper =np.array([14,255,255]) 
                mask2 = cv2.inRange(hsv,red2_lower,red2_upper)
                mask = np.bitwise_or(mask,mask2)
            #mask = self.blank_out(mask)
            self.ColVicP(mask, color)
            cv2.imshow(color, mask)



    def ColVicP(self, mask,color):
        kernel = np.ones((9, 9), np.uint8) 
        mask = cv2.erode(mask,kernel, iterations=1)
        mask = cv2.dilate(mask,kernel, iterations=1) 
        log_mask = cv2.bitwise_and(self.image, self.image, mask=mask)
        print(f"{color}: {np.count_nonzero(mask)} ")

        if np.count_nonzero(mask) > 5000 and np.count_nonzero(mask)< 110000:
            #print(np.count_nonzero(mask))
            ret,thresh = cv2.threshold(mask, 40, 255, 0)# is this necessary?
            contours, hierarchy = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
            c = max(contours, key = cv2.contourArea)
            if cv2.contourArea(c) > 5000:
                print("possible vicitm detcted")

                if True or self.safeguards_color(c, color, mask):#debug

                    self.frameDetected.append(color)
                    print(f"{color}: {cv2.contourArea(c)}")

                   # self.detected(k+self.side,victim=color)
                    #logging.info(f"found {color}, image {self.fnum}")
                   # if self.info: print(f"found {color}, image {self.fnum}")
                    #elf.log(color,img = log_mask)

                else: 
                    if self.info: print("stoped by safeguard")
                    self.log(f"F{color}",img = log_mask)
                    logging.info(f"found {color}, image {self.fnum}, but was stopped by safeguards")
