import logging
import cv2




class Helper:
    def __init__(self):
        pass






    def putText(self, text, image = None, pos = (5,5)):
        if image is None: pass #out
        font                   = cv2.FONT_HERSHEY_SIMPLEX
        fontScale              =  1/2
        color                  = (255,255,255)
        thickness              =   1
        try:
            cv2.putText(
                image = image,
                text= str(text),
                pos = pos,
                font =font,
                fontScale=fontScale,
                color=color,
                thickness=thickness)
        except Exception as ex:
            logging.exception("could not put text")
            logging.debug(f"textpos: {pos}")
            print("failed putting text")
    