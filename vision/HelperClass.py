import logging
import cv2




class Helper:
    def __init__(self):
        pass






    def putText(self, text, image = None, pos = (10,25)):
        if image is None: pass #out
        font                   = cv2.FONT_HERSHEY_SIMPLEX
        fontScale              =  1
        color                  = (255,255,255)
        thickness              =   1
        try:
            image = cv2.putText(
                img = image,
                text= str(text),
                org = pos,
                fontFace=font,
                fontScale=fontScale,
                color=color,
                thickness=thickness, )
        except Exception as ex:
            logging.exception("could not put text")
            logging.debug(f"textpos: {pos}")
            print("failed putting text")
        return image
    