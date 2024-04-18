import cv2
from ColorDetection import ColorDetection

#image_path = '/Users/arend/Desktop/NewTestImage/NormPos/Red/NewRed.jpg' #WORKS
#image_path = '/Users/arend/Desktop/NewTestImage/NormPos/Green/NewGreen.jpg' #WORKS
#image_path = '/Users/arend/Desktop/NewTestImage/NormPos/Green/LightGreen.jpeg' #WORKS
#image_path = '/Users/arend/Desktop/NewTestImage/NormPos/Yellow/NewYellow.jpg' #WORKS

# Load the image using OpenCV
image = cv2.imread(image_path)

# Create an instance of the ColorDetection class
detector = ColorDetection()

# Detect colored squares in the image
result, color, middle_point = detector.detect_colored_square(image)

# Display result
if result is not None:
    cv2.imshow('Square Detection', result)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
    print("Detected color:", color)
    print("Middle point coordinates:", middle_point)
else:
    print("No square detected.")