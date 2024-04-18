import cv2
from ColoredSquareDetection import ColoredSquareDetection

# Instance of the ColoredSquareDetection class
detector = ColoredSquareDetection()

# Load the image
#image_path = '/Users/arend/Desktop/NewTestImage/NormPos/Red/NewRed.jpg' #WORKS
#image_path = '/Users/arend/Desktop/NewTestImage/NormPos/Yellow/NewYellow.jpg' #WORKS
#image_path = '/Users/arend/Desktop/NewTestImage/NormPos/Green/GreenShadow.png' #WORKS

image = cv2.imread(image_path)

# Increase brightness
brighter_image = detector.increase_brightness(image, value=115)

# Detect the square
result = detector.detect_square(brighter_image)

if result is not None:
    # Display
    cv2.imshow('Square Detection', result)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
else:
    print("No square detected.")

    # Call yellow square detection with the original image
    result, color, middle_point = detector.detect_yellow_square(image.copy())

    if result is not None:
        # Display result
        cv2.imshow('Result', result)
        cv2.waitKey(0)
        cv2.destroyAllWindows()
        print("Detected color:", color)
        print("Middle point coordinates:", middle_point)
    else:
        print("No yellow square detected.")
