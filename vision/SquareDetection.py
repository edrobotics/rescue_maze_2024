import cv2
import numpy as np

def increase_brightness(image, value=115):
    # Convert the image to HSV
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    
    # Split the channels
    h, s, v = cv2.split(hsv)
    
    # Increase the value channel (brightness)
    v = cv2.add(v, value)
    
    # Merge the channels
    hsv = cv2.merge((h, s, v))
    
    # Convert back to BGR
    brighter_image = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)
    
    return brighter_image

def detect_square(image, min_area):
    # Convert the image to grayscale
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    
    # Apply Gaussian blur to reduce noise
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)
    
    # Detect edges using Canny edge detection
    edges = cv2.Canny(blurred, 50, 150)
    
    # Find contours in the edged image
    contours, _ = cv2.findContours(edges.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    # Loop over the contours
    for contour in contours:
        # Approximate the contour
        peri = cv2.arcLength(contour, True)
        approx = cv2.approxPolyDP(contour, 0.04 * peri, True)  # Adjust the approximation parameter
        
        # If the contour has four vertices, it might be a square
        if len(approx) == 4:
            # Check if the area of the contour is greater than the minimum area
            if cv2.contourArea(approx) > min_area:
                # Calculate the bounding box for the contour
                x, y, w, h = cv2.boundingRect(approx)
                
                # Calculate aspect ratio
                aspect_ratio = float(w) / h
                
                # Filter based on aspect ratio to ensure it resembles a square
                if 0.8 <= aspect_ratio <= 1.2:
                    # Draw a bounding box around the square
                    cv2.drawContours(image, [approx], -1, (255, 100, 0), 2)
                    return image

    return None

# Load the image
image_path = '/Users/arend/Desktop/NewTestImage/NormPos/Red/NewRed.jpg'
#image_path = '/Users/arend/Desktop/NewTestImage/NormPos/Red/NewRed.jpg'
#image_path = '/Users/arend/Desktop/NewTestImage/NormPos/Red/RedShadow.png'
#image_path = '/Users/arend/Desktop/NewTestImage/NormPos/Red/RedShaky.png' CANT DETECT (NOT SQUARE)
#image_path = '/Users/arend/Desktop/NewTestImage/NormPos/Red/RedVeryShaky.png' CANT DETECT (NOT SQUARE)
#image_path = '/Users/arend/Desktop/NewTestImage/NormPos/Red/RedRotated.png'
#image_path = '/Users/arend/Desktop/NewTestImage/NormPos/Red/RedRotatedLittleShaky.png' CANT DETECT (NOT SQUARE)

#image_path = '/Users/arend/Desktop/NewTestImage/NormPos/Green/NewGreen.jpg'
#image_path = '/Users/arend/Desktop/NewTestImage/NormPos/Green/GreenShadow.png'
#image_path = '/Users/arend/Desktop/NewTestImage/NormPos/Green/GreenOddColor.png'
#image_path = '/Users/arend/Desktop/NewTestImage/NormPos/Green/GreenShaky.png' CANT DETECT (NOT SQUARE)
#image_path = '/Users/arend/Desktop/NewTestImage/EARLPOS/Green/EarlGreen.png'
#image_path = '/Users/arend/Desktop/NewTestImage/NormPos/Green/LightGreen.jpeg' MORE TEST with LIGHT GREEN!

#image_path = '/Users/arend/Desktop/NewTestImage/NormPos/Yellow/NewYellow2.jpg' #ADJUSTED DARK
#image_path = '/Users/arend/Desktop/NewTestImage/NormPos/Yellow/NewYellow.jpg'
image_path = '/Users/arend/Desktop/NewTestImage/NormPos/Yellow/NewYellow copy 3.jpg'
#image_path = '/Users/arend/Desktop/NewTestImage/NormPos/Yellow/cam1179.png'

image = cv2.imread(image_path)

# Set the minimum area for the square
min_area = 5000  # Adjust this value according to your needs

# Increase brightness
brighter_image = increase_brightness(image, value=115)

# Detect the square
result = detect_square(brighter_image, min_area)

if result is not None:
    # Display the result
    cv2.imshow('Square Detection', result)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
else:
    print("No square detected.")
