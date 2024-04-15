import cv2
import numpy as np

def increase_brightness(image, value=115):
    # Convert image to HSV  
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    
    # Split channels
    h, s, v = cv2.split(hsv)
    
    # Increase the value channel (brightness)
    v = cv2.add(v, value)
    
    # Merge the channels
    hsv = cv2.merge((h, s, v))
    
    # Convert back to BGR
    brighter_image = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)
    
    return brighter_image

def detect_square(image, min_area):
    # Convert image to grayscale
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    
    # Gaussian blur
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)
    
    # Detect edges with Canny edge detection
    edges = cv2.Canny(blurred, 50, 150)
    
    # Find contours
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
                    
                    # Crop the square region
                    square_region = image[y:y+h, x:x+w]
                    
                    # Convert image to HSV
                    hsv_square = cv2.cvtColor(square_region, cv2.COLOR_BGR2HSV)

                    # Define color thresholds for red
                    lower_red = np.array([0, 100, 100])
                    upper_red = np.array([10, 255, 255])
                    mask = cv2.inRange(hsv_square, lower_red, upper_red)

                    # Bitwise-AND mask and original image
                    red = cv2.bitwise_and(square_region, square_region, mask=mask)

                    if np.any(red):  # If square is red
                        # Draw the square in red and add text
                        cv2.drawContours(image, [approx], -1, (0, 0, 255), 2)
                        text = "Red ({}, {})".format(x + w // 2, y + h // 2)
                        cv2.putText(image, text, (x, y), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
                        print("Color: Red, Position: ({}, {})".format(x + w // 2, y + h // 2))
                        return image
                    else:  # If no red is found, draw the square in green and add text
                        cv2.drawContours(image, [approx], -1, (0, 255, 0), 2)
                        text = "Green ({}, {})".format(x + w // 2, y + h // 2)
                        cv2.putText(image, text, (x, y), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                        print("Color: Green, Position: ({}, {})".format(x + w // 2, y + h // 2))
                        return image

    return None

def detect_yellow_square(original_image):
    # Convert image to HSV
    hsv_image = cv2.cvtColor(original_image, cv2.COLOR_BGR2HSV)

    # Define color thresholds for yellow
    lower_yellow = np.array([20, 100, 100])
    upper_yellow = np.array([30, 255, 255])

    # Create mask for yellow color
    mask_yellow = cv2.inRange(hsv_image, lower_yellow, upper_yellow)

    # Find contours
    contours, _ = cv2.findContours(mask_yellow, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Loop over the contours
    for contour in contours:
        # Calculate contour area
        area = cv2.contourArea(contour)

        # If the contour area is within the specified range, it might be the square
        if area >= 1500:  #SPECIFIED RANGE
            # Draw a bounding box around the contour
            x, y, w, h = cv2.boundingRect(contour)
            cv2.rectangle(original_image, (x, y), (x + w, y + h), (0, 255, 0), 2)
            
            # Calculate the middle point of the square
            middle_x = x + w // 2
            middle_y = y + h // 2

            return original_image, 'yellow', (middle_x, middle_y)

    # If no square is found, return None
    return None, None, None

# Load the image
#image_path = '/Users/arend/Desktop/NewTestImage/NormPos/Red/NewRed.jpg' #WORKS
#image_path = '/Users/arend/Desktop/NewTestImage/NormPos/Red/NewRed.jpg' #WORKS
#image_path = '/Users/arend/Desktop/NewTestImage/NormPos/Red/RedShadow.png' #WORKS
#image_path = '/Users/arend/Desktop/NewTestImage/NormPos/Red/RedShaky.png' #SHAKY IMAGE -> DETECTS AS YELLOW BECAUSE NOT ONLY WALL IS VISIBLE
#image_path = '/Users/arend/Desktop/NewTestImage/NormPos/Red/RedVeryShaky.png' #CANT DETECT SHAKY IMAGE
#image_path = '/Users/arend/Desktop/NewTestImage/NormPos/Red/RedRotated.png' #WORKS
#image_path = '/Users/arend/Desktop/NewTestImage/NormPos/Red/RedRotatedLittleShaky.png' #CANT DETECT SHAKY IMAGE

#image_path = '/Users/arend/Desktop/NewTestImage/NormPos/Green/NewGreen.jpg' #WORKS
#image_path = '/Users/arend/Desktop/NewTestImage/NormPos/Green/GreenShadow.png' #WORKS
#image_path = '/Users/arend/Desktop/NewTestImage/NormPos/Green/GreenOddColor.png' #WORKS
#image_path = '/Users/arend/Desktop/NewTestImage/NormPos/Green/GreenShaky.png' #CANT DETECT SHAKY IMAGE
#image_path = '/Users/arend/Desktop/NewTestImage/EARLPOS/Green/EarlGreen.png' #WORKS (SHOULD NOT DETECT)
#image_path = '/Users/arend/Desktop/NewTestImage/NormPos/Green/LightGreen.jpeg' #WORKS

#image_path = '/Users/arend/Desktop/NewTestImage/NormPos/Yellow/NewYellow2.jpg' #ADJUSTED DARK (CANT DETECT YELLOW WITH DARK LIGHTNING CONDITION)
#image_path = '/Users/arend/Desktop/NewTestImage/NormPos/Yellow/NewYellow.jpg' #WORKS
#image_path = '/Users/arend/Desktop/NewTestImage/NormPos/Yellow/NewYellow copy 2.jpg' #WORKS
#image_path = '/Users/arend/Desktop/NewTestImage/NormPos/Yellow/cam1179.png' #WORKS


image = cv2.imread(image_path)

# Minimum area for the square
min_area = 5000  

# Increase brightness
brighter_image = increase_brightness(image, value=115)

# Detect the square
result = detect_square(brighter_image, min_area)

if result is not None:
    # Display
    cv2.imshow('Square Detection', result)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
else:
    print("No square detected.")

    # Call yellow square detection with the original image
    result, color, middle_point = detect_yellow_square(image.copy())

    if result is not None:
        # Display result
        cv2.imshow('Result', result)
        cv2.waitKey(0)
        cv2.destroyAllWindows()
        print("Detected color:", color)
        print("Middle point coordinates:", middle_point)
    else:
        print("No yellow square detected.")