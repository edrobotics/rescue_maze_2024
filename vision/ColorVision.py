import cv2
import numpy as np

def is_square(approx):
    # Check if the polygon has 4 vertices and is convex (indicating a square)
    return len(approx) == 4 and cv2.isContourConvex(approx)

def check_area_and_white(contour, image):
    # Extract the region of interest (ROI) corresponding to the square
    x, y, w, h = cv2.boundingRect(contour) 
    roi = image[y:y+h, x:x+w]

    # Convert the ROI to grayscale
    gray_roi = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)

    # Threshold the grayscale image to create a binary image
    _, thresh = cv2.threshold(gray_roi, 200, 255, cv2.THRESH_BINARY)

    # Check if the thresholded image is empty (no white pixels)
    if cv2.countNonZero(thresh) == 0:
        print("Victim")
        return True  # Indicates that a white area is detected
    else:
        return False

def find_color_and_square(image):
    # Convert the image from BGR (Blue, Green, Red) to HSV (Hue, Saturation, Value) color space
    hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    # Define color ranges for red, yellow, and green in the HSV color space (TEST! SPECIAL CASES?)
    lower_red = np.array([0, 100, 100])
    upper_red = np.array([10, 255, 255])
    lower_yellow = np.array([20, 100, 100])
    upper_yellow = np.array([30, 255, 255])
    lower_green = np.array([60, 100, 100])
    upper_green = np.array([80, 255, 255])

    # Create masks for each color
    mask_red = cv2.inRange(hsv_image, lower_red, upper_red)
    mask_yellow = cv2.inRange(hsv_image, lower_yellow, upper_yellow)
    mask_green = cv2.inRange(hsv_image, lower_green, upper_green)

    # Combine the masks to detect red, yellow, and green colors in the image
    mask_combined = mask_red + mask_yellow + mask_green

    # Find contours in the combined mask
    contours, _ = cv2.findContours(mask_combined, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    for contour in contours:
        # Approximate the contour to a polygon with fewer vertices (CHANGE AND INCLUDE SPECIAL CASE?)
        epsilon = 0.02 * cv2.arcLength(contour, True)
        approx = cv2.approxPolyDP(contour, epsilon, True)

        # Check if the polygon is a square based on its vertices (CHANGE AND INCLUDE SPECIAL CASE?)
        if is_square(approx):
            color = None

            # Determine the color of the square based on a point inside it
            if cv2.pointPolygonTest(contour, tuple(approx[0][0]), True) > 0:
                color = "Red"
            elif cv2.pointPolygonTest(contour, tuple(approx[1][0]), True) > 0:
                color = "Yellow"
            elif cv2.pointPolygonTest(contour, tuple(approx[2][0]), True) > 0:
                color = "Green"

            if color:
                # Calculate the area of the contour (region enclosed by the polygon) (CHANGE AND INCLUDE SPECIAL CASE?)
                area = cv2.contourArea(contour)

                # Check if the area is approximately 16 cm^2 (CHANGE?)
                if 15 < area < 17:
                    print(f"Detected {color} color with area {area} cm^2")

                    # Check if the contour has a white square around itself (CHANGE AND INCLUDE SPECIAL CASE?)
                    if check_area_and_white(contour, image):
                        print("Square with white area (Victim)")
                    else:
                        print("Square without white area (No Victim)")
                else:
                    print(f"Detected {color} color with area {area} cm^2 (No Victim)")

# Read the input image
image = cv2.imread('/Users/arend/Desktop/ColorVisionCode/Victims/green/0E515.png')

# Check if the image is read successfully
if image is not None:
    # Call the function to find color and square in the image
    find_color_and_square(image)
else:
    print("Error reading the image.")

#Current theory: If the color has a white square around itself with a certain size it is ALWAYS a victim. The color can be detected with HIGH certainty using only thresholds.