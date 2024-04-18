import cv2
import numpy as np

class ColoredSquareDetection:
    def __init__(self, min_area=5000, debug = False):
        self.debug = False
        self.min_area = min_area

    def increase_brightness(self, image, value=115):
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

    def detect_square(self, image):
        
        # Crop the image horizontally from the upper and lower edges
        #image = image[50:-50, :]
    
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
                if cv2.contourArea(approx) > self.min_area:
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
                            if self.debug: print("Color: Red, Position: ({}, {})".format(x + w // 2, y + h // 2))
                            return image, "red", (x + w // 2, y + h // 2)
                        else:  # If no red is found, draw the square in green and add text
                            cv2.drawContours(image, [approx], -1, (0, 255, 0), 2)
                            text = "Green ({}, {})".format(x + w // 2, y + h // 2)
                            cv2.putText(image, text, (x, y), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                            if self.debug: print("Color: Green, Position: ({}, {})".format(x + w // 2, y + h // 2))
                            return image,  "green", (x + w // 2, y + h // 2) 

        return self.detect_yellow_square(original_image=image)



        #return None, None, None

    def detect_yellow_square(self, original_image):
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
