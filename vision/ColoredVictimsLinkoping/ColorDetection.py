import cv2
import numpy as np

class ColorDetection:
    def __init__(self, min_area=5500, white_balance=None):
        self.min_area = min_area
        self.white_balance = white_balance

    def apply_white_balance(self, image):
        if self.white_balance is not None:
            image = cv2.cvtColor(image, cv2.COLOR_BGR2LAB)
            avg_a = np.average(image[:, :, 1])
            avg_b = np.average(image[:, :, 2])
            image[:, :, 1] = image[:, :, 1] - ((avg_a - self.white_balance[0]) * (image[:, :, 0] / 255.0) * 1.1)
            image[:, :, 2] = image[:, :, 2] - ((avg_b - self.white_balance[1]) * (image[:, :, 0] / 255.0) * 1.1)
            image = cv2.cvtColor(image, cv2.COLOR_LAB2BGR)
        return image

    def detect_colored_square(self, image):
        # Apply white balance
        image = self.apply_white_balance(image)

        # Crop the image horizontally from the upper and lower edges
        #image = image[50:-50, :]

        # Convert image to HSV
        hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # Define color thresholds
        lower_red = np.array([0, 100, 100])
        upper_red = np.array([10, 255, 255])

        lower_yellow = np.array([20, 100, 100])
        upper_yellow = np.array([30, 255, 255])

        lower_green_1 = np.array([74, 49, 32])  # Darker green
        upper_green_1 = np.array([95, 131, 65])  # Darker green

        lower_green_2 = np.array([35, 100, 100])  # Common nuance of green
        upper_green_2 = np.array([85, 255, 255])  # Common nuance of green

        # Combine all masks
        mask_red = cv2.inRange(hsv_image, lower_red, upper_red)
        mask_yellow = cv2.inRange(hsv_image, lower_yellow, upper_yellow)
        mask_green_1 = cv2.inRange(hsv_image, lower_green_1, upper_green_1)
        mask_green_2 = cv2.inRange(hsv_image, lower_green_2, upper_green_2)

        mask = mask_red + mask_yellow + mask_green_1 + mask_green_2

        # Find contours
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Loop over the contours
        for contour in contours:
            # Calculate contour area
            area = cv2.contourArea(contour)

            # If the contour area is within the specified range, it might be the square
            if self.min_area <= area:
                # Draw a bounding box around the contour
                x, y, w, h = cv2.boundingRect(contour)
                cv2.rectangle(image, (x, y), (x + w, y + h), (0, 255, 0), 2)
                
                # Calculate the middle point of the square
                middle_x = x + w // 2
                middle_y = y + h // 2
                
                # Determine the color detected
                if cv2.mean(mask_red[y:y+h, x:x+w])[0] > 0:  
                    color = 'red'
                elif cv2.mean(mask_yellow[y:y+h, x:x+w])[0] > 0: 
                    color = 'yellow'
                elif cv2.mean(mask_green_1[y:y+h, x:x+w])[0] > 0 or cv2.mean(mask_green_2[y:y+h, x:x+w])[0] > 0:
                    color = 'green'
                else:
                    color = 'unknown'

                self.image = image
                return image, color, (middle_x, middle_y)
        # If no square is found, return None
        return None, None, None