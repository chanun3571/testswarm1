import cv2 
import numpy as np
greenBGR = np.uint8([[[202,9,240]]])
 
hsv_green = cv2.cvtColor(greenBGR,cv2.COLOR_BGR2HSV)
print (hsv_green)