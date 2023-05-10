import numpy as np
import cv2


# Camera parameters to undistort and rectify images
cv_file = cv2.FileStorage()
cv_file.open('stereoMap.xml', cv2.FileStorage_READ)

stereoMapL_x = cv_file.getNode('stereoMapL_x').mat()
stereoMapL_y = cv_file.getNode('stereoMapL_y').mat()
stereoMapR_x = cv_file.getNode('stereoMapR_x').mat()
stereoMapR_y = cv_file.getNode('stereoMapR_y').mat()


# Open both cameras
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 360)



while cap.isOpened():
    success, img = cap.read()
    # img = cv2.flip(img,0)
    left = img[:360,0:640]
    right = img[:360,640:]


    # Undistort and rectify images
    right = cv2.remap(right, stereoMapR_x, stereoMapR_y, cv2.INTER_LANCZOS4, cv2.BORDER_CONSTANT, 0)
    left = cv2.remap(left, stereoMapL_x, stereoMapL_y, cv2.INTER_LANCZOS4, cv2.BORDER_CONSTANT, 0)
    # img = cv2.remap(img, stereoMapL_x, stereoMapL_y, cv2.INTER_LANCZOS4, cv2.BORDER_CONSTANT, 0)
         
    # Show the frames
    cv2.imshow("left cam", left)
    cv2.imshow("right cam", right) 
    # cv2.imshow("right cam", img) 


    # Hit "q" to close the window
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break


# Release and destroy all windows before termination
cap.release()

cv2.destroyAllWindows()