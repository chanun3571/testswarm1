import cv2

cap = cv2.VideoCapture(0)
# cap.set(cv2.CAP_PROP_FRAME_WIDTH, 2560)
# cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 360)

num = 0

while cap.isOpened():

    succes1, img = cap.read()
    # img = cv2.flip(img,0)
    # left = img[:720,0:1280]
    # right = img[:720,1280:]
    left = img[:360,0:640]
    right = img[:360,640:]
    k = cv2.waitKey(5)

    if k == 27:
        break
    elif k == ord('s'): # wait for 's' key to save and exit
        cv2.imwrite('images/stereoLeft/imageL' + str(num) + '.png', left)
        cv2.imwrite('images/stereoRight/imageR' + str(num) + '.png', right)
        print("images saved!")
        num += 1

    cv2.imshow('left cam',left)
    cv2.imshow('right cam',right)

# Release and destroy all windows before termination
cap.release()
cv2.destroyAllWindows()