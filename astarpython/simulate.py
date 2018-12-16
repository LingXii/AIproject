import cv2
import os
import numpy as np
radius = 5
drawing = False
def draw_circle(event,x,y,flags,param):
    global drawing,img,radius

    if event == cv2.EVENT_LBUTTONDOWN:
        drawing = True

    elif event == cv2.EVENT_MOUSEMOVE:
        if drawing == True:
            cv2.circle(img, (x, y), radius, 255 , -1)

    elif event == cv2.EVENT_LBUTTONUP:
        drawing = False
        cv2.circle(img, (x, y), radius, 255, -1)

img = np.zeros((600,600))
cv2.namedWindow('image', cv2.WINDOW_NORMAL)
cv2.setMouseCallback('image', draw_circle)
while(1):
    cv2.imshow('image', img)
    k = cv2.waitKey(1) & 0xFF
    if k == ord('s'):
        cv2.imwrite('map.png',img)
        break