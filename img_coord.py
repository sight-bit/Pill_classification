import cv2
import numpy as np

def p(event, x, y, flag, param):
    print("x : ", x, " y : ", y)

img = cv2.imread('./image/00/0.jpg')
cv2.namedWindow('image')
cv2.setMouseCallback('image', p)

while(1):
    cv2.imshow('image', img)
    if cv2.waitKey(25) == 27:
        cv2.destroyAllWindows()
        break
