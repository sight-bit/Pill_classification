import cv2
import os
import glob
import numpy as np

for i in range(5,16):
    os.system('mkdir ./test/s'+ str(i).zfill(2))
    for j in range(10):
        img_name = './image/s' + str(i).zfill(2) + '/' + str(j) + '.jpg'
        image = cv2.imread(img_name)
        gray = cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)
        
        (h, w) = image.shape[:2]


        mask = np.zeros(gray.shape,dtype=np.uint8)
        mask2= image.copy()

        m = np.zeros(gray.shape,dtype=np.uint8)

        cv2.circle(mask, (410,382), 340, (255, 255, 255), -1)
        cv2.circle(mask2, (410,382), 340, (0, 0, 0), -1)
        cv2.circle(mask2, (1200,375), 278, (0, 0, 0), -1)
        cv2.circle(m, (1200,375), 278, (255, 255, 255), -1)

        cX=410  
        cY=382
        c2X=1200
        c2Y=375

        dst = cv2.bitwise_and(image,image,None,mask)
        dst2 = cv2.bitwise_and(image,image,None,m)

        for k,ra in enumerate(range(0,350,10)):
            M = cv2.getRotationMatrix2D((cX, cY), ra, 1.0)
            rotated = cv2.warpAffine(dst, M, (w, h))
            
            ra_i = (-1) * ra
            M2 = cv2.getRotationMatrix2D((c2X, c2Y), ra_i, 1.0)
            rotated2 = cv2.warpAffine(dst2, M2, (w, h))
            
            res = cv2.bitwise_or(mask2,rotated)
            res = cv2.bitwise_or(res,rotated2)
            
            cv2.imwrite('test/s'+ str(i).zfill(2) +'/' + str(j) + '_' +str(k)+'.jpg',res)
