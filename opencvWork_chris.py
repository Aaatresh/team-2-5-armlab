import argparse
from ctypes import sizeof
import sys
import cv2
import numpy as np


img = 'armlab_opencv_examples-master/image_all_blocks.png'
imgD = 'armlab_opencv_examples-master/depth_all_blocks.png'

lower = 900
upper = 975

rgb_image = cv2.imread(img)
depth_data = cv2.imread(imgD, cv2.IMREAD_ANYCOLOR | cv2.IMREAD_ANYDEPTH)
cv2.namedWindow("Image window", cv2.WINDOW_NORMAL)
cv2.namedWindow("Threshold window", cv2.WINDOW_NORMAL)

"""mask out arm & outside board"""
mask = np.zeros_like(depth_data, dtype=np.uint8)
cv2.rectangle(mask, (200,117),(573,718), 255, cv2.FILLED)
cv2.rectangle(mask, (722,117),(1100,719), 255, cv2.FILLED)
cv2.rectangle(mask, (573,117),(722,440), 255, cv2.FILLED)

rgbraw = rgb_image.copy() #make a copy without markup

cv2.rectangle(rgb_image, (200,117),(573,718), (255, 0, 0), 2)
cv2.rectangle(rgb_image, (722,117),(1100,719), (255, 0, 0), 2)
cv2.rectangle(rgb_image, (573,117),(722,440), (255, 0, 0), 2)

# cv2.rectangle(rgb_image, (575,414),(723,720), (255, 0, 0), 2)
print(depth_data)
thresh = cv2.bitwise_and(cv2.inRange(depth_data, lower, upper), mask)

# depending on your version of OpenCV, the following line could be:
contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
# _, contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

print(depth_data.max())
cv2.drawContours(rgb_image, contours, -1, (0,255,255), 3)
print(thresh)
# cv2.imshow("Threshold window", thresh)
cv2.imshow("Image window", rgb_image)
# cv2.imshow("Threshold window", depth_data)


# we have the masked depth and area, now colorize the mask
imgMasked = cv2.bitwise_and(rgbraw,rgb_image,mask=thresh)
# cv2.imshow("Threshold window", thresh)
cv2.imshow("Threshold window", imgMasked)

print(rgb_image.shape)
print(depth_data.shape)


k = cv2.waitKey(0)
if k == 27:
    cv2.destroyAllWindows()