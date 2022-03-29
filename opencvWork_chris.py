import argparse
from ctypes import sizeof
import sys
import cv2
import numpy as np

hue_val_low = 32768
hue_val_high = 32768


def on_trackbar_hueLO(val):
    global hue_val_low
    hue_val_low = val
    show_image()

def on_trackbar_hueHI(val):
    global hue_val_high
    hue_val_high = val
    show_image()

def show_image():
    RedMask = cv2.inRange(imgMasked_hsv,hue_val_low, hue_val_high)
    redsOnly = cv2.bitwise_and(imgMasked_rgb,imgMasked_rgb,mask=RedMask)
    cv2.imshow("Threshold window", redsOnly)


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
imgMasked_rgb = cv2.bitwise_and(rgbraw,rgbraw,mask=thresh)
imgMasked_hsv = cv2.cvtColor(imgMasked_rgb,cv2.COLOR_BGR2HSV)
# cv2.imshow("Threshold window", thresh)
# cv2.imshow("Threshold window", imgMasked_rgb)

redLowHSV = np.array([20,0,0])
redHighHSV = np.array([100,255,255])
RedMask = cv2.inRange(imgMasked_hsv,redLowHSV,redHighHSV)
redsOnly = cv2.bitwise_and(imgMasked_rgb,imgMasked_rgb,mask=RedMask)

cv2.imshow("Threshold window", redsOnly)
cv2.createTrackbar("Hue High Limit", "Threshold window" , 0, 255, on_trackbar_hueHI)
cv2.createTrackbar("Hue Low Limit", "Threshold window" , 0, 255, on_trackbar_hueLO)

while True:
  k = cv2.waitKey(10)
  if k == 27:
    break
cv2.destroyAllWindows()