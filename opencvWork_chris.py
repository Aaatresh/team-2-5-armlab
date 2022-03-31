import argparse
from ctypes import sizeof
import sys
import cv2
import numpy as np

hue_val_low = 0
hue_val_high = 179

sat_val_low = 0
sat_val_high = 255

val_val_low = 0
val_val_high = 255

def on_trackbar_hueLO(val):
    global hue_val_low
    hue_val_low = val
    show_image()

def on_trackbar_hueHI(val):
    global hue_val_high
    hue_val_high = val
    show_image()

def on_trackbar_satLO(val):
    global sat_val_low
    sat_val_low = val
    show_image()

def on_trackbar_satHI(val):
    global sat_val_high
    sat_val_high = val
    show_image()

def on_trackbar_valLO(val):
    global val_val_low
    val_val_low = val
    show_image()

def on_trackbar_valHI(val):
    global val_val_high
    val_val_high = val
    show_image()

def show_image():
 
    redLowHSV[0] = hue_val_low
    redHighHSV[0] = hue_val_high    
    redLowHSV[1] = sat_val_low
    redHighHSV[1] = sat_val_high   
    redLowHSV[2] = val_val_low
    redHighHSV[2] = val_val_high   
    mask = cv2.inRange(imgMasked_hsv,redLowHSV, redHighHSV)
    aftersliders = cv2.bitwise_and(imgMasked_rgb,imgMasked_rgb,mask=mask)
    cv2.imshow("Image window", imgMasked_rgb)
    cv2.imshow("Threshold window", aftersliders)


img = 'armlab_opencv_examples-master/image_all_blocks.png'
imgD = 'armlab_opencv_examples-master/depth_all_blocks.png'


# img = 'armlab_opencv_examples-master/image_test.png'
# imgD = 'armlab_opencv_examples-master/depth_test.png'


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
# print(depth_data)
thresh = cv2.bitwise_and(cv2.inRange(depth_data, lower, upper), mask)

# depending on your version of OpenCV, the following line could be:
contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
# _, contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

# print(depth_data.max())
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

redLowHSV = np.array([0,0,0])
redHighHSV = np.array([179,255,255])
RedMask = cv2.inRange(imgMasked_hsv,redLowHSV,redHighHSV)
redsOnly = cv2.bitwise_and(imgMasked_rgb,imgMasked_rgb,mask=RedMask)

cv2.imshow("Threshold window", redsOnly)
cv2.createTrackbar("Hue High Limit", "Threshold window" , 0, 179, on_trackbar_hueHI)
cv2.createTrackbar("Hue Low Limit", "Threshold window" , 0, 179, on_trackbar_hueLO)
cv2.createTrackbar("Sat High Limit", "Threshold window" , 0, 255, on_trackbar_satHI)
cv2.createTrackbar("Sat Low Limit", "Threshold window" , 0, 255, on_trackbar_satLO)
cv2.createTrackbar("Val High Limit", "Threshold window" , 0, 255, on_trackbar_valHI)
cv2.createTrackbar("Val Low Limit", "Threshold window" , 0, 255, on_trackbar_valLO)

# ###Done identifying color ranges, now use contours to id pixel regions corresponding to blocks

# thresh = cv2.bitwise_and(cv2.inRange(depth_data, lower, upper), mask)
# # depending on your version of OpenCV, the following line could be:
# contours, depth_data_ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

# ###block pixel sets id-ed, now check their avg color to see which of our ranges it falls in



purpleLowHSV = np.array([0,0,0])
purpleHighHSV = np.array([179,255,255])
purple_hue = np.array([123,173])
purpleLowHSV[0] = purple_hue[0]
purpleHighHSV[0] = purple_hue[1]
purpleMask = cv2.inRange(imgMasked_hsv,purpleLowHSV,purpleHighHSV)
purpleOnly = cv2.bitwise_and(imgMasked_rgb,imgMasked_rgb,mask=purpleMask)
contours, _ = cv2.findContours(purpleMask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
print(contours)
cv2.drawContours(purpleOnly, contours, -1, (0,255,255), thickness=1)
cv2.namedWindow("Purple window", cv2.WINDOW_NORMAL)
cv2.imshow("Purple window", purpleOnly)


while True:
  k = cv2.waitKey(10)
  if k == 27:
    break
cv2.destroyAllWindows()