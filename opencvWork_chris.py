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


# Python3 program change RGB Color
# Model to HSV Color Model
 
def bgr_to_hsv(b, g, r):
 
    # R, G, B values are divided by 255
    # to change the range from 0..255 to 0..1:
    r, g, b = r / 255.0, g / 255.0, b / 255.0
 
    # h, s, v = hue, saturation, value
    cmax = max(r, g, b)    # maximum of r, g, b
    cmin = min(r, g, b)    # minimum of r, g, b
    diff = cmax-cmin       # diff of cmax and cmin.
 
    # if cmax and cmax are equal then h = 0
    if cmax == cmin:
        h = 0
     
    # if cmax equal r then compute h
    elif cmax == r:
        h = (60 * ((g - b) / diff) + 360) % 360
 
    # if cmax equal g then compute h
    elif cmax == g:
        h = (60 * ((b - r) / diff) + 120) % 360
 
    # if cmax equal b then compute h
    elif cmax == b:
        h = (60 * ((r - g) / diff) + 240) % 360
 
    # if cmax equal zero
    if cmax == 0:
        s = 0
    else:
        s = (diff / cmax) * 100
 
    # compute v
    v = cmax * 100
    return h, s, v
 

def retrieve_area_color(data, contour, labels):
    colorfound = "none"
    mask = np.zeros(data.shape[:2], dtype="uint8")
    cv2.drawContours(mask, [contour], -1, 255, -1)
    mean = np.uint8(cv2.mean(data, mask=mask)[:3])
    meanHSV_h, meanHSV_s, meanHSV_v = bgr_to_hsv(mean[0],mean[1],mean[2])
    meanHSV_h = meanHSV_h/2
    for label in labels:
        if label["color"][0] <= meanHSV_h <= label["color"][1]:
            colorfound = label["id"]
    if colorfound == "redHI" or colorfound == "redLO":
        colorfound = "red"
    return colorfound, meanHSV_h


font = cv2.FONT_HERSHEY_SIMPLEX
colors = list((
    {'id': 'redHI', 'color': (170, 179)},
    {'id': 'redLO', 'color': (0, 6)},
    {'id': 'orange', 'color': (7, 15)},
    {'id': 'yellow', 'color': (17, 38)},
    {'id': 'green', 'color': (55, 94)},
    {'id': 'blue', 'color': (84, 112)},
    {'id': 'purple', 'color': (123, 173)})
)
img = 'armlab_opencv_examples-master/image_blocks.png'
imgD = 'armlab_opencv_examples-master/depth_blocks.png'

annotate = {'red': (10, 10, 127),
    'orange': (30, 75, 150),
    'yellow': (30, 150, 200),
    'green': (20, 60, 20),
    'blue': (100, 50, 0),
    'purple': (100, 40, 80),
    'none': (0,0,0)}


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
contoursOG, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
# _, contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

# print(depth_data.max())
cv2.drawContours(rgb_image, contoursOG, -1, (0,255,255), 3)
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
purple_hue = np.array([55,94])
# purple_hue = np.array([123,173])
purpleLowHSV[0] = purple_hue[0]
purpleHighHSV[0] = purple_hue[1]
purpleMask = cv2.inRange(imgMasked_hsv,purpleLowHSV,purpleHighHSV)
purpleOnly = cv2.bitwise_and(imgMasked_rgb,imgMasked_rgb,mask=purpleMask)
contours, _ = cv2.findContours(purpleMask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
viableContours = []
for contour in contoursOG:
    if cv2.contourArea(contour) > 200:
        viableContours.append(contour)




for contour in viableContours:
    contourcolor, meanHSV = retrieve_area_color(rgbraw,contour,colors)
    # print(contourcolor)
    annotateColor = annotate[contourcolor]
    x = 30
    annotateColor = [annotateColor[0]+x,annotateColor[1]+x,annotateColor[2]+x]
    print(annotateColor)
    #cv2.drawContours(rgbraw, [contour], -1, [0,255,255], thickness=2)
    cv2.drawContours(rgbraw, [contour], -1, annotateColor, thickness=2)




cv2.namedWindow("Annotation window", cv2.WINDOW_NORMAL)
cv2.imshow("Annotation window", rgbraw)

while True:
  k = cv2.waitKey(10)
  if k == 27:
    break
cv2.destroyAllWindows()