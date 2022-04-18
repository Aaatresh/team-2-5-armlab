"""!
Class to represent the camera.
"""

from tkinter import CENTER
import cv2
import time
import numpy as np
from PyQt4.QtGui import QImage
from PyQt4.QtCore import QThread, pyqtSignal, QTimer
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from apriltag_ros.msg import *
from cv_bridge import CvBridge, CvBridgeError

from collections import OrderedDict

class Camera():
    """!
    @brief      This class describes a camera.
    """
    def __init__(self):
        """!
        @brief      Construcfalsets a new instance.
        """
        self.VideoFrame = np.zeros((720, 1280, 3)).astype(np.uint8)
        self.BlockFrame = np.zeros((720, 1280, 3)).astype(np.uint8)
        self.TagImageFrame = np.zeros((720, 1280, 3)).astype(np.uint8)
        self.DepthFrameRaw = np.zeros((720, 1280)).astype(np.uint16)
        """ Extra arrays for colormaping the depth image"""
        self.DepthFrameHSV = np.zeros((720, 1280, 3)).astype(np.uint8)
        self.DepthFrameRGB = np.array([])

        # mouse clicks & calibration variables
        self.cameraCalibrated = False
        self.intrinsic_matrix = np.array([])
        self.extrinsic_matrix = np.array([])
        self.last_click = np.array([0, 0])
        self.new_click = False
        self.rgb_click_points = np.zeros((5, 2), int)
        self.depth_click_points = np.zeros((5, 2), int)
        self.tag_detections = np.array([])
        self.tag_locations = [[-250, -25], [250, -25], [250, 275]]
        """ block info """
        self.block_contours = np.array([])
        self.color_indices = np.array([])
        self.block_detections = np.array([])

        self.block_detectionsCAMCOORD = np.array([])
        self.block_colors = []
        self.block_colors_H = []
        self.block_sizes = []
        self.block_color_nums = []

        self.color_indices = np.array([])

        self.block_colorsSTACKED = []
        self.block_colors_HSTACKED = []
        self.block_sizesSTACKED = []
        self.block_detectionsSTACKED =[]
        self.block_detectionsCAMCOORDSTACKED = np.array([])
        self.block_contoursSTACKED = []
        self.color_indicesSTACKED = np.array([])

        # https://thispointer.com/how-to-create-and-initialize-a-list-of-lists-in-python/
        for i in range(5):
            self.block_contoursSTACKED.append([])
            self.block_detectionsSTACKED.append([])

    def processVideoFrame(self):
        """!
        @brief      Process a video frame
        """
        cv2.drawContours(self.VideoFrame, self.block_contours, -1,
                         (255, 0, 255), 3)

    def ColorizeDepthFrame(self):
        """!
        @brief Converts frame to colormaped formats in HSV and RGB
        """
        self.DepthFrameHSV[..., 0] = self.DepthFrameRaw >> 1
        self.DepthFrameHSV[..., 1] = 0xFF
        self.DepthFrameHSV[..., 2] = 0x9F
        self.DepthFrameRGB = cv2.cvtColor(self.DepthFrameHSV,
                                          cv2.COLOR_HSV2RGB)

    def loadVideoFrame(self):
        """!
        @brief      Loads a video frame.
        """
        self.VideoFrame = cv2.cvtColor(
            cv2.imread("data/rgb_image.png", cv2.IMREAD_UNCHANGED),
            cv2.COLOR_BGR2RGB)

    def loadDepthFrame(self):
        """!
        @brief      Loads a depth frame.
        """
        self.DepthFrameRaw = cv2.imread("data/raw_depth.png",
                                        0).astype(np.uint16)

    def convertQtVideoFrame(self):
        """!
        @brief      Converts frame to format suitable for Qt

        @return     QImage
        """

        try:
            frame = cv2.resize(self.VideoFrame, (1280, 720))
            img = QImage(frame, frame.shape[1], frame.shape[0],
                         QImage.Format_RGB888)
            return img
        except:
            return None

    def convertQtDepthFrame(self):
        """!
       @brief      Converts colormaped depth frame to format suitable for Qt

       @return     QImage
       """
        try:
            img = QImage(self.DepthFrameRGB, self.DepthFrameRGB.shape[1],
                         self.DepthFrameRGB.shape[0], QImage.Format_RGB888)
            return img
        except:
            return None

    def convertQtTagImageFrame(self):
        """!
        @brief      Converts tag image frame to format suitable for Qt

        @return     QImage
        """

        try:
            frame = cv2.resize(self.TagImageFrame, (1280, 720))
            img = QImage(frame, frame.shape[1], frame.shape[0],
                         QImage.Format_RGB888)
            return img
        except:
            return None
    def convertQtBlockImageFrame(self):
        """!
        @brief      Converts block image frame to format suitable for Qt

        @return     QImage
        """

        frame = cv2.resize(self.BlockFrame, (1280, 720))
        # #self.blockDetector()
        # for contour in self.block_contours:
        #     cv2.drawContours(frame, [contour], -1, [255,255,255], thickness=2)

        # for coord in self.block_detectionsCAMCOORD:
        #     cv2.circle(frame, (coord[0], coord[1]), 3, (255, 255, 255), -1)

        self.blockDetector()

        img = QImage(frame, frame.shape[1], frame.shape[0],\
                          QImage.Format_RGB888)

        return img

        # try:
        #     frame = cv2.resize(self.VideoFrame, (1280, 720))
        #     # self.blockDetector()1280

        #     # for contour in self.block_contours:
        #     #     cv2.drawContours(frame, [contour], -1, [255,255,255], thickness=2)

        #     # for coord in self.block_detectionsCAMCOORD:
        #     #     cv2.circle(frame, (coord[0], coord[1]), 3, (255, 255, 255), -1)

        #     img = QImage(frame, frame.shape[1], frame.shape[0],
        #                  QImage.Format_RGB888)
        #     return img
        # except:
        #     return None

    def getAffineTransform(self, coord1, coord2):
        """!
        @brief      Find the affine matrix transform between 2 sets of corresponding coordinates.

        @param      coord1  Points in coordinate frame 1
        @param      coord2  Points in coordinate frame 2

        @return     Affine transform between coordinates.
        """
        pts1 = coord1[0:3].astype(np.float32)
        pts2 = coord2[0:3].astype(np.float32)
        print(cv2.getAffineTransform(pts1, pts2))
        return cv2.getAffineTransform(pts1, pts2)

    def loadCameraCalibration(self, file):
        """!
        @brief      Load camera intrinsic matrix from file.

                    TODO: use this to load in any calibration files you need to

        @param      file  The file
        """
        pass

    def blockDetector(self):
        """!
        @brief      Detect blocks from rgb

                    TODO: Implement your block detector here. You will need to locate blocks in 3D space and put their XYZ
                    locations in self.block_detections
        """

        self.BlockFrame = self.VideoFrame.copy()

        rgb_image = self.BlockFrame
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
            # {'id': 'redHI', 'color': (165, 179)},
            # {'id': 'redLO', 'color': (0, 6)},
            # {'id': 'orange', 'color': (7, 15)},
            # {'id': 'yellow', 'color': (17, 38)},
            # {'id': 'green', 'color': (55, 94)},
            # {'id': 'blue', 'color': (84, 112)},
            # {'id': 'purple', 'color': (123, 164)}

            {'id': 'redHI', 'color': (120, 150)},
            # {'id': 'redLO', 'color': (0, 6)},
            {'id': 'orange', 'color': (105, 115)}, #109-110 nom
            {'id': 'yellow', 'color': (80,100)}, #95 nom
            {'id': 'green', 'color': (40, 55)}, #45-51 nom
            {'id': 'blue', 'color': (10, 20)}, #15 nom
            {'id': 'purple', 'color': (160, 179)}, #170 nom
            {'id': 'purple', 'color': (0, 5)} #0 nom
            )
        )
        img = 'armlab_opencv_examples-master/image_all_blocks.png'
        imgD = 'armlab_opencv_examples-master/depth_all_blocks.png'

        # annotate = OrderedDict({'red': (10, 10, 127),
        #     'orange': (30, 75, 150),
        #     'yellow': (30, 150, 200),
        #     'green': (20, 60, 20),
        #     'blue': (100, 50, 0),
        #     'purple': (100, 40, 80),
        #     'none': (0,0,0)})
        annotate = OrderedDict()

        annotate['red'] = (10, 10, 127)
        annotate['orange'] = (30, 75, 150)
        annotate['yellow'] = (30, 150, 200)
        annotate['green'] = (20, 60, 20)
        annotate['blue'] = (100, 50, 0)
        annotate['purple'] = (100, 40, 80)
        annotate['none'] = (0,0,0)


        # img = 'armlab_opencv_examples-master/image_test.png'
        # imgD = 'armlab_opencv_examples-master/depth_test.png'


        # lower = 900
        # upper = 965
        lower = 0
        upper = 960
        # lower = 850
        # upper = 875

        halfslice = 4
        block_height_slices = np.array([[825-halfslice, 825+halfslice],[867-halfslice, 867+halfslice],[879-halfslice, 879+halfslice],[905-halfslice, 905+halfslice],[930-halfslice, 930+halfslice]])
        # #depth "slice" we consider to mean a stack is two blocks tall
        # lower2 = 0
        # upper2 = 0

        # #depth "slice" we consider to mean a stack is two blocks tall
        # lower3 = 0
        # upper3 = 0

        depth_data = self.DepthFrameRaw.copy()
        """mask out arm & outside board"""
        scoot = 25

        mask = np.zeros_like(depth_data, dtype=np.uint8)
        cv2.rectangle(mask, (200+scoot,117),(573+scoot,718), 255, cv2.FILLED)
        cv2.rectangle(mask, (722+scoot,117),(1100+scoot,719), 255, cv2.FILLED)
        cv2.rectangle(mask, (573+scoot,117),(722+scoot,420), 255, cv2.FILLED)

        rgbraw = rgb_image.copy() #make a copy without markup

        cv2.rectangle(rgb_image, (200+scoot,117),(573+scoot,718), (255, 0, 0), 2)
        cv2.rectangle(rgb_image, (722+scoot,117),(1100+scoot,719), (255, 0, 0), 2)
        cv2.rectangle(rgb_image, (573+scoot,117),(722+scoot,420), (255, 0, 0), 2)

        # cv2.rectangle(rgb_image, (575,414),(723,720), (255, 0, 0), 2)
        thresh = cv2.bitwise_and(cv2.inRange(depth_data, lower, upper), mask)
        # self.BlockFrame = thresh.copy()

        # depending on your version of OpenCV, the following line could be:
        # return_vals = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # print("return vals: ", return_vals)

        # contoursOG, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        _, contoursOG, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        # print("contoursOG")
        # print(contoursOG)
        # we have the masked depth and area, now colorize the mask
        imgMasked_rgb = cv2.bitwise_and(rgbraw,rgbraw,mask=thresh)
        imgMasked_hsv = cv2.cvtColor(imgMasked_rgb,cv2.COLOR_BGR2HSV)

        redLowHSV = np.array([0,0,0])
        redHighHSV = np.array([179,255,255])
        RedMask = cv2.inRange(imgMasked_hsv,redLowHSV,redHighHSV)
        redsOnly = cv2.bitwise_and(imgMasked_rgb,imgMasked_rgb,mask=RedMask)

        # ###Done identifying color ranges, now use contours to id pixel regions corresponding to blocks

        # ###block pixel sets id-ed, now check their avg color to see which of our ranges it falls in

        purpleLowHSV = np.array([0,0,0])
        purpleHighHSV = np.array([179,255,255])
        purple_hue = np.array([55,94])
        # purple_hue = np.array([123,173])
        purpleLowHSV[0] = purple_hue[0]
        purpleHighHSV[0] = purple_hue[1]
        purpleMask = cv2.inRange(imgMasked_hsv,purpleLowHSV,purpleHighHSV)
        purpleOnly = cv2.bitwise_and(imgMasked_rgb,imgMasked_rgb,mask=purpleMask)
        _, contours, _ = cv2.findContours(purpleMask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        viableContours = []
        # print("start area check")

        topslice = 2 #half the thickness of the top surface depth slice for towers
        topSurfs_contoursOG = []
        block_orientations = []

        for contour in contoursOG:
            topsliceXYZ_indices = []
# ####################################    ##work in progress: only ID the top of a tower
# #https://stackoverflow.com/questions/44501723/how-to-merge-contours-in-opencv
# #ctr = np.array(list_of_pts).reshape((-1,1,2)).astype(np.int32)

#             #find the top pixel in the contour
#             maxZ = 976
#             for i in range (0, len(contour)):
#                 if depth_data[contour[i][0][1],contour[i][0][0]] < maxZ and depth_data[contour[i][0][1],contour[i][0][0]] != 0:
#                     # note that depth data uses [camY][camX] indexing, NOT xy!
#                     maxZ = depth_data[contour[i][0][1],contour[i][0][0]] # note that depth data uses [camY][camX] indexing, NOT xy!

#             #define depth slice within which we consider it part of the top surface
#             maxZupperlim = maxZ + topslice
#             maxZlowerlim = maxZ - topslice
            
#             ###NEW: try making a new thresh using the range around the top of the contour
#             threshTOPSURF = cv2.bitwise_and(cv2.inRange(depth_data, maxZlowerlim, maxZupperlim), mask)
#             # make new contours with this thresh
#             _, contoursAtTop, _ = cv2.findContours(threshTOPSURF, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
#             # print("contoursAtTop")
#             print("contour:",type(contour))
#             if(maxZ != 976):
#                 for contourTop in contoursAtTop:
#                     print("contourTop",contourTop)
#                     # print("for loop pass flag")
#                     if cv2.contourArea(contourTop) > 200 and cv2.contourArea(contourTop) < 5000:# and cv2.contourArea(contour) < 7000: #check contour area, filter out anything too small
#                         #make a rotated bounding rectangle (7b at https://docs.opencv.org/4.x/dd/d49/tutorial_py_contour_features.html)
#                         rect = cv2.minAreaRect(contourTop)
#                         box = cv2.boxPoints(rect)
#                         box = np.int0(box)
#                         if 0.5 < rect[1][1]/rect[1][0] < 2.0: #check squareness. if square enough, consider a block
#                             cv2.drawContours(rgb_image,[box],0,(0,0,255),2)                        
#                             viableContours.append(contourTop)
# #         ##################################

#         centroids = []
#         centroidsCAMCOORD = []
#         color_indices = []
#         del self.block_colors[:]
#         del self.block_colors_H[:]
#         for contour in viableContours:
#             contourcolor, meanHSVh = retrieve_area_color(rgbraw,contour,colors)
#             annotateColor = annotate[contourcolor]
#             annotateColor_index = annotate.keys().index(contourcolor)
#             color_indices.append(annotateColor_index)

#             self.block_colors.append(contourcolor)
#             self.block_colors_H.append(meanHSVh)
#             self.block_sizes.append(cv2.contourArea(contour))
#             x = 30
#             annotateColor = [annotateColor[0]+x,annotateColor[1]+x,annotateColor[2]+x]
#             # cv2.drawContours(rgb_image, contour, -1, annotateColor, 3)

#             # cv2.drawContours(rgbraw, [contour], -1, [0,255,255], thickness=2)
#             # cv2.drawContours(rgbraw, [contour], -1, annotateColor, thickness=2)

#             # https://pyimagesearch.com/2016/02/01/opencv-center-of-contour/
#             # find centroids
#             M = cv2.moments(contour)
#             cX = int(M["m10"] / M["m00"])
#             cY = int(M["m01"] / M["m00"])
#             worldCoordCentroid = self.camXY2worldXYZ(cX,cY)
#             centroids.append(worldCoordCentroid)

#             cv2.circle(rgb_image, (cX, cY), 3, (255, 255, 255), -1)

#             centroidsCAMCOORD.append([cX,cY])

#         # cv2.drawContours(rgb_image, viableContours, -1, (0,255,255), 3)
#         self.block_detections = np.array(centroids)
#         self.block_detectionsCAMCOORD = np.array(centroidsCAMCOORD)
#         self.block_contours = np.array(viableContours)
#         self.color_indices = np.array(color_indices).reshape(-1, 1)
                
# #now only run the rest of the detection refinement on top surfaces

###Uncomment below here to restore
        # for contour in topSurfs_contoursOG:
        for contour in contoursOG:
##################################
            # print("contour")
            # print(contour)
            # print("area: ", cv2.contourArea(contour))
            # print(contour[0])
            if cv2.contourArea(contour) > 200 and cv2.contourArea(contour) < 5000:# and cv2.contourArea(contour) < 7000: #check contour area, filter out anything too small


                #make a rotated bounding rectangle (7b at https://docs.opencv.org/4.x/dd/d49/tutorial_py_contour_features.html)
                rect = cv2.minAreaRect(contour)
                box = cv2.boxPoints(rect)
                box = np.int0(box)
                if 0.5 < rect[1][1]/rect[1][0] < 2.0: #check squareness. if square enough, consider a block

                    cv2.drawContours(rgb_image,[box],0,(0,0,255),2)
                    # print([box])
                    # print(cv2.contourArea(contour))
                    # print(rect[1][1]/rect[1][0])
                    # print("valid contour")
                    # print(contour)
                    viableContours.append(contour)
                # elif(0.75 < rect[1][0]/rect[1][1] < 1.70):
                #     cv2.drawContours(rgb_image,[box],0,(0,0,255),2)

                #     # print(cv2.contourArea(contour))
                #     # print(rect[1][1]/rect[1][0])
                #     # print("valid contour")
                #     # print(contour)
                #     viableContours.append(contour)


        centroids = []
        centroidsCAMCOORD = []
        color_indices = []
        del self.block_colors[:]
        del self.block_colors_H[:]
        for contour in viableContours:
            contourcolor, meanHSVh = retrieve_area_color(rgbraw,contour,colors)
            annotateColor = annotate[contourcolor]
            annotateColor_index = annotate.keys().index(contourcolor)
            color_indices.append(annotateColor_index)

            self.block_colors.append(contourcolor)
            self.block_colors_H.append(meanHSVh)
            self.block_sizes.append(cv2.contourArea(contour))
            x = 30
            annotateColor = [annotateColor[0]+x,annotateColor[1]+x,annotateColor[2]+x]
            # cv2.drawContours(rgb_image, contour, -1, annotateColor, 3)

            # cv2.drawContours(rgbraw, [contour], -1, [0,255,255], thickness=2)
            # cv2.drawContours(rgbraw, [contour], -1, annotateColor, thickness=2)

            # https://pyimagesearch.com/2016/02/01/opencv-center-of-contour/
            # find centroids
            M = cv2.moments(contour)
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])
            worldCoordCentroid = self.camXY2worldXYZ(cX,cY)
            centroids.append(worldCoordCentroid)

            cv2.circle(rgb_image, (cX, cY), 3, (255, 255, 255), -1)

            centroidsCAMCOORD.append([cX,cY])

        # cv2.drawContours(rgb_image, viableContours, -1, (0,255,255), 3)
        self.block_detections = np.array(centroids)
        self.block_detectionsCAMCOORD = np.array(centroidsCAMCOORD)
        self.block_contours = np.array(viableContours)
        self.color_indices = np.array(color_indices).reshape(-1, 1)







        #################BLOCK HEIGHT SCANS################################
        centroidsSTACKED = []
        centroidsCAMCOORDSTACKED = []
        color_indicesSTACKED = []
        del self.block_colorsSTACKED[:]
        del self.block_colors_HSTACKED[:]
        for e, slice in enumerate(block_height_slices):
            depth_data = self.DepthFrameRaw.copy()
            """mask out arm & outside board"""
            scoot = 25

            mask = np.zeros_like(depth_data, dtype=np.uint8)
            cv2.rectangle(mask, (200+scoot,117),(573+scoot,718), 255, cv2.FILLED)
            cv2.rectangle(mask, (722+scoot,117),(1100+scoot,719), 255, cv2.FILLED)
            cv2.rectangle(mask, (573+scoot,117),(722+scoot,420), 255, cv2.FILLED)

            rgbraw = rgb_image.copy() #make a copy without markup

            cv2.rectangle(rgb_image, (200+scoot,117),(573+scoot,718), (255, 0, 0), 2)
            cv2.rectangle(rgb_image, (722+scoot,117),(1100+scoot,719), (255, 0, 0), 2)
            cv2.rectangle(rgb_image, (573+scoot,117),(722+scoot,420), (255, 0, 0), 2)

            # cv2.rectangle(rgb_image, (575,414),(723,720), (255, 0, 0), 2)
            thresh = cv2.bitwise_and(cv2.inRange(depth_data, slice[0], slice[1]), mask)
            # self.BlockFrame = thresh.copy()

            # depending on your version of OpenCV, the following line could be:

            _, contoursOG, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)


            # we have the masked depth and area, now colorize the mask
            imgMasked_rgb = cv2.bitwise_and(rgbraw,rgbraw,mask=thresh)
            imgMasked_hsv = cv2.cvtColor(imgMasked_rgb,cv2.COLOR_BGR2HSV)

            redLowHSV = np.array([0,0,0])
            redHighHSV = np.array([179,255,255])
            RedMask = cv2.inRange(imgMasked_hsv,redLowHSV,redHighHSV)
            redsOnly = cv2.bitwise_and(imgMasked_rgb,imgMasked_rgb,mask=RedMask)

            # ###Done identifying color ranges, now use contours to id pixel regions corresponding to blocks

            # ###block pixel sets id-ed, now check their avg color to see which of our ranges it falls in

            purpleLowHSV = np.array([0,0,0])
            purpleHighHSV = np.array([179,255,255])
            purple_hue = np.array([55,94])
            purpleLowHSV[0] = purple_hue[0]
            purpleHighHSV[0] = purple_hue[1]
            purpleMask = cv2.inRange(imgMasked_hsv,purpleLowHSV,purpleHighHSV)
            purpleOnly = cv2.bitwise_and(imgMasked_rgb,imgMasked_rgb,mask=purpleMask)
            _, contours, _ = cv2.findContours(purpleMask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            viableContours = []

            for contour in contoursOG:

                if cv2.contourArea(contour) > 200 and cv2.contourArea(contour) < 5000:# and cv2.contourArea(contour) < 7000: #check contour area, filter out anything too small


                    #make a rotated bounding rectangle (7b at https://docs.opencv.org/4.x/dd/d49/tutorial_py_contour_features.html)
                    rect = cv2.minAreaRect(contour)
                    box = cv2.boxPoints(rect)
                    box = np.int0(box)
                    if 0.5 < rect[1][1]/rect[1][0] < 2.0: #check squareness. if square enough, consider a block
                        cv2.drawContours(rgb_image,[box],0,(0,0,255),2)
                        viableContours.append(contour)

            del self.block_colorsSTACKED[:]
            del self.block_colors_HSTACKED[:]
            for contour in viableContours:
                contourcolor, meanHSVh = retrieve_area_color(rgbraw,contour,colors)
                annotateColor = annotate[contourcolor]
                annotateColor_index = annotate.keys().index(contourcolor)
                color_indicesSTACKED.append(annotateColor_index)

                self.block_colorsSTACKED.append(contourcolor)
                self.block_colors_HSTACKED.append(meanHSVh)
                self.block_sizesSTACKED.append(cv2.contourArea(contour))
                x = 30
                annotateColor = [annotateColor[0]+x,annotateColor[1]+x,annotateColor[2]+x]

                # https://pyimagesearch.com/2016/02/01/opencv-center-of-contour/
                # find centroids
                M = cv2.moments(contour)
                cX = int(M["m10"] / M["m00"])
                cY = int(M["m01"] / M["m00"])
                worldCoordCentroid = self.camXY2worldXYZ(cX,cY)
                centroidsSTACKED.append(worldCoordCentroid)

                cv2.circle(rgb_image, (cX, cY), 3, (255, 255, 255), -1)

                centroidsCAMCOORDSTACKED.append([cX,cY])

                

            cv2.drawContours(rgb_image, viableContours, -1, (0,255,0), 3)
            self.block_detectionsSTACKED[e] = np.array(centroidsSTACKED)
            self.block_detectionsCAMCOORDSTACKED = np.array(centroidsCAMCOORDSTACKED)
            self.block_contoursSTACKED[e] = np.array(viableContours)

            self.color_indicesSTACKED = np.array(color_indicesSTACKED).reshape(-1, 1)




    def detectBlocksInDepthImage(self):
        """!
        @brief      Detect blocks from depth

                    TODO: Implement a blob detector to find blocks in the depth image
        """
        pass

    def camXY2worldXYZ(self,camX,camY):

        extMtx = self.extrinsic_matrix
        # print("ExtMtx After\n")
        # print(extMtx)
        extMtxR = np.array([extMtx[0,0:3],extMtx[1,0:3],extMtx[2,0:3]])
        extMtxt = np.array([[extMtx[0,3]],[extMtx[1,3]],[extMtx[2,3]]])

        extMtxRinv = np.linalg.inv(extMtxR)
        negextMtxRinv_t = np.matmul(-extMtxRinv,extMtxt)

        invExtMtx = np.block([
            [extMtxRinv,negextMtxRinv_t],
            [np.zeros((1,3)),np.ones((1,1))]
        ])

        Kteam =   np.array([[954.6327,0,629.4831],[0,968.4867,386.4730],[0,0,1.0000]],dtype=np.float32)
        Kinv = np.linalg.inv(Kteam)

        Pteam = np.array([[975.5068,0,628.0801],[0,993.6321,386.8233],[0, 0, 1.0000]])
        Pinv = np.linalg.inv(Pteam)

        distcoeff = np.array([0.1505,-0.2453,0.0002,-0.0014]).reshape((4,1))
        
        if self.DepthFrameRaw.any() != 0:
            z = self.DepthFrameRaw[camY][camX]

            uv1 = np.array([[camX],[camY],[1]])

            xyz_c = z*np.matmul(Pinv,uv1)

            xyz1_w = np.matmul(invExtMtx,np.array([[xyz_c[0,0]],[xyz_c[1,0]],[xyz_c[2,0]],[1]]))

            if(xyz1_w[0,0] < 0):
                wpX = xyz1_w[0,0] * 100/95
            else:
                wpX = xyz1_w[0,0] * 100/94 + 2.15

            if(xyz1_w[1,0] >= 175):
                wpY = xyz1_w[1,0] * 1.0922 - 14.4444
            else:
                wpY = xyz1_w[1,0] * 100/95 - 9.21
            # wpZ = xyz1_w[2,0]
            wpZ = 976-z #just use depth cam

        return wpX, wpY, wpZ

class ImageListener:
    def __init__(self, topic, camera):
        self.topic = topic
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber(topic, Image, self.callback)
        self.camera = camera

    def callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, data.encoding)
            #cv_image = cv2.rotate(cv_image, cv2.ROTATE_180)
        except CvBridgeError as e:
            print(e)
        self.camera.VideoFrame = cv_image


class TagImageListener:
    def __init__(self, topic, camera):
        self.topic = topic
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber(topic, Image, self.callback)
        self.camera = camera

    def callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, data.encoding)
            #cv_image = cv2.rotate(cv_image, cv2.ROTATE_180)
        except CvBridgeError as e:
            print(e)
        self.camera.TagImageFrame = cv_image


class TagDetectionListener:
    def __init__(self, topic, camera):
        self.topic = topic
        self.tag_sub = rospy.Subscriber(topic, AprilTagDetectionArray,
                                        self.callback)
        self.camera = camera

    def callback(self, data):
        self.camera.tag_detections = data
        #for detection in data.detections:
        #print(detection.id[0])
        #print(detection.pose.pose.pose.position)


class CameraInfoListener:
    def __init__(self, topic, camera):
        self.topic = topic
        self.tag_sub = rospy.Subscriber(topic, CameraInfo, self.callback)
        self.camera = camera

    def callback(self, data):
        self.camera.intrinsic_matrix = np.reshape(data.K, (3, 3))
        #print(self.camera.intrinsic_matrix)


class DepthListener:
    def __init__(self, topic, camera):
        self.topic = topic
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber(topic, Image, self.callback)
        self.camera = camera

    def callback(self, data):
        try:
            cv_depth = self.bridge.imgmsg_to_cv2(data, data.encoding)
            #cv_depth = cv2.rotate(cv_depth, cv2.ROTATE_180)
        except CvBridgeError as e:
            print(e)
        self.camera.DepthFrameRaw = cv_depth
        #self.camera.DepthFrameRaw = self.camera.DepthFrameRaw/2
        self.camera.ColorizeDepthFrame()


class VideoThread(QThread):
    updateFrame = pyqtSignal(QImage, QImage, QImage, QImage)

    def __init__(self, camera, parent=None):
        QThread.__init__(self, parent=parent)
        self.camera = camera
        image_topic = "/camera/color/image_raw"
        depth_topic = "/camera/aligned_depth_to_color/image_raw"
        camera_info_topic = "/camera/color/camera_info"
        tag_image_topic = "/tag_detections_image"
        tag_detection_topic = "/tag_detections"
        block_image_topic = "/block_detections_image"
        block_detection_topic = "/block_detections"

        image_listener = ImageListener(image_topic, self.camera)
        depth_listener = DepthListener(depth_topic, self.camera)
        tag_image_listener = TagImageListener(tag_image_topic, self.camera)

        block_image_listener = TagImageListener(block_image_topic, self.camera)

        camera_info_listener = CameraInfoListener(camera_info_topic,
                                                  self.camera)
        tag_detection_listener = TagDetectionListener(tag_detection_topic,
                                                      self.camera)
        block_detection_listener = TagDetectionListener(block_detection_topic,
                                                      self.camera)

    def run(self):
        if __name__ == '__main__':
            cv2.namedWindow("Image window", cv2.WINDOW_NORMAL)
            cv2.namedWindow("Depth window", cv2.WINDOW_NORMAL)
            cv2.namedWindow("Tag window", cv2.WINDOW_NORMAL)
            time.sleep(0.5)
        while True:
            rgb_frame = self.camera.convertQtVideoFrame()
            depth_frame = self.camera.convertQtDepthFrame()
            tag_frame = self.camera.convertQtTagImageFrame()
            block_frame = self.camera.convertQtBlockImageFrame()

            if ((rgb_frame != None) & (depth_frame != None)):
                self.updateFrame.emit(rgb_frame, depth_frame, tag_frame, block_frame)
            time.sleep(0.03)
            if __name__ == '__main__':
                cv2.imshow(
                    "Image window",
                    cv2.cvtColor(self.camera.VideoFrame, cv2.COLOR_RGB2BGR))
                cv2.imshow("Depth window", self.camera.DepthFrameRGB)
                cv2.imshow(
                    "Tag window",
                    cv2.cvtColor(self.camera.TagImageFrame, cv2.COLOR_RGB2BGR))
                cv2.waitKey(3)
                time.sleep(0.03)


if __name__ == '__main__':
    camera = Camera()
    videoThread = VideoThread(camera)
    videoThread.start()
    rospy.init_node('realsense_viewer', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()
