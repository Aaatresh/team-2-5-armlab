import numpy as np
import cv2

points_2D = np.array([
                        (418, 247),  # Nose tip
 
                        (392, 329),  # Chin
 
                        (353, 199),  # Left eye corner
 
                        (434, 203),  # Right eye corner
 
                        (348, 270),  # Left mouth 
 
                        (414, 279)   # Right mouth 
 
                      ], dtype="double")

points_3D = np.array([
 
                      (0.0, 0.0, 0.0),       #Nose tip
 
                      (0.0, -330.0, -65.0),  #Chin
 
                      (-225.0, 170.0, -135.0),#Left eye corner
 
                      (225.0, 170.0, -135.0), #Right eye corner 
 
                      (-150.0, -150.0, -125.0),#Left mouth 
 
                      (150.0, -150.0, -125.0) #Right mouth 
 
  
 
                     ])

dist_coeffs = np.zeros((4,1))

Kteam =   np.array([[954.6327,0,629.4831],[0,968.4867,386.4730],[0,0,1.0000]])

success, rot, trans = cv2.solvePnP(points_3D, points_2D, Kteam, dist_coeffs, flags = 0)