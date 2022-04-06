"""
Bennet Outland
Rocker Robotics
National Robotics Challenge | Autonomous Vehicle Challenge
Blob Detection Algorithm
License: MIT

Thank you to the creators of the OpenCV Docs for the great documentation and
example code that was modified to achieve these results.

Input: USB Camera Video, Scaling Factor

Basic Process:
  . Scale down the Video
  . Create masks of given color ranges (Blue, Yellow, and Red in this case)
  . Load SimpleBlobDetector and filter by area
  . Calculate blob size and approximate turning angle to blob

Return: Blob Size, Turning Angle to Blob, Bucket Color {'Blue': 0, 'Yellow': 1, 'Red': 2}
"""

import cv2 as cv
import numpy as np
import math
import time

def distance(size):
    return 315.802553 * (size ** -1.031336)

def blob_detection(hsv, inv_mask, color):
    """
    hsv: frame converted to HSV color format
    lower_color: lowest designated HSV color [numpy array, rank 1, 3 entries]
    upper_color: highest designated HSV color [numpy array, rank 1, 3 entries]
    color: string identifying the color to be identified

    Note: color variable is used to determine whether or not the color to
    identify is red, so the circularity can be adjusted.

    Return: Keypoints.
    """

    params = cv.SimpleBlobDetector_Params()

    #Thresholds for reporting
    params.minThreshold = 200
    params.maxThreshold = 5000 #10000

    #Area filtering. Make sure that the areas are of a reasonable size
    params.filterByArea = True
    params.minArea = 150
    params.maxArea = 10000

    #Color filtering: search for black blobs
    params.filterByColor = True
    params.blobColor = 0

    #Circularity
    """
    f = (4 * np.pi * w * h) / (2 * w + 2 * h) ** 2
      = 0.78 +- 0.16 (20% tolerance) => [0.62, 0.93] (Blue/Yellow)
      = 0.65 +- 0.13 (20% tolerance) => [0.52, 0.78] (Red)
    """
    params.filterByCircularity = True
    if (color == "red" or color == "Red"):
        params.minCircularity = 0.02 #Red: 0.52, Blue/Yellow: 0.62
        params.maxCircularity = 0.98 #Red: 0.78, Blue/Yello: 0.93
    else:
        params.minCircularity = 0.52 #Red: 0.52, Blue/Yellow: 0.62
        params.maxCircularity = 0.93 #Red: 0.78, Blue/Yellow: 0.93

    #Negate the following filters
    params.filterByInertia = False
    params.filterByConvexity = False

    ver = (cv.__version__).split('.')
    if int(ver[0]) < 3:
        detector = cv.SimpleBlobDetector(params)
    else:
        detector = cv.SimpleBlobDetector_create(params)

    #Detect blobs
    keypoints = detector.detect(inv_mask)

    return keypoints

def detect(cap, scale, color):
    while (True):
        #Read each frame
        _, frame = cap.read()

        #Scale down the frame and determine the image width
        frame = cv.resize(frame,None,fx=scale, fy=scale, interpolation = cv.INTER_CUBIC)
        frame_width = frame.shape[1]

        #Convert image to HSV
        hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)

        #Define color ranges. Note: Will need to be tweaked for production runs
        #Blue:
        if (color == 'blue'):
            lower_blue = np.array([105,30,30]) #[115,50,50]
            upper_blue = np.array([135,255,255]) #[123,255,255]
            mask = cv.inRange(hsv, lower_blue, upper_blue)
            inv_mask = cv.bitwise_not(mask)
        elif (color == 'red'):
            #Red:
            #Range 1
            lower_red_1 = np.array([0, 30, 30])
            upper_red_1 = np.array([10, 255, 255])
            #Range 2
            lower_red_2 = np.array([326, 30, 30])
            upper_red_2 = np.array([359, 255, 255])
            # Threshold the HSV image to get only blue colors
            mask_1 = cv.inRange(hsv, lower_red_1, upper_red_1)
            mask_2 = cv.inRange(hsv, lower_red_1, upper_red_1)
            inv_mask = cv.bitwise_not(mask_1 + mask_2)

        elif (color == 'yellow'):
            #Yellow:
            lower_yellow = np.array([25,25,25]) #[30, 25, 25]
            upper_yellow = np.array([85,255,255])
            mask = cv.inRange(hsv, lower_yellow, upper_yellow)
            inv_mask = cv.bitwise_not(mask)
            
        else:
            print("Error: Invalid Color!")
            inv_mask = np.empty()

        kp = blob_detection(hsv, inv_mask, "blue")

        #Used for video demonstration
        frame_with_keypoints = cv.drawKeypoints(frame, kp, np.array([]), (255, 0, 0), cv.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
        
        try:
            print(distance(kp[0].size))
        except IndexError:
            print(None)
        
        cv.imshow("Keypoints", frame_with_keypoints) #frame_with_keypoints_bry
        k = cv.waitKey(5) & 0xFF
        if k == 27:
            break
    cv.destroyAllWindows()



#Comment this out you stupid idiot!
#Define camera input. Can be file or camera index
capture = cv.VideoCapture(0) # , cv.CAP_DSHOW
i = 0
while (True):
    #capture = cv.VideoCapture(0, cv.CAP_DSHOW) #'test_footage_sim_01_trim.mp4'
    #capture = capture.set(CV_CAP_PROP_POS_FRAMES,count-1)
    print(detect(capture, 0.3, 'blue'))
    i += 1


