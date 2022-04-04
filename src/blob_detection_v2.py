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
def distance(size):
	return 315.802553 * (size ** (-1.031336))

def blob_detection(hsv, lower, upper, color):
    """
    hsv: frame converted to HSV color format
    lower_color: lowest designated HSV color [numpy array, rank 1, 3 entries]
    upper_color: highest designated HSV color [numpy array, rank 1, 3 entries]
    color: string identifying the color to be identified

    Note: color variable is used to determine whether or not the color to
    identify is red, so the circularity can be adjusted.

    Return: Keypoints.
    """
    # Threshold the HSV image to get only blue colors
    mask = cv.inRange(hsv, lower_color, upper_color)
    # Bitwise-AND mask and original image
    #res = cv.bitwise_and(frame,frame, mask = mask)
    inv_mask = cv.bitwise_not(mask)

    params = cv.SimpleBlobDetector_Params()

    #Thresholds for reporting
    params.minThreshold = 50
    params.maxThreshold = 1000 #10000

    #Area filtering. Make sure that the areas are of a reasonable size
    params.filterByArea = True
    params.minArea = 50
    params.maxArea = 1000

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
        params.minCircularity = 0.52 #Red: 0.52, Blue/Yellow: 0.62
        params.maxCircularity = 0.78 #Red: 0.78, Blue/Yello: 0.93
    else:
        params.minCircularity = 0.62 #Red: 0.52, Blue/Yellow: 0.62
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
        frame = cap

        #Scale down the frame and determine the image width
        frame = cv.resize(frame,None,fx=scale, fy=scale, interpolation = cv.INTER_CUBIC)
        frame_width = frame.shape[1]

        #Convert image to HSV
        hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)

        #Define color ranges. Note: Will need to be tweaked for production runs
	if color == 'blue' or color == 'Blue':
		#Blue:
		lower = np.array([105,50,50]) #[115,50,50]
		upper = np.array([135,255,255]) #[123,255,255]
	if color == 'red' or color == 'Red':
		#Red:
		lower = np.array([150, 15, 15])
		upper = np.array([250, 255, 255])
	if color == 'yellow' or color == 'Yellow':
		#Yellow:
		lower = np.array([30,25,25]) #[30, 25, 25]
		upper = np.array([85,255,255])
	else:
		print('Error: No color recognized")

	#Report Buckets
	try:
		kp = blob_detection(hsv, lower, upper, color)
	 	bucket_size = kp[0].size
	except IndexError:
		pass

	if len(kp) == 0:
		return None
	else:
		return distance(size)

    
    #Used for video demonstration
        #frame_with_keypoints_b = cv.drawKeypoints(frame, kp_b, np.array([]), (0, 255, 0), cv.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
        #frame_with_keypoints_r = cv.drawKeypoints(frame, kp_r, np.array([]), (0, 0, 255), cv.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
        #frame_with_keypoints_y = cv.drawKeypoints(frame, kp_y, np.array([]), (0, 255, 255), cv.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
        #frame_with_keypoints_br = cv.bitwise_or(frame_with_keypoints_b, frame_with_keypoints_r)
        #frame_with_keypoints_bry = cv.bitwise_or(frame_with_keypoints_br, frame_with_keypoints_y)
    
        #cv.imshow("Keypoints", frame_with_keypoints_b) #frame_with_keypoints_bry
        k = cv.waitKey(5) & 0xFF
        if k == 27:
            break
    cv.destroyAllWindows()
    

#Define camera input. Can be file or camera index
capture = cv.VideoCapture(0) #'test_footage_sim_01_trim.mp4'

detect(capture, 0.3, 'blue')
