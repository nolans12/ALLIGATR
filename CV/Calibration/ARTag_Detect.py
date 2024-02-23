
# Import libraries
import numpy as np
import matplotlib as mpl
import pandas as pd
import cv2
import pickle

AR_LENGTH = 0.15875

# ArUco dictionaries
ARUCO_DICT = {
	"DICT_4X4_50": cv2.aruco.DICT_4X4_50,
	"DICT_4X4_100": cv2.aruco.DICT_4X4_100,
	"DICT_4X4_250": cv2.aruco.DICT_4X4_250,
	"DICT_4X4_1000": cv2.aruco.DICT_4X4_1000,
	"DICT_5X5_50": cv2.aruco.DICT_5X5_50,
	"DICT_5X5_100": cv2.aruco.DICT_5X5_100,
	"DICT_5X5_250": cv2.aruco.DICT_5X5_250,
	"DICT_5X5_1000": cv2.aruco.DICT_5X5_1000,
	"DICT_6X6_50": cv2.aruco.DICT_6X6_50,
	"DICT_6X6_100": cv2.aruco.DICT_6X6_100,
	"DICT_6X6_250": cv2.aruco.DICT_6X6_250,
	"DICT_6X6_1000": cv2.aruco.DICT_6X6_1000,
	"DICT_7X7_50": cv2.aruco.DICT_7X7_50,
	"DICT_7X7_100": cv2.aruco.DICT_7X7_100,
	"DICT_7X7_250": cv2.aruco.DICT_7X7_250,
	"DICT_7X7_1000": cv2.aruco.DICT_7X7_1000,
	"DICT_ARUCO_ORIGINAL": cv2.aruco.DICT_ARUCO_ORIGINAL,
	"DICT_APRILTAG_16h5": cv2.aruco.DICT_APRILTAG_16h5,
	"DICT_APRILTAG_25h9": cv2.aruco.DICT_APRILTAG_25h9,
	"DICT_APRILTAG_36h10": cv2.aruco.DICT_APRILTAG_36h10,
	"DICT_APRILTAG_36h11": cv2.aruco.DICT_APRILTAG_36h11
}

# Highlight the detected markers
def aruco_display(corners, ids, rejected, image):
    if(len(corners) > 0): # Are any aruco tags detected
        
        for (markerCorner, markerID) in zip(corners, ids):
            corners = markerCorner.reshape((4,2))
            (topLeft, topRight, bottomRight, bottomLeft) = corners  # Get the corners

            # Cast the data to integers
            topRight = (int(topRight[0]), int(topRight[1]))
            bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
            bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
            topLeft = (int(topLeft[0]), int(topLeft[1]))

            # Draw the lines for the AR tag detection
            cv2.line(image, topLeft, topRight, (0, 0, 255), 2)
            cv2.line(image, topRight, bottomRight, (0, 0, 255), 2)
            cv2.line(image, bottomRight, bottomLeft, (0, 0, 255), 2)
            cv2.line(image, bottomLeft, topLeft, (0, 0, 255), 2)

    return image



# # Path for aruco tags
# path = "ARTags/Markers/"

# # Get predefined dictionary
aruco_type = "DICT_6X6_250"
testDict = cv2.aruco.getPredefinedDictionary(ARUCO_DICT[aruco_type])

# # Insantiate parameters
arucoParams = cv2.aruco.DetectorParameters()


# Specify the path to your pickle file
pickle_file_path = 'Calibration/cameraMatrix.pkl'

# Open the pickle file in binary mode ('rb' for reading binary)
with open(pickle_file_path, 'rb') as file:
    # Load data from the pickle file
    camMatrix = pickle.load(file)


# Specify the path to your pickle file
pickle_file_path = 'Calibration/dist.pkl'

# Open the pickle file in binary mode ('rb' for reading binary)
with open(pickle_file_path, 'rb') as file:
    # Load data from the pickle file
    distCoeff = pickle.load(file)





# # Read in the image
# imagePath = path + "ExampleMarker_2.png"
# img = cv2.imread(imagePath, cv2.IMREAD_COLOR)

# # Display the read in image
# cv2.imshow("ArUco Marker", img)

# # Wait until a key is pressed
# cv2.waitKey(0)

# # Close all of the windows
# cv2.destroyAllWindows()

  
# # Detect the marker
# corners, ids, rejected = cv2.aruco.detectMarkers(img, testDict, parameters=arucoParams)

# # Draw the detection
# image = aruco_display(corners, ids, rejected, img)

# # Display the image with the detected AR tag
# cv2.imshow("ArUco Marker", image)

# # Image is 1280 by 720 pixels
# xPixels = 1280
# yPixels = 720


# # Wait until a key is pressed
# cv2.waitKey(0)

# # Close all of the windows
# cv2.destroyAllWindows()

# %% Video Capture With Webcam
cap = cv2.VideoCapture(0)

cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

while cap.isOpened():
    # Get the current video feed frame
    ret, img = cap.read()
    
    # Locate the Aruco tag
    corners, ids, rejected = cv2.aruco.detectMarkers(img, testDict, parameters=arucoParams)
    image = aruco_display(corners, ids, rejected, img)
    
    # Get the centroid coordinates of the AR tag and the corners
    if corners:

        firstCorners = corners[0][0]
        topLeft = firstCorners[1]
        topRight = firstCorners[2]
        bottomRight = firstCorners[3]
        bottomLeft = firstCorners[0]
        testArr = [int(topLeft[0]), int(topLeft[1]), int(topRight[0]), int(topRight[1]), int(bottomRight[0]), int(bottomRight[1]), int(bottomLeft[0]), int(bottomLeft[1])]
        # print(testArr)
        # Get the pose
        rVec, tVec, _ = cv2.aruco.estimatePoseSingleMarkers(corners, AR_LENGTH, camMatrix, distCoeff)

        print(tVec)
    

	# Display the frame
    cv2.imshow('frame', image)
    
	# Quit
    key = cv2.waitKey(1) & 0xFF
    if key == ord("q"):
        break
    
	
cv2.destroyAllWindows()
cap.release()	
	

