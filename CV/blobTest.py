
import cv2
import numpy as np

def clearBuffer(cap, bufferSize):
    for i in range(bufferSize):
        cap.read()
    

# A function to fix HSV range
def fixHSVRange(h, s, v):
    # Normal H,S,V: (0-360,0-100%,0-100%)
    # OpenCV H,S,V: (0-180,0-255 ,0-255)
    return (180 * h / 360, 255 * s / 100, 255 * v / 100)

def detectBlob(im):
    # Make a copy of Image; find the HSV range; convert it to OpenCV
    # undrestandble range and make a mask from it
    frm=im.copy()
    frm = cv2.cvtColor(frm, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(frm, fixHSVRange(0, 0, 0), fixHSVRange(360, 4.53, 100))

    # Remove the noise
    noise=cv2.dilate(mask,np.ones((5,5)))
    noise=cv2.erode(mask,np.ones((5,5)))
    noise=cv2.medianBlur(mask,7)

    # Change image channels
    mask=cv2.cvtColor(mask,cv2.COLOR_GRAY2BGR)
    noise=cv2.cvtColor(noise,cv2.COLOR_GRAY2BGR)
    cleanMask=~noise

    # Make a new mask without noise
    centerMask=cv2.cvtColor(cleanMask.copy(),cv2.COLOR_BGR2GRAY)
        
    # Image detector
    keypoints = detector.detect(centerMask)

    # Not detected
    if not keypoints:
        return -1    # Return negative for no detection

    # Zip the detected centroid arrays
    centroids_x = np.array([])
    centroids_y = np.array([])
    for keypoint in keypoints:
        centroids_x = np.append(centroids_x, keypoint.pt[0])
        centroids_y = np.append(centroids_y, keypoint.pt[1])

    # X and Y centroid coordinates of the detected RGVs
    centroids_x = centroids_x.astype(int)
    centroids_y = centroids_y.astype(int)

    return zip(centroids_x,centroids_y)



params = cv2.SimpleBlobDetector_Params()

# Change thresholds
params.filterByColor = False

# Change thresholds
params.minThreshold = 0
params.maxThreshold = 250

# Filter by Area.
params.filterByArea = True
params.minArea = 530
params.maxArea = 100000

# Filter by circularity
params.filterByCircularity = True
params.minCircularity = 0.1
#params.maxCircularity = 0.8

# Filter by inertia ratio
params.filterByInertia = True
params.maxInertiaRatio = 0.94

# FIlter by convexity
params.filterByConvexity = False
params.minConvexity = 0.4

# Create a detector with the parameters
detector = cv2.SimpleBlobDetector_create(params)

camera_found = False
attempts = 0

# Try to open the 0 index for the primary camera
camera_index = 1   

while not camera_found:
    # Setup the GStreamer Pipeline
    pipeline = 'nvarguscamerasrc sensor-id=' + str(camera_index) + ' ! video/x-raw(memory:NVMM), width=(int)1920, height=(int)1080, format=(string)NV12, framerate=(fraction)15/1 ! nvvidconv ! video/x-raw, format=(string)BGRx ! videoconvert ! video/x-raw, format=(string)BGR ! appsink'

    # Create a VideoCapture object with the GStreamer pipeline
    cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)

    # Set buffer size
    bufferSize = 5

    # Check if the camera opened successfully
    if cap.isOpened():
        cap.set(cv2.CAP_PROP_BUFFERSIZE, bufferSize);                            # Set the buffer size so it doesn't go beyond 5 frames
        cap.set(cv2.CAP_PROP_FPS, 15)                                   # Set the FPS to 15               
        camera_found = True                                             # Camera is found
        print("Camera connected")
        break            

    attempts += 1

    if attempts > 10:
        camera_found = True


# Begin the main loop that consistently outputs blob centroids when running
while True:

    if cap.isOpened():                      # Capture image while camera is opened
        # Clear the buffer
        clearBuffer(cap, bufferSize)

        # Get the current video feed frame
        ret, img = cap.read()
        cv2.imshow('frame', img)


        # Check if the image is empty, failed to get image
        if img is None:
            out_str = "Camera Connection Lost"
        else:
            # Call blob detection
            centroid = detectBlob(img)

            # Output detection
            if centroid == -1:                                      # Not Detected
                centOut = -1
            else:                                                   # Detected, format as array
                print("Detected")
                centOut = []
                for i in centroid:
                    centOut.append(i[0])
                    centOut.append(i[1])

    else:
        print("Lost Connection")

    # Quit
    key = cv2.waitKey(1) & 0xFF
    if key == ord("q"):
        break
        

cv2.destroyAllWindows()                         # Close everything and release the camera
cap.release()                                   # Release the capture object

