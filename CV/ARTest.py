#!/usr/bin/env python 
# <- This is the shebang line which tells the OS which interpreter to use
# Be aware that the tutorial said to use python2, however we built it for py3 I believe

import cv2

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


# ArUco dictionary
ARUCO_DICT = {
    "DICT_6X6_250": cv2.aruco.DICT_6X6_250,
}

# Get predefined dictionary for AR tag detection
aruco_type = "DICT_6X6_250"
finalDict = cv2.aruco.getPredefinedDictionary(ARUCO_DICT[aruco_type])


# Try to open the 0 index for the secondary camera
camera_index = 0   
camera_found = False

while not camera_found:
    # Setup the GStreamer Pipeline
    #pipeline = f'nvarguscamerasrc sensor-id={camera_index} ! video/x-raw(memory:NVMM), width=(int)1920, height=(int)1080, format=(string)NV12, framerate=(fraction)15/1 ! nvvidconv ! video/x-raw, format=(string)BGRx ! videoconvert ! video/x-raw, format=(string)BGR ! appsink'
    pipeline = 'nvarguscamerasrc sensor-id=' + str(camera_index) + ' ! video/x-raw(memory:NVMM), width=(int)1920, height=(int)1080, format=(string)NV12, framerate=(fraction)60/1 ! nvvidconv ! video/x-raw, format=(string)BGRx ! videoconvert ! video/x-raw, format=(string)BGR ! appsink'

    # Create a VideoCapture object with the GStreamer pipeline
    cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)

    # Check if the camera opened successfully
    if cap.isOpened():
        print("Camera Connected!")
        camera_found = True     # Camera is found

        # Get the frame width and height
        frame_width = int(cap.get(3)) 
        frame_height = int(cap.get(4)) 
        size = (frame_width, frame_height) 
        
        # Create video writer object
        writeObj = cv2.VideoWriter('capturedVideo.avi', cv2.VideoWriter_fourcc(*'MJPG'), 30, size) 
        
        break

    camera_index += 1

    if camera_index > 10:
        print("Camera not found after 10 attempts.")
        faux_camera = True
        camera_found = True


# Begin the main loop that consistently outputs AR tag corners when running
while True:

    boolCapture = 1
    try:
        # Get the current video feed frame
        ret, img = cap.read()

        # Locate the Aruco tag
        # corners, ids, rejected = cv2.aruco.detectMarkers(img, finalDict)
        # image = aruco_display(corners, ids, rejected, img)

        # Save the frame every other frame (30 fps)
        if boolCapture:
            writeObj.write(img)
            boolCapture = not boolCapture
        
        # Display the frame
        #cv2.imshow('frame', image)
        
        # Quit
        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            break
    
    except KeyboardInterrupt:
        writeObj.release()
        cv2.destroyAllWindows()         # Close everything and release the camera
        cap.release()



# Save images to a file
#size = (1920, 1080)    
#out = cv2.VideoWriter("arDetect.mp4",cv2.VideoWriter_fourcc(*'DIVX'), 30, size)

#Write to ouput video object
#for i in frames:
#    out.write(i)
#out.release()
    
    
writeObj.release()
cv2.destroyAllWindows()         # Close everything and release the camera
cap.release()
