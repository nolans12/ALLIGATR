## This script will record video using both the primary and secondary sensors at their full framerate
# The purpose is only to record video, there will not be CV processing or any other processes running
import cv2
import os
from datetime import datetime

# Specify the recording and save FPS
CAM_FPS = 60
SAVE_FPS = 60

# Create a new directory for the videos
def create_directory(save_location):
    # Get the current date and time
    now = datetime.now()

    # Format the date and time
    formatted_now = now.strftime("%Y_%m_%d_%H_%M_%S")

    # Create the directory name
    data_dir_name = "videos_" + formatted_now
    data_dir = save_location + "/" + data_dir_name

    # Create the directory
    os.system("mkdir " + data_dir)
    return data_dir

# Create directory to save the videos to
save_location = os.path.expanduser("~/ALLIGATR/CV/videos")

# Create a new directory for the data
data_dir = create_directory(save_location)
print("New video directory created at: " + data_dir)

# File path for the videos
primaryFile = os.path.join(data_dir, 'primaryVideo.avi')
secondaryFile = os.path.join(data_dir, 'secondaryVideo.avi')

# Define the codec and create VideoWriter objects
fourcc = cv2.VideoWriter_fourcc(*'XVID')
outPrimary = cv2.VideoWriter(primaryFile, fourcc, 60.0, (1920, 1080))
outSecondary = cv2.VideoWriter(secondaryFile, fourcc, 60.0, (1920, 1080))

# Setup the pipelines
pipelineSecondary = 'nvarguscamerasrc sensor-id=' + str(0) + ' ! video/x-raw(memory:NVMM), width=(int)1920, height=(int)1080, format=(string)NV12, framerate=(fraction)60/1 ! nvvidconv ! video/x-raw, format=(string)BGRx ! videoconvert ! video/x-raw, format=(string)BGR ! appsink'
pipelinePrimary = 'nvarguscamerasrc sensor-id=' + str(1) + ' ! video/x-raw(memory:NVMM), width=(int)1920, height=(int)1080, format=(string)NV12, framerate=(fraction)60/1 ! nvvidconv ! video/x-raw, format=(string)BGRx ! videoconvert ! video/x-raw, format=(string)BGR ! appsink'

# Create a VideoCapture object with the GStreamer pipeline
capPrimary = cv2.VideoCapture(pipelinePrimary, cv2.CAP_GSTREAMER)
capSecondary = cv2.VideoCapture(pipelineSecondary, cv2.CAP_GSTREAMER)

# Check if the cameras opened successfully
if not capPrimary.isOpened():
    print("Error: Could not open primary camera.")
    exit()
else:
    print("Successfully Opened Primary Camera")

if not capSecondary.isOpened():
    print("Error: Could not open secondary camera.")
    exit()
else:
    print("Successfully Opened Secondary Camera")


# Loop for capturing video
frameCount = 0
while True:
    frameCount += 1 # Update frame

    # Get the current video feed frame
    retPrimary, imgPrimary = capPrimary.read()
    retSecondary, imgSecondary = capSecondary.read()
    
	# Save the frames
    if frameCount % (CAM_FPS // SAVE_FPS) == 0: 
        outPrimary.write(imgPrimary)
        outSecondary.write(imgSecondary)

	# Quit
    key = cv2.waitKey(1) & 0xFF
    if key == ord("q"):
        break
    

capPrimary.release()	
capSecondary.release()
outPrimary.release()
outSecondary.release()
cv2.destroyAllWindows()