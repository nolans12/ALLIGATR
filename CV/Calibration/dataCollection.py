import cv2

# Open the camera
camera_index = 1
pipeline = 'nvarguscamerasrc sensor-id=' + str(camera_index) + ' ! video/x-raw(memory:NVMM), width=(int)1920, height=(int)1080, format=(string)NV12, framerate=(fraction)15/1 ! nvvidconv ! video/x-raw, format=(string)BGRx ! videoconvert ! video/x-raw, format=(string)BGR ! appsink'
cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)

num = 0

while cap.isOpened():

    succes, img = cap.read()

    k = cv2.waitKey(5)

    if k == ord('q'):
        break
    elif k == ord('s'): # wait for 's' key to save and exit
        cv2.imwrite('AutoImages/img' + str(num) + '.png', img)
        print("Image Saved")
        num += 1

    cv2.imshow('Img',img)

# Release and destroy all windows before termination
cap.release()
cv2.destroyAllWindows()