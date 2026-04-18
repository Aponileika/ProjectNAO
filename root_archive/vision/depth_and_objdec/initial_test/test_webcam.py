import cv2 as cv

# Initialize webcam (0 = default camera)
cam = cv.VideoCapture(0)

# Capture one frame
ret, frame = cam.read()

if ret:
    cv.imshow("Captured", frame)         
    cv.waitKey(0)                      
    cv.destroyWindow("Captured")       
else:
    print("Failed to capture image.")

cam.release()
