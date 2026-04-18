import cv2 as cv
import time

class Webcam():

    def __init__(self):
        self.cam = cv.VideoCapture(0)

    def get_frame(self):
        return self.cam.read()

    def play_video(self):
        while True:
            ret, frame = self.get_frame()
            if(ret):
                cv.imshow("Captured", frame)
            if cv.waitKey(1) != -1:
                break

        cv.destroyWindow("Captured")

    def release_cam(self):
        self.cam.release()

webcam = Webcam()
webcam.play_video()
webcam.release_cam()
