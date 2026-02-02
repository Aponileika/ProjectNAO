import cv2
import numpy as np
from naoqi import ALProxy
import time

NAO_IP = "192.168.1.100"   # <- set your robot IP
NAO_PORT = 9559
CLIENT_NAME = "opencv_client"
CAMERA_INDEX = 0           # 0 = top camera, 1 = bottom camera
RESOLUTION = 2             # 2 = 640x480
COLOR_SPACE = 11           # 11 = kBGRColorSpace
FPS = 10

def subscribe_video(video_proxy):
    return video_proxy.subscribeCamera(CLIENT_NAME, CAMERA_INDEX, RESOLUTION, COLOR_SPACE, FPS)

def unsubscribe_video(video_proxy, sub_name):
    try:
        video_proxy.unsubscribe(sub_name)
    except:
        pass

def get_frame(video_proxy, sub_name):
    # returns BGR numpy image or None
    image = video_proxy.getImageRemote(sub_name)
    if not image:
        return None
    width = image[0]
    height = image[1]
    array = image[6]
    # array is a string/bytearray of BGR pixels
    np_img = np.frombuffer(array, dtype=np.uint8)
    np_img = np_img.reshape((height, width, 3))
    return np_img

def detect_color_bgr(frame, lower_bgr, upper_bgr):
    # convert to HSV for more robust color detection
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, lower_bgr, upper_bgr)
    # optional morphological clean
    kernel = np.ones((5,5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_DILATE, kernel)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None, mask
    largest = max(contours, key=cv2.contourArea)
    M = cv2.moments(largest)
    if M["m00"] == 0:
        return None, mask
    cx = int(M["m10"]/M["m00"])
    cy = int(M["m01"]/M["m00"])
    area = cv2.contourArea(largest)
    return (cx, cy, area, largest), mask

def main():
    video = ALProxy("ALVideoDevice", NAO_IP, NAO_PORT)
    sub = subscribe_video(video)
    print("Subscribed:", sub)
    try:
        # HSV green range (adjust to your object / lighting)
        lower_green = np.array([40, 60, 40])
        upper_green = np.array([80, 255, 255])

        while True:
            frame = get_frame(video, sub)
            if frame is None:
                print("No frame")
                time.sleep(0.1)
                continue

            result, mask = detect_color_bgr(frame, lower_green, upper_green)
            display = frame.copy()
            if result is not None:
                cx, cy, area, contour = result
                cv2.circle(display, (cx, cy), 6, (0,0,255), -1)
                cv2.drawContours(display, [contour], -1, (0,255,0), 2)
                print("Detected at", cx, cy, "area", area)
            cv2.imshow("NAO BGR", display)
            cv2.imshow("mask", mask)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    finally:
        unsubscribe_video(video, sub)
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()