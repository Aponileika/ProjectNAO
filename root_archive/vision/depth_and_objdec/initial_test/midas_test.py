import cv2 as cv
import torch

import matplotlib.pyplot as plt

#model_type = "DPT_Large"     # MiDaS v3 - Large     (highest accuracy, slowest inference speed)
model_type = "DPT_Hybrid"   # MiDaS v3 - Hybrid    (medium accuracy, medium inference speed)
#model_type = "MiDaS_small"  # MiDaS v2.1 - Small   (lowest accuracy, highest inference speed)

print(f"Initiating midas of model type {model_type}")
midas = torch.hub.load("intel-isl/MiDaS", model_type)
device = torch.device("cuda") if torch.cuda.is_available() else torch.device("cpu")
midas.to(device)
midas.eval()
midas_transforms = torch.hub.load("intel-isl/MiDaS", "transforms")
print("-----------------------")

print(f"choosing transform....")
if model_type == "DPT_Large" or model_type == "DPT_Hybrid":
    transform = midas_transforms.dpt_transform
else:
    transform = midas_transforms.small_transform
print("-----------------------")

def get_depth(img):
    input_batch = transform(img).to(device)
    print(f"inference....")
    from datetime import datetime

    start = datetime.now()

    with torch.no_grad():
        print(f"predicting....")
        prediction = midas(input_batch)

        print(f"interpolate....")
        prediction = torch.nn.functional.interpolate(
            prediction.unsqueeze(1),
            size=img.shape[:2],
            mode="bicubic",
            align_corners=False,
        ).squeeze()
    end = datetime.now()
    print(f"inference took {(end - start).total_seconds()} seconds")
    print("-----------------------")

    output = prediction.cpu().numpy()
    return output

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

    def play_video_disp(self):
        while True:
            ret, frame = self.get_frame()
            if(ret):
                #cv.imshow("Captured", frame)
                frame_rgb = cv.cvtColor(frame, cv.COLOR_BGR2RGB)
                disp = get_depth(frame_rgb)
                disp_brg = cv.cvtColor(disp, cv.COLOR_RGB2BGR)
                cv.imshow("Disp_Capture", disp_brg)
            if cv.waitKey(1) != -1:
                break

        cv.destroyWindow("Disp_Capture")

    def release_cam(self):
        self.cam.release()

webcam = Webcam()
webcam.play_video_disp()
webcam.release_cam()
