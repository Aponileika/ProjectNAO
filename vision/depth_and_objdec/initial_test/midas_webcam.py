import torch
from datetime import datetime
import cv2 as cv
import numpy as np
import matplotlib.pyplot as plt


#CREDIT liu homewreckers... (TODO make this proper)
model_type = "DPT_Hybrid"   # MiDaS v3 - Hybrid    (medium accuracy, medium inference speed)

class Midas:
    def __init__(self):
        self.midas = torch.hub.load("intel-isl/MiDaS", model_type)
        self.device = torch.device("mps") if torch.backends.mps.is_available() else torch.device("cpu")
        self.midas.to(self.device)
        self.midas.eval()
        midas_transforms = torch.hub.load("intel-isl/MiDaS", "transforms")
        if model_type == "DPT_Large" or model_type == "DPT_Hybrid":
            self.transform = midas_transforms.dpt_transform
        else:
            self.transform = midas_transforms.small_transform

    def __call__(self, image, normalize=False):
        return self.get_depth(image, normalize)

    def get_depth(self, image, normalize):
        input_batch = self.transform(image).to(self.device)

        with torch.no_grad():
            prediction = self.midas(input_batch)

        pred_cpu = prediction.squeeze().cpu().float().numpy()

        if normalize:
            pred_cpu = pred_cpu - pred_cpu.min()
            pred_cpu = pred_cpu / (pred_cpu.max() + 1e-6)
            out = (pred_cpu * 255).astype("uint8")
        else:
            out = pred_cpu

        out = cv.resize(out, (image.shape[1], image.shape[0]))
        return out

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
        print("initiating midas...")
        downsample_factor = 0.2
        upsample_factor = 1.0 / downsample_factor
        depth_model = Midas()
        normalize = True
        print("disp video...")
        while True:
            start = datetime.now()
            ret, frame = self.get_frame()
            if(ret):
                frame = cv.resize(frame, dsize=None, fx=downsample_factor, fy=downsample_factor, interpolation=cv.INTER_LANCZOS4)
                frame_rgb = cv.cvtColor(frame, cv.COLOR_BGR2RGB)
                depth = depth_model(frame_rgb, normalize)
                depth = cv.resize(depth, dsize=None, fx=upsample_factor, fy=upsample_factor, interpolation=cv.INTER_LANCZOS4)

                if(normalize==False):
                    disp_norm = cv.normalize(depth, None, 0, 255, cv.NORM_MINMAX)
                    disp_uint8 = disp_norm.astype(np.uint8)
                    disp_colored = cv.applyColorMap(disp_uint8, cv.COLORMAP_JET)
                else:
                    disp_colored = cv.applyColorMap(depth, cv.COLORMAP_JET)


                cv.imshow("Depth Map", disp_colored)

            if cv.waitKey(1) != -1:
                break
            end = datetime.now()
            print(f"one frame took {(end - start).total_seconds()} seconds; {1/((end - start).total_seconds())} fps")

        cv.destroyWindow("Depth Map")

    def release_cam(self):
        self.cam.release()

webcam = Webcam()
webcam.play_video_disp()
webcam.release_cam()

