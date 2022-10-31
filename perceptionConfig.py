import numpy as np
import torch

# Intrinsic Parameters -->  published in /camera/d435/camera_info
#TODO: figure out which camera we're actually using for mono vision; USE THE LEFT IT'S PERFECTLY ALIGNED!!!
fx = 607.6275024414062
fy = 607.12451171875
cx = 322.22088623046875
cy = 245.45425415039062

K = np.array([[fx,   0,  cx],
            [0,    fy, cy],
            [0,    0,   1.0]])

# use yolov5 for image detection
YOLO = torch.hub.load('ultralytics/yolov5', 'yolov5s', pretrained=True)

#base image folder
base_image_folder_name = 'saved_imgs'