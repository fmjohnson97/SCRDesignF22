import numpy as np
import torch
from cv_bridge import CvBridge

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

#for converting image styles
bridge = CvBridge()

#Yolov5 class categories
OBSTACLE_CLASSES = ["bicycle","car","motorcycle","airplane","bus","train","truck","boat","fire hydrant","parking meter",
                    "bench","bird","cat","dog","horse","sheep","cow","elephant","bear","zebra","giraffe","backpack",
                    "handbag","suitcase","skis","snowboard","baseball bat","skateboard","surfboard","tennis racket",
                    "chair","couch","potted plant","bed","dining table","toilet","tv","laptop","mouse","remote",
                    "keyboard","cell phone","microwave","oven","toaster","sink","refrigerator","clock","vase",
                    "teddy bear","hair drier","toothbrush"]
MAYBE_TRASH_CLASSES=["umbrella","tie","sports ball","kite","baseball glove","frisbee","bowl","banana","apple",
                     "sandwich","orange","broccoli","carrot","hot dog","pizza","donut","cake","book","scissors"]
TRAFFIC_CLASSES=["traffic light","stop sign",]
TRASH_CLASSES=["bottle","wine glass","cup","fork","knife","spoon"]
PEOPLE_CLASSES=["person"]