import numpy as np
import torch
from glob import glob
import cv2
from PIL import Image
from matplotlib import pyplot as plt

# Intrinsic Parameters -->  published in /camera/d435/camera_info
#TODO: figure out which camera we're actually using for mono vision; USE THE LEFT IT'S PERFECTLY ALIGNED!!!
fx = 607.6275024414062
fy = 607.12451171875
cx = 322.22088623046875
cy = 245.45425415039062

K = np.array([[fx,   0,  cx],
            [0,    fy, cy],
            [0,    0,   1.0]])

robot_offset = np.array([0, 0, 0]) #TODO: find the offset of the camera wrt robot center (in meters???)

# use yolov5 for image detection
model = torch.hub.load('ultralytics/yolov5', 'yolov5s', pretrained=True)

# Images
imgs = glob('test_imgs/imgs/*')#['https://ultralytics.com/images/zidane.jpg']  # batch of images
depth = glob('test_imgs/depth/*')
# Inference
results = model(imgs)

# Results
results.print()
# results.show() #or .save()  # or


#dummy list of points as place holder for the ones we care about
image_points=results.xyxy
world_points = []
for i, point in enumerate(image_points):
    # point=point.transpose(0,1)
    im=cv2.imread(imgs[i])
    depth_img = cv2.imread(depth[i])
    for p in point:
        p=p.reshape(-1,2)
        for center in p:
            coords=center.tolist()
            coords=(int(coords[0]),int(coords[1]))
            # im = cv2.circle(im, coords, 5, (255, 0, 0))
            # depth = depth_img[coords[0], coords[1]] #TODO: figure out depth, maybe just filter point cloud
            z = 1
            uv = np.array([[coords[0]],
                           [coords[1]],
                           [1]])
            world_point = np.matmul(np.linalg.inv(K), z * uv)
            # breakpoint()

            world_points.append(world_point)

    # cv2.imshow('',im)
    # cv2.waitKey(100000)

    #TODO: fit a cylinder to the points with a regression????


world_points=np.stack(world_points) #this step is just to make a list of numpy arrays into 1 numpy array

print(world_points)