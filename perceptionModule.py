import torch
import numpy as np
import cv2

from glob import glob

from perceptionConfig import *

def getDetections(image):
    results = YOLO(image)
    #returns list of pandas dataframes that contain
    # xmin        ymin        xmax        ymax  confidence  class    name
    return results.pandas().xyxy

def getPointCloud(objects, depth_image):
    depth = np.array(depth_image)[:, :, 0] / 1000
    i, j = np.indices(depth.shape)
    x = (j - cx) / fx * depth
    y = (i - cy) / fy * depth
    x = x.reshape(-1)
    y = y.reshape(-1)
    depth = depth.reshape(-1)
    pcd = np.array([x, y, depth]).T

    # mask = np.nonzero(pcd[:, 2])
    # pcd = pcd[mask]
    clouds=[]
    ids=[]
    for obj in objects.iloc:
        clouds.append(pcd[int(obj[0]):int(obj[2]),int(obj[1]):int(obj[3])])
        ids.append(obj[-1])

    return clouds, ids

def read_next_image():
    rgb=glob.glob(base_image_folder_name+'/rgb_*')
    rgb.sort()
    depth=glob.glob(base_image_folder_name+'/depth_*')
    depth.sort()
    rgb_img = np.array(cv2.imread(rgb[-1]))
    depth_img = np.array(cv2.imread(depth[-1]))
    return rgb_img, depth_img

def passivePerception():
    img, depth = read_next_image()
    detections=getDetections(img)
    if detections.shape[0]>0:
        cloud, labels = getPointCloud(detections, depth)
        return cloud, labels
    return None, None


def testFunctions():
    from glob import glob
    imgs = glob('test_imgs/imgs/*')#['https://ultralytics.com/images/zidane.jpg']  # batch of images
    depth = glob('test_imgs/depth/*')
    objects=getDetections(imgs)
    for i,obj in enumerate(objects):
        clouds, ids = getPointCloud(obj, cv2.imread(depth[i]))
        assert(len(clouds)==len(obj))


if __name__=='__main__':
    testFunctions()