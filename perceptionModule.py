import torch
import numpy as np
import cv2
import rospy
from sensor_msgs.msg import Image

from glob import glob

from perceptionConfig import *

def getDetections(image):
    results = YOLO(image)
    #returns list of pandas dataframes that contain
    # xmin        ymin        xmax        ymax  confidence  class    name
    return results.pandas().xyxy

def getPointCloud(objects, depth_image):
    depth = np.array(depth_image) / 1000
    i, j = np.indices(depth.shape)
    x = (j - cx) / fx * depth
    y = (i - cy) / fy * depth
    x = x.reshape(-1)
    y = y.reshape(-1)
    depth = depth.reshape(-1)
    pcd = np.stack((x,y,depth)).T

    # mask = np.nonzero(pcd[:, 2])
    # pcd = pcd[mask]
    clouds=[]
    ids=[]
    for obj in objects:
        obj=obj.values[0]
        clouds.append(pcd[int(obj[0]):int(obj[2]),int(obj[1]):int(obj[3]),:])
        ids.append(obj[-1])

    return clouds, ids

def read_next_image():
    # rgb=glob.glob(base_image_folder_name+'/rgb_*')
    # rgb.sort()
    # depth=glob.glob(base_image_folder_name+'/depth_*')
    # depth.sort()
    print('here')
    rgb_img = rospy.wait_for_message("/locobot/camera/color/image_raw", Image)
    print('after getting rgb')
    depth_img = rospy.wait_for_message("/locobot/camera/aligned_depth_to_color/image_raw", Image)
    print('after getting depth')
    rgb_img = bridge.imgmsg_to_cv2(rgb_img, 'passthrough')
    depth_img = bridge.imgmsg_to_cv2(depth_img, 'passthrough')
    rgb_img = np.array(rgb_img)
    depth_img = np.array(depth_img)
    return rgb_img, depth_img

def passivePerception():
    img, depth = read_next_image()
    detections=getDetections(img)
    if len(detections)>0:
        cloud, labels = getPointCloud(detections, depth)
        print(labels)
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