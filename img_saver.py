"""
save rgbd image
"""


import rospy
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped
import copy
from cv_bridge import CvBridge
import cv2
import os
from sensor_msgs.msg import Image
import numpy as np
class ImageSaver:
  def __init__(self):
    self.bridge = CvBridge()
    self.base_name = 'saved_imgs'
    if not os.path.exists(self.base_name):
      os.makedirs(self.base_name)

  def save_img(self, idx):
    """
    images are named as rgb_[idx].png, depth_[idx].png
    """
    rgb_img = rospy.wait_for_message("/locobot/camera/color/image_raw", Image)
    depth_img = rospy.wait_for_message("/locobot/camera/aligned_depth_to_color/image_raw", Image)
    rgb_img = self.bridge.imgmsg_to_cv2(rgb_img, 'passthrough')
    depth_img = self.bridge.imgmsg_to_cv2(depth_img, 'passthrough')
    rgb_img = cv2.cvtColor(rgb_img.astype(np.float32), cv2.COLOR_RGB2BGR)
    cv2.imwrite(os.path.join(self.base_name, 'rgb_%d.png'%(idx)), rgb_img)
    cv2.imwrite(os.path.join(self.base_name, 'depth_%d.png'%(idx)), depth_img.astype(np.uint16))


if __name__ == "__main__":
  rospy.init_node('img_saver', anonymous=True)
  saver = ImageSaver()
  idx = -1
  while not rospy.is_shutdown():
    idx += 1
    input('next...')
    saver.save_img(idx)

