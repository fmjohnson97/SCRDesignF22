"""
relay messages from ROS2 to ROS1. Set the header time to be current
notice that we don't have footprint frame on the arm. So we need to change the published frame to
footprint on the create3 to base_link
"""

import rospy
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped
import copy
def tf_cb(data):
  pass

class ROS2Relay:
  def __init__(self):
    self.pub1 = rospy.Publisher('tf', TFMessage)
    self.sub1 = rospy.Subscriber("/locobot/tf", TFMessage, self.tf_cb)
    self.pub2 = rospy.Publisher('tf_static', TFMessage)
    self.sub2 = rospy.Subscriber("/locobot/tf_static", TFMessage, self.tf_static_cb)

    pass
  def tf_cb(self, data: TFMessage):
    msg = TFMessage()
    transforms = []
    for tf_transform in data.transforms:
      tf = copy.deepcopy(tf_transform)
      if tf.child_frame_id == 'base_footprint':
        tf.child_frame_id = 'base_link'
      else:
        continue
      tf.header.stamp = rospy.get_rostime()
      transforms.append(tf)
    msg.transforms = transforms
    self.pub1.publish(msg)

  def tf_static_cb(self, data: TFMessage):
    msg = TFMessage()
    transforms = []
    for tf_transform in data.transforms:
      tf = copy.deepcopy(tf_transform)
      if tf.child_frame_id == 'base_footprint':
        tf.child_frame_id = 'base_link'
      else:
        continue
      tf.header.stamp = rospy.get_rostime()
      transforms.append(tf)
    msg.transforms = transforms
    self.pub2.publish(msg)


if __name__ == "__main__":
  rospy.init_node('ros2_to_1_topic_relay', anonymous=True)
  relay = ROS2Relay()
  rospy.spin()
