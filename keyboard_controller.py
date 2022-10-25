"""
on input of directional key, use left and right to turn the robot. use front and back to move
"""

import rospy
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import Twist
import copy
import cv2
import time
class KeyboardController:
  def __init__(self):
    self.pub = rospy.Publisher('/locobot/cmd_vel', Twist)
    self.linear_v = 0.05
    self.ang_v = 0.05
  def control(self):
    cv2.namedWindow('control')
    while not rospy.is_shutdown():
      key = cv2.waitKey(1) & 0xFF

      if key == ord('q'):
        # destroy the window
        cv2.destroyAllWindows()
        break
      msg = Twist()
      if key == ord('w'):
          print("up")
          msg.linear.x = self.linear_v
      elif key == ord('s'):
          print("down")
          msg.linear.x = -self.linear_v

      elif key == ord('a'):
          print("left")
          msg.angular.z = self.ang_v

      elif key == ord('d'):
          print("right")
          msg.angular.z = -self.ang_v

      self.pub.publish(msg)
      rospy.sleep(0.02)


if __name__ == "__main__":
  rospy.init_node('keyboard_op', anonymous=True)
  relay = KeyboardController()
  relay.control()
