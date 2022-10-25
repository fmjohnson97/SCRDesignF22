import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from interbotix_xs_msgs.msg import JointSingleCommand
from interbotix_xs_msgs.srv import RobotInfo
class Gripper:
    def __init__(self):
        self.topic = '/locobot/gripper_controller/command'
        self.rs_topic = '/locobot/joint_states'
        self.pub = rospy.Publisher(self.topic, JointTrajectory)
        self.pub2 = rospy.Publisher('/locobot/commands/joint_single', JointSingleCommand)
        self.robot_info_srv_name = '/locobot/get_robot_info'
        self.srv_get_info = rospy.ServiceProxy(self.robot_info_srv_name, RobotInfo)
        rospy.sleep(1.0)
        gripper_info = self.srv_get_info('single', 'gripper')
        self.joint_upper_limit = 0.85
        self.joint_lower_limit = 0.02  # actually this can go up to -0.1
        print('upper limit: ', self.joint_upper_limit)
        print('lower limit: ', self.joint_lower_limit)

    def get_joint_state(self):
        rs_msg = rospy.wait_for_message(self.rs_topic, JointState)
        rs_joints = {}
        for i in range(len(rs_msg.name)):
            rs_joints[rs_msg.name[i]] = rs_msg.position[i]
        return rs_joints

    def control(self, target_joint):
        # target_joint = min(target_joint, self.joint_upper_limit)
        # target_joint = max(target_joint, self.joint_lower_limit)
        # msg = JointSingleCommand()
        # msg.name = 'gripper'
        # msg.cmd = target_joint
        # self.pub2.publish(msg)
        msg = JointTrajectory()
        msg.joint_names = ['left_finger']

        pt = JointTrajectoryPoint()
        print('target: ', target_joint)
        joints = self.get_joint_state()
        pt.positions = [joints['gripper']]
        # pt.velocities = [0.2]
        pt.time_from_start = rospy.Time(0.0)
        pt1 = pt
        msg.points = [pt1]

        pt = JointTrajectoryPoint()
        print('target: ', target_joint)
        pt.positions = [target_joint]
        # pt.velocities = [0.2]
        pt.time_from_start = rospy.Time(3)
        msg.points = [pt1, pt]
        self.pub.publish(msg)

if __name__ == '__main__':
    rospy.init_node('simple_controller_gripper', anonymous=True)
    gripper = Gripper()
    # watter bottle: 0.12
    gripper.control(0.12)
    input('next...')
    gripper.control(0.85)
    input('next...')