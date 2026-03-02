#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PoseStamped
import tf.transformations as tf_trans
import math

def pose_callback(msg):
    orientation_q = msg.pose.orientation
    quaternion = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    roll, pitch, yaw = tf_trans.euler_from_quaternion(quaternion)

    # 将 yaw 角度规范化为 [-pi, pi]
    yaw = math.atan2(math.sin(yaw), math.cos(yaw))

    rospy.loginfo("Current Yaw: %.3f rad (%.1f deg)", yaw, math.degrees(yaw))

def main():
    rospy.init_node('yaw_monitor', anonymous=True)
    rospy.Subscriber('/mavros/local_position/pose', PoseStamped, pose_callback)
    rospy.loginfo("Subscribing to /mavros/local_position/pose")
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass