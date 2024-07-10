#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Pose, Quaternion

def publish_target_pose():
    rospy.init_node('publish_target_pose')
    pub = rospy.Publisher('/target_pose', Pose, queue_size=10)
    rate = rospy.Rate(1)  # 1 Hz

    while not rospy.is_shutdown():
        # Create a Pose message
        target_pose = Pose()
        target_pose.position.x = 1.0  # Example x position
        target_pose.position.y = 2.0  # Example y position
        target_pose.position.z = 0.0  # Assuming 2D navigation
        
        # Orientation as a quaternion
        target_pose.orientation = Quaternion(0.0, 0.0, 0.0, 1.0)  # No rotation
        
        # Publish the Pose message
        pub.publish(target_pose)
        
        rospy.loginfo("Target Pose published: \n{}".format(target_pose))
        rate.sleep()

if __name__ == '__main__':
    try:
        publish_target_pose()
    except rospy.ROSInterruptException:
        pass
