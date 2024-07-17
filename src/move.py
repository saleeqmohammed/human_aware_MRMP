#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist

def move_robot(duration_secs):
    rospy.init_node('move_robot_node', anonymous=True)
    cmd_vel_pub = rospy.Publisher('robot_0/cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(10)  # 10 Hz

    start_time = rospy.get_time()
    end_time = start_time + duration_secs

    while not rospy.is_shutdown():
        current_time = rospy.get_time()
        if current_time >= end_time:
            break
        
        vel_cmd = Twist()
        vel_cmd.linear.x = -1.0  # Example: 1.0 m/s forward velocity
        cmd_vel_pub.publish(vel_cmd)
        
        rospy.loginfo("Current time: %.2f", current_time)  # Log current time
        rate.sleep()  # Sleep to maintain the loop rate

    # Stop the robot after reaching the desired duration
    vel_cmd = Twist()
    cmd_vel_pub.publish(vel_cmd)

if __name__ == '__main__':
    try:
        move_robot(5.0)  # Move for 5 seconds
    except rospy.ROSInterruptException:
        pass
