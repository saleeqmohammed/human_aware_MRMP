#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
import numpy as np
import time

def velocity_publisher(velocities):
    # Initialize the node with a name 'velocity_publisher'
    rospy.init_node('velocity_publisher', anonymous=True)

    # Create a publisher object that will publish on the '/cmd_vel' topic
    pub = rospy.Publisher('robot_0/cmd_vel', Twist, queue_size=10)

    # Set the loop rate (1 Hz means 1 second interval)
    rate = rospy.Rate(1)

    # Loop through each velocity in the array and publish it
    for vel in velocities:
        if rospy.is_shutdown():
            break
        
        # Create a Twist message
        move_cmd = Twist()

        # Set the linear and angular velocities
        move_cmd.linear.x = vel[0]  # linear.x
        move_cmd.angular.z = vel[1]  # angular.z

        # Publish the velocity
        pub.publish(move_cmd)
        rospy.loginfo("Published velocity: linear_x = %f, angular_z = %f", move_cmd.linear.x, move_cmd.angular.z)

        # Sleep for 1 second (1 Hz)
        rate.sleep()

    # Once done, stop the robot by publishing zero velocities
    stop_cmd = Twist()
    stop_cmd.linear.x = 0.0
    stop_cmd.angular.z = 0.0
    pub.publish(stop_cmd)
    rospy.loginfo("Published stop command.")

if __name__ == '__main__':
    try:
        # Example 2D array of velocities (linear.x, angular.z)
        velocities = np.array([
            [0.5, 0.0],
            [0.5, 0.2],
            [0.5, -0.2],
            [0.5, 0.0]
        ])

        velocity_publisher(velocities)
    except rospy.ROSInterruptException:
        pass
