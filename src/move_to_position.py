#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist, Pose, Point, Quaternion
from math import sqrt

class MoveToXYPosition:
    def __init__(self):
        rospy.init_node('move_to_xy_position')
        
        self.rate = rospy.get_param('~rate', 10)  # Update rate in Hz
        self.velocity = rospy.get_param('~velocity', 0.1)  # Constant velocity in m/s
        
        self.current_pose = Pose()
        self.target_pose = Pose()
        self.distance_threshold = rospy.get_param('~distance_threshold', 0.05)  # Threshold to consider position reached
        
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.target_pos_sub = rospy.Subscriber('/target_position', Pose, self.target_position_callback)

    def target_position_callback(self, msg):
        self.target_pose = msg
        self.move_to_target()

    def move_to_target(self):
        # Calculate distance to target
        distance = sqrt((self.target_pose.position.x - self.current_pose.position.x)**2 + 
                        (self.target_pose.position.y - self.current_pose.position.y)**2)
        
        # Calculate time required to reach the target
        time_to_target = distance / self.velocity
        
        # Create Twist message
        twist = Twist()
        twist.linear.x = self.velocity
        
        # Publish velocity command
        rate = rospy.Rate(self.rate)
        start_time = rospy.Time.now()
        while not rospy.is_shutdown():
            self.cmd_vel_pub.publish(twist)
            current_time = rospy.Time.now()
            elapsed_time = (current_time - start_time).to_sec()
            
            # Update current pose assuming constant velocity
            self.current_pose.position.x += self.velocity * (current_time - start_time).to_sec()
            self.current_pose.position.y += self.velocity * (current_time - start_time).to_sec()
            
            # Check if reached the target position
            if distance <= self.distance_threshold:
                rospy.loginfo("Reached target position")
                break
            
            # Break loop if exceeded calculated time
            if elapsed_time >= time_to_target:
                rospy.loginfo("Failed to reach target position in time")
                break
            
            rate.sleep()
        
        # Stop the robot
        twist.linear.x = 0.0
        self.cmd_vel_pub.publish(twist)

if __name__ == '__main__':
    try:
        node = MoveToXYPosition()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
