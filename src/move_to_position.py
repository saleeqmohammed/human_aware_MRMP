#!/usr/bin/env python3


import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

class RobotController:
    def __init__(self):
        rospy.init_node('robot_controller')
        self.cmd_vel_pub = rospy.Publisher('robot_0/cmd_vel', Twist, queue_size=10)
        self.odom_sub = rospy.Subscriber('robot_0/odom', Odometry, self.odom_callback)
        self.current_velocity = Twist()
        self.rate = rospy.Rate(10)  # 10 Hz

    def odom_callback(self, data):
        self.current_velocity = data.twist.twist

    def move_step(self):
        move_cmd = Twist()
        move_cmd.linear.x = 1.0  # Move forward with velocity 1.0 m/s
        self.cmd_vel_pub.publish(move_cmd)
        rospy.sleep(1)  # Move for 1 second
        move_cmd.linear.x = 0
        self.cmd_vel_pub.publish(move_cmd)

    def run(self):
        while not rospy.is_shutdown():
            self.move_step()
            rospy.loginfo(f'Current Velocity: {self.current_velocity.linear.x}, {self.current_velocity.angular.z}')
            self.rate.sleep()

if __name__ == '__main__':
    try:
        controller = RobotController()
        controller.run()
    except rospy.ROSInterruptException:
        pass
