#!/usr/bin/env python

import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Twist
import numpy as np
import matplotlib.pyplot as plt

class PathTrackerNode:
    def __init__(self):
        rospy.init_node('path_tracker_node', anonymous=True)

        self.path = None
        self.current_waypoint_index = 0

        # Subscribe to the path topic
        rospy.Subscriber('/path_topic', Path, self.path_callback)

        # Publisher for robot commands (Twist messages)
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        # Constants
        self.dt = 0.1  # Timestep for simulation
        self.max_speed = 1.0  # Maximum speed of the robot
        self.Kp_y = 0.5  # Proportional gain for lateral error (y_e)
        self.Kp_theta = 1.0  # Proportional gain for heading error (theta_e)

        # Initial robot state
        self.x_robot = 0.0  # Initial x position
        self.y_robot = 0.0  # Initial y position
        self.theta_robot = np.pi / 4.0  # Initial orientation angle (45 degrees)

        rospy.spin()

    def path_callback(self, msg):
        # Extract path waypoints
        self.path = msg.poses
        self.current_waypoint_index = 0

    def compute_frenet_coordinates(self):
        if self.path is None or len(self.path) == 0:
            rospy.logwarn("Path is not yet received or empty.")
            return

        nearest_waypoint = self.path[self.current_waypoint_index].pose.position
        next_waypoint = self.path[min(self.current_waypoint_index + 1, len(self.path) - 1)].pose.position
        prev_waypoint = self.path[max(self.current_waypoint_index - 1, 0)].pose.position

        # Convert waypoints to numpy arrays for easier computation
        nearest_point = np.array([nearest_waypoint.x, nearest_waypoint.y])
        next_point = np.array([next_waypoint.x, next_waypoint.y])
        prev_point = np.array([prev_waypoint.x, prev_waypoint.y])

        # Calculate heading direction of the path segment
        path_direction = next_point - prev_point
        path_heading = np.arctan2(path_direction[1], path_direction[0])

        # Vector from robot to nearest point on path
        robot_to_path = nearest_point - np.array([self.x_robot, self.y_robot])

        # Lateral error (y_e) is the projection of robot_to_path onto the path heading direction
        y_e = np.dot(robot_to_path, np.array([-np.sin(path_heading), np.cos(path_heading)]))

        # Heading error (theta_e) is the difference between robot's orientation and path heading
        theta_e = self.theta_robot - path_heading

        return y_e, theta_e

    def control_robot(self):
        if self.path is None or len(self.path) == 0:
            rospy.logwarn("Path is not yet received or empty.")
            return

        # Compute lateral and heading errors
        y_e, theta_e = self.compute_frenet_coordinates()

        # Control law - simple proportional control
        V = self.max_speed  # Constant speed for simplicity
        omega = -self.Kp_y * y_e + self.Kp_theta * theta_e

        # Publish control commands (Twist message)
        cmd_msg = Twist()
        cmd_msg.linear.x = V
        cmd_msg.angular.z = omega
        self.cmd_pub.publish(cmd_msg)

    def update_robot_state(self):
        # Update robot's position and orientation
        self.x_robot += self.max_speed * np.cos(self.theta_robot) * self.dt
        self.y_robot += self.max_speed * np.sin(self.theta_robot) * self.dt
        self.theta_robot += self.control_robot() * self.dt

    def run(self):
        rate = rospy.Rate(1 / self.dt)  # Adjust loop rate based on timestep
        while not rospy.is_shutdown():
            self.update_robot_state()
            rate.sleep()

if __name__ == '__main__':
    try:
        path_tracker = PathTrackerNode()
        path_tracker.run()
    except rospy.ROSInterruptException:
        pass
