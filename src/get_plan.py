#!/usr/bin/env python3

import rospy
from nav_msgs.srv import GetPlan
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

def make_plan(start_pose, goal_pose):
    rospy.wait_for_service('/robot_0/move_base/make_plan')
    try:
        get_plan = rospy.ServiceProxy('/robot_0/move_base/make_plan', GetPlan)
        start = PoseStamped()
        start.header.frame_id = "map"
        start.pose = start_pose

        goal = PoseStamped()
        goal.header.frame_id = "map"
        goal.pose = goal_pose

        tolerance = 0.5  # Set an appropriate tolerance for your use case
        response = get_plan(start=start, goal=goal, tolerance=tolerance)
        return response.plan
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s" % e)
        return None

def main():
    rospy.init_node('get_global_plan')

    # Define start and goal poses
    start_pose = {
        'position': {'x': 0.0, 'y': 0.0, 'z': 0.0},
        'orientation': {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0}
    }
    
    goal_pose = {
        'position': {'x': -5.0, 'y': 4.0, 'z': 0.0},
        'orientation': {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0}
    }

    start_pose_msg = PoseStamped()
    start_pose_msg.header.frame_id = "map"
    start_pose_msg.pose.position.x = start_pose['position']['x']
    start_pose_msg.pose.position.y = start_pose['position']['y']
    start_pose_msg.pose.position.z = start_pose['position']['z']
    start_pose_msg.pose.orientation.x = start_pose['orientation']['x']
    start_pose_msg.pose.orientation.y = start_pose['orientation']['y']
    start_pose_msg.pose.orientation.z = start_pose['orientation']['z']
    start_pose_msg.pose.orientation.w = start_pose['orientation']['w']
    
    goal_pose_msg = PoseStamped()
    goal_pose_msg.header.frame_id = "map"
    goal_pose_msg.pose.position.x = goal_pose['position']['x']
    goal_pose_msg.pose.position.y = goal_pose['position']['y']
    goal_pose_msg.pose.position.z = goal_pose['position']['z']
    goal_pose_msg.pose.orientation.x = goal_pose['orientation']['x']
    goal_pose_msg.pose.orientation.y = goal_pose['orientation']['y']
    goal_pose_msg.pose.orientation.z = goal_pose['orientation']['z']
    goal_pose_msg.pose.orientation.w = goal_pose['orientation']['w']

    plan = make_plan(start_pose_msg.pose, goal_pose_msg.pose)
    
    if plan:
        rospy.loginfo("Received plan with %d points." % len(plan.poses))
        for i, pose in enumerate(plan.poses):
            rospy.loginfo("Point %d: x=%f, y=%f" % (i, pose.pose.position.x, pose.pose.position.y))
    else:
        rospy.logwarn("No plan received.")

if __name__ == '__main__':
    main()
