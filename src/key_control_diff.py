#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
import sys, select, termios, tty

# Set up the key mappings
MOVE_BINDINGS = {
    'w': (1, 0),  # Move forward
    's': (-1, 0), # Move backward
    'a': (0, 1),  # Turn left
    'd': (0, -1), # Turn right
    'q': (0, 0)   # Stop
}

# Set the speed settings
LIN_VEL = 0.5  # Linear velocity (m/s)
ANG_VEL = 1.0  # Angular velocity (rad/s)

def getKey():
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def main():
    rospy.init_node('keyboard_control')

    # Use namespaces to send commands to the correct robot
    robot_ns = 'tb3_0'#rospy.get_namespace()
    cmd_vel_pub = rospy.Publisher(robot_ns + 'cmd_vel', Twist, queue_size=10)

    twist = Twist()
    print("Use 'WASD' to move the robot, 'Q' to stop. Press 'Ctrl+C' to exit.")

    while not rospy.is_shutdown():
        key = getKey()

        if key in MOVE_BINDINGS.keys():
            lin, ang = MOVE_BINDINGS[key]
            twist.linear.x = lin * LIN_VEL
            twist.angular.z = ang * ANG_VEL
        else:
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            if key == '\x03':  # Ctrl+C
                break

        cmd_vel_pub.publish(twist)
        rospy.sleep(0.1)

if __name__ == '__main__':
    settings = termios.tcgetattr(sys.stdin)
    try:
        main()
    except rospy.ROSInterruptException:
        pass
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
