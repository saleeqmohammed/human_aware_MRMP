#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
import sys, select, termios, tty

# Key mappings
moveBindings = {
    'w': (1, 0),
    's': (-1, 0),
    'd': (0, 1),
    'a': (0, -1),
    'q': (0, 0)  # To stop the robot
}

def getKey():
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def main():
    rospy.init_node('teleop_twist_keyboard')
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    speed = rospy.get_param("~speed", 0.5)

    x = 0
    y = 0
    th = 0
    status = 0

    try:
        print("Use WASD keys to move the robot")
        print("Press Q to stop the robot")
        print("Press Ctrl+C to quit")
        
        while not rospy.is_shutdown():
            key = getKey()
            if key in moveBindings.keys():
                x = moveBindings[key][0]
                y = moveBindings[key][1]
                if key == 'q':
                    x = 0
                    y = 0

                twist = Twist()
                twist.linear.x = x * speed
                twist.linear.y = y * speed
                twist.linear.z = 0
                twist.angular.x = 0
                twist.angular.y = 0
                twist.angular.z = 0
                pub.publish(twist)

            if key == '\x03':  # Detect Ctrl+C
                break

    except Exception as e:
        print(e)

    finally:
        twist = Twist()
        twist.linear.x = 0
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = 0
        pub.publish(twist)

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

if __name__ == "__main__":
    settings = termios.tcgetattr(sys.stdin)
    main()
