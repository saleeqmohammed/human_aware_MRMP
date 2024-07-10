#!/usr/bin/env python

import rospy
import numpy as np
import matplotlib.pyplot as plt
from nav_msgs.msg import OccupancyGrid
from cv_bridge import CvBridge
def calcualte_potential(map_data_binary):
    potential_field = np.zeros_like(map_data_binary)
    #generate 1/r map
    i=0
    j=0
    for i in range(map_data_binary)
    pass
def map_callback(msg):
    global map_data
    global map_width
    global map_height

    map_width = msg.info.width
    map_height = msg.info.height
    map_data = np.array(msg.data).reshape((map_height, map_width))

    # Convert map data to binary (1 for occupied, 0 for free or unknown)
    map_data_binary = np.where(map_data == 100, 1, 0)

    # Display the map as an image using matplotlib
    plt.figure(figsize=(8, 6))
    plt.imshow(map_data_binary, cmap='gray', origin='lower')
    plt.title('Occupancy Grid Map')
    plt.colorbar(label='Occupancy')
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.show()
    calculate_potential(map_data_binary)

def map_listener():
    rospy.init_node('map_to_numpy_node', anonymous=True)
    rospy.Subscriber('/map', OccupancyGrid, map_callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        map_listener()
    except rospy.ROSInterruptException:
        pass
