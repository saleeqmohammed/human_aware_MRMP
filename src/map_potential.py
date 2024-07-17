#!/usr/bin/env python3

"""
This node publishes the borders from map as borders for social force model

"""
import rospy
import numpy as np
import matplotlib.pyplot as plt
from nav_msgs.msg import OccupancyGrid
from cv_bridge import CvBridge
import cv2
from skimage.morphology import skeletonize
import pickle
def thin_edges(edge_map):
    """
    Thin the edges in the binary edge map.
    
    Parameters:
    edge_map (numpy.ndarray): Binary edge map.
    
    Returns:
    numpy.ndarray: Thinned binary edge map.
    """
    # Ensure the edge map is binary
    edge_map = (edge_map > 0).astype(np.uint8)
    
    # Perform thinning using skeletonize
    thinned = edge_map#skeletonize(edge_map).astype(np.uint8) * 255
    return thinned

def get_lines(edge_map):
    if edge_map.dtype !=np.uint8:
        edge_map = (edge_map *255).astype(np.uint8)

    lines = np.squeeze(cv2.HoughLinesP(edge_map, rho=1, theta=np.pi/180, threshold=80, minLineLength=3, maxLineGap=10))
    line_endpoints=[]
  
    # if lines is not None:
    #     for line in lines:
    #         x1,y1,x2,y2 = line[0]
    #         line_endpoints.append(((x1,y1),(x2,y2)))

    #plot the boundaries
    color_edge_map = cv2.cvtColor(edge_map, cv2.COLOR_GRAY2RGB)
    
    for line in lines:
        x1, y1, x2, y2 = line
        cv2.line(color_edge_map, (x1, y1), (x2, y2), (255, 0, 0), 1)
    
    # Plot using matplotlib
    plt.figure(figsize=(10, 10))
    plt.imshow(color_edge_map)
    plt.title('Detected Lines')
    plt.axis('off')
    # plt.show()
    return line_endpoints
def map_callback(msg):
    global map_data
    global map_width
    global map_height

    map_width = msg.info.width
    map_height = msg.info.height
    map_data = np.array(msg.data).reshape((map_height, map_width))

    # Convert map data to binary (1 for occupied, 0 for free or unknown)

    map_data_binary = np.where(map_data == 100, 1, 0)
    np.save('binary_map.npy',map_data_binary)
    thinned_map = thin_edges(map_data_binary)
    line_endpoints =get_lines(thinned_map)
    # Display the map as an image using matplotlib
    plt.figure(figsize=(8, 6))
    plt.imshow(map_data_binary, cmap='gray', origin='lower')
    plt.title('Occupancy Grid Map')
    plt.colorbar(label='Occupancy')
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.show()
    

def map_listener():
    rospy.init_node('map_to_numpy_node', anonymous=True)
    rospy.Subscriber('/map', OccupancyGrid, map_callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        map_listener()
    except rospy.ROSInterruptException:
        pass
