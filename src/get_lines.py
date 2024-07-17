#!/usr/bin/env python3
import numpy as np
from skimage.morphology import skeletonize 

import matplotlib.pyplot as plt
import cv2
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
    thinned = skeletonize(edge_map).astype(np.uint8) * 255
    
    return thinned
#16 + 12 + 3 + 12 +4 +4
def get_lines(edge_map):
    if edge_map.dtype !=np.uint8:
        edge_map = (edge_map *255).astype(np.uint8)


    lines = np.squeeze(cv2.HoughLinesP(edge_map, rho=1, theta=np.pi/180, threshold=10, minLineLength=3, maxLineGap=10))
    line_endpoints=[]
    print(len(lines))
    if lines is not None:
        for line in lines:
            x1,y1,x2,y2 = line
            line_endpoints.append([x1,x2,y1,y2])

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
    plt.show()
    return line_endpoints

edge_map = np.load('binary_map.npy')
thin_map = thin_edges(edge_map=edge_map)
line_endpoints =get_lines(thin_map)
np.save('line_endpoints.npy',line_endpoints)