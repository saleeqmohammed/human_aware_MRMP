import numpy as np
map =np.load('/home/saleeq/Projects/PySocialForce/pysocialforce/binary_map.npy')
print(map)
indeces = np.where(map==1)
resoultion = 0.050 #m/pix
x_o = indeces[0]*resoultion
y_o = indeces[1]*resoultion
obstacle_coord =[[x_i,y_i] for x_i,y_i in zip(x_o,y_o)]
obstacle_coord = obstacle_coord[:10]
print(obstacle_coord)