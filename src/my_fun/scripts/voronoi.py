#!/usr/bin/env python

import rospy
import matplotlib.pylab as plt
import numpy as np
from skimage import color
from scipy.spatial import Voronoi, voronoi_plot_2d
import scipy.ndimage as ndimage

rospy.init_node('voronoi')
img_file = '/home/davide/ros_ws/src/learning/learning_toponav/maps/office/metric.pgm'
# img = color.rgb2gray(plt.imread(img_file, format='pgm'))
img = plt.imread(img_file, format='pgm')

# points = []
# for i in range(100):
#     points.append([np.random.uniform(0, img.shape[0]),np.random.uniform(0, img.shape[1])])
# points = np.array(points)
points = []
while not rospy.has_param('/interest_points'):
    rospy.sleep(0.1)
for i in rospy.get_param('/interest_points'):
    points.append(
        (
            i['pose']['position']['x'],
            i['pose']['position']['y']
        )
    )

vor = Voronoi(points)

fig = plt.figure(figsize=(20, 20))
ax = fig.add_subplot(111)
# ax.imshow(ndimage.rotate(img, 90))
# ax.imshow(img, cmap='gray')
voronoi_plot_2d(vor, point_size=10, ax=ax)
plt.show()

# rospy.spin()
