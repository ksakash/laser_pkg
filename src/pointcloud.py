#!/usr/bin/env python3

import rospy
import numpy as np
import math
from laser_pkg.msg import PointArray

rospy.init_node ('pointcloud')
cumulative_points = np.array ([[0,0,0]])

def cb (data):
    global cumulative_points
    l = len (data.x)
    arr = np.empty ((l, 3))
    arr[:,0] = data.x
    arr[:,1] = data.y
    arr[:,2] = data.z
    cumulative_points = np.append (cumulative_points, arr, axis=0)

sub = rospy.Subscriber ('/ros/laser/pcl', PointArray, cb)

rospy.spin ()
