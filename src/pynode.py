#!/usr/bin/env python3

import rospy
import numpy as np
from laser_pkg.msg import Laser3d, PointArray
import math
from squaternion import Quaternion
from scipy.spatial.transform import Rotation as R
from geometry_msgs.msg import Point

INF = 10000000000
rospy.init_node ('laser_scan')
pub = rospy.Publisher ('/ros/laser/pcl', PointArray, queue_size=10, latch=True)

def cb (data):
    h_count = data.count
    v_count = data.vertical_count
    h_step = data.angle_step
    v_step = data.vertical_angle_step
    h_min = data.angle_min
    h_max = data.angle_max
    v_min = data.vertical_angle_min
    v_max = data.vertical_angle_max

    origin = [data.world_pose.position.x, \
              data.world_pose.position.y, \
              data.world_pose.position.z]

    w = data.world_pose.orientation.w
    x = data.world_pose.orientation.x
    y = data.world_pose.orientation.y
    z = data.world_pose.orientation.z

    r_mat = R.from_quat([w, x, y, z]).as_matrix ()
    pcl_msg = PointArray ()

    for i in range (h_count):
        for j in range (v_count):
            ind = j * h_count + i
            h_angle = h_min + h_step * i
            v_angle = v_min + v_step * j
            r = data.ranges[ind]
            if r > INF:
                continue
            x_ = r * math.cos (v_angle) * math.cos (h_angle)
            y_ = r * math.cos (v_angle) * math.sin (h_angle)
            z_ = r * math.sin (v_angle)
            p = np.array ([x_,y_,z_])
            coord = r_mat.dot (p)
            pcl_msg.x.append (coord[0])
            pcl_msg.y.append (coord[1])
            pcl_msg.z.append (coord[2])

    pub.publish (pcl_msg)

sub = rospy.Subscriber ('/ros/laser/scan', Laser3d, cb)

rospy.spin ()
