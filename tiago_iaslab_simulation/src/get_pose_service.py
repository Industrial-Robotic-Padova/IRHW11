#! /usr/bin/env python

import rospy
from std_srvs.srv import Empty, EmptyResponse # Import the service message python classes generated from Empty.srv.
from geometry_msgs.msg import PoseWithCovarianceStamped, Pose
from sensor_msgs.msg import LaserScan
import numpy as np
import math


obstacles = []
robot_pos = np.array([])
obs = []

robot_pose = Pose()

def service_callback(request):
    print("Robot Pose:")
    print(robot_pose)
    return EmptyResponse()

def sub_callback(msg):
    print(msg.pose.pose)
    rz = msg.pose.pose.orientation.z
    p = [
        [math.cos(rz), -math.sin(rz), 0, msg.pose.pose.position.x],
        [math.sin(rz), math.cos(rz), 0, msg.pose.pose.position.y],
        [0, 0, 1, msg.pose.pose.position.z],
        [0, 0, 0, 1],
    ]
    global robot_pos
    robot_pos = np.array(p)


def get_position(d, index_mid, x, y):
    len_ranges = 666
    range_min_rad = -1.91
    range_max_rad = 1.91
    range_rad = range_max_rad - range_min_rad
    alpha_rad = ( float( (index_mid-333) ) /len_ranges) * range_rad  # rate
    x += d*math.cos(alpha_rad)
    y += d*math.sin(alpha_rad)
    return x, y

def get_distance(index_start, index_end, a, x):
    range_min_rad = -1.91
    range_max_rad = 1.91
    range_rad = range_max_rad - range_min_rad
    len_ranges = 666
    len_indexes = index_end - index_start
    alpha_rad = (len_indexes/len_ranges)*range_rad  # rate
    alpha_2_rad_tg = math.tan(alpha_rad/2)
    b = alpha_2_rad_tg*a
    return b+x, b

def same_object(c1, c2, r):
        return math.sqrt((c1[0]-c2[0])**2+(c1[1]-c2[1])**2) < r

def laser_callback(msg):
    data =  np.array(list(msg.ranges))
    gdata = np.gradient(data)
    nz_gdata = np.where(abs(gdata) > 0.2)[0]
    f_gdata = [nz_gdata[i] for i in range(len(nz_gdata[:-1])) if nz_gdata[i+1] - nz_gdata[i] > 1][1:]
    nz_gdata1 = nz_gdata[::-1]
    f_gdata1 = [nz_gdata1[i] for i in range(1, len(nz_gdata1) - 1) if nz_gdata1[i] - nz_gdata1[i + 1] > 1][1:]
    objects1 = []
    p_objects = gdata[f_gdata]
    f_gdata1 = f_gdata1[::-1]
    p_objects1 = gdata[f_gdata1]
    objects = [(f_gdata[i], f_gdata1[i + 1]) for i in range(len(p_objects[:-1])) if
               p_objects[i] < 0 < p_objects1[i + 1]]
    for obj in objects:
        if np.abs(data[obj[0]] - data[obj[1]]) < 0.2:
            objects1.append(obj)
    for obj in objects1:
        d, r = get_distance(obj[0],
                         obj[1],
                         data[obj[0]],
                         data[int((obj[0]+obj[1])/2)]
                         )

        x_, y_ = get_position(
            d,
            int((obj[0]+obj[1])/2),
            robot_pos[0][3],
            robot_pos[1][3],
        )
        is_same = False
        for i in obs:
            if same_object(i, (x_,y_), 0.9):
                is_same = True

        if not is_same:
            obs.append((x_, y_))
    if len(obs)>0:
        print(obs)

rospy.init_node('service_server')
my_service = rospy.Service('/get_pose_service', Empty , service_callback) # create the Service called get_pose_service with the defined callback
sub_pose = rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, sub_callback)
laser_msg = rospy.Subscriber('/scan',LaserScan, laser_callback)
rospy.spin() # mantain the service open.
