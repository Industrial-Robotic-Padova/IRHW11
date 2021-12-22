#! /usr/bin/env python
import rospy
import time
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseResult, MoveBaseFeedback
from geometry_msgs.msg import PoseWithCovarianceStamped, Pose
from sensor_msgs.msg import LaserScan
import numpy as np
import math

# definition of the feedback callback. This will be called when feedback
# is received from the action server
# it just prints a message indicating a new message has been received
obstacles = []
robot_pos = np.array([])
obs = []

def feedback_callback(feedback):
    print('[Feedback] Going to Goal Pose...')



def sub_callback(msg):
    rz = msg.pose.pose.orientation.z
    p = [
        [math.cos(rz), -math.sin(rz), 0, msg.pose.pose.position.x],
        [math.sin(rz), math.cos(rz), 0, msg.pose.pose.position.y],
        [0, 0, 1, msg.pose.pose.position.z],
        [0, 0, 0, 1],
    ]
    global robot_pos
    robot_pos = np.array(p)
    print(msg.pose.pose)


#def get_position(d, index_mid, rp):
#    len_ranges = 666
#    range_min_rad = -1.91
#    range_max_rad = 1.91
#    range_rad = range_max_rad - range_min_rad
#    alpha_rad = ((index_mid-333) /len_ranges ) * range_rad  # rate#
#


def get_position(d, index_mid, x, y):
    len_ranges = 666
    range_min_rad = -1.91
    range_max_rad = 1.91
    range_rad = range_max_rad - range_min_rad
    alpha_rad = ( float( (index_mid-333) ) /len_ranges) * range_rad  # rate
    print("_", (index_mid-333)/666, index_mid-333 ,alpha_rad)
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
    p_objects = gdata[f_gdata]
    objects = [(f_gdata[i], f_gdata[i+1]) for i in range(len(p_objects[:-1])) if p_objects[i] < 0 and p_objects[i+1] > 0]
    for obj in objects:
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
        print(d, int((obj[0]+obj[1])/2))
        is_same = False
        for i in obs:
            if same_object(i, (x_,y_), 0.9):
                is_same = True
         
            #is_same = same_object(i,(x_,y_),r)
        print(is_same, r)
        if not is_same:
	    obs.append((x_, y_))
    print(obs)


# initializes the action client node
rospy.init_node('move_base_action_client')

# create the connection to the action server
client = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
# waits until the action server is up and running
client.wait_for_server()

sub_pose = rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, sub_callback)
laser_msg = rospy.Subscriber('/scan',LaserScan, laser_callback)
while(True):
	val = input("Enter your value: ")
	print(val)
	x,y,Rz= val
	print(x,y,Rz)
	# creates a goal to send to the action server
	goal = MoveBaseGoal()
	goal.target_pose.header.frame_id = 'map'
	goal.target_pose.pose.position.x = x
	goal.target_pose.pose.position.y = y
	goal.target_pose.pose.position.z = 0.0
	goal.target_pose.pose.orientation.x = 0.0
	goal.target_pose.pose.orientation.y = 0.0
	goal.target_pose.pose.orientation.z = Rz
	goal.target_pose.pose.orientation.w = 0.66

	# sends the goal to the action server, specifying which feedback function
	# to call when feedback received
	client.send_goal(goal, feedback_cb=feedback_callback)

	# Uncomment these lines to test goal preemption:
	#time.sleep(3.0)
	#client.cancel_goal()  # would cancel the goal 3 seconds after starting

	# wait until the result is obtained
	# you can do other stuff here instead of waiting
	# and check for status from time to time 
	# status = client.get_state()
	# check the client API link below for more info
	print('test')
	client.wait_for_result()

	print('[Result] State: %d'%(client.get_state()))
