#!/usr/bin/env python

import rospy
import tf
from geometry_msgs.msg import PoseStamped,Pose
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
import math
import numpy as np


def quaternion_to_euler(x,y,z,w):

    """
    Convert a quaternion into euler angles (roll, pitch, yaw)
    roll is rotation around x in radians (counterclockwise)
    pitch is rotation around y in radians (counterclockwise)
    yaw is rotation around z in radians (counterclockwise)
    """

    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)

    return roll_x, pitch_y, yaw_z # in radians

rospy.init_node("error")
tf_pose=tf.TransformListener()
rospy.Rate(10).sleep()

n=0
while not rospy.is_shutdown():
    
    now1=rospy.Time.now()
    while not tf_pose.canTransform("/map","/mocap_laser_link",now1):
        now1=tf_pose.getLatestCommonTime("/map","/mocap_laser_link")
        now2=tf_pose.getLatestCommonTime("/map","/base_link")

    (position1,orientation1)=tf_pose.lookupTransform("/map","/mocap_laser_link",now1)
    (position2,orientation2)=tf_pose.lookupTransform("/map","/base_link",now2)

    pose=Pose()
    head=Header()
    pose.position.x=position1[0]-position2[0]
    pose.position.y=position1[1]-position2[1]

    angle1=quaternion_to_euler(orientation1[0],orientation1[1],orientation1[2],orientation1[3])[2]
    angle2=quaternion_to_euler(orientation2[0],orientation2[1],orientation2[2],orientation2[3])[2]
    if angle2*angle1<0 and abs(angle1)>math.pi/2 and abs(angle2)>math.pi/2:
        if angle1<0:
            angle1=angle1+2*math.pi
        else:
            angle1=angle1-2*math.pi

    pose.orientation.z=angle2-angle1
    head.frame_id="map"
    head.seq=n
    head.stamp=now1
    rospy.Publisher("/error_data",PoseStamped,queue_size=1).publish(PoseStamped(header=head,pose=pose))

    rospy.Rate(30).sleep()