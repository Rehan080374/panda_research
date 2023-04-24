#!/usr/bin/env python

import rospy
import tf.transformations as transform
import tf.transformations
import numpy as np
import math
import actionlib
from franka_gripper.msg import GraspAction, GraspGoal,MoveAction,MoveGoal
from geometry_msgs.msg import Pose
import tf2_ros
import tf2_geometry_msgs

from geometry_msgs.msg import PoseStamped
from franka_msgs.msg import FrankaState

from sensor_msgs.msg import JointState 
from scipy.spatial.transform import Rotation as R
initial_pose_found = False
pose_current = PoseStamped()
pose_desired = PoseStamped()
marker_pose = PoseStamped()

p=[0,0,0]
x=0
y=0
z=0

def publisherCallback(msg):

    pub.publish(marker_pose) 

def franka_state_callback(msg):
    global pose_current, initial_pose_found,p,marker_pose
    if not initial_pose_found:
        pose_current.pose.position.x = msg.O_T_EE[12]
        pose_current.pose.position.y = msg.O_T_EE[13]
        pose_current.pose.position.z = msg.O_T_EE[14]
        initial_pose_found = True
        marker_pose=pose_current
    p=[msg.O_T_EE[12],msg.O_T_EE[13],msg.O_T_EE[14]]
    #  print(p)

def transform_pose(input_pose, from_frame, to_frame):

    # **Assuming /tf2 topic is being broadcasted
    

    pose_stamped = tf2_geometry_msgs.PoseStamped()
    pose_stamped.pose = input_pose
    pose_stamped.header.frame_id = from_frame
    pose_stamped.header.stamp = rospy.Time(0)

    try:
        # ** It is important to wait for the listener to start listening. Hence the rospy.Duration(1)
        output_pose_stamped = tf_buffer.transform(pose_stamped, to_frame, rospy.Duration(2))
        return output_pose_stamped.pose

    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        raise
def pose_callback(msg):
    pose1=[0,0,0] 
    global p,x,y,z
    r=0.4
    marker_pose.pose.position.x=p[0]
    marker_pose.pose.position.y=p[1]
    marker_pose.pose.position.z=p[2]   
    pose1=[msg.pose.position.x-r,msg.pose.position.y,msg.pose.position.z-0.08]
    
   
    # print("pose  = ",pose1)
    my_pose = Pose()
    my_pose.position.x= pose1[0]
    my_pose.position.y= pose1[1]
    my_pose.position.z= pose1[2]
  
    
    
    
 
    t_pose = transform_pose(my_pose, "zedm_left_camera_frame", "panda_link0")
  
    
  
    pose2=[t_pose.position.x,t_pose.position.y,t_pose.position.z] 
    
    
   
    scale =0.5
    error=[0,0,0]
    for i in range(3):
        error[i]=(pose2[i]-p[i])
    # print ("error = ",error)    
    if msg.pose.position.x !=x or msg.pose.position.y !=y or msg.pose.position.z !=z :  
        if abs(error[0])>0.02 :   
            marker_pose.pose.position.x += (error[0])*scale
            # marker_pose.pose.position.x=pose2[0] 
        if abs(error[1])>0.02 :   
            marker_pose.pose.position.y += (error[1])*scale
            # marker_pose.pose.position.y=pose2[1]
        if abs(error[2])>0.02 :   
            marker_pose.pose.position.z += (error[2])*scale
            # marker_pose.pose.position.z=pose2[2]
        print("pose  = ",marker_pose) 
        
    
     
    pose1=[0,0,0] 
    x=msg.pose.position.x
    y=msg.pose.position.y
    z=msg.pose.position.z
    # rospy.sleep(0.1)
        
    
    
   
if __name__ == '__main__':
    rospy.init_node('pose_publisher')
    pub = rospy.Publisher('/cartesian_pose_example_controller/equilibrium_pose', PoseStamped, queue_size=10)
    state_sub = rospy.Subscriber('/franka_state_controller/franka_states', FrankaState, franka_state_callback)
    tf_buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tf_buffer)
    pose_sub = rospy.Subscriber('my_equilibrium_pose', PoseStamped, pose_callback)

    pose_desired.header.frame_id = 'panda_link0'
    # rospy.wait_for_message('/franka_state_controller/franka_states', FrankaState)
    # rospy.sleepif ((elapsed_time_.toSec()>0.5)&&(velocity[0] <= velocity_threshold && velocity[1] <= velocity_threshold && velocity[2] <= velocity_threshold)) {
  
    # link_name = rospy.get_param("~link_name")
    while not initial_pose_found:
        rospy.sleep(1)

    state_sub.unregister()
    # rospy.Timer(rospy.Duration(0.01),
    #             lambda msg: publisherCallback(msg))

    rate = rospy.Rate(2) # 10hz
    while not rospy.is_shutdown():
        pub.publish(marker_pose)
        rate.sleep()
