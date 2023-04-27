#!/usr/bin/env python
import sys
import rospy
import tf.transformations as transform
import tf.transformations
import numpy as np
import math
import actionlib
import moveit_commander
from std_msgs.msg import Float32MultiArray
from franka_gripper.msg import GraspAction, GraspGoal,MoveAction,MoveGoal
from geometry_msgs.msg import Pose
import tf2_ros
import tf2_geometry_msgs
from interactive_markers.interactive_marker_server import \
    InteractiveMarkerServer, InteractiveMarkerFeedback
from visualization_msgs.msg import InteractiveMarker, \
    InteractiveMarkerControl
from geometry_msgs.msg import PoseStamped
from franka_msgs.msg import FrankaState

from sensor_msgs.msg import JointState 
from scipy.spatial.transform import Rotation as R
marker_pose = PoseStamped()
pose1=PoseStamped()
initial_pose_found = False
pose_pub = None
init_euler=[0,0,0]
x=0
y=0
z=0
rx=0
ry=0
rz=0
half=False
scale=1
start_time=0
previous_state="open"
gripper_c_state=''
gripper_p_state=''
item_placed=False 
# pose2=[0,0,0,0,0]
# euler=[0,0,0]
# pose1=[0,0,0]
p=[0,0,0,0,0,0]
# [[min_x, max_x], [min_y, max_y], [min_z, max_z]]
position_limits = [[-0.6, 0.6], [-0.7, 0.7], [0.02, 0.9]]

limit_l=[-4.0, -3.0,  -1.0, -0.7, -1.50, -0.5]
limit_h=[ 3.0,  3.0,   6.0,  0.6,  0.50,  0.5]       


def grasp_client():
    global gripper_state,gripper_p_state,gripper_c_state

          
    client = actionlib.SimpleActionClient('franka_gripper/grasp',GraspAction)
    print('waiting for the action server to start')
    
    gripper_p_state=gripper_state
    # print("previous state",gripper_p_state)
    timeout=rospy.Duration(2)
    client.wait_for_server(timeout)
    #goal= MoveGoal()
    
    goal = GraspGoal()
    # print("goal ==",goal)
    #goal.width = 0.058
    #goal.speed = 0.03
    goal.force = 70
    goal.epsilon.inner = 0.035
    goal.epsilon.outer = 0.0410

    goal.speed = 0.1
    goal.width = 0.04
    #print("goal=",goal)
    client.send_goal(goal)
    wait = client.wait_for_result(timeout)

    if not wait:
        rospy.logerr("Action server not available!")
        rospy.signal_shutdown("Action server not available!")
        gripper_c_state=gripper_state
    else:
        gripper_c_state=gripper_state
        # print("current state",gripper_c_state)
        return client.get_result()
            

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
def gripper_state_callback(msg):
    global gripper_state
    #  print(msg.position[0])
    if msg.position[0]>0.035:
        gripper_state='open'
    elif msg.position[0]<0.002:
        gripper_state='closed_no_obj'   
    elif msg.position[0]>0.001 and msg.position[0]<0.035:
        gripper_state='closed_with_obj'
    # print(gripper_state)         


def euler_from_quaternion(x, y, z, w):
    """
    Convert a quaternion into euler angles (roll, pitcroll is rotation around x in radians (counterclockwise)
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
def get_quaternion_from_euler(roll, pitch, yaw):
        """
        Convert an Euler angle to a quaternion.
        
        Input
            :param roll: The roll (rotation around x-axis) angle in radians.
            :param pitch: The pitch (rotation around y-axis) angle in radians.
            :param yaw: The yaw (rotation around z-axis) angle in radians.
        
        Output
            :return qx, qy, qz, qw: The orientation in quaternion [x,y,z,w] format
        """
        qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
        qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
        qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        
 
        return [qx, qy, qz, qw]
def pose_callback(msg):
    pose1=[0,0,0] 
    global p,previous_state,x,y,z,init_euler,rx,ry,rz,half,scale,gripper_p_state,gripper_c_state,start_time,item_placed
    r=0.40
    
    # pose_pub.publish(marker_pose)
    # print("gripper " , msg.header.frame_id)
    current_state=msg.header.frame_id
    
    if current_state=="close" and previous_state=="open" :
        grasp_client()  
          
    if current_state=="slow" and previous_state=="open" and half==False :   
        scale=0.1
        half=True

    elif current_state=="slow" and previous_state=="open" and half==True:   
        scale=3 
        half=False     

    if gripper_p_state=='closed_with_obj'and gripper_c_state == 'open' and item_placed== False and start_time==0:
        print("item placed") 
        item_placed=True 
        start_time=msg.header.stamp.secs
        # item_placed=False
    if gripper_p_state!='closed_with_obj'and gripper_c_state != 'open': 
        item_placed=False
    # print("scale = ",scale,half)
    # if gripper_c_state == 'open' and item_placed==True:
    #     start_time=msg.header.stamp.secs
    #     start=start_time
    #     start_time=0
    seconds =  msg.header.stamp.secs-start_time
    # print("seconds ",seconds,"start time  ",start_time)
    if seconds>5 and start_time !=0:
        # print("5 seconds passed")
        # print(" seconds passed =", seconds)
        start_time=0
        # item_placed=False    
    previous_state=current_state
       
    pose1=[msg.pose.position.x-r,msg.pose.position.y,msg.pose.position.z-0.08]
    orient=euler_from_quaternion(msg.pose.orientation.x,msg.pose.orientation.y,msg.pose.orientation.z,msg.pose.orientation.w)
    hand_quart = get_quaternion_from_euler(init_euler[0],orient[1],orient[2])
    # print("pose  = ",pose1)
    my_pose = Pose()
    my_pose.position.x= pose1[0]
    my_pose.position.y= pose1[1]
    my_pose.position.z= pose1[2]
    my_pose.orientation.x = hand_quart[0]
    my_pose.orientation.y = hand_quart[1]
    my_pose.orientation.z = hand_quart[2]
    my_pose.orientation.w = hand_quart[3]
    
    
    # print("hand euler angles = ",orient)
    # print("robot euler angles = ",p[3:6])
    t_pose = transform_pose(my_pose, "zedm_left_camera_frame", "panda_link0")
    marker_pose.pose=t_pose
    
    # t_orient=euler_from_quaternion(t_pose.orientation.x,t_pose.orientation.y,t_pose.orientation.z,t_pose.orientation.w)
    # # print("euler for hand =",t_orient[1]*180/math.pi,t_orient[2]*180/math.pi)
    # # print ("euler 1= " ,euler[1]*180/math.pi,"euler 2 =",euler[2]*180/math.pi)

    # # print("end_effector ",p,"endl hand_pose =",t_pose,"endl error =",dif)  
    # pose2=[t_pose.position.x,t_pose.position.y,t_pose.position.z,t_orient[0],t_orient[1],t_orient[2]] 
    # # if pose2[3]!=0 or pose2[4]!=0  :  
    
    # error=[]
    # for i in range(6):
    #     error.append(pose2[i]-p[i])
    # if msg.pose.position.x !=x or msg.pose.position.y !=y or msg.pose.position.z !=z :  
    #     if abs(error[0])>0.02 and seconds>5:   
    #         marker_pose.pose.position.x += (error[0])*0.08*scale
    #         # marker_pose.pose.position.x = max([min([marker_pose.pose.position.x,
    #         #                                 position_limits[0][1]]),
    #         #                                 position_limits[0][0]])
    #         # marker_pose.pose.position.x += (pose2[0]-marker_pose.pose.position.x)*0.08*scale
    #         # print("X = ", marker_pose.pose.position.x,"  x command = ",pose2[0])  
    #     if abs(error[1])>0.02 and seconds>5:   
    #         marker_pose.pose.position.y += (error[1])*0.08*scale
    #         # marker_pose.pose.position.y = max([min([marker_pose.pose.position.y,
    #         #                                 position_limits[1][1]]),
    #         #                                 position_limits[1][0]])
    #         # marker_pose.pose.position.y += (pose2[1]-marker_pose.pose.position.y)*0.08*scale
    #         # print("Y =", marker_pose.pose.position.y,"  y command = ",pose2[1]) 
    #     if abs(error[2])>0.02 :   
    #         marker_pose.pose.position.z += (error[2])*0.08*scale
    #         # marker_pose.pose.position.z = max([min([marker_pose.pose.position.z,
    #         #                                 position_limits[2][1]]),
    #         #                                 position_limits[2][0]])
    #         # marker_pose.pose.position.z += (pose2[2]-marker_pose.pose.position.z)*0.08*scale
    #         # print("z =", marker_pose.pose.position.z,"  z command = ",pose2[2])
    # roll,pitch,yaw=euler_from_quaternion(marker_pose.pose.orientation.x,marker_pose.pose.orientation.y,marker_pose.pose.orientation.z,marker_pose.pose.orientation.w)    
    # if (roll!=rx or pitch !=ry or yaw != rz ) and seconds>5 :         
    #     # roll,pitch,yaw=euler_from_quaternion(marker_pose.pose.orientation.x,marker_pose.pose.orientation.y,marker_pose.pose.orientation.z,marker_pose.pose.orientation.w)  
    #     # if abs(init_euler[0]-p[3])>0.02 and init_euler[0]!=0 :
    #     roll =  init_euler[0]
    #         # print("RX")  
    #     if abs(error[4])>0.08:
    #         pitch +=  (error[4])*0.03*scale  
    #         # print("RY")   
    #     if  abs(error[5])>0.08:
    #         yaw   +=  (error[5])*0.03*scale
    #         # print("RZ") 
    #     # print("pitch = ",r[1],"yaw =",r[2])
    #     rot=get_quaternion_from_euler(roll,pitch,yaw)
    #     rot1=transform.quaternion_from_euler(roll,pitch,yaw)
    #     marker_pose.pose.orientation.x = rot[0]
    #     marker_pose.pose.orientation.y = rot[1]
    #     marker_pose.pose.orientation.z = rot[2]
    #     marker_pose.pose.orientation.w = rot[3] 
        
    
    # # pose_pub.publish(marker_pose) 
    # pose1=[0,0,0] 
    # x=msg.pose.position.x
    # y=msg.pose.position.y
    # z=msg.pose.position.z
    # rx=roll
    # ry=pitch
    # rz=yaw
    print(marker_pose)    
    
def franka_state_callback(msg):
    
    global initial_pose_found,p,euler,init_euler
    initial_quaternion = \
        transform.quaternion_from_matrix(
            np.transpose(np.reshape(msg.O_T_EE,
                                    (4, 4))))
    # initial_quaternion = initial_quaternion / np.linalg.norm(initial_quaternion)
    if initial_pose_found == False:
            marker_pose.pose.orientation.x = initial_quaternion[0]
            marker_pose.pose.orientation.y = initial_quaternion[1]
            marker_pose.pose.orientation.z = initial_quaternion[2]
            marker_pose.pose.orientation.w = initial_quaternion[3]
            marker_pose.pose.position.x = msg.O_T_EE[12]
            marker_pose.pose.position.y = msg.O_T_EE[13]
            marker_pose.pose.position.z = msg.O_T_EE[14]
            init_euler= transform.euler_from_matrix(np.transpose(np.reshape(msg.O_T_EE,
                                    (4, 4))))
            initial_pose_found = True

    euler= transform.euler_from_matrix(np.transpose(np.reshape(msg.O_T_EE,
                                    (4, 4))))
    
    
    # print("initial x =" ,init_euler[0],"current x = ",euler[0])
    p =[msg.O_T_EE[12],msg.O_T_EE[13],msg.O_T_EE[14],euler[0],euler[1],euler[2]]
   
     
def pose_to_matrix(pose):
    x=pose.position.x
    y=pose.position.y
    z=pose.position.z
    qx=pose.orientation.x
    qy=pose.orientation.y
    qz=pose.orientation.z
    qw=pose.orientation.w
    """
    Converts a pose (x, y, z, qx, qy, qz, qw) to a 4x4 transformation matrix.
    """
    R = np.array([
        [1 - 2*qy**2 - 2*qz**2, 2*qx*qy - 2*qz*qw, 2*qx*qz + 2*qy*qw, x],
        [2*qx*qy + 2*qz*qw, 1 - 2*qx**2 - 2*qz**2, 2*qy*qz - 2*qx*qw, y],
        [2*qx*qz - 2*qy*qw, 2*qy*qz + 2*qx*qw, 1 - 2*qx**2 - 2*qy**2, z],
        [0, 0, 0, 1]
    ])
    R=np.transpose(np.reshape(R, (4, 4)))
    return R    
    
def publish_rotation_matrix(R):
    # create ROS node
        
    
    # create ROS publisher
        

    # create message with rotation matrix data
    msg = Float32MultiArray()
    msg.data = R.flatten().tolist()

    # publish message to topic
    pub.publish(msg)

   


        



if __name__ == "__main__":
    
    rospy.init_node("matrix_publisher")
    
    pose_sub = rospy.Subscriber("my_equilibrium_pose", PoseStamped, 
                                 pose_callback)
    gripper_sub = rospy.Subscriber("/franka_gripper/joint_states",
                                JointState, gripper_state_callback)
    state_sub= rospy.Subscriber("franka_state_controller/franka_states",
                                FrankaState, franka_state_callback)
    pub = rospy.Publisher('rotation_matrix', Float32MultiArray, queue_size=10)
    # pub = rospy.Publisher('rotation_matrix', Float32MultiArray, queue_size=10)
    listener1 = tf.TransformListener()
    
   
    tf_buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tf_buffer)
    # Get initial pose for the interactive marker
    while not initial_pose_found:
       rospy.sleep(1)
    #state_sub.unregister()
    # rospy.Rate(1)
    # rate = rospy.Rate(10)
    rospy.Timer(rospy.Duration(1),
                lambda msg: publish_rotation_matrix(pose_to_matrix(marker_pose.pose)))
    # while not rospy.is_shutdown():
        
        
    #     msg = Float32MultiArray()
    #     msg.data = T.flatten().tolist()

    #     # publish message to topic
    #     pub.publish(msg)
        
        # rate.sleep()
    rospy.spin()    

    
   
  

    