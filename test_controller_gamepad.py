#!/usr/bin/env python3

# Software License Agreement (BSD License)
#
# Copyright (c) 2013, SRI International
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met: 
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of SRI International nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Author: Acorn Pooley, Mike Lautman

## BEGIN_SUB_TUTORIAL imports
##
## To use the Python MoveIt interfaces, we will import the `moveit_commander`_ namespace.
## This namespace provides us with a `MoveGroupCommander`_ class, a `PlanningSceneInterface`_ class,
## and a `RobotCommander`_ class. More on these below. We also import `rospy`_ and some messages that we will use:
##

# Python 2/3 compatibility imports

from __future__ import print_function
from ast import While
from ctypes.wintypes import PCHAR
from email import header
import imp
from ossaudiodev import control_names
from pickle import TRUE
from socket import timeout
#from six.moves import input
from evdev import InputDevice, categorize, ecodes,KeyEvent



import sys
from turtle import position
from matplotlib.pyplot import tick_params
import pandas as pd 
import copy
import rospy
import actionlib
import sensor_msgs.msg
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import tty, sys, termios
import csv 
from franka_gripper.msg import GraspAction, GraspGoal,MoveAction,MoveGoal
from franka_msgs.msg import ErrorRecoveryActionGoal,ErrorRecoveryActionFeedback



filedescriptors = termios.tcgetattr(sys.stdin)
tty.setcbreak(sys.stdin)

try:
    from math import pi, tau, dist, fabs, cos
except:  # For Python 2 compatibility
    from math import pi, fabs, cos, sqrt

    tau = 2.0 * pi

    def dist(p, q):
        return sqrt(sum((p_i - q_i) ** 2.0 for p_i, q_i in zip(p, q)))


from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

## END_SUB_TUTORIAL
# opening the file in read mode
#my_file = open("/home/panda/ws_moveit/src/moveit_tutorials/doc/move_group_python_interface/scripts/joint_angles.txt", "r")
  
# reading the file
#data = my_file.read()
  
# replacing end of line('/n') with ' ' and
# splitting the text it further when '.' is seen.
#data_into_list = data.replace('\n', ' ').split(" ")
#str_list = [l.split('\n') for l in data.split(',')]
  
# printing the data
#print(str_list)
#my_file.close()
home=[0.010634786303106106, -0.7793822895183896, -0.010644634918499455, -2.4004822612227055, 0.027908380891381375, 1.6902482674546058, 0.7998376508951186]
pickup1=[-0.2679131060033514, 0.5198334925969442, -0.6354759461754247, -2.1510822642476994, 0.504498906923665, 2.554901361499585, -0.45090570525491397]
pickfinal=[-0.4247359809465052, 0.6665463957323442, -0.4483102312596579, -2.1069096967797534, 0.536215686730907, 2.6785662354893156, -0.43715002890593474]
placefinal=[0.5487171375406416, 0.9412344858018973, 0.6022579212439687, -1.657488863038272, -0.7617680575053261, 2.375601856496599, 2.3530502091860024,]
opened=[0.037,0.037]
closed=[0.0,0.0]
pc_r="/home/panda/ws_moveit/src/joint_angles.csv"
pc_w="/home/panda/ws_moveit/src/joint_angles1.csv"
laptop_r='/home/rehan/ws_moveit/src/joint_angles.csv'
laptop_w='/home/rehan/ws_moveit/src/joint_angles1.csv'
file_read=laptop_r
file_write=laptop_w

pick=[0.3330900495280648,-0.4473527946126987,0.09144908081004947,0.9160861559973409,-0.40053923792457446,0.007851307718949254,0.01711229499275589]
with open(file_read, 'w', encoding='UTF8') as f:
            writer= csv.writer(f)
            header=('joint_goal[0]','joint_goal[1]','joint_goal[2]','joint_goal[3]','joint_goal[4]','joint_goal[5]','joint_goal[6]','joint_goal[7]','joint_goal[8]')
            #header=['position.x','position.y','position.z','orientation.x','orientation.y','orientation.z','orientation.w','gripper_pose']
            #write the header
            writer.writerow(header)
            f.close()
with open(file_write, 'w', encoding='UTF8') as o:
            write= csv.writer(o)
            header=('joint_goal[0]','joint_goal[1]','joint_goal[2]','joint_goal[3]','joint_goal[4]','joint_goal[5]','joint_goal[6]','joint_goal[7]','joint_goal[8]')
            #header=['position.x','position.y','position.z','orientation.x','orientation.y','orientation.z','orientation.w','gripper_pose']
            #write the header
            write.writerow(header)
            o.close()           
def all_close(goal, actual, tolerance):
    """
    Convenience method for testing if the values in two lists are within a tolerance of each other.
    For Pose and PoseStamped inputs, the angle between the two quaternions is compared (the angle
    between the identical orientations q and -q is calculated correctly).
    @param: goal       A list of floats, a Pose or a PoseStamped
    @param: actual     A list of floats, a Pose or a PoseStamped
    @param: tolerance  A float
    @returns: bool
    """
    if type(goal) is list:
        for index in range(len(goal)):
            if abs(actual[index] - goal[index]) > tolerance:
                return False

    elif type(goal) is geometry_msgs.msg.PoseStamped:
        return all_close(goal.pose, actual.pose, tolerance)

    elif type(goal) is geometry_msgs.msg.Pose:
        x0, y0, z0, qx0, qy0, qz0, qw0 = pose_to_list(actual)
        x1, y1, z1, qx1, qy1, qz1, qw1 = pose_to_list(goal)
        # Euclidean distance
        d = dist((x1, y1, z1), (x0, y0, z0))
        # phi = angle between orientations
        cos_phi_half = fabs(qx0 * qx1 + qy0 * qy1 + qz0 * qz1 + qw0 * qw1)
        return d <= tolerance and cos_phi_half >= cos(tolerance / 2.0)

    return True


class MoveGroupPythonInterfaceTutorial(object):
    """MoveGroupPythonInterfaceTutorial"""

    def __init__(self):
        super(MoveGroupPythonInterfaceTutorial, self).__init__()

        ## BEGIN_SUB_TUTORIAL setup
        ##
        ## First initialize `moveit_commander`_ and a `rospy`_ node:
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("move_group_python_interface_tutorial", anonymous=True)
        
        #rospy.init_node('gripper_control')
            
        ## Instantiate a `RobotCommander`_ object. Provides information such as the robot's
        ## kinematic model and the robot's current joint states
        robot = moveit_commander.RobotCommander()

        ## Instantiate a `PlanningSc10
        # eneInterface`_ object.  This provides a remote interface
        ## for getting, setting, and updating the robot's internal understanding of the
        ## surrounding world:
        scene = moveit_commander.PlanningSceneInterface()
        
        ## Instantiate a `MoveGroupCommander`_ object.  This object is an interface
        ## to a planning group (group of joints).  In this tutorial the group is the primary
        ## arm joints in the Panda robot, so we set the group's name to "panda_arm".
        ## If you are using a different robot, change this value to the name of your robot
        ## arm planning group.
        ## This interface can be used to plan and execute motions:
        group_name = "panda_arm"
        move_group = moveit_commander.MoveGroupCommander(group_name)
        ## Create a `DisplayTrajectory`_ ROS publisher which is used to display
        ## trajectories in Rviz:
        display_trajectory_publisher = rospy.Publisher(
            "/move_group/display_planned_path",
            moveit_msgs.msg.DisplayTrajectory,
            queue_size=20,
        )

        ## END_SUB_TUTORIAL

        ## BEGIN_SUB_TUTORIAL basic_info
        ##
        ## Getting Basic Information
        ## ^^^^^^^^^^^^^^^^^^^^^^^^^
        # We can get the name of the reference frame for this robot:
        planning_frame = move_group.get_planning_frame()
        print("============ Planning frame: %s" % planning_frame)

        # We can also print the name of the end-effector link for this group:
        eef_link = move_group.get_end_effector_link()
        print("============ End effector link: %s" % eef_link)

        # We can get a list of all the groups in the robot:
        group_names = robot.get_group_names()
        print("============ Available Planning Groups:", robot.get_group_names())

        # Sometimes for debugging it is useful to print the entire state of the
        # robot:
        print("============ Printing robot state")
        print(robot.get_current_state())
        print("")
        ## END_SUB_TUTORIAL

        # Misc variables
        self.box_name = ""
        self.robot = robot
        self.scene = scene
        self.move_group = move_group
        self.display_trajectory_publisher = display_trajectory_publisher
        self.planning_frame = planning_frame
        self.eef_link = eef_link
        self.group_names = group_names
    def grasp_client(self,string):
	
        
        if string=='op':
            group_name = "hand"
            move_group = moveit_commander.MoveGroupCommander(group_name)
            joint_goal = move_group.get_current_joint_values()
            joint_goal[0] = opened[0]
            joint_goal[1] = opened[1]
            move_group.go(joint_goal, wait=True)
            move_group.stop()
            #client = actionlib.SimpleActionClient('franka_gripper/grasp',MoveAction)
            #print('waiting for the action server to start')
            #client.wait_for_server()
            #goal= MoveGoal()
            #print("goal ==",goal)
            #goal.width = 0.04
            #goal.speed = 0.1
            #client.send_goal(goal)
            #wait = client.wait_for_result()
            #if not wait:
                #rospy.logerr("Action server not available!")
                #rospy.signal_shutdown("Action server not available!")
            #else:
                #return client.get_result()
        elif string =='cl':
            client = actionlib.SimpleActionClient('franka_gripper/grasp',GraspAction)
            print('waiting for the action server to start')
            timeout=rospy.Duration(5)
            client.wait_for_server(timeout)
            #goal= MoveGoal()
        
            goal = GraspGoal()
            #print("goal ==",goal)
            #goal.width = 0.058
            #goal.speed = 0.03
            goal.force = 70
            goal.epsilon.inner = 0.05
            goal.epsilon.outer = 0.0510

            goal.speed = 0.1
            goal.width = 0.04
            #print("goal=",goal)
            client.send_goal(goal)
            wait = client.wait_for_result(timeout)
        
            if not wait:
                rospy.logerr("Action server not available!")
                rospy.signal_shutdown("Action server not available!")
            else:
                return client.get_result()
            

    def go_to_joint_state(self,array):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.

     
            
        
        group_name = "panda_arm"
        move_group = moveit_commander.MoveGroupCommander(group_name)
        
        #move_group = self.move_group
        
        #print("============ Printing robot start joint values")
        #print(move_group.get_current_joint_values())
        
	
        ## BEGIN_SUB_TUTORIAL plan_to_joint_state
        ##
        ## Planning to a Joint Goal
        ## ^^^^^^^^^^^^^^^^^^^^^^^^
        
        # We get the joint values from the group and change some of the values:
        joint_goal = move_group.get_current_joint_values()
        joint_goal[0] = array[0]
        joint_goal[1] = array[1]
        joint_goal[2] = array[2]
        joint_goal[3] = array[3]
        joint_goal[4] = array[4]
        joint_goal[5] = array[5] 
        joint_goal[6] = array[6]
        #if string =='op' :
        # joint_goal[7] = opened[0]
         #joint_goal[8] = opened[1]
           
        #elif string == 'cl' :
         #   joint_goal[7] = closed[0]
         #   joint_goal[8] = closed[1]
        #else :
         #   joint_goal[7] = array[7] 
         #   joint_goal[8] = array[8]
        

        # The go command can be called with joint values, poses, or without any
        # parameters if you have already set the pose or joint target for the group
        move_group.go(joint_goal, wait=True)

        # Calling ``stop()`` ensures that there is no residual movement
        move_group.stop()

        ## END_SUB_TUTORIAL
        
        # For testing:
        current_joints = move_group.get_current_joint_values()
        joints=move_group.get_current_joint_values()
        return joints,all_close(joint_goal, current_joints, 0.01)

    def go_to_pose_goal(self,goal):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        #print("current pose ",self.move_group.get_current_pose().pose)
        move_group = self.move_group

        ## BEGIN_SUB_TUTORIAL plan_to_pose
        ##
        ## Planning to a Pose Goal
        ## ^^^^^^^^^^^^^^^^^^^^^^^
        ## We can plan a motion for this group to a desired pose for the
        ## end-effector:
        #print("geometry position = ", geometry_msgs.msg.Pose())
        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.position.x = goal[0]
        pose_goal.position.y =  goal[1]
        pose_goal.position.z =  goal[2]
        pose_goal.orientation.x =  goal[3]
        pose_goal.orientation.y =   goal[4]
        pose_goal.orientation.z =  goal[5]
        pose_goal.orientation.w =   goal[6]
        #print("pose_goal= ", pose_goal)
        
        

        move_group.set_pose_target(pose_goal)
        

        ## Now, we call the planner to compute the plan and execute it.
        #plan = move_group.go(wait=True)
        # Calling `stop()` ensures that there is no residual movement
        move_group.stop()
        # It is always good to clear your targets after planning with poses.
        # Note: there is no equivalent function for clear_joint_value_targets()
        move_group.clear_pose_targets()
        #print("final pose ",self.move_group.get_current_pose().pose)

        ## END_SUB_TUTORIAL

        # For testing:
        # Note that since this section of code will not be included in the tutorials
        # we use the class variable rather than the copied state variable
        current_pose = self.move_group.get_current_pose().pose
        return all_close(pose_goal, current_pose, 0.001)

    def read_file(self,string):
            if string=='j':
                i=0
                group_name = "hand"
                move_group = moveit_commander.MoveGroupCommander(group_name)
                joint_goal = move_group.get_current_joint_values()
                #joint_goal[0] = opened[0]
                #joint_goal[1] = opened[1]
                print("initial hand pose = ",joint_goal[0])
                if joint_goal[0]<=0.03:
                    p='closed'
                else:
                    p='opened'    
                d1= pd.read_csv(file_read)
                print(d1)
                print ("number of rows in file are ==" ,len(d1))
                #print("if the data looks good press X to execute ")
                
                while  i < len(d1):
                    array1=d1.iloc[int(i)]
                    tutorial = MoveGroupPythonInterfaceTutorial()
                    tutorial.go_to_joint_state(array1)
                    if array1[7]=='open'and p=='closed':
                        tutorial.grasp_client('op')
                        p='opened'
                    elif array1[7]=='close' and p=='opened':
                        tutorial.grasp_client('cl')
                        p='closed'
        
                    print(array1)
                    i+=1
                    
            elif string=='':    
                a=input("press '1' to use saved joint angles or press any key to replicate all movements")
                if a=='1':
                    i=0
                    group_name = "hand"
                    move_group = moveit_commander.MoveGroupCommander(group_name)
                    joint_goal = move_group.get_current_joint_values()
                    #joint_goal[0] = opened[0]
                    #joint_goal[1] = opened[1]
                    print("initial hand pose = ",joint_goal[0])
                    if joint_goal[0]<=0.03:
                        p='closed'
                    else:
                        p='opened'    
                    d= pd.read_csv(file_write)
                    print(d)
                    print ("number of rows in file are ==" ,len(d))
                    n=input("if the data looks good press Y to execute ")
                    if n=='y' or n=='Y':
                        tutorial = MoveGroupPythonInterfaceTutorial()
                        
                        while  i < len(d):
                            array1=d.iloc[int(i)]
                            
                            tutorial.go_to_joint_state(array1)
                            if array1[7]=='open'and p=='closed':
                                tutorial.grasp_client('op')
                                p='opened'
                            elif array1[7]=='close' and p=='opened':
                                tutorial.grasp_client('cl')
                                p='closed'
                
                            #print(array1)
                            i+=1
                else:
                    i=0
                    group_name = "hand"
                    move_group = moveit_commander.MoveGroupCommander(group_name)
                    joint_goal = move_group.get_current_joint_values()
                    #joint_goal[0] = opened[0]
                    #joint_goal[1] = opened[1]
                    print("initial hand pose = ",joint_goal[0])
                    if joint_goal[0]<=0.03:
                        p='closed'
                    else:
                        p='opened'    
                    d1= pd.read_csv(file_read)
                    print(d1)
                    print ("number of rows in file are ==" ,len(d1))
                    n=input("if the data looks good press Y to execute ")
                    if n=='y' or n=='Y':
                        tutorial = MoveGroupPythonInterfaceTutorial()
                        
                        while  i < len(d1):
                            array1=d1.iloc[int(i)]
                            
                            tutorial.go_to_joint_state(array1)
                            if array1[7]=='open'and p=='closed':
                                tutorial.grasp_client('op')
                                p='opened'
                            elif array1[7]=='close' and p=='opened':
                                tutorial.grasp_client('cl')
                                p='closed'
                
                            print(array1)
                            i+=1

            
                
    def write_file(self,string=''):
        if string=='j':
            group_name = "panda_arm"
            move_group = moveit_commander.MoveGroupCommander(group_name)
            joint_goal = move_group.get_current_joint_values()
            

            move_group = moveit_commander.MoveGroupCommander("hand")
            gripper_pose = move_group.get_current_joint_values()
            if gripper_pose[0]<=0.02:
                joint_goal.append("close")
            else:
                joint_goal.append("open")


            print("pose =",joint_goal)
            #data= pd.read_csv('/home/panda/ws_moveit/src/joint_angles.csv')
            
            with open(file_write, 'a+', encoding='UTF8') as f:
                writer = csv.writer(f)
                # write the data
                writer.writerow(joint_goal)
                print("file should be saved") 
                f.close
                
        else:
            group_name = "panda_arm"
            move_group = moveit_commander.MoveGroupCommander(group_name)
            joint_goal = move_group.get_current_joint_values()
            

            move_group = moveit_commander.MoveGroupCommander("hand")
            gripper_pose = move_group.get_current_joint_values()
            if gripper_pose[0]<=0.02:
                joint_goal.append("close")
            else:
                joint_goal.append("open")


            print("pose =",joint_goal)
            #data= pd.read_csv('/home/panda/ws_moveit/src/joint_angles.csv')
            
            with open(file_read, 'a+', encoding='UTF8') as f:
                writer = csv.writer(f)
                # write the data
                writer.writerow(joint_goal)
                print("file should be saved") 
                f.close


    def movepose(self,goal):
        
        group_name = "panda_arm"
        #move_group = self.move_group
        move_group = moveit_commander.MoveGroupCommander(group_name)
        wpose = move_group.get_current_pose().pose
        waypoints = []
        wpose.position.x = goal[0]
        wpose.position.y =  goal[1]
        wpose.position.z =  goal[2]
        wpose.orientation.x =  goal[3]
        wpose.orientation.y =   goal[4]
        wpose.orientation.z =  goal[5]
        wpose.orientation.w =   goal[6]
        
        waypoints.append(copy.deepcopy(wpose))
        (plan, fraction) = move_group.compute_cartesian_path(waypoints, 0.01, 0.0 )  
        robot = self.robot
        display_trajectory_publisher = self.display_trajectory_publisher
        display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        display_trajectory.trajectory_start = robot.get_current_state()
        display_trajectory.trajectory.append(plan)
        display_trajectory_publisher.publish(display_trajectory)
        move_group.execute(plan, wait=True)
    def talker(self):
        

        pub = rospy.Publisher('/franka_control/error_recovery/goal',ErrorRecoveryActionGoal, queue_size=10)
        str = ErrorRecoveryActionGoal()
        str.goal={}
        pub.publish(str)
        #rospy.wait_for_message('/franka_control/error_recovery/feedback',ErrorRecoveryActionFeedback)
            

    def gripper_control(self,string):
        
        group_name = "panda_arm"
        move_group = moveit_commander.MoveGroupCommander(group_name)
        #print("============ Printing robot start joint values")
        #print(move_group.get_current_joint_values())
        
        
	
        joint_goal = move_group.get_current_joint_values()
        #print (joint_goal)
        if string =='op' :
         group_name = "hand"
         move_group = moveit_commander.MoveGroupCommander(group_name)
         joint_goal = move_group.get_current_joint_values()
         joint_goal[0] = opened[0]
         joint_goal[1] = opened[1]
           
        elif string == 'cl' :
         group_name = "hand"
         move_group = moveit_commander.MoveGroupCommander(group_name)
         joint_goal = move_group.get_current_joint_values()
         joint_goal[0] = closed[0]
         joint_goal[1] = closed[1]
        elif string == 'add' :
         joint_goal[6] = joint_goal[6]+0.5
         move_group.go(joint_goal, wait=True)
         
        elif string == 'sub' :
         joint_goal[6] = joint_goal[6]-0.5
         move_group.go(joint_goal, wait=True)
        elif string == 'add1' :
         joint_goal[5] = joint_goal[5]+0.1
         move_group.go(joint_goal, wait=True)
         
        elif string == 'sub1' :
         joint_goal[5] = joint_goal[5]-0.1
         move_group.go(joint_goal, wait=True) 
        elif string == 'add2' :
         joint_goal[4] = joint_goal[4]+0.1
         move_group.go(joint_goal, wait=True)
         
        elif string == 'sub2' :
         joint_goal[4] = joint_goal[4]-0.1
         move_group.go(joint_goal, wait=True)  
        elif string == 'add3' :
         joint_goal[3] = joint_goal[3]+0.1
         move_group.go(joint_goal, wait=True)
         
        elif string == 'sub3' :
         joint_goal[3] = joint_goal[3]-0.1
         move_group.go(joint_goal, wait=True)  
        
          
                
        move_group.go(joint_goal, wait=True)


        
        #move_group.stop()

        
        current_joints = move_group.get_current_joint_values()
        joints=move_group.get_current_joint_values()
        return joints,all_close(joint_goal, current_joints, 0.01)

    def check_hand_pose(self):
        group_name = "hand"
        #move_group = self.move_group
        move_group = moveit_commander.MoveGroupCommander(group_name)
        print(move_group)
        
    def moveup(self,scale):
        group_name = "panda_arm"
        #move_group = self.move_group
        move_group = moveit_commander.MoveGroupCommander(group_name)
        wpose = move_group.get_current_pose().pose
        waypoints = []
        wpose.position.z += scale * 0.01 # moveup
        waypoints.append(copy.deepcopy(wpose))
        (plan, fraction) = move_group.compute_cartesian_path(waypoints, 0.01, 0.0 )  
        robot = self.robot
        display_trajectory_publisher = self.display_trajectory_publisher
        display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        display_trajectory.trajectory_start = robot.get_current_state()
        display_trajectory.trajectory.append(plan)
        display_trajectory_publisher.publish(display_trajectory)
        move_group.execute(plan, wait=True)

    def movedown(self,scale):
        group_name = "panda_arm"
        #move_group = self.move_group
        move_group = moveit_commander.MoveGroupCommander(group_name)
        wpose = move_group.get_current_pose().pose
        waypoints = []
        wpose.position.z -= scale * 0.01 # movedown
        waypoints.append(copy.deepcopy(wpose))
        (plan, fraction) = move_group.compute_cartesian_path(waypoints, 0.01, 0.0 )  
        robot = self.robot
        display_trajectory_publisher = self.display_trajectory_publisher
        display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        display_trajectory.trajectory_start = robot.get_current_state()
        display_trajectory.trajectory.append(plan)
        display_trajectory_publisher.publish(display_trajectory)
        move_group.execute(plan, wait=True)

    def moveright(self,scale):
        group_name = "panda_arm"
        #move_group = self.move_group
        move_group = moveit_commander.MoveGroupCommander(group_name)
        wpose = move_group.get_current_pose().pose
        waypoints = []
        wpose.position.y += scale * 0.01 # moveright
        waypoints.append(copy.deepcopy(wpose))
        (plan, fraction) = move_group.compute_cartesian_path(waypoints, 0.01, 0.0 )  
        robot = self.robot
        display_trajectory_publisher = self.display_trajectory_publisher
        display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        display_trajectory.trajectory_start = robot.get_current_state()
        display_trajectory.trajectory.append(plan)
        display_trajectory_publisher.publish(display_trajectory)
        move_group.execute(plan, wait=True)
    def data(self,scale,string):
        while(True):
            if string == 'a':
                group_name = "panda_arm"
                #move_group = self.move_group
                move_group = moveit_commander.MoveGroupCommander(group_name)
                wpose = move_group.get_current_pose().pose
                waypoints = []
                wpose.position.y += scale * 0.01 # moveright
                waypoints.append(copy.deepcopy(wpose))
                (plan, fraction) = move_group.compute_cartesian_path(waypoints, 0.01, 0.0 )  
                
                move_group.execute(plan, wait=True)
                rospy.sleep(5)
                move_group = moveit_commander.MoveGroupCommander(group_name)
                wpose1 = move_group.get_current_pose().pose
                waypoints1 = []
                wpose1.position.y -= scale * 0.01 # moveright
                waypoints1.append(copy.deepcopy(wpose1))
                (plan, fraction) = move_group.compute_cartesian_path(waypoints1, 0.01, 0.0 ) 
                move_group.execute(plan, wait=True)
                rospy.sleep(5)
            if string == 'b':
                group_name = "panda_arm"
                move_group = self.move_group
                move_group = moveit_commander.MoveGroupCommander(group_name)
                wpose2 = move_group.get_current_pose().pose
                print("y_orientation",wpose2)
                waypoints2 = []
                wpose2.position.x=0.31849339733685345
                wpose2.position.y=0.001712022045827885
                wpose2.position.z=0.5782304066820235
                wpose2.orientation.x = -0.920268721473281 
                wpose2.orientation.y = -0.3893779208513452
                wpose2.orientation.z = -0.025048974775792317
                wpose2.orientation.w = 0.029374545002400186
                waypoints2.append(copy.deepcopy(wpose2))
                (plan, fraction) = move_group.compute_cartesian_path(waypoints2, 0.01, 0.0 )  
             
                move_group.execute(plan, wait=True)
                rospy.sleep(5)
                move_group = moveit_commander.MoveGroupCommander(group_name)
                wpose3 = move_group.get_current_pose().pose
                print("y_orientation",wpose3.orientation.y)
                waypoints3 = []
                wpose3.position.x=0.30664646468284157
                wpose3.position.y=0.003363342522298818
                wpose3.position.z=0.5623913028029793
                wpose3.orientation.x = -0.9390901126737115 
                wpose3.orientation.y = -0.3429201927839085
                wpose3.orientation.z = 0.019292033519401702
                wpose3.orientation.w = 0.011971595641731645
                waypoints3.append(copy.deepcopy(wpose3))
                (plan, fraction) = move_group.compute_cartesian_path(waypoints3, 0.01, 0.0 ) 
                move_group.execute(plan, wait=True)
                rospy.sleep(5)    
    def moveleft(self,scale):
        group_name = "panda_arm"
        #move_group = self.move_group
        move_group = moveit_commander.MoveGroupCommander(group_name)
        wpose = move_group.get_current_pose().pose
        waypoints = []
        wpose.position.y -= scale * 0.01 # moveleft
        waypoints.append(copy.deepcopy(wpose))
        (plan, fraction) = move_group.compute_cartesian_path(waypoints, 0.01, 0.0 )  
        robot = self.robot
        display_trajectory_publisher = self.display_trajectory_publisher
        display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        display_trajectory.trajectory_start = robot.get_current_state()
        display_trajectory.trajectory.append(plan)
        display_trajectory_publisher.publish(display_trajectory)
        move_group.execute(plan, wait=True)

    def moveforward(self,scale):
        group_name = "panda_arm"
        #move_group = self.move_group
        move_group = moveit_commander.MoveGroupCommander(group_name)
        wpose = move_group.get_current_pose().pose
        
        waypoints = []
        wpose.position.x += scale * 0.01 # moveforward
        waypoints.append(copy.deepcopy(wpose))
        (plan, fraction) = move_group.compute_cartesian_path(waypoints, 0.01, 0.0 )  
        robot = self.robot
        display_trajectory_publisher = self.display_trajectory_publisher
        display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        display_trajectory.trajectory_start = robot.get_current_state()
        display_trajectory.trajectory.append(plan)
        display_trajectory_publisher.publish(display_trajectory)
        move_group.execute(plan, wait=True)

    def movebackward(self,scale):
        group_name = "panda_arm"
        #move_group = self.move_group
        move_group = moveit_commander.MoveGroupCommander(group_name)
        wpose = move_group.get_current_pose().pose
        waypoints = []
        wpose.position.x -= scale * 0.01 # move back(x)
        waypoints.append(copy.deepcopy(wpose))
        (plan, fraction) = move_group.compute_cartesian_path(waypoints, 0.01, 0.0 )  
        robot = self.robot
        display_trajectory_publisher = self.display_trajectory_publisher
        display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        display_trajectory.trajectory_start = robot.get_current_state()
        display_trajectory.trajectory.append(plan)
        display_trajectory_publisher.publish(display_trajectory)
        move_group.execute(plan, wait=True)

    def plan_cartesian_path(self, scale=1):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        move_group = self.move_group

        ## BEGIN_SUB_TUTORIAL plan_cartesian_path
        ##
        ## Cartesian Paths
        ## ^^^^^^^^^^^^^^^
        ## You can plan a Cartesian path directly by specifying a list of waypoints
        ## for the end-effector to go through. If executing  interactively in a
        ## Python shell, set scale = 1.0.
        ##
        waypoints = []

        wpose = move_group.get_current_pose().pose
        wpose.position.z -= scale * 0.06  # First move up (z)
        wpose.position.y += scale * 0.06  # and sideways (y)
        waypoints.append(copy.deepcopy(wpose))

        wpose.position.x += scale * 0.06  # Second move forward/backwards in (x)
        waypoints.append(copy.deepcopy(wpose))

        wpose.position.y -= scale * 0.06  # Third move sideways (y)
        waypoints.append(copy.deepcopy(wpose))

        # We want the Cartesian path to be interpolated at a resolution of 1 cm
        # which is why we will specify 0.01 as the eef_step in Cartesian
        # translation.  We will disable the jump threshold by setting it to 0.0,
        # ignoring the check for infeasible jumps in joint space, which is sufficient
        # for this tutorial.
        print("way points =",waypoints)
        (plan, fraction) = move_group.compute_cartesian_path(
            waypoints, 0.01, 0.0  # waypoints to follow  # eef_step
        )  # jump_threshold

        # Note: We are just planning, not asking move_group to actually move the robot yet:
        return plan, fraction

        ## END_SUB_TUTORIAL

    def display_trajectory(self, plan):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        robot = self.robot
        display_trajectory_publisher = self.display_trajectory_publisher

        ## BEGIN_SUB_TUTORIAL display_trajectory
        ##
        ## Displaying a Trajectory
        ## ^^^^^^^^^^^^^^^^^^^^^^^
        ## You can ask RViz to visualize a plan (aka trajectory) for you. But the
        ## group.plan() method does this automatically so this is not that useful
        ## here (it just displays the same trajectory again):
        ##
        ## A `DisplayTrajectory`_ msg has two primary fields, trajectory_start and trajectory.
        ## We populate the trajectory_start with our current robot state to copy over
        ## any AttachedCollisionObjects and add our plan to the trajectory.
        display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        display_trajectory.trajectory_start = robot.get_current_state()
        display_trajectory.trajectory.append(plan)
        # Publish
        display_trajectory_publisher.publish(display_trajectory)

        ## END_SUB_TUTORIAL

    def execute_plan(self, plan):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        move_group = self.move_group
         
        ## BEGIN_SUB_TUTORIAL execute_plan
        ##
        ## Executing a Plan
        ## ^^^^^^^^^^^^^^^^
        ## Use execute if you would like the robot to follow
        ## the plan that has already been computed:
        move_group.execute(plan, wait=True)
        

        ## **Note:** The robot's current joint state must be within some tolerance of the
        ## first waypoint in the `RobotTrajectory`_ or ``execute()`` will fail
        ## END_SUB_TUTORIAL

    def wait_for_state_update(
        self, box_is_known=False, box_is_attached=False, timeout=4
    ):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        box_name = self.box_name
        scene = self.scene

        ## BEGIN_SUB_TUTORIAL wait_for_scene_update
        ##
        ## Ensuring Collision Updates Are Received
        ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
        ## If the Python node dies before publishing a collision object update message, the message
        ## could get lost and the box will not appear. To ensure that the updates are
        ## made, we wait until we see the changes reflected in the
        ## ``get_attached_objects()`` and ``get_known_object_names()`` lists.
        ## For the purpose of this tutorial, we call this function after adding,
        ## removing, attaching or detaching an object in the planning scene. We then wait
        ## until the updates have been made or ``timeout`` seconds have passed
        start = rospy.get_time()
        seconds = rospy.get_time()
        while (seconds - start < timeout) and not rospy.is_shutdown():
            # Test if the box is in attached objects
            attached_objects = scene.get_attached_objects([box_name])
            is_attached = len(attached_objects.keys()) > 0

            # Test if the box is in the scene.
            # Note that attaching the box will remove it from known_objects
            is_known = box_name in scene.get_known_object_names()

            # Test if we are in the expected state
            if (box_is_attached == is_attached) and (box_is_known == is_known):
                return True

            # Sleep so that we give other threads time on the processor
            rospy.sleep(0.1)
            seconds = rospy.get_time()

        # If we exited the while loop without returning then we timed out
        return False
        ## END_SUB_TUTORIAL

    def add_box(self, timeout=4):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        box_name = self.box_name
        scene = self.scene

        ## BEGIN_SUB_TUTORIAL add_box
        ##
        ## Adding Objects to the Planning Scene
        ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
        ## First, we will create a box in the planning scene between the fingers:
        box_pose = geometry_msgs.msg.PoseStamped()
        box_pose.header.frame_id = "panda_hand"
        box_pose.pose.orientation.w = 1.0
        box_pose.pose.position.z = 0.11  # above the panda_hand frame
        box_name = "box"
        scene.add_box(box_name, box_pose, size=(0.12, 0.06, 0.02))

        ## END_SUB_TUTORIAL
        # Copy local variables back to class variables. In practice, you should use the class
        # variables directly unless you have a good reason not to.
        self.box_name = box_name
        return self.wait_for_state_update(box_is_known=True, timeout=timeout)

    def attach_box(self, timeout=4):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        box_name = self.box_name
        robot = self.robot
        scene = self.scene
        eef_link = self.eef_link
        group_names = self.group_names

        ## BEGIN_SUB_TUTORIAL attach_object
        ##
        ## Attaching Objects to the Robot
        ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
        ## Next, we will attach the box to the Panda wrist. Manipulating objects requires the
        ## robot be able to touch them without the planning scene reporting the contact as a
        ## collision. By adding link names to the ``touch_links`` array, we are telling the
        ## planning scene to ignore collisions between those links and the box. For the Panda
        ## robot, we set ``grasping_group = 'hand'``. If you are using a different robot,
        ## you should change this value to the name of your end effector group name.
        grasping_group = "hand"
        touch_links = robot.get_link_names(group=grasping_group)
        scene.attach_box(eef_link, box_name, touch_links=touch_links)
        ## END_SUB_TUTORIAL

        # We wait for the planning scene to update.
        return self.wait_for_state_update(
            box_is_attached=True, box_is_known=False, timeout=timeout
        )

    def detach_box(self, timeout=4):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        box_name = self.box_name
        scene = self.scene
        eef_link = self.eef_link

        ## BEGIN_SUB_TUTORIAL detach_object
        ##
        ## Detaching Objects from the Robot
        ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
        ## We can also detach and remove the object from the planning scene:
        scene.remove_attached_object(eef_link, name=box_name)
        ## END_SUB_TUTORIAL

        # We wait for the planning scene to update.
        return self.wait_for_state_update(
            box_is_known=True, box_is_attached=False, timeout=timeout
        )

    def remove_box(self, timeout=4):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        box_name = self.box_name
        scene = self.scene

        ## BEGIN_SUB_TUTORIAL remove_object
        ##
        ## Removing Objects from the Planning Scene
        ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
        ## We can remove the box from the world.
        scene.remove_world_object(box_name)

        ## **Note:** The object must be detached before we can remove it from the world
        ## END_SUB_TUTORIAL

        # We wait for the planning scene to update.
        return self.wait_for_state_update(
            box_is_attached=False, box_is_known=False, timeout=timeout
        )


def main():
    #try:
    print("")
    print("----------------------------------------------------------")
    print("Welcome to the MoveIt MoveGroup Python Interface Tutorial")
    print("----------------------------------------------------------")
    print("Press Ctrl-D to exit at any time")
    print("")
    #input(
     #   "============ Press `Enter` to begin the tutorial by setting up the 	#			moveit_commander ..."
  #  )
    tutorial = MoveGroupPythonInterfaceTutorial()
    i=0 
    s=0
    s=(input("enter the step length between 0.1 t0 20 cm"))
    if s.isnumeric() is True:
        s=float(s)
        if s <= 20 and s>=0.1:
             speed=s
        else:
             speed=1 
             print("new step length = ", speed) 
    else: 
                speed=1
                print("wrong input new step length = ", speed)
    print("new step length = ", speed)
    print("value of s=",speed)
    i=(input("press 1 for keyboard control \n ""press 2 for joystick control \n"))   
    #speed=float(s)
    if i=='1':
        print("keyboard control started\n")
        while True:
            print("enter the direction with 'w','a','s','d'\n"  
            "up with 'q' down with 'e' \n"
            "use 'l' to change step length \n"  
            "use 'f' to open gripper \n"
            "use 'c' to close gripper\n" 
            "use 'h' to go to home pose\n"
            "use 'k' to execute poses from file\n"
            "use 'j' to go to save current pose\n"
            "use 'r' exit tutorial \n")
        
            x=sys.stdin.read(1)[0]
            event1=x
            print("You pressed", x)
            #tutorial.go_to_joint_state(home,'op')
            #print("currrent  joint values ",joints)
            # if up arrow key is pressed
            
            if event1 == 'A' or event1 == 'w':
                tutorial.moveforward(speed)
                tutorial.write_file()
                print("forward")

            # if down arrow key is pressed
            elif event1 == 'B' or event1 == 's':
                tutorial.movebackward(speed)
                tutorial.write_file()
                print("backward")
            elif event1 == 'k' or event1 == 'K':
                tutorial.read_file('')
                
            
            elif event1 == 'j' or event1 == 'J':
                #a=input("enter the pose number")
                tutorial.write_file('j')
                

            # if left arrow key is pressed
            elif event1 == 'C' or event1 == 'd':
                tutorial.moveleft(speed)
                tutorial.write_file()
                print("left")

            # if right arrow key is pressed
            elif event1 == 'D' or event1== 'a':
                tutorial.moveright(speed)
                tutorial.write_file()
                print("right")
            elif event1 == 'q' or event1 == '+':
                tutorial.moveup(speed)
                tutorial.write_file()
                print("up")    
            elif event1 == 'e' or event1 == '-':
                tutorial.movedown(speed)
                tutorial.write_file()
                print("down")
            elif event1 == 'p' or event1 == 'P':
                print ("left rotation ")
                #tutorial.rotate_left (1)
                tutorial.gripper_control('add')
                tutorial.write_file()
            elif event1 == 'g' or event1 == 'G':
                print ("data collection activated")
                
                while True:
                
                    tutorial.gripper_control('add')
                    rospy.sleep(5)
                    tutorial.gripper_control('sub')
                    rospy.sleep(5)
                     
                            
                
                               
                
            elif event1 == 'n' or event1 == 'N':
                print ("data collection activated rotation")
                tutorial.data(speed,'a')
            
                      
            elif event1 == 'o'or event1 == 'O':
                print ("right rotation ")
                #tutorial.check_hand_pose()
                tutorial.gripper_control('sub')
                tutorial.write_file()
            elif event1 == 'l' or event1 == 'l':
                s=(input("enter the step length between 0.1 t0 20 cm  \n"))
                if s.isnumeric() is True:
                    s=float(s)
                    if s <= 20 and s>=0.1:
                            speed=s
                    else:
                            speed=1 
                            print("new step length = ", speed) 
                else: 
                    speed=1
                    print("wrong input new step length = ", speed)  
            
            elif event1 == 'f' or event1 == 'F':
                tutorial.grasp_client('op')
                tutorial.write_file()
                #tutorial.gripper_control('op')
                #tutorial.check_hand_pose()
            
            elif event1 == 'm' or event1 == 'M':
                tutorial.talker()
                
            
            elif event1 == 'c' or event1 == 'C':
                tutorial.grasp_client('cl')
                tutorial.write_file()
                #tutorial.check_hand_pose()
            elif event1 == 'h' or event1 == 'H':
                tutorial.go_to_joint_state(home)
                tutorial.write_file()
                print("Home")
            elif event1 == 'r' or event1 == 'R':
                i=1
                print("exiting")
                False
                break
            else:
                print("stop")
    elif i == '2':
        gamepad = InputDevice('/dev/input/event17')
        print("joystick control started\n")
        for event in gamepad.read_loop():
            print("enter the direction with left joystck\n"  
            "move up with 'RB' \n"
            "move down with 'LB' \n"
            "use 'LT' to decrease step length \n"
            "use 'RT' to decrease step length \n"   
            "use 'A' to open gripper \n"
            "use 'B' to close gripper\n" 
            "use 'Y' to go to home pose\n"
            "use 'X' to execute poses from file\n"
            "use 'Back' to exit tutorial \n"
            "step length = ", speed ,"\n")
            if event.type == ecodes.EV_KEY:
                keyevent = categorize(event)
                #print (categorize(event))
                if keyevent.keystate == KeyEvent.key_down:
                    if keyevent.keycode == ['BTN_A', 'BTN_GAMEPAD', 'BTN_SOUTH']:
                        print ("A")
                        tutorial.grasp_client('op')
                        tutorial.write_file()
                    elif keyevent.keycode == ['BTN_WEST', 'BTN_Y']:
                        print ("Y")
                        tutorial.go_to_joint_state(home)
                        tutorial.write_file()
                        print("Home")
                    elif keyevent.keycode == ['BTN_B', 'BTN_EAST']:
                        print ("B")
                        tutorial.grasp_client('cl')
                        tutorial.write_file()
                    elif keyevent.keycode == ['BTN_NORTH', 'BTN_X']:
                        print ("X")
                        tutorial.read_file('j')

                    elif keyevent.keycode == 'BTN_TR':
                        print ("up")
                        tutorial.moveup(speed)
                        tutorial.write_file()
                    elif keyevent.keycode == 'BTN_TL':
                        print ("down") 
                        tutorial.movedown(speed)
                        tutorial.write_file() 
                    elif keyevent.keycode == 'BTN_START':
                        print ("error recovery started please wait 3 sec") 
                        tutorial.talker()
                            
                    elif keyevent.keycode == 'BTN_SELECT':
                        print("stoping")
                        break
                                  
            elif event.type == ecodes.EV_ABS:
                absevent = categorize(event)
                if ecodes.bytype[absevent.event.type][absevent.event.code] == 'ABS_Y':
                    print ('Y')
                    if absevent.event.value > 32766:
                        print ('reverse')
                        tutorial.movebackward(speed)
                        tutorial.write_file()
                        #print (absevent.event.value)
                    elif absevent.event.value < -32766:
                        print ('forward')
                        tutorial.moveforward(speed)
                        tutorial.write_file()
                        #print (absevent.event.value)
                if ecodes.bytype[absevent.event.type][absevent.event.code] == 'ABS_X':
                    
                    if absevent.event.value > 32766 :
                        print ('right')
                        tutorial.moveleft(speed)
                        tutorial.write_file()
                        #print (absevent.event.value)
                    elif absevent.event.value < -32766:
                        print ('left')
                        tutorial.moveright(speed)
                        tutorial.write_file()
                        #print (absevent.event.value)
                if ecodes.bytype[absevent.event.type][absevent.event.code] == 'ABS_RX':
                    
                    if absevent.event.value > 32766 :
                        print ('rotate right')
                        tutorial.gripper_control('add')
                        tutorial.write_file()
                        #print (absevent.event.value)
                    elif absevent.event.value < -32766:
                        print ('rotate left')
                        tutorial.gripper_control('sub')
                        tutorial.write_file()
                        #print (absevent.event.value)  
                if ecodes.bytype[absevent.event.type][absevent.event.code] == 'ABS_RY':
                    
                    if absevent.event.value >= 32767 :
                        print ('rotate up')
                        tutorial.gripper_control('sub1')
                        tutorial.write_file()
                        #print (absevent.event.value)
                    elif absevent.event.value <= -32768:
                        print ('rotate down')
                        tutorial.gripper_control('add1')
                        tutorial.write_file()
                        #print (absevent.event.value)      
                if ecodes.bytype[absevent.event.type][absevent.event.code] == 'ABS_HAT0X':
                    
                    if absevent.event.value > 0 :
                        print ('rotate up')
                        tutorial.gripper_control('sub2')
                        tutorial.write_file()
                        #print (absevent.event.value)
                    elif absevent.event.value < 0:
                        print ('rotate down')
                        tutorial.gripper_control('add2')
                        tutorial.write_file()
                        #print (absevent.event.value)  
                if ecodes.bytype[absevent.event.type][absevent.event.code] == 'ABS_HAT0Y':
                    
                    if absevent.event.value > 0 :
                        print ('rotate up')
                        tutorial.gripper_control('sub3')
                        tutorial.write_file()
                        #print (absevent.event.value)
                    elif absevent.event.value < 0:
                        print ('rotate down')
                        tutorial.gripper_control('add3')
                        tutorial.write_file()
                        #print (absevent.event.value)              
                if ecodes.bytype[absevent.event.type][absevent.event.code] == 'ABS_RZ':
                    
                    if absevent.event.value > 254 :
                        speed+=0.5
                        print("step length = ", speed)
                        #print (absevent.event.value)
                if ecodes.bytype[absevent.event.type][absevent.event.code] == 'ABS_Z':
                    
                    if absevent.event.value > 254 :
                        speed-=0.5
                        print("step length = ", speed)
                        #print (absevent.event.value)           
                # if ecodes.bytype[absevent.event.type][absevent.event.code] == 'ABS_RY':
                #     print("RY")
                # if ecodes.bytype[absevent.event.type][absevent.event.code] == 'ABS_HAT0X':
                #     print("ABS_HAT0X")
                # if ecodes.bytype[absevent.event.type][absevent.event.code] == 'ABS_HAT0Y':
                #     print("ABS_HAT0Y")
        #input("============ Press `Enter` to get to pickup pose ...")
        #tutorial.go_to_joint_state(pickup1,'op')
        #tutorial.go_to_pose_goal(pick)

        #input("============ Press `Enter` to pick up object  ...")
        #tutorial.go_to_joint_state(pickfinal,'op')
        #tutorial.add_box()
        #tutorial.attach_box()
        
        #tutorial.go_to_joint_state(pickfinal,'cl')
        #cartesian_plan, fraction = tutorial.plan_cartesian_path()

        #input(
        #   "============ Press `Enter` to move to home  )  ..."
        #)
        #tutorial.go_to_joint_state(home,'cl')
        #tutorial.display_trajectory(cartesian_plan)

        #input("============ Press `Enter` to place the object  ...")
        #tutorial.go_to_joint_state(placefinal,'cl')
        #tutorial.go_to_joint_state(placefinal,'op')
        
        #tutorial.execute_plan(cartesian_plan)

        #input("============ Press `Enter` to return home ...")
        #tutorial.go_to_joint_state(placefinal,'op')
        #tutorial.detach_box()
        #tutorial.go_to_joint_state(home,'op')
        #tutorial.add_box()

        #input("============ Press `Enter` to attach a Box to the Panda robot ...")
        #tutorial.attach_box()

        #input(
        #    "============ Press `Enter` to plan and execute a path with an attached collision object ..."
        #)
        #cartesian_plan, fraction = tutorial.plan_cartesian_path(scale=-1)
        #tutorial.execute_plan(cartesian_plan)

        #input("============ Press `Enter` to detach the box from the Panda robot ...")
        #tutorial.detach_box()

        #input(
        #    "============ Press `Enter` to remove the box from the planning scene ..."
        #)
        #tutorial.remove_box()

        #print("============ Python tutorial demo complete!")
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN,filedescriptors)
        #rospy.ROSInterruptException:
       #return
        #except KeyboardInterrupt:
        #return



        
        
    

if __name__ == "__main__":
    main()
    

## BEGIN_TUTORIAL
## .. _moveit_commander:
##    http://docs.ros.org/noetic/api/moveit_commander/html/namespacemoveit__commander.html
##
## .. _MoveGroupCommander:
##    http://docs.ros.org/noetic/api/moveit_commander/html/classmoveit__commander_1_1move__group_1_1MoveGroupCommander.html
##
## .. _RobotCommander:
##    http://docs.ros.org/noetic/api/moveit_commander/html/classmoveit__commander_1_1robot_1_1RobotCommander.html
##
## .. _PlanningSceneInterface:
##    http://docs.ros.org/noetic/api/moveit_commander/html/classmoveit__commander_1_1planning__scene__interface_1_1PlanningSceneInterface.html
##
## .. _DisplayTrajectory:
##    http://docs.ros.org/noetic/api/moveit_msgs/html/msg/DisplayTrajectory.html
##
## .. _RobotTrajectory:
##    http://docs.ros.org/noetic/api/moveit_msgs/html/msg/RobotTrajectory.html
##
## .. _rospy:
##    http://docs.ros.org/noetic/api/rospy/html/
## CALL_SUB_TUTORIAL imports
## CALL_SUB_TUTORIAL setup
## CALL_SUB_TUTORIAL basic_info
## CALL_SUB_TUTORIAL plan_to_joint_state
## CALL_SUB_TUTORIAL plan_to_pose
## CALL_SUB_TUTORIAL plan_cartesian_path
## CALL_SUB_TUTORIAL display_trajectory
## CALL_SUB_TUTORIAL execute_plan
## CALL_SUB_TUTORIAL add_box
## CALL_SUB_TUTORIAL wait_for_scene_update
## CALL_SUB_TUTORIAL attach_object
## CALL_SUB_TUTORIAL detach_object
## CALL_SUB_TUTORIAL remove_object
## END_TUTORIAL
