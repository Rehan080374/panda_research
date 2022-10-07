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

import time
import rospy
from franka_msgs.msg import FrankaState
from geometry_msgs.msg import WrenchStamped
#from franka_msgs.msg import Wrench

from itertools import count
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation



import csv
joint_effort=[]
class MoveGroupPythonInterfaceTutorial(object):
    """MoveGroupPythonInterfaceTutorial"""

    def __init__(self):
        super(MoveGroupPythonInterfaceTutorial, self).__init__()

        
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("move_group_python_interface_tutorial", anonymous=True) 
        
        robot = moveit_commander.RobotCommander()
        scene = moveit_commander.PlanningSceneInterface()
        group_name = "panda_arm"
        move_group = moveit_commander.MoveGroupCommander(group_name)
        display_trajectory_publisher = rospy.Publisher(
            "/move_group/display_planned_path",
            moveit_msgs.msg.DisplayTrajectory,
            queue_size=20,
        )
        planning_frame = move_group.get_planning_frame()
        
        eef_link = move_group.get_end_effector_link()
        

        # We can get a list of all the groups in the robot:
        group_names = robot.get_group_names()
       
        self.box_name = ""
        self.robot = robot
        self.scene = scene
        self.move_group = move_group
        self.display_trajectory_publisher = display_trajectory_publisher
        self.planning_frame = planning_frame
        self.eef_link = eef_link
        self.group_names = group_names


    def callback(self,data):
    
        #joint_effort=data
        #rospy.loginfo(rospy.get_caller_id() + 'I heard %s', data.effort)
        # joint_effort =data.tau_ext_hat_filtered
        joint_effort =[data.wrench.force.x,data.wrench.force.y,data.wrench.force.z,data.wrench.torque.x,data.wrench.torque.y,data.wrench.torque.z]
        print("effort data =",joint_effort)
        group_name = "panda_arm"
        #move_group = self.move_group
        move_group = moveit_commander.MoveGroupCommander(group_name)
        wpose = move_group.get_current_pose().pose
        waypoints = []
        if joint_effort[0]>=3 or joint_effort[0]<=-2 or  joint_effort[1]>=2 or joint_effort[1]<=-2 or joint_effort[2]>=2 or joint_effort[2]<=-2 :
            wpose.position.x +=joint_effort[0]  * 0.01 # moveup
            wpose.position.y +=joint_effort[1]  * 0.01 # moveup
            wpose.position.z +=joint_effort[2]  * 0.01 # moveup
            waypoints.append(copy.deepcopy(wpose))
            (plan, fraction) = move_group.compute_cartesian_path(waypoints, 0.01, 0.0 )  
            # robot = self.robot
            # display_trajectory_publisher = self.display_trajectory_publisher
            # display_trajectory = moveit_msgs.msg.DisplayTrajectory()
            # display_trajectory.trajectory_start = robot.get_current_state()
            # display_trajectory.trajectory.append(plan)
            # display_trajectory_publisher.publish(display_trajectory)
            move_group.execute(plan, wait=False)
        rospy.sleep(0.5)

        
        
        
        
        # with open('/home/panda/ws_moveit/src/effort_data.csv', 'a+', encoding='UTF8') as f:
        #         writer = csv.writer(f)
        #         #write the data
        #         writer.writerow(joint_effort)
        #         #print("file should be saved") 
                    
        #         f.close
                

        #time.sleep(0.1)
        return joint_effort
    def animate(self,joint_effort):
        #data = pd.DataFrame(joint_effort, columns = ['x','y','z','rx','ry','rz'])
        #data = pd.read_csv('/home/panda/ws_moveit/src/effort_data.csv')
        #x = data['x_value']
        data=joint_effort
        y1 = data[0]
        y2 = data[1]
        y3 = data[2]
        y4 = data[3]
        y5 = data[4]
        y6 = data[5]
        # y7 = data['joint_effort[6]']
        # y8 = data['joint_effort[7]']
        # y9 = data['joint_effort[8]']

        plt.cla()

        plt.plot(  y1, label='joint_effort[x]')
        plt.plot(  y2, label='joint_effort[y]') 
        plt.plot(  y3, label='joint_effort[z]')
        plt.plot(  y4, label='joint_effort[RX]')
        plt.plot(  y5, label='joint_effort[Ry]')
        plt.plot(  y6, label='joint_effort[Rz]')
        #plt.plot(  y7, label='joint_effort[6]')
        #plt.plot(  y8, label='joint_effort[7]') 
        #plt.plot(  y9, label='joint_effort[8]')
        
        plt.legend(loc='upper left')
        plt.tight_layout()


      
            
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


    def listener(self):

        # In ROS, nodes are uniquely named. If two nodes with the same
        # name are launched, the previous one is kicked off. The
        # anonymous=True flag means that rospy will choose a unique
        # name for our 'listener' node so that multiple listeners can
        # run simultaneously.
        
        #rospy.init_node('data_log', anonymous=True)

        rospy.Subscriber('/franka_state_controller/F_ext',WrenchStamped, test.callback)

        # spin() simply keeps python from exiting until this node is stopped
    
        rospy.spin()
        
        

if __name__ == '__main__':
   
    test=MoveGroupPythonInterfaceTutorial()
    rospy.Subscriber('/franka_state_controller/F_ext',WrenchStamped, test.callback)
    #ani = FuncAnimation(plt.gcf(), test.animate(joint_effort), interval=1)
    #d=test.callback( )
    print(joint_effort)
    # plt.tight_layout()
    # plt.show()
    #print(WrenchStamped.wrench.force.x)
    rospy.spin()  

    