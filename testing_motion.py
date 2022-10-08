

from turtle import shape
import numpy as np
import copy
import sys
import rospy
import moveit_commander
import moveit_msgs.msg
from time import sleep
import rospy
import actionlib
from geometry_msgs.msg import WrenchStamped
from franka_gripper.msg import GraspAction, GraspGoal,MoveAction,MoveGoal
from franka_msgs.msg import ErrorRecoveryActionGoal,ErrorRecoveryActionFeedback
from evdev import InputDevice, categorize, ecodes,KeyEvent
from pynput import keyboard

from rospy.service import rospyerr

#from franka_msgs.msg import Wrench
opened=[0.037,0.037]
closed=[0.0,0.0]

class MoveGroupPythonInterfaceTutorial(object):
    """MoveGroupPythonInterfaceTutorial"""

    def __init__(self):
        
        super(MoveGroupPythonInterfaceTutorial, self).__init__()
        c=[]
        self.a=np.array(c)
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
        #print(eef_link)
        #print(robot.get_current_state())
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
        self.data = []
        self.joint_effort=[]
    
    def callback(self,data):
        
        
        
       
        joint_effort =[data.wrench.force.x,data.wrench.force.y,data.wrench.force.z,data.wrench.torque.x,data.wrench.torque.y,data.wrench.torque.z]
        self.joint_effort=np.array(joint_effort)
        self.data.append(self.joint_effort)

    def talker(self):
        
        for i in range (0,5):
            pub = rospy.Publisher('/franka_control/error_recovery/goal',ErrorRecoveryActionGoal, queue_size=10)
            str = ErrorRecoveryActionGoal()
            str.goal={}
            pub.publish(str)   
        
  

      
            
    def move(self,scale,string=''):
        
        move_group = self.move_group
        rot=move_group.get_current_rpy(end_effector_link=self.eef_link)
        wpose = move_group.get_current_pose().pose
        if string=='x':
            print("x-motion")
            wpose.position.x -= scale * 0.01 # moveup
        elif string=='y':
            print("y-motion")
            wpose.position.y += scale * 0.01 # moveup
        elif string=='z':
            print("z-motion")
            wpose.position.z += scale * 0.01 # moveup 
        elif string=='rx':
            
            print("rx-motion")
            if scale>=0:
                rot[0] +=  0.05 # moveup
            else:
                rot[0] -=  0.05 # moveup    
            r=self.get_quaternion_from_euler(rot[0], rot[1], rot[2])
            wpose.orientation.x = r[0]
            wpose.orientation.y = r[1]
            wpose.orientation.z = r[2]
            wpose.orientation.w = r[3]
        elif string=='ry':
            print("ry-motion")
            if scale>=0:
                rot[1] +=  0.05 # moveup
            else:
                rot[1] -=  0.05 # moveup 
            r=self.get_quaternion_from_euler(rot[0], rot[1], rot[2])
            wpose.orientation.x = r[0]
            wpose.orientation.y = r[1]
            wpose.orientation.z = r[2]
            wpose.orientation.w = r[3]
        elif string=='rz':
            print("rz-motion")
            if scale>=0:
                rot[2] +=  0.1 # moveup
            else:
                rot[2] -=  0.1 # moveup 
            r=self.get_quaternion_from_euler(rot[0], rot[1], rot[2])
            wpose.orientation.x = r[0]
            wpose.orientation.y = r[1]
            wpose.orientation.z = r[2]
            wpose.orientation.w = r[3]  
        #goal=(wpose.position.x ,wpose.position.y ,wpose.position.z, rot[0],rot[1],rot[2])     
        #print(goal)  
        #move_group.set_pose_target(wpose)
        success=move_group.go(wpose)
        if success==False:
            print("error during execution. starting recovery")
            self.talker()
            
    def get_quaternion_from_euler(self,roll, pitch, yaw):
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
    def listener(self):
        # global var
        # In ROS, nodes are uniquely named. If two nodes with the same
        # name are launched, the previous one is kicked off. The
        # anonymous=True flag means that rospy will choose a unique
        # name for our 'listener' node so that multiple listeners can
        # run simultaneously.
        
        #rospy.init_node('data_log', anonymous=True)

        rospy.Subscriber('/franka_state_controller/F_ext',WrenchStamped, self.callback)
        
        # spin() simply keeps python from exiting until this node is stopped
        # print(test.a)
    def grasp_client(self,string):
	
        
        if string=='op':
            group_name = "hand"
            move_group = moveit_commander.MoveGroupCommander(group_name)
            joint_goal = move_group.get_current_joint_values()
            joint_goal[0] = opened[0]
            joint_goal[1] = opened[1]
            move_group.go(joint_goal, wait=True)
            move_group.stop()
           
        elif string =='cl':
            client = actionlib.SimpleActionClient('franka_gripper/grasp',GraspAction)
            print('waiting for the action server to start')
            timeout=rospy.Duration(5)
            client.wait_for_server(timeout)
            #goal= MoveGoal()
        
            goal = GraspGoal()
            
            goal.force = 70
            goal.epsilon.inner = 0.05
            goal.epsilon.outer = 0.0510

            goal.speed = 0.1
            goal.width = 0.04
            
            client.send_goal(goal)
            wait = client.wait_for_result(timeout)
        
            if not wait:
                rospy.logerr("Action server not available!")
                rospy.signal_shutdown("Action server not available!")
            else:
                return client.get_result()
               
        

if __name__ == '__main__':
   
    test=MoveGroupPythonInterfaceTutorial()
    
    test.listener()
    #rospy.;spin() 
    x= input("c to close gripper and f to open gripper")
    if x == 'f' or x == 'F':
                test.grasp_client('op')
    elif x == 'c' or x == 'C':
                test.grasp_client('cl')    
    while 1:
        
        #d=np.array(test.data)
        m=np.mean(test.data,axis=0) 
         #print("X mean = {} Y mean = {} Z mean = {}".format(m[0],m[1],m[2]))
        if m.size>5:
            # x=sys.stdin.read(1)[0]    
            # print("You pressed", x)
         #print("X mean = {} Y mean = {} Z mean = {}".format(m[0],m[1],m[2]))
            if abs(test.joint_effort[0]-m[0])>2:
              test.move(test.joint_effort[0],'x')
            if abs(test.joint_effort[1]-m[1])>2:
              test.move(test.joint_effort[1],'y') 
            if abs(test.joint_effort[2]-m[2])>2:
              test.move(test.joint_effort[2],'z') 
            elif abs(test.joint_effort[3]-m[3])>1:
              test.move(test.joint_effort[3],'rx')
            elif abs(test.joint_effort[4]-m[4])>1:
              test.move(test.joint_effort[4],'ry') 
            elif abs(test.joint_effort[5]-m[5])>1:
              test.move(test.joint_effort[5],'rz') 
            
                        
                        
    