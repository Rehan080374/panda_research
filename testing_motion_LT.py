
from turtle import shape
import numpy as np
import copy
import sys
import rospy
import moveit_commander
import moveit_msgs.msg
from time import sleep
import rospy
from geometry_msgs.msg import Pose
import actionlib
from franka_gripper.msg import GraspAction, GraspGoal,MoveAction,MoveGoal
#from franka_msgs.msg import Wrench
from geometry_msgs.msg import PoseStamped
joint_effort=[]
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
        self.pose = PoseStamped()
        
    def callback(self,data):
        
        self.pose=data
        print(data)
       
        # joint_effort =[data.wrench.force.x,data.wrench.force.y,data.wrench.force.z,data.wrench.torque.x,data.wrench.torque.y,data.wrench.torque.z]
        # self.joint_effort=np.array(joint_effort)
        # self.data.append(self.joint_effort)

        
        
  
    # def grasp_client(self,string):
	
        
    #     if string=='op':
    #         group_name = "hand"
    #         move_group = moveit_commander.MoveGroupCommander(group_name)
    #         joint_goal = move_group.get_current_joint_values()
    #         joint_goal[0] = 0.034
    #         joint_goal[1] = 0.034

    #         move_group.go(joint_goal, wait=True)
    #         move_group.stop()
            
    #     elif string =='cl':
    #         client = actionlib.SimpleActionClient('franka_gripper/grasp',GraspAction)
    #         print('waiting for the action server to start')
    #         timeout=rospy.Duration(5)
    #         client.wait_for_server(timeout)
    #         #goal= MoveGoal()
        
    #         goal = GraspGoal()
    #         #print("goal ==",goal)
    #         #goal.width = 0.058
    #         #goal.speed = 0.03
    #         goal.force = 70
    #         goal.epsilon.inner = 0.05
    #         goal.epsilon.outer = 0.0510

    #         goal.speed = 0.1
    #         goal.width = 0.04
    #         #print("goal=",goal)
    #         client.send_goal(goal)
    #         wait = client.wait_for_result(timeout)
        
    #         if not wait:
    #             rospy.logerr("Action server not available!")
    #             rospy.signal_shutdown("Action server not available!")
    #         else:
    #             return client.get_result()
            

      
            
    def move(self,scale):
        
        move_group = self.move_group
        rot=move_group.get_current_rpy(end_effector_link=self.eef_link)
        wpose = move_group.get_current_pose().pose
        # if string=='x':
        #     print("x-motion")
        #     wpose.position.x -= scale * 0.01 # moveup
        # elif string=='y':
        #     print("y-motion")
        #     wpose.position.y += scale * 0.01 # moveup
        # elif string=='z':
        #     print("z-motion")
        #     wpose.position.z += scale * 0.01 # moveup 
        # if string=='rx':
        #     print("rx-motion")
        #     rot[0]+= scale * 0.01 # moveup
        # elif string=='ry':
        #     print("ry-motion")
        #     rot[1] += scale * 0.01 # moveup
        # elif string=='rz':
        #     print("rz-motion")
        #     rot[2] += scale * 0.01 # moveup
        wpose.position.x += scale
        wpose.position.y += scale
        # wpose.position.z += scale
        # wpose.orientation.x += scale
        # wpose.orientation.y += scale
        # wpose.orientation.z += scale
        # wpose.orientation.w += scale
        wpose=self.pose
        goal=(wpose)     
         
        move_group.go(goal)

    def listener(self):
    
        rospy.Subscriber('equilibrium_pose',PoseStamped, self.callback)
        
        
        

if __name__ == '__main__':
   
    test=MoveGroupPythonInterfaceTutorial()
    # test.listener()
    
    while True:
        test.listener()
        print("pose data = ",test.pose)
        test.move(0.2)
        sleep(5)
        test.move(-0.2)
        print("pose data = ",test.pose)
        sleep(5)


    # test.grasp_client('op')
    # sleep(5)
    # test.grasp_client('cl')
    # sleep(5)
    # rospy.spin()
    
    