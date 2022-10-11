
from turtle import shape
import numpy as np
import copy
import sys
import rospy
import moveit_commander
import moveit_msgs.msg
from time import sleep
import rospy
from geometry_msgs.msg import WrenchStamped
#from franka_msgs.msg import Wrench

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
    
    def callback(self,data):
        
        
        
       
        joint_effort =[data.wrench.force.x,data.wrench.force.y,data.wrench.force.z,data.wrench.torque.x,data.wrench.torque.y,data.wrench.torque.z]
        self.joint_effort=np.array(joint_effort)
        self.data.append(self.joint_effort)

        
        
  

      
            
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
        if string=='rx':
            print("rx-motion")
            rot[0]+= scale * 0.01 # moveup
        elif string=='ry':
            print("ry-motion")
            rot[1] += scale * 0.01 # moveup
        elif string=='rz':
            print("rz-motion")
            rot[2] += scale * 0.01 # moveup   
        goal=(wpose.position.x ,wpose.position.y ,wpose.position.z, rot[0],rot[1],rot[2])     
        print(goal)  
        move_group.go(goal)

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
        
        

if __name__ == '__main__':
   
    test=MoveGroupPythonInterfaceTutorial()
    #rospy.Subscriber('/franka_state_controller/F_ext',WrenchStamped, test.callback)
    #ani = FuncAnimation(plt.gcf(), test.animate(joint_effort), interval=1)
    #d=test.callback( )
    #print(joint_effort)
    # plt.tight_layout()
    # plt.show()
    #print(WrenchStamped.wrench.force.x)
    #test.animate()
    test.listener()
    #rospy.;spin() 
    while 1:
        #d=np.array(test.data)
        m=np.mean(test.data,axis=0) 
        #print("X mean = {} Y mean = {} Z mean = {}".format(m[0],m[1],m[2]))
        if m.size>5:
         #print("X mean = {} Y mean = {} Z mean = {}".format(m[0],m[1],m[2]))
            if abs(test.joint_effort[0]-m[0])>1:
              test.move(test.joint_effort[0],'x')
            if abs(test.joint_effort[1]-m[1])>1:
              test.move(test.joint_effort[1],'y') 
            if abs(test.joint_effort[2]-m[2])>1:
              test.move(test.joint_effort[2],'z') 
            if abs(test.joint_effort[3]-m[3])>1:
              test.move(test.joint_effort[3],'rx')
            if abs(test.joint_effort[4]-m[4])>1:
              test.move(test.joint_effort[4],'ry') 
            if abs(test.joint_effort[5]-m[5])>1:
              test.move(test.joint_effort[5],'rz')          
        sleep(0.3)
        # x,y,z,rx,ry,rz
        # print(x,y,z)

    