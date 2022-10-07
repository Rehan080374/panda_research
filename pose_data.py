
import time
import rospy
from franka_msgs.msg import FrankaState
from geometry_msgs.msg import WrenchStamped
#from franka_msgs.msg import Wrench


import csv




def callback(data):
   
    joint_effort=[]
    #rospy.loginfo(rospy.get_caller_id() + 'I heard %s', data.effort)
    #joint_effort =data.tau_ext_hat_filtered
    joint_effort =[data.wrench.force.x,data.wrench.force.y,data.wrench.force.z,data.wrench.torque.x,data.wrench.torque.y,data.wrench.torque.z]
    print("effort data =",joint_effort)
    

    
    
    
    
    with open('/home/rehan/ws_moveit/src/effort_data.csv', 'a+', encoding='UTF8') as f:
            writer = csv.writer(f)
            #write the data
            writer.writerow(joint_effort)
            #print("file should be saved") 
                
            f.close
            

    time.sleep(1)
    
    



def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    
    rospy.init_node('data_log', anonymous=True)

    rospy.Subscriber('/franka_state_controller/F_ext',WrenchStamped, callback)

    # spin() simply keeps python from exiting until this node is stopped
   
    rospy.spin()
    
    

if __name__ == '__main__':
   

   
    with open('/home/rehan/ws_moveit/src/effort_data.csv', 'w', encoding='UTF8') as o:
            write= csv.writer(o)
            header=('joint_effort[0]','joint_effort[1]','joint_effort[2]','joint_effort[3]','joint_effort[4]','joint_effort[5]','joint_effort[6]','joint_effort[7]','joint_effort[8]')
            #header=['position.x','position.y','position.z','orientation.x','orientation.y','orientation.z','orientation.w','gripper_pose']
            #write the header
            write.writerow(header)
            o.close()
    listener()
          
    
              
    

   

