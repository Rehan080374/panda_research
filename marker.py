#! /usr/bin/env python

import rospy
from visualization_msgs.msg import InteractiveMarkerPose
from visualization_msgs.msg import InteractiveMarkerUpdate
from geometry_msgs.msg import PoseStamped

rospy.init_node('rviz_marker')

marker_pub = rospy.Publisher("/panda/cartesian_impedance_example_controller/equilibrium_pose", PoseStamped, queue_size = 10)

#marker = InteractiveMarkerUpdate()
#marker.server_id ="/panda/interactive_marker"

#rate = rospy.Rate(5)

#marker.poses = [pose]
#arker.UPDATE
pose = PoseStamped()
print(pose)
pose.header.stamp = rospy.Time.now()


while not rospy.is_shutdown():
 
 
  
  pose.header.stamp = rospy.Time.now()

  pose.header.frame_id = "panda_link8"
  # Set the pose of the marker
  pose.pose.position.x = 0.5
  pose.pose.position.y = 0
  pose.pose.position.z = 0.42
  

  pose.pose.orientation.x = 0.999760091304779
  pose.pose.orientation.y = -0.021686352789402008
  pose.pose.orientation.z = 0.003082813462242484
  pose.pose.orientation.w = -7.041404023766518e-05
  marker_pub.publish(pose)
  print(pose)
  rospy.rostime.wallsleep(4)
  pose.pose.position.x = 0.5
  pose.pose.position.y =0 
  pose.pose.position.z = 0.42
  pose.pose.orientation.x = 0.6515280604362488
  pose.pose.orientation.y = -0.7586182951927185
  pose.pose.orientation.z =  0.002006412949413061
  pose.pose.orientation.w = -0.002341622719541192
  marker_pub.publish(pose)
  print(pose)
  rospy.rostime.wallsleep(4)
  #marker.seq_num+=1
  
  #marker_pub.publish(pose)
  #rate.sleep() 
  