import sched
import numpy as np
import os
import configparser
import sys
import cv2
import wget
import mediapipe as mp
import pyzed.sl as sl
import math
import rospy
from std_msgs.msg import String
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from geometry_msgs.msg import PoseStamped
from google.protobuf.json_format import MessageToDict
marker_pose = PoseStamped()
# class Data(object):
#     def __init__(self,point_cloud, pts):
#         self.current_points = pts
#         self.current_distance=[]
#         self.previous_distance=[]
#         self.point_cloud=point_cloud
#     def check_distance_array(self) :
#         distance_array = []
#         dist=0
        
#         for i in range(0, len(self.current_points)):
#             neighbors=[]
            
#             print(self.current_points[i])
#             x,y=self.current_points[i]
#             neighbors=[x,y]
#             neighbors.append([x-1,y-1],[x-1,y],[x-1,y+1],[x,y-1],[x,y+1],[x+1,y+1],[x+1,y],[x+1,y-1])
#             if neighbors.all()>=0 : 
#                for j in range(0,len(neighbors)):
#                         x,y=neighbors[j]
#                         err,point_cloud_value = self.point_cloud.get_value(x,y)
#                         dist = math.sqrt(point_cloud_value[0] * point_cloud_value[0] +
#                                                         point_cloud_value[1] * point_cloud_value[1] +
#                                                         point_cloud_value[2] * point_cloud_value[2])
#                         if not np.isnan(dist) and not np.isinf(dist) and dist>=0 :
                            
#                             break
#                         else:  
#                             continue   
#             distance_array.append(dist)                 
#         return distance_array    

#     def minus(self, x):
#         self.n -= x
#         return self.n

        
        # self.number_subscriber = rospy.Subscriber("/number", Int64, self.callback_number)
        # self.reset_service = rospy.Service("/reset_counter", SetBool, self.callback_reset_counter)
    # def callback_number(self, msg):
    #     self.l= msg.data
    #     new_msg = list
    #     new_msg.data = self.l
    #     self.pub.publish(new_msg)
    # def callback_reset_counter(self, req):
    #     if req.data:
    #         self.counter = 0
    #         return True, "Counter has been successfully reset"
    #     return False, "Counter has not been reset"
start_x=0
start_y=0
start_z=0 
index=0

def rolling_average(a,window) : 
    global index
    array=a
    s=0
    m_average = 0
    # print(index)
    if index>window:
         
        for i in range(0,window):
            s=s+array[i+index-window]
        m_average=s/window
                    
    
    return m_average     
     
def check_direction(distance) :

    d_thumb=distance[4]-distance[0]
    d_wrist=distance[4]-distance[1]
    d_pinky=distance[4]-distance[2]
    d_middl=distance[4]-distance[3]
    if d_middl<0 and d_wrist>0:
        print("up")
    elif d_middl>0 and d_wrist<0:
        print("down")
    elif d_thumb<0 and d_pinky>0:
        print("left")
    elif d_thumb>0 and d_pinky<0:
        print("right")
    else:
        print("stop")    
    return 
def check_angle(normal,r) :
    r=r
    phi=((math.pi/2)+normal[1])#*math.pi/180
    theta=normal[2]#*math.pi/180
    global start_x,start_y,start_z
    x = r*math.sin(phi)*math.cos(theta)
    y = r*math.sin(phi)*math.sin(theta)
    z = r*math.cos(phi)
    x_temp=x-start_x
    y_temp=y-start_y
    z_temp=z-start_z
    print ("x ={},y ={},z={}".format(x,y,z))
    start_x=x
    start_y=y
    start_z=z
    # print ("temp_x ={},temp_y ={},temp_z={}".format(x_temp,y_temp,z_temp))
    
        
def check_distance_between_points(point_cloud,p1,p2) :
    if p1[0]>0 and p1[1]>0 and p2[0]>0 and p2[1]>0:
        err1,point_cloud_value1 = point_cloud.get_value(p1[0], p1[1])
        err2,point_cloud_value2 = point_cloud.get_value(p2[0], p2[1])
        
        distance = math.sqrt((point_cloud_value1[0] - point_cloud_value2[0]) **2+
                                        (point_cloud_value1[1] - point_cloud_value2[1])**2 +
                                        (point_cloud_value1[2] - point_cloud_value2[2])**2)
        if  math.isnan(distance)==False and  math.isinf(distance)==False   :                            
            distance=distance
        else:
            distance=0    
    else:
        distance=0
    return distance 

def check_depth(depth,points_array) :
    d_array = []
    dist=0
    for i in range(0, len(points_array)):
        x,y=points_array[i]
        neighbors=[[x,y],[x-1,y-1],[x-1,y],[x-1,y+1],[x,y-1],[x,y+1],[x+1,y+1],[x+1,y],[x+1,y-1]]
        for j in range(0,len(neighbors)):
            if neighbors[j][0]>=0 and neighbors[j][1]>=0:
                e,dist = depth.get_value(neighbors[j][0],neighbors[j][1])
            
                if not np.isnan(dist) and not np.isinf(dist) and dist>=0  :          
                    break 
        if not np.isnan(dist) and not np.isinf(dist) and dist>=0  :          
                d_array.append(dist) 
        else:
                 d_array.append(0)                       
    return d_array    

def check_distance_array(point_cloud,points_array) :
        distance_array = []
        dist=0
        
        for i in range(0, len(points_array)):
            neighbors=[]
            
            # print(points_array[i])
            x,y=points_array[i]
            neighbors=[[x,y],[x-1,y-1],[x-1,y],[x-1,y+1],[x,y-1],[x,y+1],[x+1,y+1],[x+1,y],[x+1,y-1]]
            nib=np.array(neighbors)
             
            for j in range(0,len(neighbors)):
                    a,b=neighbors[j]
                    if a>=0 and b>=0:
                        # print(a,b)
                    
                        err,point_cloud_value = point_cloud.get_value(a,b)
                        dist = math.sqrt(point_cloud_value[0] * point_cloud_value[0] +
                                                        point_cloud_value[1] * point_cloud_value[1] +
                                                        point_cloud_value[2] * point_cloud_value[2])
                        if not np.isnan(dist) and not np.isinf(dist) and dist>=0  :
                            # print("distance = " ,dist)
                            break
            if not np.isnan(dist) and not np.isinf(dist) and dist>=0  :          
                distance_array.append(dist) 
            else:
                 distance_array.append(0)                   
        return distance_array 
def centroid(vertexes):
     _x_list = [vertex [0] for vertex in vertexes]
     _y_list = [vertex [1] for vertex in vertexes]
     _len = len(vertexes)
   
     _x =int( sum(_x_list) / _len)
     _y = int(sum(_y_list) / _len)
     return(_x, _y)    
    
def main():
    index=0
    # rospy.init_node('finger_cordinates')
    # finger_cordinates()
    list=[0,0,0]
   
    # initialise the media pipeline model
    mp_drawing = mp.solutions.drawing_utils
    mp_drawing_styles = mp.solutions.drawing_styles
    mp_hands = mp.solutions.hands
    hands= mp_hands.Hands(static_image_mode=False,model_complexity=0,max_num_hands=1,min_detection_confidence=0.7,min_tracking_confidence=0.5) 
    

    # Create a InitParameters object and set configuration parameters
    init_params = sl.InitParameters()
    init_params.camera_resolution = sl.RESOLUTION.HD720  
    init_params.camera_fps = 30  # Set fps at 30
    init_params.depth_mode = sl.DEPTH_MODE.ULTRA
    init_params.coordinate_units=sl.UNIT.METER
    init_params.coordinate_system=sl.COORDINATE_SYSTEM.RIGHT_HANDED_Z_UP_X_FWD
    init_params.depth_minimum_distance = 0.1 # Set the minimum depth perception distance to 15cm
    # Open the camera
    cam = sl.Camera()
    err = cam.open(init_params)
    if err != sl.ERROR_CODE.SUCCESS:
        exit(1)
    runtime = sl.RuntimeParameters()
    mat = sl.Mat()
    depth = sl.Mat()
    point_cloud = sl.Mat()
    plane = sl.Plane()
  
    find_plane_status = sl.ERROR_CODE.SUCCESS
    tracking_state = sl.POSITIONAL_TRACKING_STATE.OFF
    cam.enable_positional_tracking()
    # coord=sl.unit2()
    pose=sl.Pose()
    image_width = cam.get_camera_information().camera_resolution.width
    image_height = cam.get_camera_information().camera_resolution.height
    print("image resolution '{0}'x'{1}' ".format(image_width,image_height))
    square_width=[100,100]
    center=[image_width/2,image_height/2]
    start=[int(center[0]-square_width[0]/2),int(center[1]-square_width[1]/2)]
    end=[int(center[0]+square_width[0]/2),int(center[1]+square_width[1]/2)]
    # print(start,end)
    tx,ty,tz,ox,oy,oz,ow=[np.array for _ in range(7)]
    window=2
    while cam.open():
        err = cam.grab(runtime)
        if (err == sl.ERROR_CODE.SUCCESS) :
            cam.retrieve_image(mat, sl.VIEW.LEFT)
            image=mat.get_data()
            # Retrieve depth map. Depth is aligned on the left image
            cam.retrieve_measure(depth, sl.MEASURE.DEPTH)
            # Retrieve colored point cloud. Point cloud is aligned on the left image.
            tracking_state = cam.get_position(pose) # Get the tracking state of the camera
            cam.retrieve_measure(point_cloud, sl.MEASURE.XYZRGBA)
            image.flags.writeable = False
            image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
            results = hands.process(image)
          
            image.flags.writeable = True
            image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
            # image = cv2.rectangle(image, start, end, (255,0,0), 2)
            if results.multi_hand_landmarks:
               
                lbl = str(results.multi_handedness[0].classification[0].label)
                score=float(results.multi_handedness[0].classification[0].score)
                # print(results)
                    # print(handedness_dict["classification"])
                if lbl=="Left" and score > 0.6: 
                    # print("label =",lbl,"score = ",score)   
                    for hand_landmarks in results.multi_hand_landmarks:
                        mp_drawing.draw_landmarks(image,hand_landmarks,mp_hands.HAND_CONNECTIONS,mp_drawing_styles.get_default_hand_landmarks_style(),mp_drawing_styles.get_default_hand_connections_style())
                        
                             
                        pts = np.array([
                            [hand_landmarks.landmark[5].x * image_width,
                            hand_landmarks.landmark[5].y * image_height],
                            [hand_landmarks.landmark[1].x * image_width,
                            hand_landmarks.landmark[1].y * image_height],
                            [hand_landmarks.landmark[0].x * image_width,
                            hand_landmarks.landmark[0].y * image_height],
                            [hand_landmarks.landmark[17].x * image_width
                            ,hand_landmarks.landmark[17].y * image_height]], np.int32)
                        cent=centroid(pts)
                        # print("center",cent)
                        # cent=[cent[0]-10,cent[1]-10]
                        p1=[hand_landmarks.landmark[8].x * image_width,hand_landmarks.landmark[8].y * image_height]
                        p2=[hand_landmarks.landmark[4].x * image_width,hand_landmarks.landmark[4].y * image_height]
                        p4=[hand_landmarks.landmark[16].x * image_width,hand_landmarks.landmark[16].y * image_height]
                        p3=[cent[0],cent[1]]
                        dist_1=math.dist(p3,p1)
                        dist_2=math.dist(p3,p2)
                        dist_3=math.dist(p3,p4)
                        # m=check_distance_between_points(point_cloud,p3,p1)
                        # n=check_distance_between_points(point_cloud,p3,p2)
                        # print("max=",m,"min =",n)
                        # min_dist=min(abs(p1[0]-p3[0]),abs(p1[1]-p3[1]))
                        # max_dist=min(abs(p2[0]-p3[0]),abs(p2[1]-p3[1]))
                        # print("max= ",max_dist,"min= ",min_dist)
                        # finger_distance=check_distance_between_points(point_cloud,p1,p2)
                        
                        if dist_1<dist_2:
                            # print("open")
                            st="close"
                        elif dist_2>dist_3:
                            # print("open")
                            st="slow"   
                        else:
                            # print("closed") 
                            st="open"   
                        # print (st)    
                        # print(dist_1,dist_2,dist_3)
                        # print("distance = ",finger_distance)
                        cv2.circle(image,cent,radius=3,color=(0,255,255),thickness=10) 
                        a,b=cent
                        # print(pts)
                        # if tracking_state == sl.POSITIONAL_TRACKING_STATE.OK:  
                        # Detect the plane passing by the depth value of pixel coord
                        if a>0 and b >0:
                            find_plane_status = cam.find_plane_at_hit(cent, plane)
                        if find_plane_status == sl.ERROR_CODE.SUCCESS:
                            normal = plane.get_normal() # Get the normal vector of the detected plane
                            plane_equation = plane.get_plane_equation() # Get (a,b,c,d) where ax+by+cz=d  
                            plane_pose=plane.get_pose()
                            poly=plane.get_bounds()
                            # print("polygon = ",poly)
                            # print("plane_pose = ",plane_pose)
                            # print("camera_pose = ",pose.pose_data())
                            orient = plane_pose.get_orientation().get()
                            # print("orintation = ", orient)
                            
                            tx= np.append(tx,round(plane_pose.get_translation().get()[0], 3))
                            ty = np.append(ty,round(plane_pose.get_translation().get()[1], 3))
                            tz = np.append(tz,round(plane_pose.get_translation().get()[2], 3))
                            # print("Translation: Tx: {0}, Ty: {1}, Tz {2}, \n".format(tx, ty, tz))
                            ox = np.append(ox,round(plane_pose.get_orientation().get()[0], 3))
                            oy = np.append(oy,round(plane_pose.get_orientation().get()[1], 3))
                            oz = np.append(oz,round(plane_pose.get_orientation().get()[2], 3))
                            ow = np.append(ow,round(plane_pose.get_orientation().get()[3], 3))
                            # print("Orientation: Ox: {0}, Oy: {1}, Oz {2}, Ow: {3}\n".format(ox, oy, oz, ow))
                            if index>window: 
                                # marker_pose.pose.position.x =round(np.mean(tx[index-window:index]),3)
                                # marker_pose.pose.position.y =round(np.mean(ty[index-window:index]),3)
                                # marker_pose.pose.position.z =round(np.mean(tz[index-window:index]),3) 
                                # marker_pose.pose.orientation.x =round(np.mean(ox[index-window:index]),3)
                                # marker_pose.pose.orientation.y =round(np.mean(oy[index-window:index]),3)
                                # marker_pose.pose.orientation.z =round(np.mean(oz[index-window:index]),3)
                                # marker_pose.pose.orientation.w =round(np.mean(ow[index-window:index]),3)
                                marker_pose.pose.position.x =tx[index]
                                marker_pose.pose.position.y =ty[index]
                                marker_pose.pose.position.z =tz[index] 
                                marker_pose.pose.orientation.x =ox[index]
                                marker_pose.pose.orientation.y =oy[index]
                                marker_pose.pose.orientation.z =oz[index]
                                marker_pose.pose.orientation.w =ow[index]
                                marker_pose.header.frame_id = st
                                marker_pose.header.seq = index
                                marker_pose.header.stamp = rospy.get_rostime()
                                pose_pub.publish(marker_pose)
                                # string_pub.publish("string")
                                # print(marker_pose)
                                rate.sleep()
                            
                            euler = plane_pose.get_euler_angles(radian=False)
                            # print("eulers = ",marker_pose.header.seq)
                            
                            index+=1
                        # else:
                        #      print("lost positional tracking")            

                        # points=np.append(pts,[cent],axis=0)
                        
                        # d=check_distance_array(point_cloud,points)
                        # # d=check_depth(depth,points)
                        # # print(len(f))
                        
                        # # print(d) 
                        # dist=np.array(d)  
                        # # check_direction(dist) 
                        # if not np.isnan(dist.all()) and not np.isinf(dist.all()):
                            
                        #     for i in range (0,len(points)):
                        #         x,y=points[i]
                        #         # print(i)
                        #         cv2.putText(image, str(round(d[i],3)) ,(points[i]),cv2.FONT_HERSHEY_SIMPLEX, 1,(0,0,255),2,cv2.LINE_AA)
                            
                        #     p = pts.reshape((-1,1,2))
                        #     cv2.polylines(image,[p],True,(0,255,255))    
                            
                            
                       
                   
                        sys.stdout.flush()           
                            
            # Flip the image horizontally for a selfie-view display.
            cv2.imshow('MediaPipe Hands', image)
            #cv2.imshow("left RECT", left_rect)
        
        if cv2.waitKey(30) >= 0 :
                break    
                
    cam.close()
    
    
if __name__ == "__main__":
    # pub = rospy.Publisher("/finger_cordinates", Float32MultiArray, queue_size=10)
    # rospy.init_node('finger_cordinates')
    rospy.init_node("my_equilibrium_pose_node")
    pose_pub = rospy.Publisher(
        "my_equilibrium_pose", PoseStamped, queue_size=10)
    # string_pub =rospy.Publisher(
    #     "my_equilibrium_pose1", String, queue_size=10)
    rate = rospy.Rate(20)    
    main()
    
    # rospy.spin()