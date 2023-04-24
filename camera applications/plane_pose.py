import sched
import numpy as np

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
def check_angles(point_cloud,p1,p2,p3):
    _,wrist=point_cloud.get_value(p1[0],p1[1])
    _,pinky=point_cloud.get_value(p2[0],p2[1])
    _,ind=point_cloud.get_value(p3[0],p3[1])
    # print(wrist) 
       
    v1 = np.array(
        [
            wrist[0] - ind[0],
            wrist[1] - ind[1],
            wrist[2] - ind[2],
        ]
    )
    v2 = np.array(
        [
            wrist[0] - pinky[0],
            wrist[1] - pinky[1],
            wrist[2] - pinky[2],
        ]
    )
    
    normal = np.cross(v2, v1)
    # print(normal)
    camera_vector_x = np.array([1, 1, 1]) 
    camera_vector_y = np.array([0, 1, 0]) 
    camera_vector_z = np.array([0, 0, 1])
    
    angle_x = math.atan2(np.linalg.norm(np.cross(camera_vector_x, normal)), np.dot(camera_vector_x, normal)) 
    # print(angle_x*180/math.pi,np.dot(camera_vector_x, normal))
    angle_y = math.atan2(np.linalg.norm(np.cross(camera_vector_y, normal)), np.dot(camera_vector_x, normal))
    angle_z = math.atan2(np.linalg.norm(np.cross(camera_vector_z, normal)), np.dot(camera_vector_x, normal))
    euler_angles=[angle_x*180/math.pi,angle_y*180/math.pi,angle_z*180/math.pi]
    # print(euler_angles)
    theta_y= math.acos((np.dot(camera_vector_z, normal))/(np.linalg.norm(camera_vector_z)*np.linalg.norm(normal))) 
    theta1= math.acos((np.dot(camera_vector_x, normal))/(np.linalg.norm(camera_vector_x)*np.linalg.norm(normal)))
    theta_z= math.atan2(np.linalg.norm(np.cross(camera_vector_y, normal)), np.dot(camera_vector_y, normal))
    # print(theta1*180/math.pi)
    return euler_angles    
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
    
    
    # print("image resolution '{0}'x'{1}' ".format(image_width,image_height))
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
                        
                        euler_angles = check_angles(point_cloud,(pts[2]),(pts[3]),(pts[0]))
                       
                        # print("euler angles = ",euler_angles)
                        # cent=[cent[0]-10,cent[1]-10]
                        # normal = np.cross(v2, v1)
                        p1=[hand_landmarks.landmark[8].x * image_width,hand_landmarks.landmark[8].y * image_height]
                        p2=[hand_landmarks.landmark[4].x * image_width,hand_landmarks.landmark[4].y * image_height]
                        p4=[hand_landmarks.landmark[16].x * image_width,hand_landmarks.landmark[16].y * image_height]
                        p3=[cent[0],cent[1]]
                        dist_1=math.dist(p3,p1)
                        dist_2=math.dist(p3,p2)
                        dist_3=math.dist(p3,p4)
                        
                        
                        if dist_1<dist_2:
                            # print("open")
                            st="close"
                        elif dist_2>dist_3:
                            # print("open")
                            st="slow"   
                        else:
                            # print("closed") 
                            st="open"   
                       
                        cv2.circle(image,cent,radius=3,color=(0,255,255),thickness=10) 
                        a,b=cent
                      
                        if a>0 and b >0:
                            find_plane_status = cam.find_plane_at_hit(cent, plane)
                        if find_plane_status == sl.ERROR_CODE.SUCCESS:
                            normal = plane.get_normal() # Get the normal vector of the detected plane
                            plane_equation = plane.get_plane_equation() # Get (a,b,c,d) where ax+by+cz=d  
                            # print(normal)
                            plane_pose=plane.get_pose()
                            poly=plane.get_bounds()
                            # print("polygon = ",poly)
                            # print("plane_pose = ",plane_pose)
                            # print("camera_pose = ",pose.pose_data())
                            orient = plane_pose.get_orientation().get()
                            translation=plane_pose.get_translation().get()
                            # print("orintation = ", orient)
                            
                            if cent[0]>0 and cent[0]>0:
                                _,xyz=point_cloud.get_value(cent[0],cent[1])
                                print("x = ",round(xyz[0],3),"y = ",round(xyz[1],3),"z = ",round(xyz[2],3),"index=",index)
                                marker_pose.pose.position.x =round(translation[0], 3)
                                marker_pose.pose.position.y =round(translation[1], 3)
                                marker_pose.pose.position.z =round(translation[2], 3)
                                # marker_pose.pose.position.x =round(xyz[0], 3)
                                # marker_pose.pose.position.y =round(xyz[1], 3)
                                # marker_pose.pose.position.z =round(xyz[2], 3)
                                marker_pose.pose.orientation.x =round(orient[0], 3)
                                marker_pose.pose.orientation.y =round(orient[1], 3)
                                marker_pose.pose.orientation.z =round(orient[2], 3)
                                marker_pose.pose.orientation.w =round(orient[3], 3)
                                # marker_pose.header.frame_id = st
                                marker_pose.header.frame_id = st
                                marker_pose.header.seq = index

                                marker_pose.header.stamp = rospy.get_rostime()
                                pose_pub.publish(marker_pose)
                                # print(marker_pose)
                                # string_pub.publish("string")
                                # print(marker_pose)
                                rate.sleep()
                                
                                euler = plane_pose.get_euler_angles(radian=False)
                                # print("eulers = ",euler)
                                
                                index+=1
                        
                            
                       
                   
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