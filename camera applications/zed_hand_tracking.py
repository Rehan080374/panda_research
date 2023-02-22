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
from std_msgs.msg import Float32MultiArray
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from geometry_msgs.msg import PoseStamped
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
    
        
def check_distance(point_cloud,x,y) :
    err,point_cloud_value = point_cloud.get_value(x, y)
    
    distance = math.sqrt(point_cloud_value[0] * point_cloud_value[0] +
                                    point_cloud_value[1] * point_cloud_value[1] +
                                    point_cloud_value[2] * point_cloud_value[2])
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
    hands= mp_hands.Hands(static_image_mode=False,model_complexity=0,max_num_hands=1,min_detection_confidence=0.5,min_tracking_confidence=0.5) 
    

    # Create a InitParameters object and set configuration parameters
    init_params = sl.InitParameters()
    init_params.camera_resolution = sl.RESOLUTION.HD720  # Use HD1080 video mode
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

            # Draw the hand annotations on the image.
            image.flags.writeable = True
            image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
            # image = cv2.rectangle(image, start, end, (255,0,0), 2)
            if results.multi_hand_landmarks:
                
                for hand_landmarks in results.multi_hand_landmarks:
                    # mp_drawing.draw_landmarks(image,hand_landmarks,mp_hands.HAND_CONNECTIONS,mp_drawing_styles.get_default_hand_landmarks_style(),mp_drawing_styles.get_default_hand_connections_style())
                    #print(f'Index finger tip coordinate: (',f'{hand_landmarks.landmark[mp_hands.HandLandmark.INDEX_FINGER_TIP].x * image_size.width}, 'f'{hand_landmarks.landmark[mp_hands.HandLandmark.INDEX_FINGER_TIP].y * image_size.height})')
                    # x=int(hand_landmarks.landmark[mp_hands.HandLandmark.INDEX_FINGER_TIP].x * image_width)
                    # y=int(hand_landmarks.landmark[mp_hands.HandLandmark.INDEX_FINGER_TIP].y * image_height)
                    # print(f'Index finger tip coordinate: (',f'{x},'f'{y})')
                    # pts = np.array([
                    #     [hand_landmarks.landmark[mp_hands.HandLandmark.THUMB_MCP].x * image_width,
                    #     hand_landmarks.landmark[mp_hands.HandLandmark.THUMB_MCP].y * image_height],
                    #     [hand_landmarks.landmark[mp_hands.HandLandmark.WRIST].x * image_width,
                    #     hand_landmarks.landmark[mp_hands.HandLandmark.WRIST].y * image_height],
                    #     [hand_landmarks.landmark[mp_hands.HandLandmark.PINKY_MCP].x * image_width,
                    #     hand_landmarks.landmark[mp_hands.HandLandmark.PINKY_MCP].y * image_height],
                    #     [hand_landmarks.landmark[mp_hands.HandLandmark.MIDDLE_FINGER_TIP].x * image_width
                    #     ,hand_landmarks.landmark[mp_hands.HandLandmark.MIDDLE_FINGER_TIP].y * image_height]], np.int32)      
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
                    a,b=cent
                    # print(pts)
                    if tracking_state == sl.POSITIONAL_TRACKING_STATE.OK:  
                        # Detect the plane passing by the depth value of pixel coord
                        find_plane_status = cam.find_plane_at_hit(cent, plane)
                    if find_plane_status == sl.ERROR_CODE.SUCCESS:
                        normal = plane.get_normal() # Get the normal vector of the detected plane
                        plane_equation = plane.get_plane_equation() # Get (a,b,c,d) where ax+by+cz=d  
                        plane_pose=plane.get_pose()
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
                            marker_pose.pose.position.x =round(np.mean(tx[index-window:index]),3)
                            marker_pose.pose.position.y =round(np.mean(ty[index-window:index]),3)
                            marker_pose.pose.position.z =round(np.mean(tz[index-window:index]),3) 
                            marker_pose.pose.orientation.x =round(np.mean(ox[index-window:index]),3)
                            marker_pose.pose.orientation.y =round(np.mean(oy[index-window:index]),3)
                            marker_pose.pose.orientation.z =round(np.mean(oz[index-window:index]),3)
                            marker_pose.pose.orientation.w =round(np.mean(ow[index-window:index]),3)
                            marker_pose.header.frame_id = "panda_hand_tcp"
                            marker_pose.header.stamp = rospy.Time(0)
                            pose_pub.publish(marker_pose)
                            rate.sleep()
                        euler = plane_pose.get_euler_angles(radian=False)
                        print("euler_angles = ", euler)
                        # check_angle(euler,tx)
                        # if index>10:
                        #     print("tx ", tx[index],"average tx =",np.mean(tx[index-10:index]))
                            # print("tY ", ty[index],"average tY =",rolling_average(ty,10))
                        # plane_transform = sl.Transform()
                        # plane_equation = plane.get_transform()
                        # print("plane equation = ",plane_equation," plane normal = ",10*normal)
                        # p=check_distance(point_cloud,a,b)
                        # if not np.isnan(p) and not np.isinf(p) and p>=0  : 
                        #     check_angle(100*normal,p)
                            
                            # Create axis
                        index+=1
                              

                    points=np.append(pts,[cent],axis=0)
                    # print(pts)
                    # center_distance,p=check_distance(point_cloud,a,b)

                    # print("pointcloud vlaue = ",p,"depth",depth.get_value(a,b))
                    # print("center ",cent ," at distance  = ",center_distance)
                    d=check_distance_array(point_cloud,points)
                    # d=check_depth(depth,points)
                    # print(len(f))
                    
                    # print(d) 
                    dist=np.array(d)  
                    # check_direction(dist) 
                    if not np.isnan(dist.all()) and not np.isinf(dist.all()):
                        # cv2.putText(image, str(round(center_distance,2)) ,(cent),cv2.FONT_HERSHEY_SIMPLEX, 1,(0,0,255),2,cv2.LINE_AA)
                        # list=[distance,x,y]
                        # pub.publish(data=list)
                        # print("Distance to Camera at ({}, {}) (index finger tip): {:1.3} m".format(x, y, distance), end="\r")
                        for i in range (0,len(points)):
                            x,y=points[i]
                            # print(i)
                            cv2.putText(image, str(round(d[i],3)) ,(points[i]),cv2.FONT_HERSHEY_SIMPLEX, 1,(0,0,255),2,cv2.LINE_AA)
                        
                        p = pts.reshape((-1,1,2))
                        cv2.polylines(image,[p],True,(0,255,255))    
                        # if x>start[0] and x>end[0] and y<start[1] and y<end[1]:
                        #     cv2.putText(image, 'top right' ,start,cv2.FONT_HERSHEY_SIMPLEX, 1,(0,0,255),2,cv2.LINE_AA)
                        # elif x>start[0] and x>end[0] and y>start[1] and y>end[1]:
                        #     cv2.putText(image, 'bottom right' ,start,cv2.FONT_HERSHEY_SIMPLEX, 1,(0,0,255),2,cv2.LINE_AA)
                        # elif x<start[0] and x<end[0] and y>start[1] and y>end[1]:
                        #     cv2.putText(image, 'bottom left' ,start,cv2.FONT_HERSHEY_SIMPLEX, 1,(0,0,255),2,cv2.LINE_AA) 
                        # elif x<start[0] and x<end[0] and y<start[1] and y<end[1]:
                        #     cv2.putText(image, 'top left' ,start,cv2.FONT_HERSHEY_SIMPLEX, 1,(0,0,255),2,cv2.LINE_AA)           
                        # elif x>start[0] and x>end[0]:
                        #     cv2.putText(image, 'right' ,start,cv2.FONT_HERSHEY_SIMPLEX, 1,(0,0,255),2,cv2.LINE_AA)
                        # elif x<start[0] and x<end[0]:
                        #     cv2.putText(image, 'left' ,start,cv2.FONT_HERSHEY_SIMPLEX, 1,(0,0,255),2,cv2.LINE_AA)
                        # elif y<start[1] and y<end[1]:
                        #     cv2.putText(image, 'up' ,start,cv2.FONT_HERSHEY_SIMPLEX, 1,(0,0,255),2,cv2.LINE_AA)
                        # elif y>start[1] and y>end[1]:
                        #     cv2.putText(image, 'down' ,start,cv2.FONT_HERSHEY_SIMPLEX, 1,(0,0,255),2,cv2.LINE_AA)
                        # else:  
                        #     cv2.putText(image, 'center' ,start,cv2.FONT_HERSHEY_SIMPLEX, 1,(0,0,255),2,cv2.LINE_AA)  
                            
                        # Increment the loop
                        
                    # else:
                    #     print("Can't estimate distance at this position.")
                    #     print("Your camera is probably too close to the scene, please move it backwards.\n")
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
    rate = rospy.Rate(100)    
    main()
    
    # rospy.spin()