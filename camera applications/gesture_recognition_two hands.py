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


def centroid(vertexes):
     _x_list = [vertex [0] for vertex in vertexes]
     _y_list = [vertex [1] for vertex in vertexes]
     _len = len(vertexes)
   
     _x =int( sum(_x_list) / _len)
     _y = int(sum(_y_list) / _len)
     return(_x, _y)    
    
def main():
    index=0
   
    list=[0,0,0]
   
    # initialise the media pipeline model
    mp_drawing = mp.solutions.drawing_utils
    mp_drawing_styles = mp.solutions.drawing_styles
    mp_hands = mp.solutions.hands
    hands= mp_hands.Hands(static_image_mode=False,model_complexity=0,max_num_hands=2,min_detection_confidence=0.7,min_tracking_confidence=0.5) 
    BaseOptions = mp.tasks.BaseOptions
    GestureRecognizer = mp.tasks.vision.GestureRecognizer
    GestureRecognizerOptions = mp.tasks.vision.GestureRecognizerOptions
    GestureRecognizerResult = mp.tasks.vision.GestureRecognizerResult
    VisionRunningMode = mp.tasks.vision.RunningMode

    options = GestureRecognizerOptions(
    base_options=BaseOptions(model_asset_path='/home/panda/test_ws/src/panda_repository/panda_research/camera applications/gesture_recognizer.task'),
    running_mode=VisionRunningMode.IMAGE,num_hands=2, )

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
   
   
    with GestureRecognizer.create_from_options(options) as recognizer:
        while cam.open():
            err = cam.grab(runtime)
            if (err == sl.ERROR_CODE.SUCCESS) :
                cam.retrieve_image(mat, sl.VIEW.LEFT)
                image=mat.get_data()
                # image= cv2.flip(image, 1)
                # Retrieve depth map. Depth is aligned on the left image
                cam.retrieve_measure(depth, sl.MEASURE.DEPTH)
                # Retrieve colored point cloud. Point cloud is aligned on the left image.
                tracking_state = cam.get_position(pose) # Get the tracking state of the camera
                cam.retrieve_measure(point_cloud, sl.MEASURE.XYZRGBA)
                image.flags.writeable = False
                image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
                results = hands.process(image)
                mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=image)
                gesture_results = recognizer.recognize(mp_image)
                image.flags.writeable = True
                image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
                # image = cv2.rectangle(image, start, end, (255,0,0), 2)
                

                if results.multi_hand_landmarks:
                
                    
                    # print(results)
                        # print(handedness_dict["classification"])
                    # if lbl=="Left" and score > 0.6: 
                        # print("label =",lbl,"score = ",score)   
                    for idx, hand_landmarks in enumerate(results.multi_hand_landmarks):
                        # print(idx)
                        mp_drawing.draw_landmarks(image,hand_landmarks,mp_hands.HAND_CONNECTIONS,mp_drawing_styles.get_default_hand_landmarks_style(),mp_drawing_styles.get_default_hand_connections_style())
                        lbl = str(results.multi_handedness[idx].classification[0].label)
                        score=float(results.multi_handedness[idx].classification[0].score)
                        # print(lbl)
                        if lbl=="Left" and score > 0.6: 
                            # print("label =",lbl,"score = ",score)      
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
                            
                            if dist_1<dist_2:
                                # print("open")
                                st="close"
                            elif dist_2>dist_3:
                                # print("open")
                                st="slow"   
                            else:
                                # print("closed") 
                                st="open"   
                            
                            
                            a,b=cent
                            
                            if a>0 and b >0:
                                find_plane_status = cam.find_plane_at_hit(cent, plane)
                            if find_plane_status == sl.ERROR_CODE.SUCCESS:
                                # cv2.circle(image,cent,radius=3,color=(0,255,255),thickness=10)
                                plane_pose=plane.get_pose()
                                translation=plane_pose.get_translation().get()
                                orient=plane_pose.get_orientation().get()
                                # print(plane_pose)

                                marker_pose.pose.position.x =round(translation[0], 3)
                                marker_pose.pose.position.y =round(translation[1], 3)
                                marker_pose.pose.position.z =round(translation[2], 3)
                                marker_pose.pose.orientation.x =round(orient[0], 3)
                                marker_pose.pose.orientation.y =round(orient[1], 3)
                                marker_pose.pose.orientation.z =round(orient[2], 3)
                                marker_pose.pose.orientation.w =round(orient[3], 3)
                                marker_pose.header.frame_id = st
                                marker_pose.header.seq = index
                                marker_pose.header.stamp = rospy.get_rostime()
                                pose_pub.publish(marker_pose)
                                # print(marker_pose)
                                # string_pub.publish("string")
                                # print(marker_pose)
                                rate.sleep()
                                    
                                
                                # euler = plane_pose.get_euler_angles(radian=False)
                                index+=1 
                    g=[]
                    
                    i=0
                    for idx,gestures in enumerate(gesture_results.gestures):
                        # print(idx)
                        gesture=gesture_results.gestures[idx][0].category_name
                        print(gesture)
                        hand=gesture_results.handedness[idx][0].category_name
                        if hand=="Left":
                            g=np.append(g,'Right')
                            g=np.append(g,gesture)
                        else:    
                            g=np.append(g,'Left')
                            g=np.append(g,gesture)
                                
                        
                        
                            
                            # print("here",hand)
                            # i+=1
                            # cv2.putText(image, gestures, (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 
                            # 1, (0,0,255), 2, cv2.LINE_AA)
                        # print(g)
                        
                        cv2.putText(image, str(g), (10, 50), cv2.FONT_HERSHEY_SIMPLEX,1, (0,0,255), 2, cv2.LINE_AA)
                    
                        
                                
                    
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