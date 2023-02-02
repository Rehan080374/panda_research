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

def main():
    # rospy.init_node('finger_cordinates')
    # finger_cordinates()
    list=[0,0,0]
   
    # initialise the media pipeline model
    mp_drawing = mp.solutions.drawing_utils
    mp_drawing_styles = mp.solutions.drawing_styles
    mp_hands = mp.solutions.hands
    hands= mp_hands.Hands(static_image_mode=False,model_complexity=0,min_detection_confidence=0.5,min_tracking_confidence=0.5) 
    

    # Create a InitParameters object and set configuration parameters
    init_params = sl.InitParameters()
    init_params.camera_resolution = sl.RESOLUTION.HD720  # Use HD1080 video mode
    init_params.camera_fps = 30  # Set fps at 30
    init_params.depth_mode = sl.DEPTH_MODE.ULTRA
    init_params.coordinate_units=sl.UNIT.METER
    init_params.coordinate_system=sl.COORDINATE_SYSTEM.RIGHT_HANDED_Y_UP
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
   
    image_width = cam.get_camera_information().camera_resolution.width
    image_height = cam.get_camera_information().camera_resolution.height
    print("image resolution '{0}'x'{1}' ".format(image_width,image_height))
    square_width=[100,100]
    center=[image_width/2,image_height/2]
    start=[int(center[0]-square_width[0]/2),int(center[1]-square_width[1]/2)]
    end=[int(center[0]+square_width[0]/2),int(center[1]+square_width[1]/2)]
    print(start,end)
    while cam.open():
        err = cam.grab(runtime)
        if (err == sl.ERROR_CODE.SUCCESS) :
            cam.retrieve_image(mat, sl.VIEW.LEFT)
            image=mat.get_data()
            # Retrieve depth map. Depth is aligned on the left image
            cam.retrieve_measure(depth, sl.MEASURE.DEPTH)
            # Retrieve colored point cloud. Point cloud is aligned on the left image.
            cam.retrieve_measure(point_cloud, sl.MEASURE.XYZRGBA)
            image.flags.writeable = False
            image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
            results = hands.process(image)

            # Draw the hand annotations on the image.
            image.flags.writeable = True
            image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
            image = cv2.rectangle(image, start, end, (255,0,0), 2)
            if results.multi_hand_landmarks:
                
                for hand_landmarks in results.multi_hand_landmarks:
                    mp_drawing.draw_landmarks(image,hand_landmarks,mp_hands.HAND_CONNECTIONS,mp_drawing_styles.get_default_hand_landmarks_style(),mp_drawing_styles.get_default_hand_connections_style())
                    #print(f'Index finger tip coordinate: (',f'{hand_landmarks.landmark[mp_hands.HandLandmark.INDEX_FINGER_TIP].x * image_size.width}, 'f'{hand_landmarks.landmark[mp_hands.HandLandmark.INDEX_FINGER_TIP].y * image_size.height})')
                    x=int(hand_landmarks.landmark[mp_hands.HandLandmark.INDEX_FINGER_TIP].x * image_width)
                    y=int(hand_landmarks.landmark[mp_hands.HandLandmark.INDEX_FINGER_TIP].y * image_height)
                    # print(f'Index finger tip coordinate: (',f'{x},'f'{y})')
                    pts = np.array([[int(hand_landmarks.landmark[mp_hands.HandLandmark.INDEX_FINGER_MCP].x * image_width)
                    ,int(hand_landmarks.landmark[mp_hands.HandLandmark.INDEX_FINGER_MCP].y * image_height)],
                    [int(hand_landmarks.landmark[mp_hands.HandLandmark.THUMB_MCP].x * image_width),
                    int(hand_landmarks.landmark[mp_hands.HandLandmark.THUMB_MCP].y * image_height)],
                    [int(hand_landmarks.landmark[mp_hands.HandLandmark.WRIST].x * image_width),
                    int(hand_landmarks.landmark[mp_hands.HandLandmark.WRIST].y * image_height)],
                    [int(hand_landmarks.landmark[mp_hands.HandLandmark.PINKY_MCP].x * image_width),
                    int(hand_landmarks.landmark[mp_hands.HandLandmark.PINKY_MCP].y * image_height)]], np.int32)      
                    if x >=0 and y>=0:
                        err, point_cloud_value = point_cloud.get_value(x, y)
                        distance = math.sqrt(point_cloud_value[0] * point_cloud_value[0] +
                                    point_cloud_value[1] * point_cloud_value[1] +
                                    point_cloud_value[2] * point_cloud_value[2])
                            
                        if not np.isnan(distance) and not np.isinf(distance):
                            list=[distance,x,y]
                            pub.publish(data=list)
                            print("Distance to Camera at ({}, {}) (index finger tip): {:1.3} m".format(x, y, distance), end="\r")
                            cv2.putText(image, str(round(distance,2)) ,(x,y),cv2.FONT_HERSHEY_SIMPLEX, 1,(0,0,255),2,cv2.LINE_AA)
                            
                            pts = pts.reshape((-1,1,2))
                            cv2.polylines(image,[pts],True,(0,255,255))    
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
                        
                    else:
                        print("Can't estimate distance at this position.")
                        print("Your camera is probably too close to the scene, please move it backwards.\n")
                    sys.stdout.flush()           
                        
            # Flip the image horizontally for a selfie-view display.
            cv2.imshow('MediaPipe Hands', image)
            #cv2.imshow("left RECT", left_rect)
        
        if cv2.waitKey(30) >= 0 :
                break    
    
    cam.close()
    
    
if __name__ == "__main__":
    pub = rospy.Publisher("/finger_cordinates", Float32MultiArray, queue_size=10)
    rospy.init_node('finger_cordinates')
    main()
    rospy.spin()