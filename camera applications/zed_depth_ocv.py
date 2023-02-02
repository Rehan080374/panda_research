import numpy as np
import os
import configparser
import sys
import cv2
import wget
import mediapipe as mp
import pyzed.sl as sl
import math
def check_distance(point_cloud,x,y) :
    err,point_cloud_value = point_cloud.get_value(x, y)
    distance = math.sqrt(point_cloud_value[0] * point_cloud_value[0] +
                                    point_cloud_value[1] * point_cloud_value[1] +
                                    point_cloud_value[2] * point_cloud_value[2])
    return distance

def main():
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
    #plane tracking
    pose = sl.Pose() 
    plane = sl.Plane() # Structure that stores the estimated plane
    
    cam.enable_positional_tracking()
    while cam.open():
        err = cam.grab(runtime)
        if (err == sl.ERROR_CODE.SUCCESS) :
            
            tracking_state = cam.get_position(pose) # Get the tracking state of the camera
                

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
            
            if results.multi_hand_landmarks:
                
                for hand_landmarks in results.multi_hand_landmarks:
                    mp_drawing.draw_landmarks(image,hand_landmarks,mp_hands.HAND_CONNECTIONS,mp_drawing_styles.get_default_hand_landmarks_style(),mp_drawing_styles.get_default_hand_connections_style())
                    #print(f'Index finger tip coordinate: (',f'{hand_landmarks.landmark[mp_hands.HandLandmark.INDEX_FINGER_TIP].x * image_size.width}, 'f'{hand_landmarks.landmark[mp_hands.HandLandmark.INDEX_FINGER_TIP].y * image_size.height})')
                    x=int(hand_landmarks.landmark[mp_hands.HandLandmark.INDEX_FINGER_MCP].x * image_width)
                    y=int(hand_landmarks.landmark[mp_hands.HandLandmark.INDEX_FINGER_MCP].y * image_height)
                    # print(f'Index finger tip coordinate: (',f'{x},'f'{y})')

                    coord = (x,y)
                    
                    px=int(hand_landmarks.landmark[mp_hands.HandLandmark.PINKY_MCP].x * image_width)
                    py=int(hand_landmarks.landmark[mp_hands.HandLandmark.PINKY_MCP].y * image_height)
                    if x >=0 and y>=0 and  px >=0 and py>=0 :
                        
                        # point_cloud_value2 = point_cloud.get_value(px, py)
                        distance1= check_distance(point_cloud,x,y)
                        distance2= check_distance(point_cloud,px,py)
                        # if not np.isnan(distance) and not np.isinf(distance):
                            # print("Distance to Camera at ({}, {}) (index finger tip): {:1.3} m".format(x, y, distance), end="\r")
                        cv2.putText(image, str(round(distance1,3)) ,(x,y),cv2.FONT_HERSHEY_SIMPLEX, 1,(0,0,255),2,cv2.LINE_AA)
                        cv2.putText(image, str(round(distance2,3)) ,(px,py),cv2.FONT_HERSHEY_SIMPLEX, 1,(0,0,255),2,cv2.LINE_AA) 
                            # Increment the loop
                        tracking_state = cam.get_position(pose) # Get the tracking state of the camera
                        if tracking_state == sl.POSITIONAL_TRACKING_STATE.OK :  
                        # Detect the plane passing by the depth value of pixel coord
                            find_plane_status = cam.find_plane_at_hit(coord, plane)
                            normal = plane.get_normal() # Get the normal vector of the detected plane
                            plane_equation = plane.get_plane_equation() # Get (a,b,c,d) where ax+by+cz=d
                            print("plane = ",plane_equation,normal)    
                        
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
    main()
