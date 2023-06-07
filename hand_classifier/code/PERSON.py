import cv2
import numpy as np
import mediapipe as mp
from detection import HandDetector
import tensorflow as tf
import pyzed.sl as sl
import sys
import ogl_viewer.viewer as gl
tf.config.set_visible_devices([], 'GPU')
def main():

    cam = sl.Camera()
    # Create a InitParameters object and set configuration parameters
    init_params = sl.InitParameters()
    init_params.camera_resolution = sl.RESOLUTION.HD1080 
    init_params.camera_fps = 30  # Set fps at 30
    init_params.depth_mode = sl.DEPTH_MODE.ULTRA
    init_params.coordinate_units=sl.UNIT.METER
    init_params.coordinate_system=sl.COORDINATE_SYSTEM.RIGHT_HANDED_Z_UP_X_FWD
    init_params.depth_minimum_distance = 0.1 # Set the minimum depth perception distance to 15cm
     # Set runtime parameters
    runtime_parameters = sl.RuntimeParameters()
    # Open the camera
    err = cam.open(init_params)
    
    err = cam.open(init_params)
    if err != sl.ERROR_CODE.SUCCESS:
        print("camera not connected properly")
        exit(1)
    # Enable object detection module
    obj_param = sl.ObjectDetectionParameters()
    # Defines if the object detection will track objects across images flow.
    obj_param.enable_tracking = True       # if True, enable positional tracking
    if obj_param.enable_tracking:
        cam.enable_positional_tracking()
        
    cam.enable_object_detection(obj_param)
        # Configure object detection runtime parameters
    obj_runtime_param = sl.ObjectDetectionRuntimeParameters()
    obj_runtime_param.detection_confidence_threshold = 60
    obj_runtime_param.object_class_filter = [sl.OBJECT_CLASS.PERSON]    # Only detect Persons

    objects = sl.Objects()
    # image = sl.Mat()
    mat = sl.Mat()
    depth = sl.Mat()
    point_cloud = sl.Mat()
    # plane = sl.Plane()
    pose_camera=sl.Pose()
    image_width = cam.get_camera_information().camera_resolution.width
    image_height = cam.get_camera_information().camera_resolution.height
    print(image_width,image_height)
    hand_pose=HandDetector(maxHands=1,detectionCon=0.5)
    # cap = cv2.VideoCapture(0)
    while cam.open():
        err = cam.grab(runtime_parameters)
        # success, img = cap.read()
        if (err != sl.ERROR_CODE.SUCCESS) :
            break
        # cv2.imshow('MediaPipe Pose1', img)
        cam.retrieve_image(mat, sl.VIEW.LEFT)
        # Retrieve depth map. Depth is aligned on the left image
        cam.retrieve_measure(depth, sl.MEASURE.DEPTH)
        # Retrieve objects
        cam.retrieve_objects(objects, obj_runtime_param)
        # Retrieve colored point cloud. Point cloud is aligned on the left image.
        tracking_state = cam.get_position(pose_camera) # Get the tracking state of the camera
        cam.retrieve_measure(point_cloud, sl.MEASURE.XYZRGBA)
        img=mat.get_data()
        img.flags.writeable = False
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        
        h,w,c=img.shape
        center=[int(w/2),int(h/2)]
        # image_hands=img.copy()
        # pose, img = hand_pose.findPose(img, blk=False)
       
      
        hands, img = hand_pose.findHands(img, blk=False) 
        # img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        
        # left_hand = None
        # right_hand = None
       
        
           
        # if len(pose) !=0  :
        #     p1=[pose["pose_bbox"][0],pose["pose_bbox"][1],0]
        #     p2=[center[0],center[1],0]
        #     p3=[pose["pose_bbox"][0]+pose["pose_bbox"][2],pose["pose_bbox"][1],0]
            
        #     print("point=",p1,p2,p3,w)
        #     if   p1[0]>0 and p3[0] >=w  :
        #         print("right side")
        #     elif  p1[0]<0 and p3[0] <w  :
        #         print("left side")
            
        #     elif p1[0] >0 and p3[0]< w :
        #         print("center")
            
        cv2.imshow('MediaPipe hands', img)
        # print("distance=",distance,info)
        if cv2.waitKey(5) & 0xFF == 27:
            break
    # image.free(memory_type=sl.MEM.CPU)
    # Disable modules and close camera
    cam.disable_object_detection()
    cam.disable_positional_tracking()
 
    cam.close()  
    
if __name__ == "__main1__":
    main()
