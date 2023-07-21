
import sys
import ogl_viewer.viewer as gl
import pyzed.sl as sl
import cv2
import numpy as np
from detection import HandDetector
import tensorflow as tf
import mediapipe as mp
import rospy
from geometry_msgs.msg import PointStamped
def check_depth(depth,point) :
   
    dist=0
    
    x,y=point
    neighbors=[[x,y],[x-1,y-1],[x-1,y],[x-1,y+1],[x,y-1],[x,y+1],[x+1,y+1],[x+1,y],[x+1,y-1]]
    for j in range(0,len(neighbors)):
        if neighbors[j][0]>=0 and neighbors[j][1]>=0:
            e,dist = depth.get_value(neighbors[j][0],neighbors[j][1])
        
            if not np.isnan(dist) and not np.isinf(dist) and dist>=0  :          
                break 
    if not np.isnan(dist) and not np.isinf(dist) and dist>=0  :          
            dist=dist
    else:
                dist=0
                          
    return dist   

def main():    
    # Create a Camera object
    object_length=0.8 #meters
    work_space=2 #meters
    p = PointStamped()
    zed = sl.Camera()
    hand_detector=HandDetector(maxHands=1,detectionCon=0.5)
    # Create a InitParameters object and set configuration parameters
    init_params = sl.InitParameters()
    init_params.camera_resolution = sl.RESOLUTION.HD1080  # Use HD1080 video mode    
    init_params.coordinate_units = sl.UNIT.METER
    init_params.camera_fps = 30                          # Set fps at 30
    init_params.coordinate_system = sl.COORDINATE_SYSTEM.RIGHT_HANDED_Z_UP_X_FWD
    init_params.depth_minimum_distance = 0.1 # Set the minimum depth perception distance to 10cm

    # If applicable, use the SVO given as parameter
    # Otherwise use ZED live stream
    # if len(sys.argv) == 2:
    #     filepath = sys.argv[1]
    #     print("Using SVO file: {0}".format(filepath))
    #     init_params.set_from_svo_file(filepath)

    # Set runtime parameters
    runtime_parameters = sl.RuntimeParameters()

    # Open the camera
    err = zed.open(init_params)
    if err != sl.ERROR_CODE.SUCCESS:
        exit(1)

    # Enable object detection module
    obj_param = sl.ObjectDetectionParameters()
    # Defines if the object detection will track objects across images flow.
    obj_param.enable_tracking = True       # if True, enable positional tracking

    if obj_param.enable_tracking:
        zed.enable_positional_tracking()
        
    zed.enable_object_detection(obj_param)

    camera_info = zed.get_camera_information()
    # Create OpenGL viewer
    # viewer = gl.GLViewer()
    width=camera_info.camera_resolution.width
    height=camera_info.camera_resolution.height
    print("resolution = ",width,height)
    # viewer.init(camera_info.calibration_parameters.left_cam, obj_param.enable_tracking)

    # Configure object detection runtime parameters
    obj_runtime_param = sl.ObjectDetectionRuntimeParameters()
    obj_runtime_param.detection_confidence_threshold = 60
    obj_runtime_param.object_class_filter = [sl.OBJECT_CLASS.PERSON]    # Only detect Persons

    # Create ZED objects filled in the main loop
    objects = sl.Objects()
    image = sl.Mat()
    depth=sl.Mat()
    while zed.open():
        # Grab an image, a RuntimeParameters object must be given to grab()
        if zed.grab(runtime_parameters) == sl.ERROR_CODE.SUCCESS:
            # Retrieve left image
            zed.retrieve_image(image, sl.VIEW.LEFT)
            img=image.get_data()
            zed.retrieve_measure(depth, sl.MEASURE.DEPTH)
            # img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
            # Retrieve objects
            zed.retrieve_objects(objects, obj_runtime_param)
            # Update GL view
            # viewer.update_view(image, objects)
           
            # img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
            hands, img = hand_detector.findHands(img, blk=False) 
            position="none"
            # img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
            for object in objects.object_list:
                print(len(objects.object_list))
                position="none"
                object_position=object.position
                print("object position {} object id {} ".format(object_position,object.id))
                print(object.id)
                if object_position[0]<work_space :
                    if object_position[1] >0 and object_position[0]<=object_length:
                        position="left"
                    elif object_position[1] <0 and object_position[0]<=object_length:
                        position="right"
                    elif object_position[0]>object_length:
                        position="center"
                    p.header.frame_id=position    
                    bbox= object.bounding_box_2d
                    bbox = np.asarray(bbox, dtype = 'int')
                    cv2.rectangle(img, (int(bbox[0][0]),int(bbox[0][1])),(int(bbox[2][0]),int(bbox[2][1])),
                                            (255, 0, 255), 5)  

                    # cv2.putText(img, position, ((bbox[0][0]) + 20, bbox[0][1] - 30), cv2.FONT_HERSHEY_PLAIN,
                                        # 2, (255, 0, 255), 2) 
                    cv2.putText(img, position, (500,200), cv2.FONT_HERSHEY_PLAIN,
                                        2, (255, 0, 255), 2)
                else:
                     position ="none"   
            for hand in hands:
                lm=hand["lmList"]
                # print(lm[0][0]*width,lm[0][1]*height)
                # center=hand["center"]
                if (hand["type"]=="Left" and position=="right") or (hand["type"]=="Right" and position=="left"):
                    center=[lm[17][0]*width,lm[17][1]*height]
                    # print(center)
                    distance =round(check_depth(depth,center),3)
                    # print("radius = ",distance) 
                    if distance >=object_length and position=="center":
                        distance =0
                    p.point.x=distance 
                    r=('radius' ,distance)
                    cv2.putText(img, str(r), ((bbox[0][0]) +50, bbox[0][1] +50), cv2.FONT_HERSHEY_PLAIN,
                                        2, (255, 0, 255), 2)   
                else:
                    p.point.x=0       
           
              
            pub.publish(p)     
            cv2.imshow('MediaPipe Pose', img)    
            sys.stdout.flush()
        
        if cv2.waitKey(30) >= 0 :
                break        
    # viewer.exit()
    

    image.free(memory_type=sl.MEM.CPU)
    # Disable modules and close camera
    zed.disable_object_detection()
    zed.disable_positional_tracking()

    zed.close()
if __name__ == "__main__":
    
    rospy.init_node('finger_cordinates')
    pub = rospy.Publisher("/position_publisher", PointStamped, queue_size=10)
    # rospy.init_node("my_equilibrium_pose_node")
    # pose_pub = rospy.Publisher(
    #     "my_equilibrium_pose", PoseStamped, queue_size=10)
    # string_pub =rospy.Publisher(
    #     "my_equilibrium_pose1", String, queue_size=10)
    rate = rospy.Rate(100)
    main()