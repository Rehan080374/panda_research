
import cv2
import os
import numpy as np
import math
import time
import mediapipe as mp
import pickle
import tensorflow as tf
import rospy
from std_msgs.msg import String
import psutil
from detection import HandDetector
import pyttsx3
tf.config.set_visible_devices([], 'GPU')
from keras.models import load_model
def is_roscore_running():
    for proc in psutil.process_iter(['name']):
        if proc.info['name'] == 'roscore':
            return True
    return False
def is_master_running():
    try:
        rospy.get_master().getUri()
        return True
    except rospy.ROSException:
        return False


def main():
    
    # rospy.init_node("gesture_node")
    # gesture_pub = rospy.Publisher(
    #     "gesture_pose", String, queue_size=10)
    folder = "/home/panda/model_data/model4/"
    # folder1=os.path.join(folder,class_name)

    model_path=os.path.join(folder,"keras_model.h5")
    label_path=os.path.join(folder,"labels.txt")
    scaler_file = os.path.join(folder,"scaler.pkl")
    normalizer_file= os.path.join(folder,"normalizer.pkl")
    # folder_orignal = "/home/rehan/catkin_ws/src/panda_research/hand_classifier/data/cnn_data"
    # class_name="rotate_ccw"
    # iteration = 0 
    # limit =2000
    # start_time=None
    # recorded_array=None
    # labelp=None
    model=load_model(model_path)
    with open(label_path, 'r') as f:
        labels = f.read().splitlines()
    # Loading the scaler and normalizer objects
    with open(scaler_file, 'rb') as scaler_file  :
        scaler = pickle.load(scaler_file)

    with open(normalizer_file, 'rb') as normalizer_file:
        normalizer = pickle.load(normalizer_file)    


    # Create a HandDetector object
    handDetector = HandDetector()

    # Open a video capture object
    # if it throws img.shape is none check camera number
    # cap = cv2.VideoCapture(0)
    capture = cv2.VideoCapture(0, cv2.CAP_V4L2)
    capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
    width = 1280
    height = 720
    capture.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    capture.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    # Initialize variables to store left and right hand landmarks
    
    left_hand_landmarks = []
    left_hand_distances = []
    right_hand_landmarks = []
    right_hand_distances = []
    combined_array=[]
    while True:
        
        # Read the video frame
        success, img = capture.read()
        image_orignal=img.copy()
        
        # Find hands in the frame
        hands, img, blk_img = handDetector.findHands(img, blk=True)

        # Check if both hands are visible
        if len(hands) >= 2:
            left_hand = None
            right_hand = None

            # Process the detected hands
            for hand in hands:
                hand_type = hand["type"]
                if hand_type == "Left" and left_hand is None:
                    left_hand = hand
                    left_hand_landmarks.append(left_hand["lmList"])
                elif hand_type == "Right" and right_hand is None:
                    right_hand = hand
                    right_hand_landmarks.append(right_hand["lmList"])

            # Collect landmarks and calculate distances for left hand
            if left_hand is not None:
                
                for landmark in left_hand["lmList"] :
                    try:

                        length = handDetector.calculate_distance(left_hand["lmList"][0], landmark)
                        left_hand_distances.append(length)
                        if right_hand is not None:
                            length_with_right = handDetector.calculate_distance(right_hand["lmList"][0], landmark)
                            left_hand_distances.append(length_with_right)
                    except Exception as e:
                        print(e)
            # Collect landmarks and calculate distances for right hand
            if right_hand is not None:
                
                for landmark in right_hand["lmList"]:
                    try:
                        length = handDetector.calculate_distance(right_hand["lmList"][0], landmark)
                        right_hand_distances.append(length)
                        if left_hand is not None:
                            length_with_left = handDetector.calculate_distance(left_hand["lmList"][0], landmark)
                            right_hand_distances.append(length_with_left)
                            
                    except Exception as e:
                        print(e)    

        
            
            # print("iteration = ", loop_count,"   ",len(right_hand_distances))
            # Check if the recording should be started based on the delay
            if left_hand_landmarks and left_hand_distances and right_hand_landmarks and right_hand_distances  is not None :
                
                # Convert the collected landmarks and distances to flattened NumPy arrays
                    left_hand_landmarks = np.array(left_hand_landmarks).flatten()
                    left_hand_distances = np.array(left_hand_distances)
                    right_hand_landmarks = np.array(right_hand_landmarks).flatten()
                    right_hand_distances = np.array(right_hand_distances)

                    # Concatenate the arrays in the specified order
                    combined_array = np.concatenate((left_hand_landmarks, left_hand_distances,
                                                    right_hand_landmarks, right_hand_distances))
                    combined_array=(combined_array).flatten()
                    print(combined_array.shape)
                    if len(combined_array)==210:
                        # data_normalized = normalizer.transform(scaler.transform(combined_array))
                        # Generate a new file name for each iteration
                        # input_data = np.reshape(combined_array, (combined_array.shape[0], ))
                        # predictions = np.around(model.predict(input_data))
                        # Single input array for testing
                        test_input = np.random.random((210,))

                        # Reshape the input array to have shape (1, 210)
                        test_input = combined_array.reshape((1, 210))
                        data_normalized = normalizer.transform(scaler.transform(test_input))
                        # Make predictions using the model
                        predictions = model.predict(data_normalized)
                        label =labels[np.argmax(predictions)]
                        cv2.putText(blk_img,label, (50,50), cv2.FONT_HERSHEY_PLAIN,
                                2, (255, 100, 0), 2)
                        cv2.putText(img,label, (50,50), cv2.FONT_HERSHEY_PLAIN,
                                2, (255, 100, 0), 2)
                        
                        gesture_pub.publish(label)
                        
                        # if labelp is not None and labelp != label:
                        #     # text_to_speech(label)
                        #     # engine.say(label)
    
                        #     # engine.runAndWait()
                        # labelp=label
                    # print("Recorded Array:", (combined_array.shape))
                    # Increment the iteration counterc
                    
                    # Clear the lists for the next recording
                    left_hand_landmarks = []
                    left_hand_distances = []
                    right_hand_landmarks = []
                    right_hand_distances = []
                    combined_array=[]


            
              
    
      
        if cv2.waitKey(30) >= 0 :
                break  
        cv2.imshow("Frame", img)
        # cv2.imshow("orignal", image_orignal)
    cv2.destroyAllWindows()    
if __name__ == "__main__":
    rospy.init_node("gesture_node")
    gesture_pub = rospy.Publisher(
        "gesture_pose", String, queue_size=10)
    main()
