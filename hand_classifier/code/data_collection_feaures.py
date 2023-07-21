
import cv2
import os
import numpy as np
import math
import time
import mediapipe as mp
from detection import HandDetector

def main():
    folder = "/home/panda/model_data/cnn_data"
    # folder_orignal = "/home/rehan/catkin_ws/src/panda_research/hand_classifier/data/cnn_data"
    class_name="fast"
    iteration = 0 
    limit =2000
    start_time=None
    recorded_array=None

    # Create a HandDetector object
    handDetector = HandDetector()

    # Open a video capture object
    cap = cv2.VideoCapture(0)

    # Initialize variables to store left and right hand landmarks
    
    while True:
        
        left_hand_landmarks = []
        left_hand_distances = []
        right_hand_landmarks = []
        right_hand_distances = []
        combined_array=[]
        # Read the video frame
        success, img = cap.read()
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
            # Collect landmarks and calculate distances for right handq
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
            if left_hand_landmarks and left_hand_distances and right_hand_landmarks and right_hand_distances and start_time is not None and time.time() >= start_time and iteration<=limit:
                
                # Convert the collected landmarks and distances to flattened NumPy arrays
                    left_hand_landmarks = np.array(left_hand_landmarks).flatten()
                    left_hand_distances = np.array(left_hand_distances)
                    right_hand_landmarks = np.array(right_hand_landmarks).flatten()
                    right_hand_distances = np.array(right_hand_distances)

                    # Concatenate the arrays in the specified order
                    combined_array = np.concatenate((left_hand_landmarks, left_hand_distances,
                                                    right_hand_landmarks, right_hand_distances))
                    if len(combined_array)==210:

                        # Generate a new file name for each iteration
                        file_name = os.path.join(folder,class_name, f"{class_name}_feature_{iteration}.npy")

                        # Save the recorded array to the new file using np.save()
                        np.save(file_name, combined_array)
                        print(f"Recorded array has been saved to {file_name}")
                        folder1=os.path.join(folder,class_name)
                        cv2.imwrite(f'{folder1}/image_{class_name}_{iteration}.jpg',image_orignal) 
                    print("Recorded Array:", len(combined_array))
                    # print("Recorded Array:", (combined_array.shape))
                    # Increment the iteration counter
                    iteration += 1

                    # Clear the lists for the next recording
                    left_hand_landmarks = []
                    left_hand_distances = []
                    right_hand_landmarks = []
                    right_hand_distances = []
                    combined_array=[]


            
              
    
        # Display the frame
        
        if cv2.waitKey(1) & 0xFF == ord('s'):
                start_time = time.time() + 10  # Set the start time with a 10-second delay
                cv2.putText(img,"data recording will start in 10 seconds", (50,50), cv2.FONT_HERSHEY_PLAIN,
                                2, (255, 0, 255), 2)
                print("data recording will start in 10 seconds")
        # Break the loop if 'q' is pressed
        if cv2.waitKey(1) & 0xFF == ord('q') or iteration>=limit:
            cv2.destroyAllWindows()
            break
        cv2.imshow("Frame", img)
        # cv2.imshow("orignal", image_orignal)
    cv2.destroyAllWindows()    
if __name__ == "__main__":
    main()
