
import cv2
import os
import numpy as np
import math
import time
import mediapipe as mp
class HandDetector:
    """
    Finds Hands using the mediapipe library. Exports the landmarks
    in pixel format. Adds extra functionalities like finding how
    many fingers are up or the distance between two fingers. Also
    provides bounding box info of the hand found.
    """

    def __init__(self, mode=False, maxHands=2, detectionCon=0.5, minTrackCon=0.5):
        """
        :param mode: In static mode, detection is done on each image: slower
        :param maxHands: Maximum number of hands to detect
        :param detectionCon: Minimum Detection Confidence Threshold
        :param minTrackCon: Minimum Tracking Confidence Threshold
        """
        self.mode = mode
        self.maxHands = maxHands
        self.detectionCon = detectionCon
        self.minTrackCon = minTrackCon

        self.mpHands = mp.solutions.hands
        self.hands = self.mpHands.Hands(static_image_mode=self.mode, max_num_hands=self.maxHands,
                                        min_detection_confidence=self.detectionCon,
                                        min_tracking_confidence=self.minTrackCon)
        self.mpDraw = mp.solutions.drawing_utils
        self.tipIds = [4, 8, 12, 16, 20]
        self.fingers = []
        self.lmList = []

    def findHands(self,img,blk,draw=True,flipType=True):
        """
        Finds hands in a BGR image.
        :param img: Image to find the hands in.
        :param blk: Flag to draw the output on the black image.
        :param draw: Flag to draw the output on the image.
        :return: Images with or without drawings
        """
        # imgshape=img.shape
        blk_img = np.zeros((img.shape), dtype = np.uint8)
        imgRGB = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        self.results = self.hands.process(imgRGB)
        allHands = []
        h, w, c = img.shape
        if self.results.multi_hand_landmarks:
            for handType, handLms in zip(self.results.multi_handedness, self.results.multi_hand_landmarks):
                myHand = {}
                ## lmList
                mylmList = []
                xList = []
                yList = []
                for id, lm in enumerate(handLms.landmark):
                    px, py, pz = int(lm.x * w), int(lm.y * h), int(lm.z * w)
                    # mylmList.append([px, py, pz])
                    
                    mylmList.append([lm.x, lm.y, lm.z])
                    xList.append(px)
                    yList.append(py)

                ## bbox
                xmin, xmax = min(xList), max(xList)
                ymin, ymax = min(yList), max(yList)
                boxW, boxH = xmax - xmin, ymax - ymin
                bbox = xmin, ymin, boxW, boxH
                cx, cy = bbox[0] + (bbox[2] // 2), \
                         bbox[1] + (bbox[3] // 2)

                myHand["lmList"] = mylmList
                myHand["bbox"] = bbox
                myHand["center"] = (cx, cy)

                if flipType:
                    if handType.classification[0].label == "Right":
                        myHand["type"] = "Left"
                    else:
                        myHand["type"] = "Right"
                else:
                    myHand["type"] = handType.classification[0].label
                allHands.append(myHand)

                ## draw
                if draw==True and blk==True:
                    
                    self.mpDraw.draw_landmarks(img, handLms,
                                               self.mpHands.HAND_CONNECTIONS)
                    cv2.rectangle(img, (bbox[0] - 20, bbox[1] - 20),
                                  (bbox[0] + bbox[2] + 20, bbox[1] + bbox[3] + 20),
                                  (255, 0, 255), 2)
                    cv2.putText(img, myHand["type"], (bbox[0] - 20, bbox[1] - 30), cv2.FONT_HERSHEY_PLAIN,
                                2, (255, 0, 255), 2)
                    if blk==True:
                        self.mpDraw.draw_landmarks(blk_img, handLms,
                                               self.mpHands.HAND_CONNECTIONS)
                                               
                   
                if blk==False and draw==True:
                    self.mpDraw.draw_landmarks(img, handLms,
                                            self.mpHands.HAND_CONNECTIONS)
                    cv2.putText(img, myHand["type"], (bbox[0] - 20, bbox[1] - 30), cv2.FONT_HERSHEY_PLAIN,
                            2, (255, 0, 255), 2)    
        # cv2.imshow("black Image", blk_img)           
        if draw==True and blk==False:
            # print("here1")
            return allHands, img
        
        elif draw==True and blk==True:
            # print("here2")
            return allHands,img,blk_img
        else:
            return allHands

   

    def findDistance(self, p1, p2, img=None):
        """
        Find the distance between two landmarks based on their
        index numbers.
        :param p1: Point1
        :param p2: Point2
        :param img: Image to draw on.
        :param draw: Flag to draw the output on the image.
        :return: Distance between the points
                 Image with output drawn
                 Line information
        """

        x1, y1,z1= p1
        x2, y2,z2 = p2
        cx, cy = (x1 + x2) // 2, (y1 + y2) // 2
        length = math.hypot(x2 - x1, y2 - y1)
        info = (x1, y1, x2, y2, cx, cy)
        if img is not None:
            cv2.circle(img, (x1, y1), 5, (255, 0, 255), cv2.FILLED)
            cv2.circle(img, (x2, y2), 5, (255, 0, 255), cv2.FILLED)
            cv2.line(img, (x1, y1), (x2, y2), (255, 0, 255), 3)
            cv2.circle(img, (cx, cy), 5, (255, 0, 255), cv2.FILLED)
            return length, info, img
        else:
            return length, info
    def calculate_distance(self,landmark1, landmark2):
            """
            Calculates the Euclidean distance between two landmarks.
            :param landmark1: First landmark in the format [x1, y1, z1]
            :param landmark2: Second landmark in the format [x2, y2, z2]
            :return: Distance between the landmarks
            """
            x1, y1, z1 = landmark1
            x2, y2, z2 = landmark2

            distance = math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2 + (z2 - z1) ** 2)
            return distance
def main():
    folder = "/home/panda/model_data/cnn_data"
    # folder_orignal = "/home/rehan/catkin_ws/src/panda_research/hand_classifier/data/cnn_data"
    class_name="rotate_cw"
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
