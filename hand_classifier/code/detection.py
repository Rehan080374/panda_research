
import cv2
import numpy as np
import math
import mediapipe as mp
# import tensorflow as tf
# tf.config.set_visible_devices([], 'GPU')
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
        self.mpPose =  mp.solutions.pose
        self.pose = self.mpPose.Pose(static_image_mode=self.mode, min_detection_confidence=self.detectionCon,
                                            min_tracking_confidence=self.minTrackCon)
        
        self.mpHands = mp.solutions.hands
        self.hands = self.mpHands.Hands(static_image_mode=self.mode, max_num_hands=self.maxHands,
                                        min_detection_confidence=self.detectionCon,
                                        min_tracking_confidence=self.minTrackCon)
        self.mpDraw = mp.solutions.drawing_utils
        self.tipIds = [4, 8, 12, 16, 20]
        self.fingers = []
        self.lmList = []

    def findPose(self,img,blk,draw=True,flipType=True):
            """
            Finds hands in a BGR image.
            :param img: Image to find the hands in.
            :param blk: Flag to draw the output on the black image.
            :param draw: Flag to draw the output on the image.
            :return: Images with or without drawings
            """
            # imgshape=img.shape
            blk_img = np.zeros((img.shape), dtype = np.uint8)
            img.flags.writeable = True
            img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
            self.pose_results = self.pose.process(img)
            img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
            allPoses = []
            h, w, c = img.shape
            if self.pose_results.pose_landmarks:
                # for  poseLms in zip(self.pose_results.pose_landmarks):
                    # print(self.pose_results.pose_landmarks)
                    myPose = {}
                    # myPose=self.pose_results.pose_landmarks
                    ## lmList
                    myposeList = []
                    nlm_List=[]
                    xList = []
                    yList = []
                    for idx, lm in enumerate(self.pose_results.pose_landmarks.landmark):
                        # print(idx)
                        px, py, pz = int(lm.x * w), int(lm.y * h), int(lm.z * w)
                        myposeList.append([px, py, pz])
                        
                        nlm_List.append([lm.x, lm.y, lm.z])
                        xList.append(px)
                        yList.append(py)

                    ## bbox
                    xmin, xmax = min(xList), max(xList)
                    ymin, ymax = min(yList), max(yList)
                    boxW, boxH = xmax - xmin, ymax - ymin
                    pose_bbox = xmin, ymin, boxW, boxH
                    cx, cy = pose_bbox[0] + (pose_bbox[2] // 2), \
                            pose_bbox[1] + (pose_bbox[3] // 2)
                    myPose["pose_nlm_List"] = nlm_List
                    myPose["pose_lmList"] = myposeList
                    myPose["pose_bbox"] = pose_bbox
                    myPose["pose_center"] = (cx, cy)

                    # if flipType:
                    #     if handType.classification[0].label == "Right":
                    #         myHand["type"] = "Left"
                    #     else:
                    #         myHand["type"] = "Right"
                    # else:
                    #     myHand["type"] = handType.classification[0].label
                    allPoses=myPose

                    # draw
                    if draw==True and blk==True:
                        
                        self.mpDraw.draw_landmarks(img, self.pose_results.pose_landmarks,
                                                self.mpPose.POSE_CONNECTIONS)
                        cv2.rectangle(img, (pose_bbox[0] - 20, pose_bbox[1] - 20),
                                    (pose_bbox[0] + pose_bbox[2] + 20, pose_bbox[1] + pose_bbox[3] + 20),
                                    (255, 0, 255), 2)
                        cv2.putText(img, "pose1", (pose_bbox[0] - 20, pose_bbox[1] - 30), cv2.FONT_HERSHEY_PLAIN,
                                2, (255, 0, 255), 2)
                        if blk==True:
                            self.mpDraw.draw_landmarks(img, self.pose_results.pose_landmarks,
                                                self.mpPose.POSE_CONNECTIONS)
                                                
                    
                    if blk==False and draw==True:
                        self.mpDraw.draw_landmarks(img, self.pose_results.pose_landmarks,
                                                self.mpPose.POSE_CONNECTIONS)
                        cv2.rectangle(img, (pose_bbox[0] - 20, pose_bbox[1] - 20),
                                    (pose_bbox[0] + pose_bbox[2] + 20, pose_bbox[1] + pose_bbox[3] + 20),
                                    (255, 0, 255), 2)
                        cv2.putText(img, "pose1", (pose_bbox[0] - 20, pose_bbox[1] - 30), cv2.FONT_HERSHEY_PLAIN,
                                2, (255, 0, 255), 2)    
            # cv2.imshow("black Image", blk_img)           
            if draw==True and blk==False:
                # print("here1")
                return allPoses, img
            
            elif draw==True and blk==True:
                # print("here2")
                return allPoses,img,blk_img
            else:
                return allPoses
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
        img.flags.writeable = True
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        self.hand_results = self.hands.process(img)
        img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        allHands = []
        h, w, c = img.shape
        if self.hand_results.multi_hand_landmarks:
            for handType, handLms in zip(self.hand_results.multi_handedness, self.hand_results.multi_hand_landmarks):
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
                if blk==True and draw==False:
                        self.mpDraw.draw_landmarks(blk_img, handLms,
                                               self.mpHands.HAND_CONNECTIONS)
                                               
                   
                if blk==False and draw==True:
                    self.mpDraw.draw_landmarks(img, handLms,
                                            self.mpHands.HAND_CONNECTIONS)
                    cv2.rectangle(img, (bbox[0] - 20, bbox[1] - 20),
                                  (bbox[0] + bbox[2] + 20, bbox[1] + bbox[3] + 20),
                                  (255, 0, 255), 2)
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