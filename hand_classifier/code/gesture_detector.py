import cv2
import mediapipe as mp
import math
import numpy as np
from HandTrackingModule import HandDetector




def main():
    cap = cv2.VideoCapture(4)
    detector = HandDetector(detectionCon=0.8, maxHands=2)
    blk=False
    while True:
        # Get image frame
        success, img = cap.read()
        # Find the hand and its landmarks
        # hands, img,blk_img = detector.findHands(img)  # with draw
        if blk==True:
            hands, img,blk_img= detector.findHands(img,blk)  # with draw
        else:
            hands, img = detector.findHands(img,blk)  # with draw
        # hands = detector.findHands(img, draw=False)  # without draw

        if hands:
            # Hand 1
            hand1 = hands[0]
            lmList1 = hand1["lmList"]  # List of 21 Landmark points
            bbox1 = hand1["bbox"]  # Bounding box info x,y,w,h
            centerPoint1 = hand1['center']  # center of the hand cx,cy
            handType1 = hand1["type"]  # Handtype Left or Right

            fingers1 = detector.fingersUp(hand1)

            if len(hands) == 2:
                # Hand 2
                hand2 = hands[1]
                lmList2 = hand2["lmList"]  # List of 21 Landmark points
                bbox2 = hand2["bbox"]  # Bounding box info x,y,w,h
                centerPoint2 = hand2['center']  # center of the hand cx,cy
                handType2 = hand2["type"]  # Hand Type "Left" or "Right"

                fingers2 = detector.fingersUp(hand2)

               
        # Display
        cv2.imshow("Image", img) 
        if blk==True:
            cv2.imshow("black Image", blk_img)
    
               
        cv2.waitKey(1)


if __name__ == "__main__":
    main()
