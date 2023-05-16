import cv2
import mediapipe as mp
import math
import numpy as np
from keras.models import load_model
import tensorflow as tf
from HandTrackingModule import HandDetector
# from classifier import Classifier
tf.config.set_visible_devices([], 'GPU')


def main():
    folder ="/home/rehan/catkin_ws/src/panda_research/hand_classifier/data/none"
    model_path="/home/rehan/catkin_ws/src/panda_research/hand_classifier/model11/keras_model.h5"
    label_path="/home/rehan/catkin_ws/src/panda_research/hand_classifier/model11/labels.txt"
    model=load_model(model_path)
    with open(label_path, 'r') as f:
        labels = f.read().splitlines()

    cap = cv2.VideoCapture(4)
    detector = HandDetector(detectionCon=0.8, maxHands=2)
    blk=True
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
                # detector.findDistance(centerPoint1, centerPoint2, blk_img) 
                detector.findDistance(lmList1[8][0:2], lmList2[8][0:2], blk_img) 
                detector.findDistance(lmList1[4][0:2], lmList2[4][0:2], blk_img)
                detector.findDistance(lmList1[12][0:2], lmList2[12][0:2], blk_img)
                detector.findDistance(lmList1[16][0:2], lmList2[16][0:2], blk_img)
                detector.findDistance(lmList1[20][0:2], lmList2[20][0:2], blk_img)
                # Find Distance between two Landmarks. Could be same hand or different hands
                # length, info, img = detector.findDistance(lmList1[8][0:2], lmList2[8][0:2], img)  # with draw
                # length, info = detector.findDistance(lmList1[8], lmList2[8])  # with draw
                try:
                        # frame = cv2.resize(Hori, (300, 600))
                        frame =blk_img
                        # print(Hori.shape)
                        frame = np.expand_dims(frame, axis=0)
                        frame = frame / 255.0

                        # make a prediction
                        prediction = np.around(model.predict(frame),3)
                        # if prediction[np.amax(prediction)]>0.6:
                        prediction_label = labels[np.argmax(prediction)]
                        print(prediction[0][np.argmax(prediction)])
                        # display the prediction label on the frame
                        if prediction[0][np.argmax(prediction)]>0.9:
                                cv2.putText(blk_img, prediction_label, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (45, 255, 12), 2)
                        # print(prediction)
                        # display the frame
                        # cv2.imshow('frame', frame)
                        # predict,index=Class.getPrediction(Hori)
                        # print("prediction = ",predict," index = ",index)
                        # cv2.imshow('left', Hori)
                except Exception as g:
                      print(g)
        # Display
        cv2.imshow("Image", img) 
        if blk==True:
            cv2.imshow("black Image", blk_img)
    
               
        cv2.waitKey(1)


if __name__ == "__main__":
    main()
