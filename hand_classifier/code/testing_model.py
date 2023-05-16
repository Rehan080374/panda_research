import cv2
from HandTrackingModule import HandDetector
import numpy as np
import math
import time
import tensorflow as tf
# from classifier import Classifier
tf.config.set_visible_devices([], 'GPU')
from keras.models import load_model
cap = cv2.VideoCapture(4) #
detect=HandDetector(maxHands=0)
count=0
# cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
# cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
blk=True
xbuffer=20
var=False
ybuffer=20
time_passed=0
folder ="/home/rehan/catkin_ws/src/panda_research/hand_classifier/data/none"
model_path="/home/rehan/catkin_ws/src/panda_research/hand_classifier/model11/keras_model.h5"
label_path="/home/rehan/catkin_ws/src/panda_research/hand_classifier/model11/labels.txt"
image_size=300
# Class=Classifier
model=load_model(model_path)
with open(label_path, 'r') as f:
    labels = f.read().splitlines()

while True:
    suc,img =cap.read()
#     img = cv2.flip(img, 1)
#     print('Resolution: ' + str(img.shape[0]) + ' x ' + str(img.shape[1]))
    if blk==True:
            hands, img,blk_img= detect.findHands(img,blk) 
            
            
    else:
            hands, img = detect.findHands(img,blk)  
    
    
    if hands:
            # Hand 1
           
            if len(hands) == 2:
                # Hand 2s
                hand1 = hands[0]
                lmList1 = hand1["lmList"]  # List of 21 Landmark points
                bbox1 = hand1["bbox"]  # Bounding box info x,y,w,h
                x,y,w,h=bbox1
                # print(bbox1)
                centerPoint1 = hand1['center']  # center of the hand cx,cy
                handType1 = hand1["type"]  # Handtype Left or Right
                img_crop1=blk_img[y-ybuffer:y+h+ybuffer,x-xbuffer:x+w+xbuffer]  #for black image
                # img_crop1=img[y-ybuffer:y+h+ybuffer,x-xbuffer:x+w+xbuffer]
                hand2 = hands[1]
                lmList2 = hand2["lmList"]  # List of 21 Landmark points
                bbox2 = hand2["bbox"]  # Bounding box info x,y,w,h
                centerPoint2 = hand2['center']  # center of the hand cx,cy
                handType2 = hand2["type"]  # Hand Type "Left" or "Right
                x2,y2,w2,h2=bbox2
                img_crop2=blk_img[y2-ybuffer:y2+h2+ybuffer,x2-xbuffer:x2+w2+xbuffer] #for black image
                # img_crop2=img[y2-ybuffer:y2+h2+ybuffer,x2-xbuffer:x2+w2+xbuffer]
                image_black1= np.zeros((image_size,image_size,3),np.uint8)
                image_black2= np.zeros((image_size,image_size,3),np.uint8)
                # print("bbox1 = ",bbox1,"  bbox2 = ",bbox2)
                A_R1=h/w
                A_R2=h2/w2
                imgcropshape1=np.array(img_crop1.shape)
                imgcropshape2=np.array(img_crop2.shape)
                detect.findDistance(lmList1[8][0:2], lmList2[8][0:2], blk_img) 
                detect.findDistance(lmList1[4][0:2], lmList2[4][0:2], blk_img)
                detect.findDistance(lmList1[12][0:2], lmList2[12][0:2], blk_img)
                detect.findDistance(lmList1[16][0:2], lmList2[16][0:2], blk_img)
                detect.findDistance(lmList1[20][0:2], lmList2[20][0:2], blk_img)
                # try:
                # #     k1=image_size/h
                # #     wcal1=math.ceil(k1+w)
                # #     imgresize1=cv2.resize(img_crop1,(wcal1,image_size))
                # #     imgresizeshape1=imgresize1.shape
                # #     wgap1=math.ceil((image_size-wcal1)/2)
                # #     image_black1[:,wgap1:wgap1+wcal1]=imgresize1
                #     image_black1=cv2.resize(img_crop1,(300,300))
                    
                # except Exception as e:
                #       print(e)
                
               
                # try:
                # #     k2=image_size/h2
                # #     wcal2=math.ceil(k2+w2)
                # #     imgresize2=cv2.resize(img_crop2,(wcal2,image_size))
                # #     imgresizeshape2=imgresize2.shape
                # #     wgap2=math.ceil((image_size-wcal2)/2)
                # #     image_black2[:,wgap2:wgap2+wcal2]=imgresize2 
                #     image_black2=cv2.resize(img_crop2,(300,300))
                # except Exception as f:
                #       print(f)
                 
                    
                # if handType1=="Left":
                
                #         Hori = np.concatenate((image_black1, image_black2), axis=1)
                # elif handType1=="Right":
                #         Hori = np.concatenate((image_black2, image_black1), axis=1)   
                # else:    
                #         Hori = np.concatenate((image_black2, image_black1), axis=1)       
                # imgshape=Hori.shape
                # # print(imgshape)
                # # concatenate image Vertically
                # #     Verti = np.concatenate((img1, img2), axis=0)
                
                
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
                if blk==True:
                        cv2.imshow("black Image", blk_img)
    
    
    
    
    # if blk==True:
    #     # cv2.imshow("black Image", blk_img)
    
    cv2.imshow("image",img)



#     if cv2.waitKey(30) >= 0 :
#                 break
    key=cv2.waitKey(1)
    
   
    if key ==ord("c"):  
          break    