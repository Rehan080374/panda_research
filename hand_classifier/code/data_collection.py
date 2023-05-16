import cv2
from HandTrackingModule import HandDetector
import numpy as np
import math
import time
cap = cv2.VideoCapture(4) #
detect=HandDetector(maxHands=2)
count=0
# cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
# cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
blk=True
xbuffer=20
var=False
ybuffer=20
time_passed=0
folder ="/home/rehan/catkin_ws/src/panda_research/hand_classifier/data/train_data_bw/down"
image_size=300
while True:
    suc,img =cap.read()
#     img = cv2.flip(img, 1)
    print('Resolution: ' + str(img.shape[0]) + ' x ' + str(img.shape[1]))
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
                # img_crop1=blk_img[y-ybuffer:y+h+ybuffer,x-xbuffer:x+w+xbuffer]  #for black image
                img_crop1=img[y-ybuffer:y+h+ybuffer,x-xbuffer:x+w+xbuffer]
                hand2 = hands[1]
                lmList2 = hand2["lmList"]  # List of 21 Landmark points
                bbox2 = hand2["bbox"]  # Bounding box info x,y,w,h
                centerPoint2 = hand2['center']  # center of the hand cx,cy
                handType2 = hand2["type"]  # Hand Type "Left" or "Right
                x2,y2,w2,h2=bbox2
                # img_crop2=blk_img[y2-ybuffer:y2+h2+ybuffer,x2-xbuffer:x2+w2+xbuffer] #for black image
                img_crop2=img[y2-ybuffer:y2+h2+ybuffer,x2-xbuffer:x2+w2+xbuffer]
                image_black1= np.zeros((image_size,image_size,3),np.uint8)
                image_black2= np.zeros((image_size,image_size,3),np.uint8)
                # print("bbox1 = ",bbox1,"  bbox2 = ",bbox2)
                A_R1=h/w
                A_R2=h2/w2
                imgcropshape1=np.array(img_crop1.shape)
                imgcropshape2=np.array(img_crop2.shape)
                
                try:
                    k1=image_size/h
                    wcal1=math.ceil(k1+w)
                    imgresize1=cv2.resize(img_crop1,(wcal1,image_size))
                    imgresizeshape1=imgresize1.shape
                    wgap1=math.ceil((image_size-wcal1)/2)
                    image_black1[:,wgap1:wgap1+wcal1]=imgresize1
                except Exception as e:
                      print(e)
                
               
                try:
                    k2=image_size/h2
                    wcal2=math.ceil(k2+w2)
                    imgresize2=cv2.resize(img_crop2,(wcal2,image_size))
                    imgresizeshape2=imgresize2.shape
                    wgap2=math.ceil((image_size-wcal2)/2)
                    image_black2[:,wgap2:wgap2+wcal2]=imgresize2 
                except Exception as f:
                      print(f)
                detect.findDistance(lmList1[8][0:2], lmList2[8][0:2], blk_img) 
                detect.findDistance(lmList1[4][0:2], lmList2[4][0:2], blk_img)
                detect.findDistance(lmList1[12][0:2], lmList2[12][0:2], blk_img)
                detect.findDistance(lmList1[16][0:2], lmList2[16][0:2], blk_img)
                detect.findDistance(lmList1[20][0:2], lmList2[20][0:2], blk_img) # with draw
                # if A_R1>1:
                #        k1=image_size/h
                #        wcal1=math.ceil(k1+w)
                #        imgresize1=cv2.resize(img_crop1,(wcal1,image_size))
                #        imgresizeshape1=imgresize1.shape
                #        wgap1=math.ceil((image_size-wcal1)/2)
                #        image_black1[:,wgap1:wgap1+wcal1]=imgresize1
                # else:
                #        k1=image_size/w
                #        hcal1=math.ceil(k1+h)
                #        imgresize1=cv2.resize(img_crop1,(image_size,hcal1))
                #        imgresizeshape1=imgresize1.shape
                #        hgap1=math.ceil((image_size-hcal1)/2)
                #        image_black1[hgap1:hgap1+hcal1, :]=imgresize1
                # if A_R2>1:
                #        k2=image_size/h2
                #        wcal2=math.ceil(k2+w2)
                #        imgresize2=cv2.resize(img_crop2,(wcal2,image_size))
                #        imgresizeshape2=imgresize2.shape
                #        wgap2=math.ceil((image_size-wcal2)/2)
                #        image_black2[:,wgap2:wgap2+wcal2]=imgresize2    
                # else:
                #        k2=image_size/w2
                #        hcal2=math.ceil(k2+h2)
                #        imgresize2=cv2.resize(img_crop2,(image_size,hcal2))
                #        imgresizeshape2=imgresize2.shape
                #        hgap2=math.ceil((image_size-hcal2)/2)
                #        image_black2[hgap2:hgap2+hcal2,:]=imgresize2           
                # final_crop=img_crop1+img_crop2
                # imgshape=img.shape
                # print("bbox1 = ",imgshape)
                # if x >0 and y>0 and x2>0 and y2>0 and x+w<imgshape[0] and y+h<imgshape[1] and x2+w2<imgshape[0] and y2+h2<imgshape[1]:
                # print (np.any(imgcropshape1,where =[0]))
                # try:
                #         image_black1=cv2.resize(img_crop1,(image_size,image_size))
                #         image_black2=cv2.resize(img_crop2,(image_size,image_size))
                # except Exception as e:
                #       print(e)        
                if handType1=="Left":
                
                        Hori = np.concatenate((image_black1, image_black2), axis=1)
                elif handType1=="Right":
                        Hori = np.concatenate((image_black2, image_black1), axis=1)   
                else:    
                        Hori = np.concatenate((image_black2, image_black1), axis=1)       

                # concatenate image Vertically
                #     Verti = np.concatenate((img1, img2), axis=0)
                
                cv2.imshow('left', blk_img)
                #     cv2.imshow('right', image_black2)
                #     cv2.imshow('VERTICAL', Verti)
                #     cv2.imshow("cropped Image_1", img_crop1)
                #     cv2.imshow("cropped Image_2", img_crop2)
            # else:
            #     hand1 = hands[0]
            #     lmList1 = hand1["lmList"]  # List of 21 Landmark points
            #     bbox1 = hand1["bbox"]  # Bounding box info x,y,w,h
            #     x,y,w,h=bbox1
            #     print(bbox1)
            #     centerPoint1 = hand1['center']  # center of the hand cx,cy
            #     handType1 = hand1["type"]  # Handtype Left or Right
            #     img_crop1=blk_img[y-buffer:y+h+buffer,x-buffer:x+w+buffer]
            #     # fingers1 = detec.fingersUp(hand1)
            #     cv2.imshow("cropped Image_1", img_crop1)        

                # fingers2 = detector.fingersUp(hand2)
    
    
    
    
    
    
    
    # if blk==True:
    #     # cv2.imshow("black Image", blk_img)
    
    cv2.imshow("image",img)



#     if cv2.waitKey(30) >= 0 :
#                 break
    key=cv2.waitKey(1)
    if key ==ord("s"):
        var=True
        seconds=time.time()
            
    if time_passed<5 and var==True:
        print("waiting for " ,time_passed )
        time_passed=time.time()-seconds
    
    if time_passed>5 and var ==True :
           

        count +=1
        print("image count = ",count)        
        cv2.imwrite(f'{folder}/image_{count}.jpg',blk_img)  
    if key ==ord("c"):  
          break    