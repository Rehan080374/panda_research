'''
///////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2018, STEREOLABS.
//
// All rights reserved.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
///////////////////////////////////////////////////////////////////////////

/*****************************************************************************************
 ** This sample demonstrates how to capture stereo images and calibration parameters    **
 ** from the ZED camera with OpenCV without using the ZED SDK.                          **
 *****************************************************************************************/
'''

import numpy as np
import os
import configparser
import sys
import cv2
import mediapipe as mp
# from mediapipe.tasks.python import vision
# from mediapipe.tasks import python
# from mediapipe_model_maker import gesture_recognizer

BaseOptions = mp.tasks.BaseOptions
GestureRecognizer = mp.tasks.vision.GestureRecognizer
GestureRecognizerOptions = mp.tasks.vision.GestureRecognizerOptions
GestureRecognizerResult = mp.tasks.vision.GestureRecognizerResult
VisionRunningMode = mp.tasks.vision.RunningMode
# Create a gesture recognizer instance with the live stream mode:
def print_result(result: GestureRecognizerResult, output_image: mp.Image, timestamp_ms: int):
    print('gesture recognition result: {}'.format(result))

class Resolution :
     width = 1280
     height = 720

def main() :
    serial_number = 19116715
    mp_drawing = mp.solutions.drawing_utils
    mp_drawing_styles = mp.solutions.drawing_styles
    mp_hands = mp.solutions.hands
    # if len(sys.argv) == 1 :
    #     print('Please provide ZED serial number')
    #     exit(1)

    # Open the ZED camera
    cap = cv2.VideoCapture(0)
    if cap.isOpened() == 0:
        exit(-1)

    # image_size = Resolution()
    # image_size.width = 1280
    # image_size.height = 720

    # Set the video resolution to HD720
    # cap.set(cv2.CAP_PROP_FRAME_WIDTH, image_size.width)
    # cap.set(cv2.CAP_PROP_FRAME_HEIGHT, image_size.height)
    # STEP 2: Create an GestureRecognizer object.
    
    options = GestureRecognizerOptions(
    base_options=BaseOptions(model_asset_path='/home/panda/test_ws/src/panda_repository/panda_research/camera applications/gesture_recognizer.task'),
    running_mode=VisionRunningMode.IMAGE,
    )


        
    # with mp_hands.Hands(
    # model_complexity=0,
    # min_detection_confidence=0.5,
    # min_tracking_confidence=0.5) as hands:
    with GestureRecognizer.create_from_options(options) as recognizer:

        while cap.isOpened():
            # Get a new frame from camera
            retval, frame = cap.read()
            # Extract left and right images from side-by-side
            left_right_image = np.split(frame, 2, axis=1)
            image = cv2.cvtColor(left_right_image[0], cv2.COLOR_BGR2RGB)
            mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=image)
            # recognizer.recognize_async(mp_image, frame_timestamp_ms)
            width = image.shape[0]
            height = image.shape[1]
            # image=left_right_image[0]
            

            # image = mp.Image.create_from_file(image)
            # image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB) 
            # image_packet = mp.packet_creator.create_image( image_format=unit., data=image)

            # image.flags.writeable = True
            # image_frame_packet = mp.packet_creator.create_image_frame(image_format=mp.ImageFormat.SRGB, data=image)
            # image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
            results = recognizer.recognize(mp_image)
            # image = cv2.cvtColor(left_right_image[0], cv2.COLOR_RGB2BGR)
            # print(results)
            #  results = hands.process(image)
            # top_gesture = recognition_result.Gestures[0].Categories[0].catagoryName 
            hand_landmarks = np.array(results.hand_landmarks)
            
            # Draw the hand annotations on the image.
            # image.flags.writeable = True
            image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
            
            if results.handedness:
                print(results.gestures[0][0].category_name)
                # print("handedness=",results.handedness[0].display_name)
            #     # print(i)
            #     # i+=1
                # landmarks=[]
                for i in range(20):
                    # x=round(hand_landmarks[0][i].x * width)
                    # y=round(hand_landmarks[0][i].y* height)
                    # cent=(x,y)
                    p1=[hand_landmarks[0][i].x * width,hand_landmarks[0][i].y * height]
                #   print(f'Index finger tip coordinate: ',i,'  (',f'{hand_landmarks[0][i].x * image_size.width}, 'f'{hand_landmarks[0][i].y * image_size.height})')  
                    cv2.circle(image,p1,radius=3,color=(0,255,255),thickness=10)
                # for  hand_landmark in results.hand_landmarks:
                    # mp_drawing.draw_landmarks(image,Landmark,mp_hands.HAND_CONNECTIONS,mp_drawing_styles.get_default_hand_landmarks_style(),mp_drawing_styles.get_default_hand_connections_style())
            #          mp_drawing.draw_landmarks(image,landmarks,mp_hands.HAND_CONNECTIONS,mp_drawing_styles.get_default_hand_landmarks_style(),mp_drawing_styles.get_default_hand_connections_style())
                #    for Landmark in  hand_landmarks:     
                # mp_drawing.draw_landmarks(image,results.hand_world_landmarks, mp_hands.HAND_CONNECTIONS)
                    # print(f'Index finger tip coordinate: (',f'{landmark[mp_hands.HandLandmark.INDEX_FINGER_TIP].x * image_size.width}, 'f'{landmark[mp_hands.HandLandmark.INDEX_FINGER_TIP].y * image_size.height})')
                    print(x,y )
                # for landmark in  landmarks:
                # mp_drawing.draw_landmarks(image,landmarks, mp_hands.HAND_CONNECTIONS)

            cv2.imshow('MediaPipe Hands', image)            
            # Flip the image horizontally for a selfie-view display.
            # cv2.imshow('MediaPipe Hands', cv2.flip(image, 1))
            #cv2.imshow("left RECT", left_rect)

            #cv2.imshow("right RECT", right_rect)
            if cv2.waitKey(30) >= 0 :
                break

    exit(0)

if __name__ == "__main__":
    main()
