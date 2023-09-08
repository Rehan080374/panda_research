from evdev import InputDevice, categorize, ecodes,KeyEvent
import evdev
# dev ='/dev/input/event17'
# gamepad = InputDevice('/dev/input/event5')

# for event in gamepad.read_loop():
#                 # print("button pressed",event )
#                 if event.type == ecodes.EV_KEY:
#                     keyevent = categorize(event)
#                     # print ("ev codes ",categorize(event))
#                     if keyevent.keystate == KeyEvent.key_down:
                        
#                         print ("key code for the button is ",keyevent.keycode) 
                            
#                 elif event.type == ecodes.EV_ABS:
#                     absevent = categorize(event)   
#                     print( "joystick ",ecodes.bytype[absevent.event.type][absevent.event.code] )
#                     print ("value =",absevent.event.value)
# Import the required module for text
# to speech conversion
from gtts import gTTS
from playsound import playsound
# This module is imported so that we can
# play the converted audio
import os

# The text that you want to convert to audio
mytext = 'Welcome to geeksforgeeks!'




# Passing the text and language to the engine,
# here we have marked slow=False. Which tells
# the module that the converted audio should
# have a high speed

def text_to_speech(text, output_file):
    tts = gTTS(text=text, lang='en' ,slow=False)
    tts.save(output_file)
    playsound(output_file)
# Saving the converted audio in a mp3 file named
# welcome
output_file = "output.mp3"  # Use WAV format instead of MP3
# play the audio file
# playsound("welco.mp3")
text_to_speech(mytext,output_file)
# Playing the converted file

