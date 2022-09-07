from evdev import InputDevice, categorize, ecodes,KeyEvent
gamepad = InputDevice('/dev/input/event3')
#if your event is not event3, then change to yours
print (gamepad)
for event in gamepad.read_loop():
    if event.type == ecodes.EV_KEY:
        keyevent = categorize(event)
        print (categorize(event))
        if keyevent.keystate == KeyEvent.key_down:
            if keyevent.keycode == ['BTN_A', 'BTN_GAMEPAD', 'BTN_SOUTH']:
                print ("A")
            elif keyevent.keycode == ['BTN_WEST', 'BTN_Y']:
                print ("Y")
            elif keyevent.keycode == ['BTN_B', 'BTN_EAST']:
                print ("B")
            elif keyevent.keycode == ['BTN_NORTH', 'BTN_X']:
                print ("X")
            elif keyevent.keycode == 'BTN_TR':
                print ("up")
            elif keyevent.keycode == 'BTN_TL':
                print ("down")        
    elif event.type == ecodes.EV_ABS:
        absevent = categorize(event)
        print (categorize(event))
        print (absevent.event.value)
        if ecodes.bytype[absevent.event.type][absevent.event.code] == 'ABS_Y':
            if absevent.event.value > 32766:
                print ('reverse')
                
                print (absevent.event.value)
            elif absevent.event.value < -32766:
                print ('forward')
                print (absevent.event.value)
        if ecodes.bytype[absevent.event.type][absevent.event.code] == 'ABS_X':
            if absevent.event.value > 32766 :
                print ('right')
                print (absevent.event.value)
            elif absevent.event.value < -32766:
                print ('left')
                print (absevent.event.value)
   
                 