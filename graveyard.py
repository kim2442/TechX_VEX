    apply_claw_state(2) #clamp
    move_bot(0.2,60)                                                                                                                                                         #Distance the robot from wall
    wait(0.2,SECONDS)
    control_arm(70, 100)
    wait(0.1,SECONDS)
    control_arm(0)

    #Initial toggle switch 

    move_bot(0.2,-60)
    wait(0.2,SECONDS)
    control_arm(25, 20) #hopefully consistently flips the toggle twice


    #ultrasafe toggle sequence (uncomment if the above doesn't work consistently)
    wait(0.1,SECONDS) 
    control_arm(5)   
    wait(0.1,SECONDS)
    move_bot(0.15,-40) 
    control_arm(25, 20)
    
    #place pin
    wait(0.1,SECONDS)
    move_bot(0.2,50)
    wait(0.1,SECONDS)
    turn_in_place(-32)
    wait(0.2,SECONDS)
    move_bot(0.7,100)
    wait(0.2,SECONDS)
    move_bot(0.35,-20)
    wait(0.4,SECONDS)
    control_arm(3)
    wait(0.2,SECONDS)
    move_bot(0.125,-30)
    apply_claw_state(0, is_wide=True) #open
    wait(0.2,SECONDS)
    
    #grab second pin    
    move_bot(0.2,-50)
    wait(0.2,SECONDS)
    """turn_in_place(-40)
    wait(0.2,SECONDS)
    move_bot(0.5,50)
    
    #return    

    #release
    control_arm(25, 100)
    wait(0.2,SECONDS)
    apply_claw_state(0, is_wide=True) #open"""
    
    #prepare for recalibration of Claw sensor
    apply_claw_state(1)

def apply_claw_state(desired_state, is_wide = False):
    """Applies the desired claw state to the claw motor and syncs is_claw_open.

    desired_state: 0 = open, 1 = loosely closed, 2 = clamp
    is_wide: if True, opens the claw wider
    """
    
    #REVERSE is closing the claw, FORWARD is opening the claw
    global is_claw_open
    is_claw_open = (desired_state == 0) #keep is_claw_open in sync no matter who moves the claw (A, X, autonomous)

    claw_time = 0.03 
    if is_wide:
        claw_time = 0.07

    if desired_state == 0: #open
        Claw.spin(FORWARD, 7, VOLT)
        wait(claw_time, SECONDS) #half-open
        Claw.stop()
            
    if desired_state == 1: #loosly closed
        Claw.spin(REVERSE, 7, VOLT)
        wait(0.08, SECONDS) 
        Claw.stop()

    if desired_state == 2: #clamp
        #controller_1.rumble(".")
        Claw.spin(REVERSE, 7, VOLT)

# def control_claw(is_wide = False):
#     global claw_state
#     apply_claw_state(claw_state % 3, is_wide) #wrap around to 0,1,2
#     claw_state += 1


def controller_a_pressed_callback():
    """Handles button A press (regular claw control). EXPERIMENTAL"""
    #global claw_state
    global is_claw_open

    #controller_1.screen.clear_row(2)
    #controller_1.screen.set_cursor(2,1)
    #controller_1.screen.print(is_claw_open)
    #print(is_claw_open)
    

    if is_claw_open: #if open, close
        start_time = brain.timer.time(MSEC)
        reached = False
        Claw.spin(REVERSE, 7, VOLT) #REVERSE closes the claw
        while brain.timer.time(MSEC) - start_time < 500:
            if claw_sensor.position(DEGREES) > -5:
                reached = True
                break
            wait(5, MSEC)
        if reached:
            Claw.set_stopping(COAST)
            Claw.stop()
        else:
            Claw.set_stopping(HOLD)
            Claw.stop()
            controller_1.rumble(".")
        is_claw_open = False
    else:
        Claw.set_stopping(HOLD)
        Claw.spin(FORWARD, 9, VOLT)
        wait(0.1, SECONDS) #half-open
        Claw.stop()
        is_claw_open = True
        
        #apply_claw_state(0) #open (also sets is_claw_open = True)
        
def controller_x_pressed_callback():
    """Makes the claw open all the way (for grabbing the pin in middle of the field) or clamp down hard instead of resting neutrally"""
    #currently broken; need to copy above code
    global claw_state
    #control_claw(is_wide=True)