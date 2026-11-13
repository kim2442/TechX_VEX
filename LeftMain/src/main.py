# ---------------------------------------------------------------------------- #
#                                                                              #
# 	Module:       main.py                                                      #
# 	Author:       wawa                                                         #
# 	Created:      15/10/2026, 10:06:34                                         #
# 	Description:  V5 project                                                   #
#                                                                              #
# ---------------------------------------------------------------------------- #

from vex import *
#import urandom
import math
from shared.driver import *

# add a small delay to make sure we don't print in the middle of the REPL header
wait(200, MSEC)
# clear the console to make sure we don't have the REPL in the console
print("\033[2J")

# --- Main autonomous routine ---
def autonomous(): #(robot should be placed right in the center with pin held)

    #Free toggle mechanism (initially tied down by rubber bands for space)
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
    turn_in_place(-40)
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

# Clear console output buffer before starting main loop
print("\033[2J")
wait(200, MSEC)

competition = Competition(driver_control, autonomous)