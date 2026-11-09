# ---------------------------------------------------------------------------- #
#                                                                              #
# 	Module:       main.py                                                      #
# 	Author:       wawa                                                         #
# 	Created:      15/10/2026, 10:06:34                                         #
# 	Description:  V5 project                                                   #
#                                                                              #
# ---------------------------------------------------------------------------- #

# Library imports
from vex import *
#import urandom
import math

def play_vexcode_sound(sound_name):
    # Helper to make playing sounds from the V5 in VEXcode easier and
    # keeps the code cleaner by making it clear what is happening.
    print("VEXPlaySound:" + sound_name)
    wait(5, MSEC)

# add a small delay to make sure we don't print in the middle of the REPL header
wait(200, MSEC)
# clear the console to make sure we don't have the REPL in the console
print("\033[2J")

#endregion VEXcode Generated Robot Configuration

# --- 1. Robot and Motor Configuration ---

brain = Brain()

Arm_motor_a = Motor(Ports.PORT12, GearSetting.RATIO_36_1, True)
Arm_motor_b = Motor(Ports.PORT20, GearSetting.RATIO_36_1, False)
Arm = MotorGroup(Arm_motor_a, Arm_motor_b)

Claw = Motor(Ports.PORT13, GearSetting.RATIO_18_1, False)

RightMotors_motor_a = Motor(Ports.PORT1, GearSetting.RATIO_18_1, False) #this should be true, but too lazy to swap everything
RightMotors_motor_b = Motor(Ports.PORT3, GearSetting.RATIO_18_1, False) #this should be true, but too lazy to swap everything
RightMotors = MotorGroup(RightMotors_motor_a, RightMotors_motor_b)

LeftMotors_motor_a = Motor(Ports.PORT2, GearSetting.RATIO_18_1, False)
LeftMotors_motor_b = Motor(Ports.PORT4, GearSetting.RATIO_18_1, False)
LeftMotors = MotorGroup(LeftMotors_motor_a, LeftMotors_motor_b)

arm_sensor = Rotation(Ports.PORT10, False) #unused
claw_sensor = Rotation(Ports.PORT14, False) 
inertial_sensor = Inertial(Ports.PORT5)
controller_1 = Controller(PRIMARY)

# wait for rotation sensor to fully initialize
wait(30, MSEC)


# --- 2. Misc State Variables (Global State Management) ---

# Display precision settings
screen_precision = 0
console_precision = 0
controller_precision = 0


# --- 3. Utility Functions ---

"""def initialize_random_seed():
    wait(100, MSEC)
    # Use voltage and current to create a unique seed
    seed_value = brain.battery.voltage(MV) + brain.battery.current(CurrentUnits.AMP) * 100 + brain.timer.system_high_res()
    urandom.seed(int(seed_value))
    print("Random seed initialized.")"""

def console_format(variable):
    """Formats a variable for printing with specified decimal precision."""
    if isinstance(variable, str):
        return variable
    
    # Determine precision
    precision = 6 if console_precision is None else console_precision
    
    # Format the number
    return "{0:.{1}f}".format(variable, precision)


# --- 4. Drive logic ---

forwardback = 0.0
turning = 0.0
leftspeed = 0.0
rightspeed = 0.0
claw_state: int = 1 #!!!!!!!!!!!!!!!!!!!!!!!!

is_claw_open = True #experimental

slow_mode = False #toggled by controller B; halves drive speed
#Claw.set_timeout(1.3, SECONDS)
#Claw.set_stopping(HOLD)
Claw.set_velocity(70,PERCENT)
arm_sensor.set_position(0, DEGREES)
Arm.set_position(0, DEGREES)
Arm.set_stopping(HOLD) 

def update_drive():
    global forwardback, turning, leftspeed, rightspeed, slow_mode

    # >>> RR-EDIT (record/replay): originals were controller_1.axis3.position() / controller_1.axis4.position()
    forwardback  = -rr_axis3()
    turning = - 0.007 * rr_axis4() * abs(rr_axis4())
    # <<< RR-EDIT END
        
    # deadzone 
    if abs(forwardback) < 5:
        forwardback = 0
    if abs(turning) < 5:
        turning = 0

    # Slow mode: scale inputs down for finer control
    if slow_mode:
        turning  *= 0.5
        forwardback *= 0.7

    leftspeed = forwardback + turning
    rightspeed = turning - forwardback

    # Keep the ratio between the two sides
    max_speed = max(100, abs(leftspeed), abs(rightspeed))
    leftspeed = leftspeed / max_speed * 100
    rightspeed = rightspeed / max_speed * 100


def move_bot(t, speed): 
    """in seconds (negative to go backward), percents"""
    LeftMotors.set_velocity(speed, PERCENT)
    RightMotors.set_velocity(speed, PERCENT)
    LeftMotors.spin(REVERSE)
    RightMotors.spin(FORWARD)
    wait(t, SECONDS)
    LeftMotors.stop()
    RightMotors.stop()

def turn_in_place(deg):
    target_rotation = inertial_sensor.rotation(DEGREES) + deg
    LeftMotors.set_velocity(50, PERCENT)
    RightMotors.set_velocity(50, PERCENT)
    
    if (deg < 0): #turn left
        LeftMotors.spin(FORWARD) 
        RightMotors.spin(FORWARD)
        while (inertial_sensor.rotation(DEGREES) > target_rotation):
            wait(5, MSEC)
    else: #turn right
        LeftMotors.spin(REVERSE)
        RightMotors.spin(REVERSE)
        while (inertial_sensor.rotation(DEGREES) < target_rotation):
            wait(5, MSEC)
    #ms = abs(deg)*7.2 #TODO: find proper coefficient
    #wait(ms, MSEC)
    LeftMotors.stop()
    RightMotors.stop()    
    
def control_arm(raise_pct, velocity = 60):
    #Maximum rotation of arm_sensor is 105 degrees
    #Maximum rotation of Arm is 405 degrees
    goal_rotation = (raise_pct/100) * 405
    Arm.spin_to_position(goal_rotation, DEGREES, velocity, PERCENT)

def apply_claw_state(desired_state, is_wide = False):
    """Applies the current claw state (open/closed/clamp) to the claw motor."""
    """
    state
    0 = open
    1 = closed
    2 = clamp
    """
    
    #REVERSE is closing the claw, FORWARD is opening the claw
    global is_claw_open
    is_claw_open = (desired_state == 0) #keep is_claw_open in sync no matter who moves the claw (A, X, autonomous)

    claw_time = 0.03 #0.06 orig claw
    if is_wide:
        claw_time = 0.07 #0.14 orig claw

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

def control_claw(is_wide = False):
    global claw_state
    apply_claw_state(claw_state % 3, is_wide) #wrap around to 0,1,2
    claw_state += 1


# --- Event Callbacks (Input Handling) ---

# Drive callbacks
def controller_axis_changed_callback():
    update_drive()

def controller_b_pressed_callback():
    """Toggles slow mode (halves drive speed) for finer control."""
    global slow_mode
    slow_mode = not slow_mode
    
    controller_1.screen.clear_row(1)
    controller_1.screen.set_cursor(1,1)
    if (slow_mode):
        controller_1.screen.print("SLOW MODE")
    else:
        controller_1.screen.print("FAST MODE")
    update_drive()


# Arm callbacks
def controller_r1_pressed_callback():
    """Callback for when the R1 button is pressed (Forward Arm)."""
    if (Arm.position(DEGREES) < 400): #max height is 405 (here, 5 deg margin)
        Arm.spin(FORWARD, 6, VOLT)

def controller_r1_released_callback():
    """Callback for when the R1 button is released."""
    Arm.stop()

def controller_r2_pressed_callback():
    """Callback for when the R2 button is pressed (Reverse Arm)."""
    Arm.spin(REVERSE, 3, VOLT) #ignore pylance issue; works fine
    
def controller_r2_released_callback():
    """Callback for when the R2 button is released."""
    Arm.stop()


def controller_down_pressed_callback(): #workaround for firmware bug where motors lock up after program termination
    Arm.set_stopping(BRAKE)
    #TODO: use brain.program_stop()

#isArmMoving = False #temp

# Claw callbacks
# def controller_a_pressed_callback():
#     global claw_state
#     control_claw()

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
            Claw.set_stopping(BRAKE)
            Claw.stop()
        else:
            Claw.set_stopping(HOLD)
            Claw.stop()
            controller_1.rumble(".")
        is_claw_open = False
    else:
        Claw.set_stopping(BRAKE)
        Claw.spin(FORWARD, 7, VOLT)
        wait(0.1, SECONDS) #half-open
        Claw.stop()
        is_claw_open = True
        
        #apply_claw_state(0) #open (also sets is_claw_open = True)
        
def controller_x_pressed_callback():
    """Makes the claw open all the way (for grabbing the pin in middle of the field) or clamp down hard instead of resting neutrally"""
    #currently broken; need to copy above code
    global claw_state
    control_claw(is_wide=True)


# --- Debug functions ---


# ============================================================================ #
# >>> RECORD/REPLAY ADDITION START  (to revert: delete down to "RECORD/REPLAY  #
#     ADDITION END", then undo the two small "RR-EDIT" blocks in update_drive  #
#     and autonomous)                                                          #
# ============================================================================ #
#
# How to use:
#   1. Plug the brain (or the tethered controller) into this computer via USB and
#      run the program so the VEX terminal in VS Code shows the robot's output.
#   2. In driver control, press Y to START recording. Drive your auton. Press Y
#      again to STOP (auto-stops after RR_MAX_SAMPLES).
#   3. The recording is printed to the terminal between "RR_BEGIN" and "RR_END".
#      Copy those lines and paste them over the two REPLAY_* lines below.
#   4. Autonomous now replays it. Press LEFT during driver control to test-replay.
#
# Each sample is (axis3, axis4, button_bitmask), taken every RR_INTERVAL_MS.

USE_REPLAY_AUTON = True   # set False to always run the old scripted autonomous
RR_INTERVAL_MS = 20       # sample period
RR_MAX_SAMPLES = 1500     # 1500 * 20ms = 30 s cap

# ---- paste recording output over these two lines ----
REPLAY_SLOW_MODE = False
REPLAY_DATA = [
    (0, 0, 0), (0, 0, 0), (0, 0, 0), (0, 0, 0), (0, 0, 0), (0, 0, 0), (0, 0, 0), (0, 0, 0), (0, 0, 0), (0, 0, 0),
    (0, 0, 0), (0, 0, 0), (0, 0, 0), (0, 0, 0), (0, 0, 0), (0, 0, 0), (0, 0, 0), (0, 0, 0), (0, 0, 0), (0, 0, 0),
    (0, 0, 0), (0, 0, 0), (0, 0, 0), (0, 0, 0), (0, 0, 0), (0, 0, 0), (0, 0, 0), (0, 0, 0), (0, 0, 0), (0, 0, 0),
    (0, 0, 0), (0, 0, 0), (3, 0, 0), (3, 0, 0), (3, 0, 0), (22, 0, 0), (29, 0, 0), (36, 0, 0), (36, 0, 0), (36, 0, 0),
    (59, 0, 0), (63, 0, 0), (66, 0, 0), (66, 0, 0), (66, 0, 0), (67, 0, 0), (67, 0, 0), (67, 0, 0), (67, 0, 0), (67, 0, 0),
    (67, 0, 0), (67, 0, 0), (67, 0, 0), (67, 0, 0), (67, 0, 0), (67, 0, 0), (67, 0, 0), (67, 0, 0), (67, 0, 0), (67, 0, 0),
    (67, 0, 0), (67, 0, 0), (67, 0, 0), (67, 0, 0), (67, 0, 0), (67, 0, 0), (67, 0, 0), (67, 0, 0), (67, 0, 0), (33, 57, 0),
    (16, 81, 0), (0, 100, 0), (0, 100, 0), (0, 100, 0), (0, 100, 0), (0, 100, 0), (0, 100, 0), (0, 100, 0), (0, 100, 0), (0, 100, 0),
    (0, 100, 0), (0, 100, 0), (0, 100, 0), (0, 100, 0), (0, 100, 0), (0, 100, 0), (0, 100, 0), (0, 93, 0), (2, 51, 0), (2, 51, 0),
    (4, 0, 0), (0, -2, 0), (0, -25, 0), (0, -60, 0), (0, -60, 0), (0, -100, 0), (0, -100, 0), (0, -100, 0), (0, -100, 0), (0, -100, 0),
    (0, -100, 0), (0, -100, 0), (0, -100, 0), (0, -100, 0), (0, -100, 0), (0, -100, 0), (0, -100, 0), (0, -100, 0), (0, -100, 0), (0, -100, 0),
    (0, -100, 0), (0, -84, 0), (0, -40, 0), (0, 0, 0), (0, 0, 0), (0, 0, 0), (0, 0, 0), (0, 0, 0), (0, 0, 0), (0, 0, 0),
    (0, 0, 0), (0, 0, 0), (-14, 0, 0), (-33, 0, 0), (-33, 0, 0), (-33, 0, 0), (-62, 0, 0), (-74, 0, 0), (-79, 0, 0), (-81, 0, 0),
    (-80, 0, 0), (-82, 0, 0), (-87, 0, 0), (-87, 0, 0), (-92, 0, 0), (-93, 0, 0), (-93, 0, 0), (-89, 0, 0), (-89, 0, 0), (-89, 0, 0),
    (0, 0, 0), (0, 0, 0), (0, 0, 0), (0, 0, 0), (0, 0, 0), (0, 0, 0), (0, 0, 0), (0, 0, 0), (0, 0, 0), (0, 0, 0),
    (0, 0, 0), (0, 0, 0), (0, 0, 0), (0, 0, 0), (0, 0, 0), (0, 0, 0), (0, 0, 0), (0, 0, 0), (0, 0, 0), (0, 0, 0),
    (0, 0, 0), (0, 0, 0), (0, 0, 0), (0, 0, 0), (0, 0, 0), (0, 0, 0), (0, 0, 0), (0, 0, 0), (0, 0, 0), (0, 0, 0),
    (0, 0, 0), (0, 0, 0), (0, 0, 0), (0, 0, 0), (0, 0, 0), (0, 0, 0), (0, 0, 0), (0, 0, 0), (0, 0, 0),
]
# ------------------------------------------------------

RR_BTN_A = 1
RR_BTN_B = 2
RR_BTN_X = 4
RR_BTN_R1 = 8
RR_BTN_R2 = 16
RR_BTN_DOWN = 32

rr_recording = False
rr_replaying = False
rr_axis3_val = 0
rr_axis4_val = 0

def rr_axis3():
    return rr_axis3_val if rr_replaying else controller_1.axis3.position()

def rr_axis4():
    return rr_axis4_val if rr_replaying else controller_1.axis4.position()

def rr_wait_until(t_ms):
    remaining = t_ms - brain.timer.time(MSEC)
    if remaining > 0:
        wait(remaining, MSEC)

def rr_read_buttons():
    m = 0
    if controller_1.buttonA.pressing():    m |= RR_BTN_A
    if controller_1.buttonB.pressing():    m |= RR_BTN_B
    if controller_1.buttonX.pressing():    m |= RR_BTN_X
    if controller_1.buttonR1.pressing():   m |= RR_BTN_R1
    if controller_1.buttonR2.pressing():   m |= RR_BTN_R2
    if controller_1.buttonDown.pressing(): m |= RR_BTN_DOWN
    return m

def rr_dump(samples, start_slow):
    """Prints the recording as Python you can paste straight into this file."""
    print("# ---- RR_BEGIN (paste over REPLAY_SLOW_MODE / REPLAY_DATA) ----")
    print("REPLAY_SLOW_MODE = " + str(start_slow))
    print("REPLAY_DATA = [")
    for k in range(0, len(samples), 10):
        print("    " + ", ".join([str(s) for s in samples[k:k + 10]]) + ",")
        wait(10, MSEC)  # don't flood the serial link
    print("]")
    print("# ---- RR_END (" + str(len(samples)) + " samples) ----")

def rr_record_thread():
    global rr_recording
    samples = []
    start_slow = slow_mode
    start = brain.timer.time(MSEC)
    i = 0
    while rr_recording and len(samples) < RR_MAX_SAMPLES:
        samples.append((controller_1.axis3.position(),
                        controller_1.axis4.position(),
                        rr_read_buttons()))
        i += 1
        rr_wait_until(start + i * RR_INTERVAL_MS)
    rr_recording = False
    controller_1.screen.clear_row(3)
    controller_1.screen.set_cursor(3, 1)
    controller_1.screen.print("REC DONE " + str(len(samples)))
    rr_dump(samples, start_slow)

def rr_replay():
    """Feeds REPLAY_DATA back through the normal drive/button code."""
    global rr_replaying, rr_axis3_val, rr_axis4_val, slow_mode
    if not REPLAY_DATA:
        print("RR: no REPLAY_DATA to replay")
        return

    on_press = [(RR_BTN_A, controller_a_pressed_callback),
                (RR_BTN_B, controller_b_pressed_callback),
                (RR_BTN_X, controller_x_pressed_callback),
                (RR_BTN_R1, controller_r1_pressed_callback),
                (RR_BTN_R2, controller_r2_pressed_callback),
                (RR_BTN_DOWN, controller_down_pressed_callback)]
    on_release = [(RR_BTN_R1, controller_r1_released_callback),
                  (RR_BTN_R2, controller_r2_released_callback)]

    slow_mode = REPLAY_SLOW_MODE
    rr_replaying = True
    prev = 0
    start = brain.timer.time(MSEC)
    i = 0
    for sample in REPLAY_DATA:
        rr_axis3_val = sample[0]
        rr_axis4_val = sample[1]
        btns = sample[2]
        update_drive()

        changed = btns ^ prev
        for bit, cb in on_press:
            if (changed & bit) and (btns & bit):
                Thread(cb)  # threads, like real events, so blocking callbacks don't stall replay
        for bit, cb in on_release:
            if (changed & bit) and not (btns & bit):
                Thread(cb)
        prev = btns

        # same motor output as driver_control's loop
        arm_height = max(0, min(405, Arm.position(DEGREES)))
        arm_speed_factor = 1 - 0.35 * (arm_height / 405)
        LeftMotors.set_velocity(leftspeed * arm_speed_factor, PERCENT)
        RightMotors.set_velocity(rightspeed * arm_speed_factor, PERCENT)
        LeftMotors.spin(FORWARD)
        RightMotors.spin(FORWARD)

        i += 1
        rr_wait_until(start + i * RR_INTERVAL_MS)

    rr_axis3_val = 0
    rr_axis4_val = 0
    update_drive()
    rr_replaying = False
    LeftMotors.stop()
    RightMotors.stop()
    print("RR: replay done")

def rr_y_pressed_callback():
    global rr_recording
    if rr_replaying:
        return
    controller_1.screen.clear_row(3)
    controller_1.screen.set_cursor(3, 1)
    if not rr_recording:
        rr_recording = True
        controller_1.screen.print("RECORDING...")
        Thread(rr_record_thread)
    else:
        rr_recording = False  # thread finishes and prints the data

def rr_left_pressed_callback():
    if rr_recording or rr_replaying:
        return
    controller_1.screen.clear_row(3)
    controller_1.screen.set_cursor(3, 1)
    controller_1.screen.print("REPLAYING...")
    rr_replay()

controller_1.buttonY.pressed(rr_y_pressed_callback)
controller_1.buttonLeft.pressed(rr_left_pressed_callback)

# ============================================================================ #
# <<< RECORD/REPLAY ADDITION END                                               #
# ============================================================================ #


# --- 8. Autonomous ---
def autonomous(): #(robot should be placed right in the center with pin held)

    # >>> RR-EDIT (record/replay): if a recording exists, replay it instead of the scripted auto
    if USE_REPLAY_AUTON and REPLAY_DATA:
        rr_replay()
        return
    # <<< RR-EDIT END

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
    control_arm(40, 100) #hopefully consistently flips the toggle twice
    
    #ultrasafe
    """
    wait(0.1,SECONDS) #safe
    control_arm(5)   #safe
    wait(0.1,SECONDS) #safe
    control_arm(40, 30) #safe"""

    #place pin
    wait(0.1,SECONDS)
    move_bot(0.2,50)
    wait(0.1,SECONDS)
    turn_in_place(-40)
    wait(0.2,SECONDS)
    move_bot(1.2,100)
    wait(0.2,SECONDS)
    move_bot(0.2,-50)
    wait(0.4,SECONDS)
    control_arm(3)
    wait(0.2,SECONDS)
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



# --- Driver Control (Main Loop) ---

def driver_control():
    global forwardback, leftspeed, rightspeed, claw_state
    claw_sensor.set_position(0, DEGREES) #reset claw sensor to 0 at start of driver control
    controller_1.screen.clear_screen()
    controller_1.screen.print("driver control!!")

    #oldRotation = "0" #debug
    oldClaw = "0" #debug

    while True:
        
        # --- DEBUG ---
        # newRotation = str(claw_sensor.position(DEGREES)) #debug
        # if (oldRotation != newRotation):

        #     controller_1.screen.clear_row(1)
        #     controller_1.screen.set_cursor(1,1)
        #     controller_1.screen.print(newRotation)
            
        #     controller_1.screen.clear_row(2)
        #     controller_1.screen.set_cursor(2,1)
        #     controller_1.screen.print(claw_sensor.position(DEGREES))
        #     #print(str(arm_sensor.position(DEGREES)) + " " + str(newRotation))

        # oldRotation = str(claw_sensor.position(DEGREES))
        
        
        # newClaw = str(claw_sensor.position(DEGREES)) #debug
        # if (oldClaw != newClaw):
        #     brain.screen.clear_row(1)
        #     brain.screen.set_cursor(1,1)
        #     brain.screen.print(newClaw)
            
        #     controller_1.screen.clear_row(1)
        #     controller_1.screen.set_cursor(1,1)
        #     controller_1.screen.print(newClaw)
        # oldClaw = str(claw_sensor.position(DEGREES)) #debug
        
        #print(Claw.current(CurrentUnits.AMP))
        
        # --- END DEBUG ---

        # Slow the drive slightly as the arm is raised (up to 35% slower at full height)
        arm_height = max(0, min(405, Arm.position(DEGREES)))
        arm_speed_factor = 1 - 0.35 * (arm_height / 405)

        LeftMotors.set_velocity(leftspeed * arm_speed_factor, PERCENT)
        RightMotors.set_velocity(rightspeed * arm_speed_factor, PERCENT)
        
        # Drive Movement
        LeftMotors.spin(FORWARD)
        RightMotors.spin(FORWARD)
        
        wait(5, MSEC) # Small delay for stability

# --- 10. Setup and Main Execution ---

# Set random seed once
#initialize_random_seed()

# Clear console output buffer before starting main loop
print("\033[2J")
wait(200, MSEC)


# --- Event Binding ---
controller_1.buttonR1.released(controller_r1_released_callback)
controller_1.buttonR2.released(controller_r2_released_callback)
controller_1.buttonR1.pressed(controller_r1_pressed_callback)
controller_1.buttonR2.pressed(controller_r2_pressed_callback)

controller_1.axis4.changed(controller_axis_changed_callback)
controller_1.axis3.changed(controller_axis_changed_callback)
controller_1.buttonA.pressed(controller_a_pressed_callback)
controller_1.buttonB.pressed(controller_b_pressed_callback)
controller_1.buttonX.pressed(controller_x_pressed_callback)
controller_1.buttonDown.pressed(controller_down_pressed_callback)

competition = Competition(driver_control, autonomous)