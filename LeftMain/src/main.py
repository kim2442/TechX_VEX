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

# add a small delay to make sure we don't print in the middle of the REPL header
wait(200, MSEC)
# clear the console to make sure we don't have the REPL in the console
print("\033[2J")

#endregion VEXcode Generated Robot Configuration

# --- Robot and Motor Configuration ---

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

# Display precision settings
screen_precision = 0
console_precision = 0
controller_precision = 0


# --- Utility Functions  ---

# def initialize_random_seed():
#     wait(100, MSEC)
#     # Use voltage and current to create a unique seed
#     seed_value = brain.battery.voltage(MV) + brain.battery.current(CurrentUnits.AMP) * 100 + brain.timer.system_high_res()
#     urandom.seed(int(seed_value))
#     print("Random seed initialized.")

def console_format(variable):
    """Formats a variable for printing with specified decimal precision."""
    if isinstance(variable, str):
        return variable
    
    # Determine precision
    precision = 6 if console_precision is None else console_precision
    
    # Format the number
    return "{0:.{1}f}".format(variable, precision)


# --- Drive logic ---

forwardback = 0.0
turning = 0.0
leftspeed = 0.0
rightspeed = 0.0
#claw_state: int = 1 
is_claw_open = True #experimental

slow_mode = False
#Claw.set_timeout(1.3, SECONDS)
#Claw.set_stopping(HOLD)
Claw.set_velocity(70,PERCENT)
arm_sensor.set_position(0, DEGREES)
Arm.set_position(0, DEGREES)
Arm.set_stopping(HOLD) 

def update_drive():
    """Reads the joysticks and recomputes left/right drive speeds (deadzone, slow mode, and ratio-preserving scaling applied)."""
    global forwardback, turning, leftspeed, rightspeed, slow_mode

    fb_raw  = controller_1.axis3.position() / 100.0
    turn_raw = controller_1.axis4.position() / 100.0
    forwardback  = - math.copysign (1, fb_raw) * ((2** abs(fb_raw))-1)*100 
    turning = (-((3/4 * (turn_raw) * abs(turn_raw)) + (1/4 * turn_raw))*100) #cubic scaling for finer control at low speeds 
    
        
    # deadzone 
    if abs(fb_raw) < 0.02:
        forwardback = 0
    if abs(turn_raw) < 0.02:
        turning = 0

    # Slow mode: scale inputs down for finer control
    if slow_mode:
        turning  *= 0.5
        forwardback *= 0.7

    leftspeed = forwardback + turning
    rightspeed = turning - forwardback

    # Keep the ratio between the two sides
    max_speed = max(abs(leftspeed), abs(rightspeed))

    if max_speed > 100:
        leftspeed = leftspeed / max_speed * 100
        rightspeed = rightspeed / max_speed * 100
# --- Autonomous logic ---

def move_bot(t, speed): 
    """Drives straight for t seconds at the given speed percent (negative speed drives backward)."""
    LeftMotors.set_velocity(speed, PERCENT)
    RightMotors.set_velocity(speed, PERCENT)
    LeftMotors.spin(REVERSE)
    RightMotors.spin(FORWARD)
    wait(t, SECONDS)
    LeftMotors.stop()
    RightMotors.stop()

def turn_in_place(deg):
    """Turns the robot in place by deg degrees using the inertial sensor (negative = left, positive = right)."""
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
    """Moves the arm to raise_pct percent of its maximum height (405 degrees) at the given velocity percent."""
    #Maximum rotation of arm_sensor is 105 degrees
    #Maximum rotation of Arm is 405 degrees
    goal_rotation = (raise_pct/100) * 405
    Arm.spin_to_position(goal_rotation, DEGREES, velocity, PERCENT)

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


# --- Event Callbacks (Input Handling) ---

# Drive callbacks
def controller_axis_changed_callback():
    """Recomputes drive speeds whenever a joystick axis changes."""
    update_drive()

def controller_b_pressed_callback():
    """Toggles slow mode for finer control."""
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
        Arm.spin(FORWARD, 6, VOLT) # type: ignore #ignore pylance issue; works fine

def controller_r1_released_callback():
    """Stops moving arm when R1 button is released."""
    Arm.stop()

def controller_r2_pressed_callback():
    """Callback for when the R2 button is pressed (Reverse Arm)."""
    Arm.spin(REVERSE, 3, VOLT)  # type: ignore
    
def controller_r2_released_callback():
    """Stops moving arm when R2 button is released."""
    Arm.stop()


def controller_down_pressed_callback(): #workaround for firmware bug where motors lock up after program termination
    Arm.set_stopping(BRAKE)
    #TODO: use brain.program_stop()

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
    #control_claw(is_wide=True)



# --- Driver Control (Main Loop) ---

def driver_control():
    global forwardback, leftspeed, rightspeed, claw_state
    claw_sensor.set_position(0, DEGREES) #reset claw sensor to 0 at start of driver control
    controller_1.screen.clear_screen()
    controller_1.screen.print("driver control!!")

    #oldRotation = "0" #debug
    #oldClaw = "0" #debug

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

# --- Setup and Main Execution ---

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

# Clear console output buffer before starting main loop
print("\033[2J")
wait(200, MSEC)

competition = Competition(driver_control, autonomous)
