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

fb_filtered = 0.0 #for exponentional moving average
turn_filtered = 0.0 #for exponentional moving average

def update_drive():
    """Reads the joysticks and recomputes left/right drive speeds (deadzone, slow mode, and ratio-preserving scaling applied)."""
    global forwardback, turning, leftspeed, rightspeed, slow_mode
    global fb_filtered, turn_filtered


    fb_raw  = controller_1.axis3.position() / 100.0
    turn_raw = controller_1.axis4.position() / 100.0

    EMA_SIGMA = 0.3 #for exponentional moving averag (lower = more smoothing)
    
    fb_filtered += (fb_raw - fb_filtered) * EMA_SIGMA 
    turn_filtered += (turn_raw - turn_filtered) * EMA_SIGMA

    # deadzone 
    DRIVE_DEADZONE = 0.02  
    
    if abs(fb_filtered) < DRIVE_DEADZONE:
        forwardback = 0
    else:
        forwardback = -math.copysign((2**abs(fb_filtered) - 1) * 100, fb_filtered) #TODO: need update notebook

    if abs(turn_filtered) < DRIVE_DEADZONE:
        turning = 0
    else:
        turning = -((3/4 * turn_filtered * abs(turn_filtered)) + (1/4 * turn_filtered)) * 100 #TODO: need update notebook

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

# --- Claw logic ---

CLAW_HOLD_DELAY_MS = 250      # hold A this long before the claw reverses
CLAW_X_SPIN_MS = 1500         # how long X spins the claw forward

claw_prev_a = False
claw_prev_x = False
claw_a_pressed_at = None
claw_reversing = False
claw_reverse_started_at = 0

def update_claw():
    """X: spin claw forward for 1.5 s, then hold. A: stop; hold A for 250 ms to spin in reverse."""
    global claw_prev_a, claw_prev_x, claw_a_pressed_at, claw_reversing, claw_reverse_started_at

    a_pressed = controller_1.buttonA.pressing()
    x_pressed = controller_1.buttonX.pressing()
    now = brain.timer.time(MSEC)

    if x_pressed and not claw_prev_x:
        Claw.spin(FORWARD, 100, PERCENT)
        claw_reversing = True
        claw_reverse_started_at = now
        claw_a_pressed_at = None

    if a_pressed and not claw_prev_a:
        claw_reversing = False
        Claw.stop(COAST)
        claw_a_pressed_at = now
    elif a_pressed and claw_a_pressed_at is not None and not claw_reversing:
        if now - claw_a_pressed_at >= CLAW_HOLD_DELAY_MS:
            Claw.spin(REVERSE, 100, PERCENT)

    if claw_prev_a and not a_pressed:
        Claw.stop(COAST)
        claw_a_pressed_at = None

    if claw_reversing and now - claw_reverse_started_at >= CLAW_X_SPIN_MS:
        Claw.stop(HOLD)
        claw_reversing = False

    claw_prev_a = a_pressed
    claw_prev_x = x_pressed

# --- Arm logic --- #TODO: smooth arm movement 

ARM_MAX_DEG = 400   # max height is 405 (here, 5 deg margin)
ARM_UP_VOLT = 6
ARM_DOWN_VOLT = 3

arm_direction = None 

def update_arm():
    """R1: raise arm. R2: lower arm."""
    global arm_direction

    if controller_1.buttonR1.pressing() and Arm.position(DEGREES) < ARM_MAX_DEG: #to avoid overextension
        target_direction = "up"
    elif controller_1.buttonR2.pressing():
        target_direction = "down"
    else: 
        target_direction = None

    if target_direction == arm_direction:
        return #only send motor commands when direction of the arm changes
    arm_direction = target_direction

    if target_direction == "up":
        Arm.spin(FORWARD, ARM_UP_VOLT, VOLT) # type: ignore #<-- ignore pylance issue; works fine
    elif target_direction == "down":
        Arm.spin(REVERSE, ARM_DOWN_VOLT, VOLT) # type: ignore
    else:
        Arm.stop()

# --- Autonomous side selector ---

auton_side = "left" #as in the direction of the colored goal
side_selector_locked = False #D-pad shoudln't change it mid-match

def draw_side_selector():
    """Shows the selected auton side on the controller screen and the brain screen."""
    controller_1.screen.clear_screen()
    controller_1.screen.set_cursor(1, 1)
    controller_1.screen.print("AUTON: " + auton_side.upper())
    controller_1.screen.set_cursor(2, 1)
    controller_1.screen.print("D-pad L/R to change")

    brain.screen.clear_screen()
    brain.screen.set_font(FontType.MONO20)
    brain.screen.set_pen_color(Color.WHITE)
    brain.screen.set_fill_color(Color.BLACK)

    brain.screen.set_font(FontType.MONO40)
    for side, x in (("left", 10), ("right", 250)):
        fill = Color.GREEN if side == auton_side else Color.BLACK
        brain.screen.set_pen_color(Color.WHITE)
        brain.screen.set_fill_color(fill)
        brain.screen.draw_rectangle(x, 45, 220, 180)
        brain.screen.set_pen_color(Color.BLACK if side == auton_side else Color.WHITE)
        brain.screen.print_at(side.upper(), x=x + 65, y=145)

def select_auton_side(side):
    """Sets the auton side (ignored once the match has started)."""
    global auton_side
    if side_selector_locked or side == auton_side:
        return
    auton_side = side
    draw_side_selector()

def controller_left_pressed_callback():
    select_auton_side("left")

def controller_right_pressed_callback():
    select_auton_side("right")

draw_side_selector()

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
    """Turns the robot in place by deg degrees using the inertial sensor (negative = left, positive = right).
    Mirrored automatically when RIGHT is selected with the controller D-pad."""
    if auton_side == "right":
        deg = -deg
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

# --- Event Callbacks (Input Handling) ---

# Drive callbacks
# def controller_axis_changed_callback():
#     """Recomputes drive speeds whenever a joystick axis changes."""
#     update_drive()

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

def controller_down_pressed_callback(): #workaround for firmware bug where motors lock up after program termination
    Arm.set_stopping(BRAKE)
    #TODO: use brain.program_stop()

# --- Driver Control (Main Loop) ---

def driver_control():
    global forwardback, leftspeed, rightspeed, side_selector_locked
    side_selector_locked = True
    claw_sensor.set_position(0, DEGREES) #reset claw sensor to 0 at start of driver control
    controller_1.screen.clear_screen()
    controller_1.screen.print("driver control!!")

    while True:
        update_drive()
        update_claw()
        update_arm()

        # --- DEBUG ---
        #oldRotation = "0" #debug
        #oldClaw = "0" #debug

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
# controller_1.axis4.changed(controller_axis_changed_callback)
# controller_1.axis3.changed(controller_axis_changed_callback)
#controller_1.buttonA.pressed(controller_a_pressed_callback)
controller_1.buttonB.pressed(controller_b_pressed_callback)
#controller_1.buttonX.pressed(controller_x_pressed_callback)
controller_1.buttonDown.pressed(controller_down_pressed_callback)
controller_1.buttonLeft.pressed(controller_left_pressed_callback)
controller_1.buttonRight.pressed(controller_right_pressed_callback)


# add a small delay to make sure we don't print in the middle of the REPL header
wait(200, MSEC)
# clear the console to make sure we don't have the REPL in the console
print("\033[2J")


# --- Main autonomous routine ---
def autonomous(): #(robot should be placed right in the center with pin held)
    global side_selector_locked
    side_selector_locked = True
    
    #Free toggle mechanism (initially tied down by rubber bands for space)
    move_bot(0.2, 60)
    wait(0.2, SECONDS)
    control_arm(70, 50)
    wait(0.1, SECONDS)
    control_arm(0, 20)

    move_bot(0.2, -60)
    wait(0.2, SECONDS)
    control_arm(25, 20)

    wait(0.1, SECONDS)
    control_arm(5)
    wait(0.1, SECONDS)
    move_bot(0.3, -20)
    control_arm(35, 20)

    wait(0.1, SECONDS)
    move_bot(0.2, 50)
    wait(0.1, SECONDS)
    turn_in_place(-40)
    wait(0.2, SECONDS)
    move_bot(1.4, 50)
    wait(0.2, SECONDS)
    move_bot(0.35, -20)
    wait(0.4, SECONDS)
    control_arm(5, 20)
    wait(0.2, SECONDS)
    move_bot(0.125, -30)
    Claw.spin(FORWARD, 100, PERCENT)
    wait(1, SECONDS)
    Claw.stop()

    move_bot(0.2, -50)
    wait(0.2, SECONDS)

# Clear console output buffer before starting main loop
print("\033[2J")
wait(200, MSEC)

competition = Competition(driver_control, autonomous)
