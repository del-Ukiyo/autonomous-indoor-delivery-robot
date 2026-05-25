from controller import Robot, Camera, LED
import math

# Constants
TIME_STEP = 64
MAX_SPEED = 6.28

# IR sensor thresholds
OBSTACLE_THRESHOLD = 150.0
EMERGENCY_THRESHOLD = 3500.0

# Camera resolution (official e-puck specs)
CAM_WIDTH = 52
CAM_HEIGHT = 39

# Red colour detection thresholds
RED_R_MIN = 100
RED_G_MAX = 70
RED_B_MAX = 70

# Odometry values (official e-puck specs)
WHEEL_RADIUS = 0.0205
AXLE_WIDTH = 0.052

# FSM States
WANDER = "WANDER"
AVOID = "AVOID"
TARGET_SEEK = "TARGET_SEEK"
ARRIVAL = "ARRIVAL"

# Robot setup
robot = Robot()

# Motors
left_motor = robot.getDevice('left wheel motor')
right_motor = robot.getDevice('right wheel motor')
left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))
left_motor.setVelocity(0.0)
right_motor.setVelocity(0.0)

# IR Sensors
ps = []
for i in range(8):
    s = robot.getDevice(f'ps{i}')
    s.enable(TIME_STEP)
    ps.append(s)

# Camera
camera = robot.getDevice('camera')
camera.enable(TIME_STEP)

# Body LED signals delivery complete
led = robot.getDevice('led8')

# Wheel encoders for tracking position
left_enc = robot.getDevice('left wheel sensor')
right_enc = robot.getDevice('right wheel sensor')
left_enc.enable(TIME_STEP)
right_enc.enable(TIME_STEP)

# State variables
state = WANDER
pos_x, pos_y, heading = 0.0, 0.0, 0.0
prev_enc_l = prev_enc_r = 0.0
prev_ir = [0.0] * 8
stuck_counter = 0


def read_sensors():
    # Read all 8 IR sensors and apply a rolling average to smooth out noise
    global prev_ir
    raw = [ps[i].getValue() for i in range(8)]
    smoothed = [(raw[i] + prev_ir[i]) / 2.0 for i in range(8)]
    prev_ir = raw
    front = max(smoothed[0], smoothed[7])
    right = max(smoothed[1], smoothed[2])
    left  = max(smoothed[5], smoothed[6])
    return front, right, left, smoothed[0], smoothed[7]


def see_red():
    # Scan the camera image pixel by pixel looking for red pixels
    # Returns whether red is detected and which direction to steer toward it
    image = camera.getImage()
    red_count = 0
    centroid_x = 0
    for x in range(CAM_WIDTH):
        for y in range(CAM_HEIGHT):
            r = Camera.imageGetRed(image, CAM_WIDTH, x, y)
            g = Camera.imageGetGreen(image, CAM_WIDTH, x, y)
            b = Camera.imageGetBlue(image, CAM_WIDTH, x, y)
            if r > RED_R_MIN and g < RED_G_MAX and b < RED_B_MAX:
                red_count += 1
                centroid_x += x
    ratio = red_count / (CAM_WIDTH * CAM_HEIGHT)
    detected = ratio >= 0.05
    offset = 0.0
    if red_count > 0:
        offset = (centroid_x / red_count - CAM_WIDTH / 2) / (CAM_WIDTH / 2)
    return detected, offset


def update_position():
    # Track the robots position using wheel encoder readings
    # Uses the difference in left and right wheel distances to calculate movement
    global pos_x, pos_y, heading, prev_enc_l, prev_enc_r
    l = left_enc.getValue()
    r = right_enc.getValue()
    dl = (l - prev_enc_l) * WHEEL_RADIUS
    dr = (r - prev_enc_r) * WHEEL_RADIUS
    prev_enc_l, prev_enc_r = l, r
    dist = (dl + dr) / 2.0
    heading += (dr - dl) / AXLE_WIDTH
    pos_x += dist * math.cos(heading)
    pos_y += dist * math.sin(heading)


def drive(left, right):
    # Set motor speeds and make sure they dont exceed the maximum
    left_motor.setVelocity(max(-MAX_SPEED, min(MAX_SPEED, left)))
    right_motor.setVelocity(max(-MAX_SPEED, min(MAX_SPEED, right)))


def recover():
    # If the robot gets stuck reverse then turn to get free
    # Reset odometry after recovery to stop position drift building up
    global pos_x, pos_y, heading, prev_enc_l, prev_enc_r
    print("RECOVERY: Reversing...")
    drive(-MAX_SPEED * 0.5, -MAX_SPEED * 0.5)
    for _ in range(20):
        robot.step(TIME_STEP)
    print("RECOVERY: Turning...")
    drive(MAX_SPEED * 0.6, -MAX_SPEED * 0.6)
    for _ in range(25):
        robot.step(TIME_STEP)
    pos_x, pos_y, heading = 0.0, 0.0, 0.0
    prev_enc_l = left_enc.getValue()
    prev_enc_r = right_enc.getValue()
    print("Odometry reset after recovery")


# Main loop - runs every simulation timestep
while robot.step(TIME_STEP) != -1:

    # Read sensors each timestep
    front, right, left, fr, fl = read_sensors()
    red_detected, red_offset = see_red()
    update_position()

    # Emergency stop if something is dangerously close in front
    if front > EMERGENCY_THRESHOLD:
        drive(0.0, 0.0)
        print("EMERGENCY STOP")
        continue

    # Count how long the robot has been near an obstacle
    if front > OBSTACLE_THRESHOLD * 0.7:
        stuck_counter += 1
    else:
        stuck_counter = 0

    # If stuck for too long trigger the recovery routine
    if stuck_counter > 60:
        recover()
        stuck_counter = 0
        continue

    # FSM - decide what to do based on current state and sensor readings

    if state == ARRIVAL:
        # Robot has reached the target - stop and turn on the LED
        drive(0.0, 0.0)
        led.set(1)

    elif red_detected:
        # Camera can see the red target
        if front > 50:
            # Close enough to the target - switch to arrival
            state = ARRIVAL
            print("STATE: ARRIVAL - Delivery complete!")
            print(f"Final position: ({pos_x:.2f}, {pos_y:.2f})")
        else:
            # Steer toward the red target using the camera centroid
            state = TARGET_SEEK
            turn = red_offset * MAX_SPEED * 0.5
            drive(MAX_SPEED * 0.4 - turn, MAX_SPEED * 0.4 + turn)
            print("STATE: TARGET_SEEK")

    elif front > OBSTACLE_THRESHOLD:
        # Obstacle detected in front - steer away from it
        state = AVOID
        if fr > fl:
            # Obstacle is more on the right so turn left
            drive(MAX_SPEED * 0.15, MAX_SPEED * 0.7)
        else:
            # Obstacle is more on the left so turn right
            drive(MAX_SPEED * 0.7, MAX_SPEED * 0.15)
        print("STATE: AVOID")

    else:
        # No obstacle and no target visible - follow the right wall down the corridor
        state = WANDER
        if right < 50:
            # Too far from the right wall so steer toward it
            drive(MAX_SPEED * 0.7, MAX_SPEED * 0.4)
        elif right > 200:
            # Too close to the right wall so steer away
            drive(MAX_SPEED * 0.4, MAX_SPEED * 0.7)
        else:
            # Good distance from the wall so go straight
            drive(MAX_SPEED * 0.7, MAX_SPEED * 0.7)
        print(f"STATE: WANDER - Position: ({pos_x:.2f}, {pos_y:.2f})")