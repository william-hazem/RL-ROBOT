#   +-----------------------------------------------+
#   | RL-ROBOT. Reinforcement Learning for Robotics |
#   | William Henrique A. Martins                   |
#   | Federal University of Campina Grande, 2024    |
#   +-----------------------------------------------+
""" Link with Compelia sim using ZMQ API simulator """
import time
import sys
import numpy as np

from third.coppeliasim_zmqremoteapi_client import RemoteAPIClient

import vrep  # Vrep API library (todo: remove)

client = None
# client.timeout = 5
# client.connect()

sim = None

# When simulation is not running, ZMQ message handling could be a bit
# slow, since the idle loop runs at 8 Hz by default. So let's make
# sure that the idle loop runs at full speed for this program:
# defaultIdleFps = sim.getInt32Param(sim.intparam_idle_fps)
# sim.setInt32Param(sim.intparam_idle_fps, 0)

WAIT_RESPONSE = False  # True: Synchronous response (too much delay)

LASER_DISTRIBUTION = (
    "sensor_front",
    "sensor_front_left",
    "sensor_left",
    "sensor_rear_left",
    "sensor_rear",
    "sensor_rear_right",
    "sensor_right",
    "sensor_front_right",
)
HAS_KINECT = False
HAS_ARM = False
HAS_GOAL_OBJECT = False

# V-REP data transmission modes:
WAIT = vrep.simx_opmode_oneshot_wait
ONESHOT = vrep.simx_opmode_oneshot
STREAMING = vrep.simx_opmode_streaming
BUFFER = vrep.simx_opmode_buffer

if WAIT_RESPONSE:
    MODE_INI = WAIT
    MODE = WAIT
else:
    MODE_INI = STREAMING
    MODE = BUFFER

N_LASERS = 8  # 1 point laser each

robotID = -1
laserID = [-1] * N_LASERS
left_motorID = -1
right_motorID = -1
clientID = -1

armID = -1
bicepsID = -1
forearmID = -1
gripperID = -1

ballID = -1  # Goal object to reach for 2d navigation and 3d arm motion tasks

kinect_rgb_ID = -1  # not used so far
kinect_depth_ID = -1  # not used so far

distance = np.full(N_LASERS, -1, dtype=np.float64)  # distances from lasers (m)
pose = np.full(3, -1, dtype=np.float64)  # Pose 2d base: x(m), y(m), theta(rad)

# @todo: implement message communication
def show_msg(message):
    """send a message for printing in ZMQ"""
    # sim.simxAddStatusbarMessage(clientID, message, WAIT)
    sim.addStatusbarMessage(message)
    return


def connect():
    global client, sim
    client = RemoteAPIClient()
    sim = client.getObject('sim')
    """Connect to the simulator"""
    ip = "127.0.0.1"
    port = 19997
    
    sim.stopSimulation()        # closes previous simulation
    sim.startSimulation()
    # result, sim_state = client.simxGetSimulationState()
    # result, sim_state = client.simxGetIntegerSignal('simRun', client.clientID)
    # clientID = client.clientID
    # clientID = vrep.simxStart(ip, port, True, True, 3000, 5)
    # Connect to V-REP
    # if sim_state == 0:
    #     import sys

    #     sys.exit(
    #         "\nZMQ remote API server connection failed ("
    #         + ip
    #         + ":"
    #         + str(port)
    #         + "). Is V-REP running?"
    #     )
    print("Connected to Robot")
    # show_msg("Python: Hello")
    time.sleep(0.5)
    return


def disconnect():
    """Disconnect from the simulator"""
    # Make sure that the last command sent has arrived
    # vrep.simxGetPingTime(clientID)
    # show_msg("RL-ROBOT: Bye")
    # Now close the connection to V-REP:
    # vrep.simxFinish(clientID)
    sim.stopSimulation()
    time.sleep(0.5)
    return


def start():
    """Start the simulation (force stop and setup)"""
    stop()
    setup_devices()
    sim.startSimulation()
    # vrep.simxStartSimulation(clientID, ONESHOT)
    time.sleep(0.5)
    # Solve a rare bug in the simulator by repeating:
    # setup_devices()
    # vrep.simxStartSimulation(clientID, ONESHOT)
    # time.sleep(0.5)
    return


def stop():
    """Stop the simulation"""
    # vrep.simxStopSimulation(clientID, ONESHOT)
    wait = False
    sim.stopSimulation(wait)
    time.sleep(0.5)


def setup_devices():
    """Assign the devices from the simulator to specific IDs"""
    global robotID, left_motorID, right_motorID, laserID
    global armID, bicepsID, forearmID, gripperID
    global kinect_rgb_ID, kinect_depth_ID
    global ballID
    global sim

    # differently from vrep legacy api, zmq version specify Coppelia as topics under root folder (/<Object>)
    # e.g. to get robot's handle it's necessary to specify '/' before object name
    robotID = sim.getObject("/robot")

    # motors
    left_motorID = sim.getObject("/robot/leftMotor")
    right_motorID = sim.getObject("/robot/rightMotor")
    # lasers
    for idx, item in enumerate(LASER_DISTRIBUTION):
        laserID[idx] = sim.getObject('/' + item)
    # arm
    if HAS_ARM:
        armID     = sim.getObject("/arm_joint")
        bicepsID  = sim.getObject("/biceps_joint")
        forearmID = sim.getObject("/forearm_joint")
        gripperID = sim.getObject("/gripper_1_visual")
    # Kinect
    if HAS_KINECT:
        kinect_rgb_ID   = sim.getObject("/kinect_rgb")
        kinect_depth_ID = sim.getObject("/kinect_depth")
    # ball
    if HAS_GOAL_OBJECT:
        ballID = sim.getObject("/Ball")
    # start up devices

    # wheels
    sim.setJointTargetVelocity(left_motorID, 0)
    sim.setJointTargetVelocity(right_motorID, 0)
    # pose
    # vrep.simxGetObjectPosition(clientID, robotID, -1, MODE_INI)
    # vrep.simxGetObjectOrientation(clientID, robotID, -1, MODE_INI)
    # distances from lasers
    for i in laserID:
        # vrep.simxReadProximitySensor(clientID, i, MODE_INI)
        continue # @todo: update above line
    if HAS_ARM:
        sim.setJointTargetVelocity(armID    , 0)
        sim.setJointTargetVelocity(bicepsID , 0)
        sim.setJointTargetVelocity(forearmID, 0)
        sim.getObjectPosition(gripperID, -1)

    if HAS_GOAL_OBJECT:
        # vrep.simxGetObjectPosition(clientID, ballID, -1, MODE_INI)
        sim.getObjectPosition(ballID, -1)

    if HAS_KINECT:
        raise "Not implemented to ZMQ" #todo: implement this to zmq api
        rc, resolution, image = vrep.simxGetVisionSensorImage(
            clientID, kinect_rgb_ID, 0, MODE_INI
        )
        im = np.array(image, dtype=np.uint8)
        im.resize([resolution[1], resolution[0], 3])
        # plt.imshow(im, origin='lower')
        # return_code, resolution, depth = vrep.simxGetVisionSensorImage(
        #     clientID, kinect_depth_ID, 0, MODE_INI)
        # de = np.array(depth)
        time.sleep(0.5)
    return


def get_image_rgb():
    """Get RGB image from a Kinect"""
    raise "Not implemented to ZMQ" #todo: implement this to zmq api
    rc, resolution, image = vrep.simxGetVisionSensorImage(
        clientID, kinect_rgb_ID, 0, MODE
    )

    im = np.array(image, dtype=np.uint8)
    im.resize([resolution[1], resolution[0], 3])
    # im.shape
    # plt.imshow(im,origin='lower')
    return im


def get_image_depth():
    """get image Depth from a Kinect"""
    raise "Not implemented to ZMQ" #todo: implement this to zmq api
    rc, resolution, depth = vrep.simxGetVisionSensorImage(
        clientID, kinect_depth_ID, 0, MODE
    )
    de = np.array(depth)
    return de


def get_mobilebase_pose2d():
    """return the pose of the robot:  [ x(m), y(m), Theta(rad) ]"""
    pos = sim.getObjectPosition(robotID, -1)
    ori = sim.getObjectOrientation(robotID, -1)
    pos = np.array([pos[0], pos[1], ori[2]])
    return pos


def get_distance_obstacle():
    """return an array of distances measured by lasers (m)"""
    for i in range(0, N_LASERS):
        # int res, float dist, list point, int obj, list n = sim.readProximitySensor(int sensorHandle)
        rc, ds, detected_point, doh, dsn = sim.readProximitySensor(laserID[i])
        distance[i] = detected_point[2]
    return distance


def move_wheels(v_left, v_right):
    """move the wheels. Input: Angular velocities in rad/s"""
    sim.setJointTargetVelocity(left_motorID, v_left)
    sim.setJointTargetVelocity(right_motorID, v_right)
    return


def stop_motion():
    """stop the base wheels"""
    return move_wheels(0.0, 0.0)


def move_arm(w):
    """move arm joint. Angular velocities in rad/s (+anticlockwise)"""
    sim.setJointTargetVelocity(armID, w)


def move_biceps(w):
    """move biceps joint. Angular velocities in rad/s(+anticlockwise)"""
    sim.setJointTargetVelocity(bicepsID, w)


def move_forearm(w):
    """move forearm joint. Angular velocities in rad/s (+anticlockwise)"""
    sim.setJointTargetVelocity(forearmID, w)

def stop_arm_all():
    """stop arm joints"""
    move_arm(0)
    move_biceps(0)
    move_forearm(0)


def get_gripper_pose3d():
    """Returns the position of the gripper:  [ x(m), y(m), z(m) ]"""
    pos = sim.getObjectPosition(gripperID, -1)
    return np.array(pos)


def get_goal_pose_3d():
    """Returns the position of the goal object:  [ x(m), y(m), z(m) ]"""
    pos = sim.getObjectPosition(ballID, -1)
    return np.array(pos)
