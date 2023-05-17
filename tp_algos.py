#!/usr/bin/python3
'''
    CentraleSupelec TP 2A/3A
    Aarsh THAKKER,2023
    (all variables in SI unit)

###########################################################################################
============================ READ THIS BEFORE STARTING TO CODE ============================
    You ONLY modify the part that is marked "to be modified" in the functions
    variables used by the functions of this script
        - robotNo: no of the current robot of same type for which control is coputed (1 .. nbRobots)
        - nbTB3: number of total tb3 robots in the fleet (>=0)
        - nbCF: number of total crazyflie nano drones in the fleet (>=0)
        - nbRMTT: number of total dji robomaster TT drones in the fleet (>=0)
        - nbRMS1: number of total dji robomaster S1 in the fleet (>=0)  (YOU CAN ONLY CONTROL THIS ROBOT MANUALLY USING YOUR MOBILE PHONE AND GET POSITION IN HERE TO BE USED)
        - nbOBSTACLE: number of total obstacle positions in the environment (>=0)
        - tb3_poses:  size (3 x nbTB3)
            eg. of use: for robot number 'robotNo', position of the robot can be obtained by:
                    tb3_poses[:,robotNo-1]   (indexes in Python start from 0 !)
                    tb3_poses[0,robotNo-1]: x-coordinate of robot position (in m)
                    tb3_poses[1,robotNo-1]: y-coordinate of robot position (in m)
                    tb3_poses[2,robotNo-1]: orientation angle of robot (in rad)
        - cf_poses:  size (3 x nbCF)
            eg. of use: for robot number 'robotNo', position of the robot can be obtained by:
                    cf_poses[:,robotNo-1]   (indexes in Python start from 0 !)
                    cf_poses[0,robotNo-1]: x-coordinate of robot position (in m)
                    cf_poses[1,robotNo-1]: y-coordinate of robot position (in m)
                    cf_poses[2,robotNo-1]: z-coordinate of robot position (in m)
        - rmtt_poses:  size (3 x nbRMTT)
            eg. of use: for robot number 'robotNo', position of the robot can be obtained by:
                    rmtt_poses[:,robotNo-1]   (indexes in Python start from 0 !)
                    rmtt_poses[0,robotNo-1]: x-coordinate of robot position (in m)
                    rmtt_poses[1,robotNo-1]: y-coordinate of robot position (in m)
                    rmtt_poses[2,robotNo-1]: z-coordinate of robot position (in m)
        - rms1_poses:  size (3 x nbRMS1)
            eg. of use: for robot number 'robotNo', position of the robot can be obtained by:
                    tb3_poses[:,robotNo-1]   (indexes in Python start from 0 !)
                    tb3_poses[0,robotNo-1]: x-coordinate of robot position (in m)
                    tb3_poses[1,robotNo-1]: y-coordinate of robot position (in m)
                    tb3_poses[2,robotNo-1]: orientation angle of robot (in rad)
        - obstacle_pose:  size (5 x nbOBSTACLE)
                    This can be used to define cube/sphere shaped obstacle in the environment.
                    obstacle_pose[:,nbOBSTACLE-1]   (indexes in Python start from 0 !)
                    obstacle_pose[0,nbOBSTACLE-1]: x-coordinate of center position of obstacle (in m)
                    obstacle_pose[1,nbOBSTACLE-1]: y-coordinate of center position of obstacle (in m)
                    obstacle_pose[2,nbOBSTACLE-1]: z-coordinate of center position of obstacle (in m)
                    obstacle_pose[3,nbOBSTACLE-1]: size of the obstacle (from center to the side of the cube/radius of sphere)
                    obstacle_pose[4,nbOBSTACLE-1]: type of obstacle (0= sphere, 1= cube)


###########################################################################################

'''

import numpy as np
import math
import time

# ==============   "GLOBAL" VARIABLES KNOWN BY ALL THE FUNCTIONS ===================
# all variables declared here will be known by functions below
# use keyword "global" inside a function if the variable needs to be modified by the function

# depend on Fleet definition in Simu.py
nbCF = 3
nbTB3 = 1

# in Mode possibility for crazyfly
# 0 : vol en formation
# 1 : descente
# 2 : remonte
# 3 : tourne autour
# TODO: anticollision

Mode = np.zeros(nbCF+nbTB3)

cf_mask = np.zeros(Mode.shape, dtype=bool)
cf_mask[:nbCF] = True
tb3_mask = np.zeros(Mode.shape, dtype=bool)
tb3_mask[nbCF:] = True

dist_inter_cf_mode_0 = 2
t0 = time.time()

list_resources = []
attrib_resources = {i:None for i in range(nbCF)}


def reg(n):
    global dist_inter_cf_mode_0

    Poses = [(0, 0, 0)]
    r = dist_inter_cf_mode_0/(2*np.cos(np.pi/n))
    for i in range(n):
        Poses.append((r*np.cos(2*np.pi*i/n), r*np.sin(2*np.pi*i/n), 0))
    return Poses


def ref(t):
    return (-4, 0, 1.5)


def get_absolute_index(robotNo, cf_poses, robot_type):
    if robot_type == "Crazy":
        return robotNo-1
    if robot_type == "Turtle":
        return robotNo-1 + len(cf_poses[0])


# ===================================================================================

# Control function for turtlebot3 ground vehicle to be modified
# should ONLY return (vx,vy) for the robot command
# max useable numbers of robots = 6 (1 for waffle model)
# ====================================


def tb3_control_fn(robotNo, tb3_poses, cf_poses, rmtt_poses, rms1_poses, obstacle_pose):
    # ====================================

    nbTB3 = len(tb3_poses[0])  # number of total tb3 robots in the use
    nbCF = len(cf_poses[0])  # number of total crazyflie nano drones in the use
    nbRMTT = len(rmtt_poses[0])  # number of total dji rmtt drones in the use
    nbRMS1 = len(rms1_poses[0])  # number of total dji rms1 in the use
    # number of total obstacle positions in the environment
    nbOBSTACLE = len(obstacle_pose[0])

    #  --- TO BE MODIFIED ---
    vx = 1.0
    vy = 0.0
    # -----------------------

    return vx, vy
# ====================================


# ====================================
# Control function for crazyflie nano drones to be modified
# should ONLY return (vx,vy,vz,wz,takeoff,land) for the robot command
# max useable numbers of drones = 3
# ====================================
def cf_control_fn(robotNo, tb3_poses, cf_poses, rmtt_poses, rms1_poses, obstacle_pose):
    # ====================================
    nbTB3 = len(tb3_poses[0])  # number of total tb3 robots in the use
    nbCF = len(cf_poses[0])  # number of total crazyflie nano drones in the use
    nbRMTT = len(rmtt_poses[0])  # number of total dji rmtt drones in the use
    nbRMS1 = len(rms1_poses[0])  # number of total dji rms1 in the use
    # number of total obstacle positions in the environment
    nbOBSTACLE = len(obstacle_pose[0])

    global t0
    global list_resources
    global attrib_resources

    #  --- TO BE MODIFIED ---
    vx = 0.0
    vy = 0.0
    vz = 0.0
    takeoff = False
    land = False

    kp = 1
    ko = 10
    Poses = reg(nbCF)
    abs_cf_id = get_absolute_index(robotNo, cf_poses, "Crazy")

    if Mode[abs_cf_id] == 0:
        Poses = reg(Mode[Mode == 0].shape[0])
        robot_in_same_mode = np.logical_and(Mode == 0, cf_mask)
        for i in np.where(robot_in_same_mode)[0]:
            if i != robotNo:
                vx += -kp*(cf_poses[0, robotNo-1] - cf_poses[0,
                           i-1] - (Poses[abs_cf_id+1][0] - Poses[i][0]))
                vy += -kp*(cf_poses[1, robotNo-1] - cf_poses[1,
                           i-1] - (Poses[abs_cf_id+1][1] - Poses[i][1]))
                vz += -kp*(cf_poses[2, robotNo-1] - cf_poses[2,
                           i-1] - (Poses[abs_cf_id+1][2] - Poses[i][2]))

        Ref = ref(0)
        vx += -ko*(cf_poses[0, robotNo-1] - Ref[0] -
                   (Poses[robotNo][0] - Poses[0][0]))
        vy += -ko*(cf_poses[1, robotNo-1] - Ref[1] -
                   (Poses[robotNo][1] - Poses[0][1]))
        vz += -ko*(cf_poses[2, robotNo-1] - Ref[2] -
                   (Poses[robotNo][2] - Poses[0][2]))

    elif Mode[abs_cf_id] == 1:
        pass
    elif Mode[abs_cf_id] == 2:
        pass
    elif Mode[abs_cf_id] == 3:
        pass

    # --- TRANSITIONS ---

    # From 0 (formation) to 1 (descente)
    for resource in list_resources:
        if (Mode[abs_cf_id] == 0) and (np.linalg.norm(ref(time.time()-t0)[:2] - resource[:2]) < 1.1) and (abs_cf_id == np.where(Mode == 0)[0][0]):
            list_resources.remove(resource)
            attrib_resources[abs_cf_id] = resource  # resource becomes the drone's target
            print('Drone ' + str(abs_cf_id) + ' is getting the resource at ' + str(resource))
            Mode[abs_cf_id] = 1
    
    # From 1 (descente) to 2 (remonte)
    if (Mode[abs_cf_id] == 1) and (np.linalg.norm(cf_poses[:,robotNo-1] - resource) < 0.3):
        print('The resource at ' + str(attrib_resources[abs_cf_id]) + ' was collected by drone ' + str(abs_cf_id))
        Mode[abs_cf_id] = 2
    
    # From 2 (remonte) to 3 (retourne a la formation)
    tb3_poses[2] = 0
    if (Mode[abs_cf_id] == 2) and (np.linalg.norm(cf_poses[:,robotNo-1] - tb3_poses) < 0.4):
        print('The resource that was at ' + str(attrib_resources[abs_cf_id]) + ' was delivered by drone ' + str(abs_cf_id))
        attrib_resources[abs_cf_id] = None
        Mode[abs_cf_id] = 3
    
    # From 3 (retourne a la formation) to 0 (formation)
    if (Mode[abs_cf_id] == 3) and (np.linalg.norm(cf_poses[:,robotNo-1] - ref(time.time()-t0)) < 1):
        print('Drone ' + str(abs_cf_id) + ' has returned to formation')
        Mode[abs_cf_id] = 0

    # -----------------------

    return vx, vy, vz, takeoff, land
# ====================================


# ====================================
# Control function for dji rmtt drones to be modified
# should ONLY return (vx,vy,vz,wz,takeoff,land,led) for the robot command
# max useable numbers of drones = 1
# ====================================
def rmtt_control_fn(robotNo, tb3_poses, cf_poses, rmtt_poses, rms1_poses, obstacle_pose):
    # ====================================
    nbTB3 = len(tb3_poses[0])  # number of total tb3 robots in the use
    nbCF = len(cf_poses[0])  # number of total crazyflie nano drones in the use
    nbRMTT = len(rmtt_poses[0])  # number of total dji rmtt drones in the use
    nbRMS1 = len(rms1_poses[0])  # number of total dji rms1 in the use
    # number of total obstacle positions in the environment
    nbOBSTACLE = len(obstacle_pose[0])

    #  --- TO BE MODIFIED ---
    vx = 0.0
    vy = 0.0
    vz = 0.0
    takeoff = False
    land = False
    led = [0, 0, 0]  # RGB
    # -----------------------

    return vx, vy, vz, takeoff, land, led
# ====================================


# ======== ! DO NOT MODIFY ! ============
def tb3_controller(robotNo, poses, cf_poses, rmtt_poses, rms1_poses, obstacle_pose):
    vx, vy = tb3_control_fn(robotNo, poses, cf_poses,
                            rmtt_poses, rms1_poses, obstacle_pose)
    return vx, vy


def cf_controller(robotNo, tb3_poses, cf_poses, rmtt_poses, rms1_poses, obstacle_pose):
    vx, vy, vz, takeoff, land = cf_control_fn(
        robotNo, tb3_poses, cf_poses, rmtt_poses, rms1_poses, obstacle_pose)
    return vx, vy, vz, takeoff, land


def rmtt_controller(robotNo, tb3_poses, cf_poses, rmtt_poses, rms1_poses, obstacle_pose):
    vx, vy, vz, takeoff, land, led = rmtt_control_fn(
        robotNo, tb3_poses, cf_poses, rmtt_poses, rms1_poses, obstacle_pose)
    return vx, vy, vz, takeoff, land, led
# =======================================
