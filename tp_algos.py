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

# ==============   "GLOBAL" VARIABLES KNOWN BY ALL THE FUNCTIONS ===================
# all variables declared here will be known by functions below
# use keyword "global" inside a function if the variable needs to be modified by the function


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
    vx = 0.0
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

    #  --- TO BE MODIFIED ---
    vx = 0.0
    vy = 0.0
    vz = 0.0
    takeoff = False
    land = False
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
def tb3_controller(robotNo, nbRobots, poses, cf_poses, rmtt_poses, rms1_poses, obstacle_pose):
    vx, vy = tb3_control_fn(robotNo, nbRobots, poses,
                            cf_poses, rmtt_poses, rms1_poses, obstacle_pose)
    return vx, vy


def cf_controller(robotNo, nbRobots, tb3_poses, cf_poses, rmtt_poses, rms1_poses, obstacle_pose):
    vx, vy, vz, takeoff, land = cf_control_fn(
        robotNo, nbRobots, tb3_poses, cf_poses, rmtt_poses, rms1_poses, obstacle_pose)
    return vx, vy, vz, takeoff, land


def rmtt_controller(robotNo, nbRobots, tb3_poses, cf_poses, rmtt_poses, rms1_poses, obstacle_pose):
    vx, vy, vz, takeoff, land, led = rmtt_control_fn(
        robotNo, nbRobots, tb3_poses, cf_poses, rmtt_poses, rms1_poses, obstacle_pose)
    return vx, vy, vz, takeoff, land, led
# =======================================
