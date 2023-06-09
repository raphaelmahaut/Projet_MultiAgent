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
import random as rd
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

# in Mode possibility for wafflebot
# 0 : suivie de la formation de drone
# 1 : consensus avec le drone qui lui apporte
# 2 : attérissage du drone/ immobile

Mode = np.zeros(nbCF+nbTB3)

cf_mask = np.zeros(Mode.shape, dtype=bool)
cf_mask[:nbCF] = True
tb3_mask = np.zeros(Mode.shape, dtype=bool)
tb3_mask[nbCF:] = True


dist_inter_cf_mode_0 = 0.4
radius_inter_robot_security_xyz = 0.2
flee_speed = 0.5
attente = []

t0 = 0


list_resources = [
    np.array([[rd.uniform(-1, 3)], [rd.uniform(-2, 2)], [0.2]]) for i in range(3)]
attrib_resources = {i: None for i in range(nbCF)}


def reg(n):
    global dist_inter_cf_mode_0

    Poses = []
    if n <= 1:
        r = 0
    else:
        r = dist_inter_cf_mode_0/(2*np.sin(np.pi/n))
    for i in range(n):
        Poses.append((r*np.cos(2*np.pi*i/n), r*np.sin(2*np.pi*i/n), 0))
    Poses.append((0, 0, 0))
    return Poses


def reg_vec(n):
    global dist_inter_cf_mode_0

    Poses = []
    if n <= 1:
        r = 0
    else:
        r = dist_inter_cf_mode_0/(2*np.sin(np.pi/n))
    for i in range(n):
        Poses.append(
            np.array([r*np.cos(2*np.pi*i/n), r*np.sin(2*np.pi*i/n), 0]))
    Poses.append(np.array([0., 0., 0.]))
    return Poses


Path = [[(-3, 0), (-3, 1)], [(-3, 1), (3, 1)],
        [(3, 1), (3, -1)], [(3, -1), (-3, -1)]]


def ref(t):
    global Path, t0
    vit = 0.3
    if not Path:
        return np.array([[-3], [0], [1.5]])

    tps = np.sqrt((Path[0][0][0] - Path[0][1][0])**2 +
                  (Path[0][0][1] - Path[0][1][1])**2)/vit
    # print(tps, t)
    if (t - t0 > tps):
        x = Path[0][1][0]
        y = Path[0][1][1]
        t0 = t
        Path.pop(0)
    else:
        x = Path[0][0][0] - ((t-t0)/tps)*(Path[0][0][0] - Path[0][1][0])
        y = Path[0][0][1] - ((t-t0)/tps)*(Path[0][0][1] - Path[0][1][1])

    return np.array([[x], [y], [1.5]])


def get_absolute_index(robotNo, cf_poses, robot_type):
    if robot_type == "Crazy":
        return robotNo-1
    if robot_type == "Turtle":
        return robotNo-1 + len(cf_poses[0, :])

# Anticollision


def measure_inter_dist(A):
    assert len(A.shape) == 2, "wrong shape"
    N = A.shape[0]
    d = A.shape[1]

    A_temp = np.expand_dims(A, axis=-1)
    ones_temp = np.ones((N, 1, N))
    prod_temp = A_temp@ones_temp
    prod_temp = np.transpose(prod_temp, (0, 2, 1))
    transposed = np.transpose(prod_temp, (1, 0, 2))
    sqr_dist = ((np.abs(prod_temp-transposed)**2)@np.ones((N, d, 1)))[:, :, 0]
    dist = np.sqrt(sqr_dist)
    return dist


def apply_anticollision(robotNo, tb3_poses, cf_poses, vx, vy, vz, takeoff, land):
    global radius_inter_robot_security_xyz, flee_speed

    all_poses = np.concatenate((cf_poses, tb3_poses), axis=-1)
    inter_dist_xyz = measure_inter_dist(all_poses.T)
    robots_at_risk = np.where(
        inter_dist_xyz[robotNo-1] <= radius_inter_robot_security_xyz)[0]

    if len(robots_at_risk) > 1:
        barycenter = np.mean(all_poses[:, robots_at_risk], axis=-1)
        flee_vector = cf_poses[:, robotNo-1]-barycenter
        flee_vector = flee_vector/np.linalg.norm(flee_vector)
        flee_vector = flee_speed*flee_vector
        vx, vy, vz = flee_vector
        takeoff, land = False, False

    return vx, vy, vz, takeoff, land


def apply_sat_cf(vx, vy, vz, limit=0.5):
    vx = min(limit, vx)
    vx = max(-limit, vx)
    vy = min(limit, vy)
    vy = max(-limit, vy)
    vz = min(limit, vz)
    vz = max(-limit, vz)
    return vx, vy, vz


# ===================================================================================

# Control function for turtlebot3 ground vehicle to be modified
# should ONLY return (vx,vy) for the robot command
# max useable numbers of robots = 6 (1 for waffle model)
# ====================================


def tb3_control_fn(robotNo, tb3_poses, cf_poses, rmtt_poses, rms1_poses, obstacle_pose, t):
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

    abs_tb_id = get_absolute_index(robotNo, cf_poses, "Turtle")
    kp = 0.5
    kc = 1.5

    if Mode[abs_tb_id] == 0:
        for i in range(nbTB3):

            vx += -kp*(tb3_poses[0, robotNo - 1] - cf_poses[0, i])
            vy += -kp*(tb3_poses[1, robotNo - 1] - cf_poses[1, i])

    if Mode[abs_tb_id] == 1:
        for i in range(nbTB3):
            if Mode[i] == 2:
                vx += -kc*(tb3_poses[0, robotNo - 1] - cf_poses[0, i])
                vy += -kc*(tb3_poses[1, robotNo - 1] - cf_poses[1, i])

    vx, vy, _ = apply_sat_cf(vx, vy, 0, 0.05)

    # -----------------------

    return vx, vy
# ====================================


# ====================================
# Control function for crazyflie nano drones to be modified
# should ONLY return (vx,vy,vz,wz,takeoff,land) for the robot command
# max useable numbers of drones = 3
# ====================================
def cf_control_fn(robotNo, tb3_poses, cf_poses, rmtt_poses, rms1_poses, obstacle_pose, t):
    # ====================================
    global t0, list_resources, attrib_resources, attente

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

    v = np.array([0., 0, 0])
    kp = 1
    ko = 0.1
    kc = 0.5
    abs_cf_id = get_absolute_index(robotNo, cf_poses, "Crazy")

    if Mode[abs_cf_id] == 0:

        robot_in_same_mode = np.logical_and(Mode == 0, cf_mask)
        Poses = reg_vec(
            robot_in_same_mode[robot_in_same_mode == True].shape[0])

        id_pose = np.sum(robot_in_same_mode[:abs_cf_id])

        for i in np.where(robot_in_same_mode)[0]:
            id_pose_i = np.sum(robot_in_same_mode[:i])
            if i != abs_cf_id:
                # vx += -kp*(cf_poses[0, robotNo-1] - cf_poses[0, i] -
                #            (Poses[id_pose][0] - Poses[id_pose_i][0]))
                # vy += -kp*(cf_poses[1, robotNo-1] - cf_poses[1, i] -
                #            (Poses[id_pose][1] - Poses[id_pose_i][1]))
                # vz += -kp*(cf_poses[2, robotNo-1] - cf_poses[2, i] -
                #            (Poses[id_pose][2] - Poses[id_pose_i][2]))
                unit = (cf_poses[:, robotNo-1] - cf_poses[:, i]) / \
                    np.linalg.norm(cf_poses[:, robotNo-1] - cf_poses[:, i])
                v += -kp*(np.linalg.norm(cf_poses[:, robotNo-1] - cf_poses[:, i]) - np.linalg.norm(
                    Poses[id_pose] - Poses[id_pose_i]))*unit
                vz += -kp*(cf_poses[2, robotNo-1] - cf_poses[2, i] -
                           (Poses[id_pose][2] - Poses[id_pose_i][2]))

        Ref = ref(t)
        unit = (cf_poses[:, robotNo-1] - Ref[:, 0]) / \
            np.linalg.norm(cf_poses[:, robotNo-1] - Ref[:, 0])
        v += -ko*(np.linalg.norm(cf_poses[:, robotNo-1] - Ref[:, 0]) -
                  np.linalg.norm(Poses[id_pose][:] - Poses[-1][:]))*unit
        vx = v[0]
        vy = v[1]
        vz += -ko*(cf_poses[2, robotNo-1] - Ref[2, 0] -
                   (Poses[id_pose][2] - Poses[-1][2]))

    elif Mode[abs_cf_id] == 1:
        # Aller vers la ressource de manière autonome
        vx = -kp*(cf_poses[0, robotNo-1] - attrib_resources[abs_cf_id][0, 0])
        vy = -kp*(cf_poses[1, robotNo-1] - attrib_resources[abs_cf_id][1, 0])
        vz = -kp*(cf_poses[2, robotNo-1] - attrib_resources[abs_cf_id][2, 0])

    elif Mode[abs_cf_id] == 2:
        # Consensus entre le robot et le waffle
        vx = -kc*(cf_poses[0, robotNo-1] - tb3_poses[0, 0])
        vy = -kc*(cf_poses[1, robotNo-1] - tb3_poses[1, 0])
        vz = -kp*(cf_poses[2, robotNo-1] - 0.8)

    elif Mode[abs_cf_id] == 3:
        Ref = ref(t)
        vx += -kc*(cf_poses[0, robotNo-1] - Ref[0, 0])
        vy += -kc*(cf_poses[1, robotNo-1] - Ref[1, 0])
        vz += -kc*(cf_poses[2, robotNo-1] - Ref[2, 0])

    # --- TRANSITIONS ---
    # From 0 (formation) to 1 (descente)
    for index in range(len(list_resources)):
        if (Mode[abs_cf_id] == 0) and (np.linalg.norm(ref(t)[:2] - list_resources[index][:2]) < 1.1) and (abs_cf_id == np.where(Mode == 0)[0][0]):
            attrib_resources[abs_cf_id] = np.copy(list_resources[index])

            # resource becomes the drone's target
            print('Drone ' + str(abs_cf_id) +
                  ' is getting the resource at ' + str(list_resources[index]))
            Mode[abs_cf_id] = 1
            list_resources.pop(index)
            break

    # From 1 (descente) to 2 (remonte)
    if (Mode[abs_cf_id] == 1) and (np.linalg.norm(cf_poses[:, robotNo-1:robotNo] - attrib_resources[abs_cf_id]) < 0.01):
        print('The resource at ' +
              str(attrib_resources[abs_cf_id]) + ' was collected by drone ' + str(abs_cf_id))
        if attente:
            attente.append(abs_cf_id)
            Mode[abs_cf_id] = -1
        else:
            attente.append(abs_cf_id)
            Mode[nbCF] = 1
            Mode[abs_cf_id] = 2

    # From 2 (remonte) to 3 (retourne a la formation)
    tb3_poses[2] = 0
    if (Mode[abs_cf_id] == 2) and (np.linalg.norm(cf_poses[:2, robotNo-1:robotNo] - tb3_poses[:2, :]) < 0.01):
        print('The resource that was at ' +
              str(attrib_resources[abs_cf_id]) + ' was delivered by drone ' + str(abs_cf_id))
        attrib_resources[abs_cf_id] = None
        attente.pop(0)
        Mode[nbCF] = 0
        if attente:
            Mode[nbCF] = 1
            Mode[attente[0]] = 2
        Mode[abs_cf_id] = 3

    # From 3 (retourne a la formation) to 0 (formation)
    if (Mode[abs_cf_id] == 3) and (np.linalg.norm(cf_poses[:, robotNo-1:robotNo] - ref(t)) < 1):
        print('Drone ' + str(abs_cf_id) + ' has returned to formation')
        Mode[abs_cf_id] = 0

    # -----------------------

    vx, vy, vz, takeoff, land = apply_anticollision(
        robotNo, tb3_poses, cf_poses, vx, vy, vz, takeoff, land)
    vx, vy, vz = apply_sat_cf(vx, vy, vz, 0.5)

    return vx, vy, vz, takeoff, land
# ====================================


# ====================================
# Control function for dji rmtt drones to be modified
# should ONLY return (vx,vy,vz,wz,takeoff,land,led) for the robot command
# max useable numbers of drones = 1
# ====================================
def rmtt_control_fn(robotNo, tb3_poses, cf_poses, rmtt_poses, rms1_poses, obstacle_pose, t):
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
def tb3_controller(robotNo, poses, cf_poses, rmtt_poses, rms1_poses, obstacle_pose, t):
    vx, vy = tb3_control_fn(robotNo, poses, cf_poses,
                            rmtt_poses, rms1_poses, obstacle_pose, t)
    return vx, vy


def cf_controller(robotNo, tb3_poses, cf_poses, rmtt_poses, rms1_poses, obstacle_pose, t):
    vx, vy, vz, takeoff, land = cf_control_fn(
        robotNo, tb3_poses, cf_poses, rmtt_poses, rms1_poses, obstacle_pose, t)
    return vx, vy, vz, takeoff, land


def rmtt_controller(robotNo, tb3_poses, cf_poses, rmtt_poses, rms1_poses, obstacle_pose, t):
    vx, vy, vz, takeoff, land, led = rmtt_control_fn(
        robotNo, tb3_poses, cf_poses, rmtt_poses, rms1_poses, obstacle_pose, t)
    return vx, vy, vz, takeoff, land, led
# =======================================
