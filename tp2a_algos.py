#!/usr/bin/env python
'''
   CentraleSupelec TP3A
   Sylvain BERTRAND, 2023
   (all variables in SI unit)
   
   
   variables used by the functions of this script
       - nbRobots: nb of robots in the fleet (>1)    
       - robotNo: no of the current robot for which control is coputed (1 .. nbRobots)
       - poses:  size (3 x nbRobots) 
                 eg. of use: for robot no 'robotNo', its pose can be obtained by: poses[:,robotNo-1]   (indexes in Python start from 0 !)
                           poses[0,robotNo-1]: x-coordinate of robot position (in m)
                           poses[1,robotNo-1]: y-coordinate of robot position (in m)
                           poses[2,robotNo-1]: orientation angle of robot (in rad)
   
'''

import numpy as np
import math
#import rospy




# ==============   "GLOBAL" VARIABLES KNOWN BY ALL THE FUNCTIONS ===================
# all variables declared here will be known by functions below
# if a variable here needs to be modified by a function, use 'global' keyword inside the function

toto = 1.0

# ===================================================================================




# =======================================
def consensus(robotNo, nbRobots, poses):
# =======================================
    
    #  --- TO BE MODIFIED ---
    vx = 0.0
    vy = 0.0
    # -----------------------
        
    return vx,vy
# ====================================        






# ============================================    
def leaderFollower(robotNo, nbRobots, poses):
# ============================================
    
    
    # example of use of global variable
    # ---------------------------------------
    global toto  # this variable needs to be modified by the function
    
    toto = toto + 1
    # ---------------------------------------
    
    
    #  --- TO BE MODIFIED ---
    vx = 0.0
    vy = 0.0
    # -----------------------
        
    return vx,vy

# ====================================    
    



# ======== ! DO NOT MODIFY ! ============
def controller(robotNo, nbRobots, poses):
# =======================================
    
    # UNCOMMENT THE ONE TO BE TESTED FOR EXPERIMENT
    vx,vy = consensus(robotNo, nbRobots, poses)
    #vx,vy = leaderFollower(robotNo, nbRobots, poses)
    
    return vx,vy
    
# ====================================   