# -*- coding: utf-8 -*-
"""
Robot Class

(c) S. Bertrand
"""

import numpy as np
import matplotlib.pyplot as plt


# =============================================================================
class Robot:
    # =============================================================================

    # -------------------------------------------------------------------------
    def __init__(self, dynamics='singleIntegrator2D', initState=[], index=0):
        # -------------------------------------------------------------------------
        self.index = index
        self.dynamics = dynamics

        if (dynamics == 'singleIntegrator2D'):
            # state
            self.stateDim = 2
            # if (initState==None).any():   # depending on numpy version
            if (len(initState) == 0):
                self.state = np.array([[0], [0]])
            else:
                self.state = initState[index]

            # control
            self.ctrlDim = 2
            self.ctrl = np.array([[0], [0]])

        if (dynamics == 'drone'):
            self.stateDim = 3

            if (len(initState) == 0):
                self.state = np.array([[0], [0], [0]])
            else:
                self.state = initState[index]

    # -------------------------------------------------------------------------

    def setCtrl(self, ctrl):
        # -------------------------------------------------------------------------
        self.ctrl = ctrl

    # -------------------------------------------------------------------------

    def integrateMotion(self, Te):
        # -------------------------------------------------------------------------
        # integrate robot motion over one sampling period (Euler discretization) applying control input u

        if (self.dynamics == 'singleIntegrator2D'):
            self.state = self.state + Te * self.ctrl

    # -------------------------------------------------------------------------

    def __repr__(self):
        # -------------------------------------------------------------------------
        """Display in command line"""
        message = "\nRobot:\n index: {}\n".format(self.index)
        message += " state: {}".format(self.state)
        return message+"\n"

    # -------------------------------------------------------------------------
    def __str__(self):
        # -------------------------------------------------------------------------
        """Display with print function"""
        message = "\nRobot:\n index: {}\n".format(self.index)
        message += " state: {}".format(self.state)
        return message+"\n"

# ====================== end of class Robot ===================================


# =============================================================================
class Fleet:
    # =============================================================================

    # -------------------------------------------------------------------------
    def __init__(self, nbOfRobots, dynamics='singleIntegrator2D', initState=[]):
        # -------------------------------------------------------------------------
        self.nbOfRobots = nbOfRobots

        self.robot = []
        index = 0
        for i in range(0, self.nbOfRobots):
            self.robot.append(Robot(dynamics, initState, index))
            index += 1
        # !!!!!!! PREVOIR LA STRUCTURE DE DONNEES POUR LA TOPOLOGIE

    # -------------------------------------------------------------------------
    def integrateMotion(self, Te):
        # -------------------------------------------------------------------------
        # integrate fleet motion over one sampling period (Euler discretization) applying control input u

        for i in range(0, self.nbOfRobots):
            self.robot[i].integrateMotion(Te)

    # -------------------------------------------------------------------------

    def __repr__(self):
        # -------------------------------------------------------------------------
        """Display in command line"""
        message = "\nFleet\n"
        for rob in self.robot:
            message += " Robot:\n index: {}\n".format(rob.index)
            message += "  state: {}\n".format(rob.state)
        return message+"\n"

    # -------------------------------------------------------------------------
    def __str__(self):
        # -------------------------------------------------------------------------
        """Display with print function"""
        message = "\nFleet\n"
        for rob in self.robot:
            message += "Robot:\n index: {}\n".format(rob.index)
            message += " state: {}\n".format(rob.state)
        return message+"\n"

# ====================== end of class Fleet ===================================


# ============================== MAIN =========================================
if __name__ == '__main__':
    # =============================================================================

    initState = [np.array([[-1.], [2.]])]
    robot = Robot(dynamics='singleIntegrator2D', initState=initState, index=0)

    print(robot)

    nbOfRobots = 8

    # , initState=initState)
    fleet = Fleet(nbOfRobots, dynamics='singleIntegrator2D')

    for i in range(0, nbOfRobots):
        # random init btw -10, +10
        fleet.robot[i].state = 20*np.random.rand(2, 1)-10

    print(fleet)
