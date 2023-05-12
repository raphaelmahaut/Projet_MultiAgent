# -*- coding: utf-8 -*-
"""
Simulation Class

(c) S. Bertrand
"""

import numpy as np
import matplotlib.pyplot as plt
import Robot


# =============================================================================
class RobotSimulation:
    # =============================================================================

    # -------------------------------------------------------------------------
    def __init__(self, robot, t0=0.0, tf=10.0, dt=0.01):
        # -------------------------------------------------------------------------

        # associated robot
        self.robot = robot

        # time
        self.t0 = t0  # init time of simulation (in sec)
        self.tf = tf  # final time of simulation (in sec)
        self.dt = dt  # sampling period for numerical integration (in sec)
        self.t = np.arange(t0, tf, dt)  # vector of time stamps

        # to save robot state and input during simulation
        self.state = np.zeros([self.robot.stateDim, int(self.t.shape[0])])
        self.ctrl = np.zeros([self.robot.ctrlDim, int(self.t.shape[0])])


#        # to save reference position of current WP during simulation
#        self.xr = np.zeros_like(self.t)
#        self.yr = np.zeros_like(self.t)

        # index of current stored data (from 0 to len(self.t)-1 )
        self.currentIndex = 0

    # -------------------------------------------------------------------------

    def addDataFromRobot(self, robot):
        # -------------------------------------------------------------------------
        # store state data
        for i in range(0, robot.stateDim):
            self.state[i, self.currentIndex] = robot.state[i]
        # store ctrl data
        for i in range(0, robot.ctrlDim):
            self.ctrl[i, self.currentIndex] = robot.ctrl[i]
        # increment storage index
        self.currentIndex = self.currentIndex + 1

    # -------------------------------------------------------------------------

    def plot(self, figNo=1, xmin=-10, xmax=10, ymin=-10, ymax=10, mod=None):
        # -------------------------------------------------------------------------

        if (self.robot.dynamics == 'singleIntegrator2D'):

            fig1 = plt.figure(figNo)
            graph = plt.subplot(111, aspect='equal', autoscale_on=False, xlim=(
                xmin, xmax), ylim=(ymin, ymax))
#            graph.plot(self.state[0,:], self.state[1,:], color = 'r')
#            graph.plot(self.state[0,-1], self.state[1,-1], color = 'r', marker='o')
            graph.plot(self.state[0, ::mod], self.state[1, ::mod], marker='.')
            graph.grid(True)
            graph.set_xlabel('x (m)')
            graph.set_ylabel('y (m)')

            fig2 = plt.figure(figNo+1)
            graph = plt.subplot(111)
            graph.plot(self.t[::mod], self.state[0, ::mod])
            graph.grid(True)
            graph.set_xlabel('t (s)')
            graph.set_ylabel('x (m)')

            fig3 = plt.figure(figNo+2)
            graph = plt.subplot(111)
            graph.plot(self.t[::mod], self.state[1, ::mod])
            graph.grid(True)
            graph.set_xlabel('t (s)')
            graph.set_ylabel('y (m)')


# ====================== end of class RobotSimulation==========================


# =============================================================================
class FleetSimulation:
    # =============================================================================

    # -------------------------------------------------------------------------
    def __init__(self, fleet, t0=0.0, tf=10.0, dt=0.01):
        # -------------------------------------------------------------------------

        self.nbOfRobots = fleet.nbOfRobots

        self.robotSimulation = []

        for i in range(0, self.nbOfRobots):
            self.robotSimulation.append(
                RobotSimulation(fleet.robot[i], t0, tf, dt))

        # time
        self.t0 = t0  # init time of simulation (in sec)
        self.tf = tf  # final time of simulation (in sec)
        self.dt = dt  # sampling period for numerical integration (in sec)
        self.t = np.arange(t0, tf, dt)  # vector of time stamps


# !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
# !!!!!!! PRévoir un add data from robot avec l index du robot
# !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

    # -------------------------------------------------------------------------

    def addDataFromFleet(self, fleet):
        # -------------------------------------------------------------------------
        for i in range(0, self.nbOfRobots):
            self.robotSimulation[i].addDataFromRobot(fleet.robot[i])

    # -------------------------------------------------------------------------

    def plot(self, figNo=1, xmin=-10, xmax=10, ymin=-10, ymax=10, mod=None):
        # -------------------------------------------------------------------------

        for i in range(0, self.nbOfRobots):
            self.robotSimulation[i].plot(figNo, xmin, xmax, ymin, ymax, mod)

    # -------------------------------------------------------------------------

    def plotFleet(self, figNo=1, xmin=-10, xmax=10, ymin=-10, ymax=10, mod=None, links=True):
        # -------------------------------------------------------------------------

        fig1 = plt.figure(figNo)
        graph = fig1.add_subplot(
            111, aspect='equal', autoscale_on=False, xlim=(xmin, xmax), ylim=(ymin, ymax))
#            graph.plot(self.state[0,:], self.state[1,:], color = 'r')
#            graph.plot(self.state[0,-1], self.state[1,-1], color = 'r', marker='o')

        # print int(self.t.shape[0])

        plt.gca().set_prop_cycle(None)

        for tt in range(0, int(self.t.shape[0]))[::mod]:
            for i in range(0, self.nbOfRobots):
                for j in range(0, self.nbOfRobots):
                    xi = self.robotSimulation[i].state[0, tt]
                    yi = self.robotSimulation[i].state[1, tt]
                    xj = self.robotSimulation[j].state[0, tt]
                    yj = self.robotSimulation[j].state[1, tt]
                    if (links == True):
                        graph.plot([xi, xj], [yi, yj], color='grey',
                                   alpha=0.3, linestyle='--')
                graph.plot(xi, yi, marker='8', linestyle="None", markersize=10)
            plt.gca().set_prop_cycle(None)
        graph.grid(True)
        graph.set_xlabel('x (m)')
        graph.set_ylabel('y (m)')


# ====================== end of class RobotSimulation==========================


# ============================== MAIN =========================================
if __name__ == '__main__':
    # =============================================================================

    # ---- one robot simulation test
    #
    #    initState = np.array([ [-1.], [2.] ])
    #    robot = Robot.Robot(dynamics='singleIntegrator2D', initState=initState, index=0)
    #
    #    Te = 0.01
    #    simulation = RobotSimulation(robot, t0=0.0, tf=20.0, dt=Te)
    #
    #    # reference
    #    referenceState= np.array([[2.] , [1.] ])
    #
    #    # control gain
    #    kp = 0.4
    #
    #    for t in simulation.t:
    #
    #        robot.ctrl = kp* (referenceState - robot.state)
    #
    #        simulation.addDataFromRobot(robot)
    #
    #        robot.integrateMotion(Te)
    #
    #    print robot
    #
    #    simulation.plot(figNo=1)

    # ---- fleet simulation test

    nbOfRobots = 3

    # , initState=initState)
    fleet = Robot.Fleet(nbOfRobots, dynamics='singleIntegrator2D')

    for i in range(0, nbOfRobots):
        # random init btw -10, +10
        fleet.robot[i].state = 20*np.random.rand(2, 1)-10

    Te = 0.01
    simulation = FleetSimulation(fleet, t0=0.0, tf=20.0, dt=Te)

    # control gain
    kp = 0.4

    for t in simulation.t:

        #        #proportional control law to common reference state
        #        referenceState= np.array([[2.] , [1.] ])
        #        for r in range(0, fleet.nbOfRobots):
        #            fleet.robot[r].ctrl = kp* (referenceState - fleet.robot[r].state)

        # consensus
        for r in range(0, fleet.nbOfRobots):
            fleet.robot[r].ctrl = np.zeros((2, 1))
            for n in range(0, fleet.nbOfRobots):
                fleet.robot[r].ctrl += kp * \
                    (fleet.robot[n].state -
                     fleet.robot[r].state) / fleet.nbOfRobots

        simulation.addDataFromFleet(fleet)

        fleet.integrateMotion(Te)

    simulation.plot(figNo=1)

    simulation.plotFleet(figNo=4, mod=200)
    plt.show()
