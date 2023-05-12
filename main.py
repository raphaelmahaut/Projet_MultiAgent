# -*- coding: utf-8 -*-
"""
Simulation to test script for experiments

S. Bertrand 2022
"""

import numpy as np
import matplotlib.pyplot as plt
import Robot
import Simulation
import tp2a_algos as mas_ctrl


nbOfRobots = 5

fleet = Robot.Fleet(nbOfRobots, dynamics='singleIntegrator2D')

np.random.seed(100)
for i in range(0, nbOfRobots):
    # random init btw -10, +10
    fleet.robot[i].state = 10*np.random.rand(2, 1)-5


Te = 0.01
simulation = Simulation.FleetSimulation(fleet, t0=0.0, tf=20.0, dt=Te)


for t in simulation.t:

    for i in range(0, fleet.nbOfRobots):

        poses = np.zeros((3, fleet.nbOfRobots))

        for j in range(0, fleet.nbOfRobots):
            poses[0, j] = fleet.robot[j].state[0]
            poses[1, j] = fleet.robot[j].state[1]

        vxi, vyi = mas_ctrl.controller(i, fleet.nbOfRobots, poses)

        fleet.robot[i].ctrl = np.zeros_like(fleet.robot[i].state)
        fleet.robot[i].ctrl[0] = float(vxi)
        fleet.robot[i].ctrl[1] = float(vyi)

    simulation.addDataFromFleet(fleet)

    fleet.integrateMotion(Te)

simulation.plot(figNo=1)

simulation.plotFleet(figNo=4, mod=100, links=False)
plt.show()
