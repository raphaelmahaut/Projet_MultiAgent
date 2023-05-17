import matplotlib.pyplot as plt
import numpy as np
from tp_algos import tb3_controller, cf_controller, rmtt_controller


def reg(n):
    global a

    Poses = []
    r = a/(2*np.cos(np.pi/n))
    for i in range(n):
        Poses.append((r*np.cos(2*np.pi*i/n), r*np.sin(2*np.pi*i/n), 1.5))
    return Poses


# =============================================================================
class Robot:
    # =============================================================================

    # -------------------------------------------------------------------------
    def __init__(self, type="Turtle", initState=None, index=0):
        # -------------------------------------------------------------------------
        self.index = index
        self.type = type
        self.ctrl = np.zeros((3, 1))
        try:
            self.state = initState[:, index - 1: index]
        except:
            self.state = np.zeros((3, 1))

        self.takeoff = False
        self.land = False
        self.led = False

    def integrateMotion(self, Te, vx, vy, vz=0):
        # -------------------------------------------------------------------------
        # integrate robot motion over one sampling period (Euler discretization) applying control input

        if self.takeoff:
            pass
        elif self.land:
            pass
        else:
            self.ctrl = np.array([[vx], [vy], [vz]])
            self.state += Te * self.ctrl

    # -------------------------------------------------------------------------

    def __repr__(self):
        # -------------------------------------------------------------------------
        """Display in command line"""
        message = "Robot:\n type: {}\n".format(self.type)
        message += "index: {}\n".format(self.index)
        message += " state: {}\n".format(self.state)
        return message+"\n"

    # -------------------------------------------------------------------------
    def __str__(self):
        # -------------------------------------------------------------------------
        """Display with print function"""
        message = "Robot:\n type: {}\n".format(self.type)
        message += "index: {}\n".format(self.index)
        message += " state: {}\n".format(self.state)
        return message+"\n"

# ====================== end of class Robot ===================================


# =============================================================================
class Fleet:
    # =============================================================================

    # -------------------------------------------------------------------------
    def __init__(self, nbTB3=0, nbCF=0, nbRMTT=0, nbRMS1=0, PosesTB3=[], PosesCF=[], PosesRMTT=[], PosesRMS1=[]):
        # -------------------------------------------------------------------------
        self.nbTB3 = nbTB3
        self.nbCF = nbCF
        self.nbRMTT = nbRMTT
        self.nbRMS1 = nbRMS1

        self.tb3_poses = np.zeros((3, nbTB3))
        if PosesTB3:
            for i in range(self.nbTB3):
                self.tb3_poses[0, i] = PosesTB3[i][0]
                self.tb3_poses[1, i] = PosesTB3[i][1]

        self.cf_poses = np.zeros((3, nbCF))
        if PosesTB3:
            for i in range(self.nbCF):
                self.cf_poses[0, i] = PosesCF[i][0]
                self.cf_poses[1, i] = PosesCF[i][1]
                self.cf_poses[2, i] = PosesCF[i][2]

        self.rmtt_poses = np.zeros((3, nbRMTT))
        if PosesRMTT:
            for i in range(self.nbRMTT):
                self.rmtt_poses[0, i] = PosesRMTT[i][0]
                self.rmtt_poses[1, i] = PosesRMTT[i][1]
                self.rmtt_poses[1, i] = PosesRMTT[i][2]

        self.rms1_poses = np.zeros((3, nbRMS1))
        if PosesRMS1:
            for i in range(self.nbRMS1):
                self.rms1_poses[0, i] = PosesRMS1[i][0]
                self.rms1_poses[1, i] = PosesRMS1[i][1]

        self.obstacle_pose = np.zeros((5, 0))

        self.robot = []

        for i in range(self.nbTB3):
            self.robot.append(Robot('Turtle', self.tb3_poses, i+1))

        for i in range(self.nbCF):
            self.robot.append(Robot('Crazy', self.cf_poses, i+1))

        for i in range(self.nbRMTT):
            self.robot.append(Robot('DJI', self.rmtt_poses, i+1))

        for i in range(self.nbRMS1):
            self.robot.append(Robot('Autre', self.rms1_poses, i+1))

    # -------------------------------------------------------------------------
    def integrateMotion(self, Te):
        # -------------------------------------------------------------------------
        # integrate fleet motion over one sampling period (Euler discretization) applying control input u

        for rob in self.robot:
            if rob.type == 'Turtle':
                vx, vy = tb3_controller(rob.index, self.tb3_poses, self.cf_poses,
                                        self.rmtt_poses, self.rms1_poses, self.obstacle_pose)
                rob.integrateMotion(Te, vx, vy)
            if rob.type == 'Crazy':
                vx, vy, vz, rob.takeoff, rob.land = cf_controller(rob.index, self.tb3_poses, self.cf_poses,
                                                                  self.rmtt_poses, self.rms1_poses, self.obstacle_pose)
                rob.integrateMotion(Te, vx, vy, vz)
            if rob.type == 'DJI':
                vx, vy, vz, rob.takeoff, rob.land, rob.led = rmtt_controller(rob.index, self.tb3_poses, self.cf_poses,
                                                                             self.rmtt_poses, self.rms1_poses, self.obstacle_pose)
                rob.integrateMotion(Te, vx, vy, vz)
    # -------------------------------------------------------------------------

    def __repr__(self):
        # -------------------------------------------------------------------------
        """Display in command line"""
        message = "\nFleet\n"
        for rob in self.robot:
            message += "Robot:\n type: {}\n".format(rob.type)
            message += "index: {}\n".format(rob.index)
            message += " state: {}\n".format(rob.state)
        return message+"\n"

    # -------------------------------------------------------------------------
    def __str__(self):
        # -------------------------------------------------------------------------
        """Display with print function"""
        message = "\nFleet\n"
        for rob in self.robot:
            message += "Robot:\n type: {}\n".format(rob.type)
            message += "index: {}\n".format(rob.index)
            message += " state: {}\n".format(rob.state)
        return message+"\n"

# ====================== end of class Fleet ===================================


class Simulateur:
    # -------------------------------------------------------------------------
    def __init__(self, fleet, t0=0.0, tf=10.0, dt=0.01):
        # -------------------------------------------------------------------------

        # associated robot
        self.fleet = fleet

        # time
        self.t0 = t0  # init time of simulation (in sec)
        self.tf = tf  # final time of simulation (in sec)
        self.dt = dt  # sampling period for numerical integration (in sec)
        self.t = np.arange(t0, tf, dt)  # vector of time stamps
        self.nb = int(self.t.shape[0])

        # to save robot state and input during simulation
        self.state = np.zeros([3, len(fleet.robot), int(self.t.shape[0])])
        self.ctrl = np.zeros([3, len(fleet.robot), int(self.t.shape[0])])

        # index of current stored data (from 0 to len(self.t)-1 )
        self.currentIndex = 0

    # -------------------------------------------------------------------------

    def addDataFromRobot(self, robot, i):
        # -------------------------------------------------------------------------
        # store state data
        self.state[:, i, self.currentIndex] = np.copy(robot.state).ravel()
        # store ctrl data
        self.ctrl[:, i, self.currentIndex] = np.copy(robot.ctrl).ravel()

    # -------------------------------------------------------------------------

    def plot(self, xmin=-10, xmax=10, ymin=-10, ymax=10, mod=None):
        # -------------------------------------------------------------------------

        while self.currentIndex < self.nb:
            fig1 = plt.figure(1)
            fig1.clf()
            ax = fig1.add_subplot(111, projection='3d', proj_type='ortho')
            for i in range(len(self.fleet.robot)):
                rob = self.fleet.robot[i]
                self.addDataFromRobot(rob, i)
                if rob.type == 'Crazy':
                    ax.scatter(
                        rob.state[0], rob.state[1], rob.state[2], color='red', label='crazyflies', zorder=1)
                elif rob.type == 'Turtle':
                    ax.scatter(rob.state[0], rob.state[1], rob.state[2],
                               color='green', label='turtlebot/waffle', zorder=1)
                elif rob.type == 'Crazy':
                    ax.scatter(rob.state[0], rob.state[1],
                               rob.state[2], color='blue', label='dji', zorder=1)

            handles, labels = ax.get_legend_handles_labels()
            unique_labels = list(set(labels))
            unique_handles = [handles[labels.index(
                label)] for label in unique_labels]

            x = [-4, 4, 4, -4]
            y = [-2, -2, 2, 2]
            z = [0, 0, 0, 0]
            ax.plot_trisurf(x, y, z, linewidth=0.2,
                            color='grey', alpha=0.5, zorder=2)
            ax.legend(unique_handles, unique_labels)

            ax.set_xlim3d(-6, 6)
            ax.set_ylim3d(-6, 6)
            ax.set_zlim3d(-1, 9)

            plt.pause(0.01)
            self.fleet.integrateMotion(self.dt)
            self.currentIndex = self.currentIndex + 1
            print(
                f"simulation en cours: {100*self.currentIndex/self.nb:.2f}%", end='\r')

        for i in range(len(self.fleet.robot)):
            fig2 = plt.figure(2)
            graph = plt.subplot(111, aspect='equal', autoscale_on=False, xlim=(
                xmin, xmax), ylim=(ymin, ymax))

            graph.plot(self.state[0, i, ::mod],
                       self.state[1, i, ::mod], marker='.')
            graph.grid(True)
            graph.set_xlabel('x (m)')
            graph.set_ylabel('y (m)')

            fig3 = plt.figure(3)
            graph = plt.subplot(111)
            graph.plot(self.t[::mod], self.state[0, i, ::mod])
            graph.grid(True)
            graph.set_xlabel('t (s)')
            graph.set_ylabel('x (m)')

            fig4 = plt.figure(4)
            graph = plt.subplot(111)
            graph.plot(self.t[::mod], self.state[1, i, ::mod])
            graph.grid(True)
            graph.set_xlabel('t (s)')
            graph.set_ylabel('y (m)')

            fig5 = plt.figure(5)
            graph = plt.subplot(111)
            graph.plot(self.t[::mod], self.state[2, i, ::mod])
            graph.grid(True)
            graph.set_xlabel('t (s)')
            graph.set_ylabel('z (m)')

        plt.show()


# ====================== end of class RobotSimulation==========================s

if __name__ == '__main__':
    a = 0.3
    n = 3

    fleet = Fleet(nbCF=3, nbTB3=1, PosesCF=[
                  (-4, -1, 1.5), (-4, 0, 1.5), (-4, 1, 1.5)], PosesTB3=[(-4, 0, 0)])
    Sim = Simulateur(fleet)
    Sim.plot()
