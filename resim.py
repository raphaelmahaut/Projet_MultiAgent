import matplotlib.pyplot as plt
import numpy as np


def plot(xmin=-10, xmax=10, ymin=-10, ymax=10, mod=None):
    # -------------------------------------------------------------------------
    list_resources = [np.array([[-1], [1.5], [0.2]]),
                      np.array([[1], [-0.3], [0.2]]), np.array([[3], [-1.7], [0.2]])]

    State = []
    for i in range(4):
        State.append(np.loadtxt("sauvegarde/rob"+str(i)+".csv", delimiter=","))

    t = np.loadtxt("sauvegarde/T.csv", delimiter=",")

    for index in range(0, len(t), 50):
        fig1 = plt.figure(1)
        fig1.clf()
        ax = fig1.add_subplot(111, projection='3d', proj_type='ortho')
        for i in range(4):
            if i >= 1:
                ax.scatter(
                    State[i][0, index], State[i][1, index], State[i][2, index], color='red', label='crazyflies', zorder=1)
            else:
                ax.scatter(State[i][0, index], State[i][1, index], State[i][2, index],
                           color='green', label='turtlebot/waffle', zorder=1)

        handles, labels = ax.get_legend_handles_labels()
        unique_labels = list(set(labels))
        unique_handles = [handles[labels.index(
            label)] for label in unique_labels]

        for res in list_resources:
            ax.scatter(res[0], res[1], res[2], color='orange',
                       label='ressource', zorder=1)

        # Dessin du rectangle gris représentant
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

    for i in range(4):
        fig2 = plt.figure(2)
        graph = plt.subplot(111, aspect='equal', autoscale_on=False, xlim=(
            xmin, xmax), ylim=(ymin, ymax))

        graph.plot(State[i][0, ::mod],
                   State[i][1, ::mod], marker='.')
        graph.grid(True)
        graph.set_xlabel('x (m)')
        graph.set_ylabel('y (m)')

        fig3 = plt.figure(3)
        graph = plt.subplot(111)
        graph.plot(t[::mod], State[i][0, ::mod])
        graph.grid(True)
        graph.set_xlabel('t (s)')
        graph.set_ylabel('x (m)')

        fig4 = plt.figure(4)
        graph = plt.subplot(111)
        graph.plot(t[::mod], State[i][1, ::mod])
        graph.grid(True)
        graph.set_xlabel('t (s)')
        graph.set_ylabel('y (m)')

        fig5 = plt.figure(5)
        graph = plt.subplot(111)
        graph.plot(t[::mod], State[i][2, ::mod])
        graph.grid(True)
        graph.set_xlabel('t (s)')
        graph.set_ylabel('z (m)')

    plt.show()


plot()
