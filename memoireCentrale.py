#!/usr/bin/env python3

import rospy
import sys
import time
import numpy as np
from std_msgs.msg import Int8MultiArray, Bool
from geometry_msgs.msg import Point


def callback(data, pub_mode, pub_res):
    global Modes, Ressources, Avail

    msg = data.data
    print("message reçu", msg)

    if msg[1] == 1:
        if Avail[msg[2] - 1] and Modes[msg[0]] == 0:
            Avail[msg[2]-1] = False
            Modes[msg[0]] = msg[1]
            Ressources[msg[0]] = msg[2]
            msg_mode = Int8MultiArray()
            msg_res = Int8MultiArray()
            msg_mode.data = Modes
            msg_res.data = Ressources
            pub_mode.publish(msg_mode)
            pub_res.publish(msg_res)
    else:
        Modes[msg[0]] = msg[1]
        msg_mode = Int8MultiArray()
        msg_mode.data = Modes
        pub_mode.publish(msg_mode)


def lancement(data):
    global t0, init
    init = data.data
    t0 = time.time()
    print("C'est parti")


def ref():
    global Path, t0, Modes, end
    vit = 0.3
    Ref = Point()
    Ref.z = 1.5

    if not Path:
        Ref.x = -3
        Ref.y = 0
        if sum(Modes) == 0:
            end = True
        return Ref

    tps = np.sqrt((Path[0][0][0] - Path[0][1][0])**2 +
                  (Path[0][0][1] - Path[0][1][1])**2)/vit

    t = time.time()
    print("temps : ", t-t0)
    if (t - t0 > tps):
        Ref.x = Path[0][1][0]
        Ref.y = Path[0][1][1]
        t0 = t
        Path.pop(0)
    else:
        Ref.x = Path[0][0][0] - ((t-t0)/tps)*(Path[0][0][0] - Path[0][1][0])
        Ref.y = Path[0][0][1] - ((t-t0)/tps)*(Path[0][0][1] - Path[0][1][1])

    return Ref


def publisher(nbcf, nbtb, nbres):
    global init, t0

    rospy.init_node('Memoire', anonymous=False)

    pub_mode = rospy.Publisher('Modes', Int8MultiArray, queue_size=10)
    pub_res = rospy.Publisher('Ressources', Int8MultiArray, queue_size=10)
    pub_path = rospy.Publisher('Path', Point, queue_size=10)

    rospy.Subscriber('Changement', Int8MultiArray,
                     lambda data: callback(data, pub_mode, pub_res))
    rospy.Subscriber('Lancement', Bool, lancement)
    rate = rospy.Rate(50)  # 10hz

    saveTime = time.time()
    stopTime = saveTime

    while not rospy.is_shutdown():
        if init != end:
            if 0 in Modes[:nbcf]:
                t0 += stopTime - saveTime
                saveTime = time.time()
                stopTime = saveTime
                pub_path.publish(ref())
            else:
                stopTime = time.time()

        rate.sleep()


if __name__ == '__main__':
    try:

        nbcf = int(sys.argv[1])
        nbtb = int(sys.argv[2])
        nbres = int(sys.argv[3])

        init = False
        end = False
        t0 = time.time()
        Modes = [0 for i in range(nbcf + nbtb)]
        Ressources = [0 for i in range(nbres)]
        Avail = [True for i in range(nbres)]
        Path = [[(-3, 0), (-3, 1)], [(-3, 1), (3, 1)],
                [(3, 1), (3, -1)], [(3, -1), (-3, -1)]]

        publisher(nbcf, nbtb, nbres)
    except rospy.ROSInterruptException:
        pass
