#!/usr/bin/env python3

import rospy
import sys
import time
import numpy as np
from std_msgs.msg import Int8MultiArray
from geometry_msgs.msg import Point


def callback_mod(data):
    global Modes
    Modes = data.data
    print(Modes)
    #rospy.loginfo(rospy.get_caller_id() + "Des ressources ont été prises %s")


def callback_res(data):
    global Ressources
    Ressources = data.data
    print("Ressources prise")
    #rospy.loginfo(rospy.get_caller_id() + "Des ressources ont été prises %s")


def callback_path(data):
    global Ref
    Ref = np.array([[data.x], [data.y], [data.z]])
    print("pos", data.x, data.y, data.z, "Modes", Modes)
    #rospy.loginfo(rospy.get_caller_id() + "La référence a bougé")


def subscriber():
    global RobotNo, Modes, Ressources, Ref
    rospy.init_node('CF'+str(RobotNo), anonymous=False)

    rospy.Subscriber('Modes', Int8MultiArray, callback_mod)
    rospy.Subscriber('Ressources', Int8MultiArray, callback_res)
    rospy.Subscriber('Path', Point, callback_path)

    pub_ch = rospy.Publisher('Changement', Int8MultiArray, queue_size=10)
    rate = rospy.Rate(10)  # 10hz

    t0 = time.time()

    while not rospy.is_shutdown():
        if (time.time() - t0) > 10:
            t0 = time.time()
            mod = (Modes[RobotNo - 1] + 1) % 3
            msg = Int8MultiArray()
            msg.data = [RobotNo-1, mod, RobotNo]
            pub_ch.publish(msg)
            print("message envoyé")

        rate.sleep()

    rospy.spin()


if __name__ == '__main__':
    RobotNo = int(sys.argv[1])

    Modes = [0 for i in range(3)]
    Ressources = [0 for i in range(3)]
    Ref = np.array([[-3], [0], [1.5]])

    subscriber()
