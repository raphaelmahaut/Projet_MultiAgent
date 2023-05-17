#!/usr/bin/env python

import rospy
from std_msgs.msg import Int8MultiArray


def callback(data):
    rospy.loginfo("Received Int8MultiArray: %s", data.data)


def pub_sub_node():
    rospy.init_node('pub_sub_node', anonymous=True)
    pub = rospy.Publisher('Mode', Int8MultiArray, queue_size=10)
    rospy.Subscriber('Mode', Int8MultiArray, callback)
    rate = rospy.Rate(10)  # Fréquence de publication (10 Hz)

    while not rospy.is_shutdown():
        # Créer un message Int8MultiArray
        message = Int8MultiArray()
        message.data = [1, 2, 3, 4, 5]  # Exemple de données

        # Publier le message
        pub.publish(message)

        rate.sleep()


if __name__ == '__main__':
    try:
        pub_sub_node()
    except rospy.ROSInterruptException:
        pass
