#!/usr/bin/env python

import rospy
from mani_test.msg import Mdata

msg = Mdata()

def talker():
    pub = rospy.Publisher("Motor_data", Mdata, queue_size=1)
    rospy.init_node('mani_test_publisher', anonymous=False)
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():

        msg.id      = [0,1,2,3]
        msg.speed   = [400,1424,400,1424]
        msg.second  = [0.2,0.5,0.65,0.9]

        rospy.loginfo(msg)
        pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
