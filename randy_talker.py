#!/usr/bin/env python3
import rospy
from std_msgs.msg import String

def talker():
    rospy.init_node('puppy_talks', anonymous=True)
    pub = rospy.Publisher('/puppy_speech', String, queue_size=10)
    rate = rospy.Rate(1)  # 1 message per second

    while not rospy.is_shutdown():
        message = "Hello, I am PuppyPi!"
        rospy.loginfo(message)
        pub.publish(message)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass

