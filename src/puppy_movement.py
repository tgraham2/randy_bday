#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
from puppy_control.msg import Pose

class PuppyMovement:
    def __init__(self):
        rospy.init_node('puppy_movement')
        self.pose_pub = rospy.Publisher('/puppy_control/pose', Pose, queue_size=10)
        self.command_sub = rospy.Subscriber('/puppy_control/command', String, self.command_callback)
        rospy.loginfo("Puppy Movement Node Started!")

    def command_callback(self, msg):
        command = msg.data.lower().strip()
        rospy.loginfo(f"Received command: {command}")

        if command == "raise right paw":
            self.raise_right_paw()
        elif command == "sit":
            self.sit()
        elif command == "stand":
            self.stand()
        elif command == "wave":
            self.wave()
        else:
            rospy.logwarn(f"Unknown command: {command}")

    def raise_right_paw(self):
        rospy.loginfo("Raising right paw...")
        pose = Pose()
        pose.stance_x = -2
        pose.stance_y = 4
        pose.x_shift = 0.5
        pose.height = -8
        pose.roll = 5
        pose.pitch = 0
        pose.yaw = 0
        pose.run_time = 1000
        self.pose_pub.publish(pose)

    def sit(self):
        rospy.loginfo("Sitting down...")
        pose = Pose()
        pose.stance_x = 0
        pose.stance_y = 0
        pose.x_shift = 0
        pose.height = -15
        pose.roll = 0
        pose.pitch = -10
        pose.yaw = 0
        pose.run_time = 1000
        self.pose_pub.publish(pose)

    def stand(self):
        rospy.loginfo("Standing up...")
        pose = Pose()
        pose.stance_x = 0
        pose.stance_y = 0
        pose.x_shift = 0
        pose.height = -10
        pose.roll = 0
        pose.pitch = 0
        pose.yaw = 0
        pose.run_time = 1000
        self.pose_pub.publish(pose)

    def wave(self):
        rospy.loginfo("Waving paw...")
        for _ in range(3):
            self.raise_right_paw()
            rospy.sleep(1)
            self.stand()
            rospy.sleep(1)

if __name__ == '__main__':
    try:
        PuppyMovement()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
