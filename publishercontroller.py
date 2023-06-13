#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Pose, Point, Quaternion

def pose_publisher():
    rospy.init_node("xarm_target_pose_publisher")
    pub = rospy.Publisher("xarm_target_pose", Pose, queue_size=10)
    rate = rospy.Rate(10)  # Publish rate (10 Hz)

    while not rospy.is_shutdown():
        # Get user input for position and orientation
        x = float(input("Enter X coordinate: "))
        y = float(input("Enter Y coordinate: "))
        z = float(input("Enter Z coordinate: "))
        qx = float(input("Enter X component of the orientation quaternion: "))
        qy = float(input("Enter Y component of the orientation quaternion: "))
        qz = float(input("Enter Z component of the orientation quaternion: "))
        qw = float(input("Enter W component of the orientation quaternion: "))

        # Create a Pose message with the user input
        pose_msg = Pose()
        pose_msg.position = Point(x=x, y=y, z=z)
        pose_msg.orientation = Quaternion(x=qx, y=qy, z=qz, w=qw)

        # Publish the pose message
        pub.publish(pose_msg)

        rate.sleep()

if __name__ == "__main__":
    try:
        pose_publisher()
    except rospy.ROSInterruptException:
        pass
