#!/usr/bin/env python


import rospy
from xarm_msgs.srv import Move, MoveRequest

def move_callback(home_msg):
    tar_home = Move()
    tar_home.pose = home_msg.pose
    tar_home.mvvelo = home_msg.mvvelo
    tar_home.mvacc = home_msg.mvacc
    tar_home.mvtime = home_msg.mvtime
    tar_home.mvradii = home_msg.mvradii
    
if __name__ == '__main__':
    rospy.init_node('go_home_node')
    rospy.wait_for_service('/ufactory/go_home')
    go_home_service = rospy.ServiceProxy('/ufactory/go_home', Move)
    rospy.Subscriber("go_home", Move, move_callback)

    rospy.spin()