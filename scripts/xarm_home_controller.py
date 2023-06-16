#!/usr/bin/env python


import rospy
from xarm_msgs.srv import Move, MoveRequest, SetInt16, SetInt16Request
from std_msgs.msg import Int16

def move_callback(home_msg):
    tar_home = MoveRequest()
    tar_home.pose = home_msg.pose
    tar_home.mvvelo = home_msg.mvvelo
    tar_home.mvacc = home_msg.mvacc
    tar_home.mvtime = home_msg.mvtime
    tar_home.mvradii = home_msg.mvradii
    go_home_service(tar_home)
def state_callback(state_val):
    state_value = SetInt16()
    state_value = state_val.data
    set_state_service(state_value)
    
if __name__ == '__main__':
    rospy.init_node('go_home_node')
    srv_go = SetInt16()
    srv_go.data = 0
    set_state_service = rospy.ServiceProxy('ufactory/set_state', SetInt16)
    go_home_service = rospy.ServiceProxy('/ufactory/go_home', Move)


    rospy.Subscriber("set_state", Int16, state_callback)
    rospy.Subscriber("go_home", MoveRequest, move_callback)
    rospy.wait_for_service('/ufactory/go_home')
    rospy.wait_for_service('/ufactory/set_state')

    rospy.spin()