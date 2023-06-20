#!/usr/bin/env python
import rospy
from std_msgs.msg import Bool
from xarm_planner.srv import pose_plan, exec_plan, joint_plan
from xarm_msgs.srv import SetInt16, Move
from geometry_msgs.msg import Pose, Point, Quaternion
from sensor_msgs.msg import JointState
import time

def request_plan(client, srv):
    response = client(srv.target)
    return response.success

def request_exec(client, srv):
    response = client(True)
    return response.success

def create_motion_plan(tar_joint):
    rospy.wait_for_service("xarm_pose_plan")
    print("Create Plan")
    try:
        joint_plan_service = rospy.ServiceProxy("xarm_pose_plan", joint_plan)
        response = joint_plan_service(tar_joint)
        if response.success:
            rospy.loginfo("Motion plan created successfully!")
            return response
        else:
            rospy.logwarn("Failed to create motion plan.")
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s", str(e))


def joint_callback(joint_msg):
    # Process the received pose message and create a motion plan
    tar_joint = Move()
    tar_joint.pose = joint_msg.data
    motion_plan = create_motion_plan(tar_joint)
    
    if motion_plan is not None:
        rospy.loginfo("Motion plan created. Executing...")
        srv_exec = exec_plan()
        srv_exec.execute = True
        if not request_exec(client_exec, srv_exec):
            srv_stop.data = 4
            client_stop(srv_stop)
            rospy.logwarn("Execution failed. Stopping...")
    else:
        rospy.logwarn("Failed to create motion plan.")

if __name__ == "__main__":
    rospy.init_node("xarm_position_joint")
    client = rospy.ServiceProxy("xarm_joint_plan", joint_plan)
    client_exec = rospy.ServiceProxy("xarm_exec_plan", exec_plan)
    client_stop = rospy.ServiceProxy("ufactory/set_state", SetInt16)
    exec_pub = rospy.Publisher("xarm_planner_exec", Bool, queue_size=10)
    srv = joint_plan()
    srv_stop = SetInt16()

    rospy.Subscriber("xarm_target_joint", JointState.position, joint_callback)
    rospy.wait_for_service("xarm_joint_plan")
    rospy.wait_for_service("xarm_exec_plan")
    rospy.wait_for_service("ufactory/set_state")

    rospy.spin()

    #rospy.signal_shutdown("Program terminated")
