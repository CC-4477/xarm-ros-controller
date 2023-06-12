#!/usr/bin/env python
import rospy
from std_msgs.msg import Bool
from xarm_planner.srv import pose_plan, pose_plan, exec_plan
from xarm_msgs.srv import SetInt16
from geometry_msgs.msg import Pose
import time
def request_plan(client, srv):
    response = client(srv.target)
    return response.success
def request_exec(client, srv):
    response = client(True)
    return response.success
def create_motion_plan(target_pose):
    rospy.wait_for_service("xarm_pose_plan")
    try:
        pose_plan_service = rospy.ServiceProxy("xarm_pose_plan", pose_plan)
        response = pose_plan_service(target_pose)
        if response.success:
            rospy.loginfo("Motion plan created successfully!")
            return response
        else:
            rospy.logwarn("Failed to create motion plan.")
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s", str(e))
if __name__ == "__main__":
    tar_pos = Pose()
    tar_pos.position.x = 0.25
    tar_pos.position.y = 0.25
    tar_pos.position.z = 0.25
    tar_pos.orientation.x = 1.0
    tar_pos.orientation.y = 0.0
    tar_pos.orientation.z = 0.0
    tar_pos.orientation.w = 0.0
    rospy.init_node("xarm_simple_planner_client")
    client = rospy.ServiceProxy("xarm_pose_plan", pose_plan)
    client_exec = rospy.ServiceProxy("xarm_exec_plan", exec_plan)
    client_stop = rospy.ServiceProxy("xarm/set_state", SetInt16)
    exec_pub = rospy.Publisher("xarm_planner_exec", Bool, queue_size=10)
    srv = pose_plan()
    srv_exec = exec_plan()
    srv_stop = SetInt16()
    rospy.wait_for_service("xarm_exec_plan")
    rospy.wait_for_service("ufactory/set_state")
    motion_plan = create_motion_plan(tar_pos)
    if motion_plan is not None:
        while not rospy.is_shutdown():
            srv.target = tar_pos
            if request_plan(client, srv):
                rospy.loginfo("Plan SUCCESS! Executing...")
                srv_exec.execute = True
                if not request_exec(client_exec, srv_exec):
                    srv_stop.data = 4
                    client_stop(srv_stop)
                    rospy.logwarn("OOPS! SETTING STOP...")
                    break
    else:
        rospy.logwarn("Failed to create motion plan. Exiting...")
    rospy.signal_shutdown("Program terminated")