#!/usr/bin/env python
import rospy
from moveit_commander import PlanningSceneInterface
from moveit_msgs.msg import CollisionObject
from geometry_msgs.msg import Pose
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import Bool

def add_obstacle():
    rospy.loginfo("Adding obstacle to the planning scene...")

    # Create a PlanningSceneInterface object
    scene = PlanningSceneInterface()

    # Create a CollisionObject for the box obstacle
    obstacle = CollisionObject()
    obstacle.id = "obstacle"
    obstacle.header.frame_id = "link_base"

    # Define the pose of the obstacle (position and orientation)
    obstacle_pose = Pose()
    obstacle_pose.position.x = 0.5  # Example position
    obstacle_pose.position.y = 0.2
    obstacle_pose.position.z = 0.3
    obstacle_pose.orientation.w = 1.0  # Example orientation
    obstacle.primitive_poses.append(obstacle_pose)

    # Define the shape of the obstacle as a box
    box = SolidPrimitive()
    box.type = SolidPrimitive.BOX
    box.dimensions = [1.2, 1.2, 1.2]  # Example dimensions (length, width, height)
    obstacle.primitives.append(box)

    # Add the obstacle to the planning scene
    scene.add_object(obstacle)

    rospy.loginfo("Obstacle added to the planning scene!")

if __name__ == '__main__':
    rospy.init_node("obstacle_node")
    rospy.Subscriber("add_object", Bool, add_obstacle)

    add_obstacle()
    rospy.spin()
