#!/usr/bin/env python3
# Python 2/3 compatibility imports
from __future__ import print_function
from os import wait


from six.moves import input
import yaml

import sys
import time
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import rospkg

try:
    from math import pi, tau, dist, fabs, cos
except:  # For Python 2 compatibility
    from math import pi, fabs, cos, sqrt

    tau = 2.0 * pi

    def dist(p, q):
        return sqrt(sum((p_i - q_i) ** 2.0 for p_i, q_i in zip(p, q)))


from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list


from tf.transformations import quaternion_from_euler
import logging
loger = logging.getLogger('moveit mario')
loger.setLevel(logging.INFO)

ch = logging.StreamHandler()
ch.setLevel(logging.INFO)

formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
ch.setFormatter(formatter)
loger.addHandler(ch)

#logging.basicConfig(format='%(asctime)s - %(message)s',level=logging.DEBUG)

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node("python_moveit", anonymous=True)
rospack = rospkg.RosPack()

## Instantiate a `RobotCommander`_ object. Provides information such as the robot's
## kinematic model and the robot's current joint states
robot = moveit_commander.RobotCommander()
print(robot.get_group_names())

## Instantiate a `PlanningSceneInterface`_ object.  This provides a remote interface
## for getting, setting, and updating the robot's internal understanding of the
## surrounding world:
scene = moveit_commander.PlanningSceneInterface()

## Instantiate a `MoveGroupCommander`_ object.  This object is an interface
## to a planning group (group of joints).  In this tutorial the group is the primary
## arm joints in the Panda robot, so we set the group's name to "panda_arm".
## If you are using a different robot, change this value to the name of your robot
## arm planning group.
## This interface can be used to plan and execute motions:
arm_1_id = "panda_arm_1"
arm_1_move_group = moveit_commander.MoveGroupCommander(arm_1_id)
arm_3_id = "panda_arm_3"
arm_3_move_group = moveit_commander.MoveGroupCommander(arm_3_id)

hand_1_id = "hand_1"
hand_1_move_group = moveit_commander.MoveGroupCommander(hand_1_id)
hand_3_id = "hand_3"
hand_3_move_group = moveit_commander.MoveGroupCommander(hand_3_id)



## Create a `DisplayTrajectory`_ ROS publisher which is used to display
## trajectories in Rviz:
display_trajectory_publisher = rospy.Publisher(
    "/move_group/display_planned_path",
    moveit_msgs.msg.DisplayTrajectory,
    queue_size=20,
)

robot_poses = []
with open(rospack.get_path('mario_package')+"/config/poses.yaml") as f:
    robot_poses = yaml.load(f, Loader=yaml.SafeLoader)
    logging.info(robot_poses)

def basic_info():
    ## BEGIN_SUB_TUTORIAL basic_info
    ##
    ## Getting Basic Information
    ## ^^^^^^^^^^^^^^^^^^^^^^^^^
    # We can get the name of the reference frame for this robot:
    planning_frame = arm_1_move_group.get_planning_frame()
    print("============ Planning frame: %s" % planning_frame)

    # We can also print the name of the end-effector link for this group:
    eef_link = arm_1_move_group.get_end_effector_link()
    print("============ End effector link: %s" % eef_link)

    # We can get a list of all the groups in the robot:
    group_names = robot.get_group_names()
    print("============ Available Planning Groups:",group_names)

    # Sometimes for debugging it is useful to print the entire state of the
    # robot:
    print("============ Printing robot state")
    print(robot.get_current_state())
    print("")

def wait_for_state_update(object_name='',object_is_attached_check=False,object_is_dettached_check=False,timeout=4):

    ## Ensuring Collision Updates Are Received
    ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    ## If the Python node was just created (https://github.com/ros/ros_comm/issues/176),
    ## or dies before actually publishing the scene update message, the message
    ## could get lost and the box will not appear. To ensure that the updates  
    ## We do 3 checks:
    ## - If a box name is provided, check it exists
    ## - If we need it attached, we check that it is attached. If we need it to be dettached, we also check it
    ## - General timeout
    start = rospy.get_time()
    seconds = rospy.get_time()
    while (seconds - start < timeout) and not rospy.is_shutdown():
        # Test if the box is in attached objects
        if object_name:

            # Test if the box is in the scene.
            # Note that attaching the box will remove it from known_objects
            is_object = object_name in scene.get_known_object_names()
            if is_object:
                if object_is_attached_check or object_is_dettached_check:
                    attached_objects = scene.get_attached_objects([object_name])
                    is_attached = len(attached_objects.keys()) > 0
                    if is_attached and object_is_attached_check:
                        return True
                    if not is_attached and object_is_dettached_check:
                        return True
                else:
                    return True


            # Sleep so that we give other threads time on the processor
            rospy.sleep(0.1)
            seconds = rospy.get_time()

    # If we exited the while loop without returning then we timed out
    return False

def add_boxes():
    name = 'box1'
    mesh_path = rospack.get_path('mario_package')+"/models/assembly_box/lacaja.stl"

    box_pose = geometry_msgs.msg.PoseStamped()
    box_pose.header.frame_id ="panda_3_link0"
    box_pose.pose.position.z = 0.95
    box_pose.pose.position.x = 0.15
    box_pose.pose.position.y = 0.2
      
      # RPY to convert: 0deg, 180deg, 0deg
    q = quaternion_from_euler(0, 1.5707*2, 0)
    box_pose.pose.orientation.x = q[0]
    box_pose.pose.orientation.y = q[1]
    box_pose.pose.orientation.z = q[2]
    box_pose.pose.orientation.w = q[3]
    scene.add_mesh(name,box_pose,mesh_path,size=(0.015,0.015,0.015))
    wait_for_state_update(object_name=name)

    name = 'box2'
    box2_pose = geometry_msgs.msg.PoseStamped()
    box2_pose.header.frame_id ="panda_1_link0"
    box2_pose.pose.position.z = 1.1
    box2_pose.pose.position.x = 0.1

    q = quaternion_from_euler(0, 1.5707*2, 0)
    box2_pose.pose.orientation.x = q[0]
    box2_pose.pose.orientation.y = q[1]
    box2_pose.pose.orientation.z = q[2]
    box2_pose.pose.orientation.w = q[3]

    scene.add_mesh(name,box2_pose,mesh_path,size=(0.015,0.015,0.015))
    wait_for_state_update(object_name=name)

    pass

def open_hand(hand_group):
    loger.info(f"Opening {hand_group.get_name()}")
    joint_goal = hand_group.get_current_joint_values()
    joint_goal[0] = 0.04
    joint_goal[1] = 0.04

    # The go command can be called with joint values, poses, or without any
    # parameters if you have already set the pose or joint target for the group
    hand_group.go(joint_goal, wait=True)

    # Calling ``stop()`` ensures that there is no residual movement
    hand_group.stop()

def close_hand(hand_group):
    loger.info(f"Closing {hand_group.get_name()}")
    joint_goal = hand_group.get_current_joint_values()
    joint_goal[0] = 0
    joint_goal[1] = 0

    # The go command can be called with joint values, poses, or without any
    # parameters if you have already set the pose or joint target for the group
    hand_group.go(joint_goal, wait=True)

    # Calling ``stop()`` ensures that there is no residual movement
    hand_group.stop()

def arm_1_grab_box():
    # hand_1_move_group.set_pose_target()
    open_hand(hand_1_move_group)

    # move in front of box
    joint_goal = robot_poses["arm_1_pre_box_position"]
    arm_1_move_group.go(joint_goal,wait=True)
    arm_1_move_group.stop()

    # group.set_pose_target(pose_goal)    # attach to box
    time.sleep(1)
    joint_goal = robot_poses["arm_1_box_position"]
    arm_1_move_group.go(joint_goal,wait=True)
    arm_1_move_group.stop()
    
    # attach box
    arm_1_move_group.attach_object('box2',link_name='panda_1_hand')

def arm_3_grab_box():
    # hand_1_move_group.set_pose_target()
    open_hand(hand_3_move_group)

    # move in front of box
    joint_goal = robot_poses["arm_3_pre_box_position"]
    arm_3_move_group.go(joint_goal,wait=True)
    arm_3_move_group.stop()

    # group.set_pose_target(pose_goal)    # attach to box
    time.sleep(1)
    joint_goal = robot_poses["arm_3_box_position"]
    arm_3_move_group.go(joint_goal,wait=True)
    arm_3_move_group.stop()
    
    # attach box
    arm_3_move_group.attach_object('box1',link_name='panda_3_hand')

def move_pre_attachment():
    joint_goal = robot_poses["arm_1_pre_attachment"]
    arm_1_move_group.go(joint_goal,wait=True)
    arm_1_move_group.stop()
    joint_goal = robot_poses["arm_3_pre_attachment"]
    arm_3_move_group.go(joint_goal,wait=True)
    arm_3_move_group.stop()

def attach_pieces():
    joint_goal = robot_poses["arm_1_attachment"]
    arm_1_move_group.go(joint_goal,wait=True)
    arm_1_move_group.stop()
    joint_goal = robot_poses["arm_3_attachment"]
    arm_3_move_group.go(joint_goal,wait=True)
    arm_3_move_group.stop()

    scene.remove_attached_object('panda_1_hand','box2')
    scene.remove_attached_object('panda_3_hand','box1')

def move_away_attachment():
    joint_goal = robot_poses["arm_1_dettach"]
    arm_1_move_group.go(joint_goal,wait=True)
    arm_1_move_group.stop()
    joint_goal = robot_poses["arm_3_dettach"]
    arm_3_move_group.go(joint_goal,wait=True)
    arm_3_move_group.stop()

def main():

    basic_info()
    input("============ Press `Enter` to add a box to the planning scene ...")
    add_boxes()
    input("============ Press `Enter` to move arm 1 to get box ...")
    arm_1_grab_box()
    input("============ Press `Enter` to move arm 3 to get box ...")
    arm_3_grab_box()
    input("============ Press `Enter` move arms to preattach ...")
    move_pre_attachment()
    input("============ Press `Enter` to do the attachment ...")
    attach_pieces()
    input("============ Press `Enter` to move away from the attachment ...")
    move_away_attachment()
    #loger.info(arm_1_move_group.get_current_state())
    #loger.info(arm_1_move_group.get_current_joint_values())
    #loger.info(arm_3_move_group.get_current_joint_values())
    #loger.info(arm_1_move_group.get_named_targets())
    #loger.info(arm_1_move_group.get_remembered_joint_values())
    pass

if __name__=="__main__":
    main()
