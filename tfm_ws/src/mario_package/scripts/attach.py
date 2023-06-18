#!/usr/bin/env python3

import rospy
from gazebo_ros_link_attacher.srv import Attach, AttachRequest, AttachResponse

# Attach srv
#rospy.init_node('attach_gazebo')
rospy.loginfo("Creating ServiceProxy to /link_attacher_node/attach")
attach_srv = rospy.ServiceProxy('/link_attacher_node/attach',
                                Attach)
attach_srv.wait_for_service()
rospy.loginfo("Created ServiceProxy to /link_attacher_node/attach")

# Dettach srv
#rospy.init_node('dettach_gazebo')
rospy.loginfo("Creating ServiceProxy to /link_attacher_node/detach")
dettach_srv = rospy.ServiceProxy('/link_attacher_node/detach',
                                Attach)
dettach_srv.wait_for_service()
rospy.loginfo("Created ServiceProxy to /link_attacher_node/detach")

def attach_gazebo(arm, cube):
    rospy.loginfo(f"Attaching {arm} and {cube}")
    req = AttachRequest()
    req.model_name_1 = "quad_arm"
    req.link_name_1 = f"{arm}_link7"
    req.model_name_2 = cube
    req.link_name_2 = "box"

    attach_srv.call(req)

def dettach_gazebo(arm, cube):
    rospy.loginfo(f"Dettaching {arm} and {cube}")
    req = AttachRequest()
    req.model_name_1 = "quad_arm"
    req.link_name_1 = f"{arm}_link7"
    req.model_name_2 = cube
    req.link_name_2 = "box"

    dettach_srv.call(req)

if __name__ == '__main__':

    input("Press Enter to attach")
    attach_gazebo("panda_3","box1")
    input("Press Enter to dettach")
    dettach_gazebo("panda_3", "box1")
