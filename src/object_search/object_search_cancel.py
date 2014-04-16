#! /usr/bin/env python

import roslib; roslib.load_manifest('object_search')
import rospy

# Brings in the SimpleActionClient
import actionlib

import object_search_action.msg

def object_search_client():
    # Creates the SimpleActionClient, passing the type of the action
    # to the constructor.
    client = actionlib.SimpleActionClient('object_search_action', object_search_action.msg.SearchAction)

    # Waits until the action server has started up and started
    # listening for goals.
    client.wait_for_server()

    # Sends the goal to the action server.
    client.cancel_all_goals()


if __name__ == '__main__':
    try:
        # Initializes a rospy node so that the SimpleActionClient can
        # publish and subscribe over ROS.
        rospy.init_node('object_search_cancel')
        object_search_client()
    except rospy.ROSInterruptException:
        print "program interrupted before completion"
