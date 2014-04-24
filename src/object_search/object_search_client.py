#! /usr/bin/env python

import roslib; roslib.load_manifest('object_search')
import rospy
import sys

# Brings in the SimpleActionClient
import actionlib

import object_search_action.msg

def object_search_client(obj_type):
    # Creates the SimpleActionClient, passing the type of the action
    # to the constructor.
    client = actionlib.SimpleActionClient('object_search_action', object_search_action.msg.SearchAction)

    # Waits until the action server has started up and started
    # listening for goals.
    client.wait_for_server()

    # Creates a goal to send to the action server.
    json_obj_type = '{"type" : "' +  obj_type  + '"}'			
    goal = object_search_action.msg.SearchGoal(obj_desc = json_obj_type)

    # Sends the goal to the action server.
    client.send_goal(goal)

    # Waits for the server to finish performing the action.
    client.wait_for_result()

    # Prints out the result of executing the action
    return client.get_result()  

if __name__ == '__main__':
    try:
        rospy.init_node('object_search_client')
	if len(sys.argv) != 2:
        	rospy.loginfo('Please provide excactly one object type (not %i)', len(sys.argv)-1)
		sys.exit()

	obj_type = sys.argv[1]
	rospy.loginfo('Searching for a %s', obj_type)
        # Initializes a rospy node so that the SimpleActionClient can
        # publish and subscribe over ROS.
        result = object_search_client(obj_type)
        print "Result:", result.obj_found
    except rospy.ROSInterruptException:
        print "program interrupted before completion"
