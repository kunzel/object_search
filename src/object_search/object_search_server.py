#! /usr/bin/env python

import roslib; roslib.load_manifest('object_search')
import rospy

import threading

import actionlib

from object_search_action.msg import *

from search_agent import SearchAgent


class ObjectSearchServer(object):
    # create messages that are used to publish feedback/result
    _feedback = object_search_action.msg.SearchFeedback()
    _result   = object_search_action.msg.SearchResult()
    
    def __init__(self, name):
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, object_search_action.msg.SearchAction, execute_cb=self.execute_cb, auto_start = False)

        self._as.start()
        rospy.loginfo('Started action server for object search')
        
        
    def execute_cb(self, goal):

        rospy.loginfo('Received object search request: %s', goal)
        
        # helper variables
        r = rospy.Rate(1)
        self.success = True

        # create an agent
        self.agent = SearchAgent()

        # set userdata: object type, ...
        self.agent.set_sm_userdata(goal.obj_desc)        
        # run agent's state machine with or without introspection server
        # outcome = agent.execute_sm()

        smach_thread = threading.Thread(target = self.agent.execute_sm_with_introspection)
        smach_thread.start()

        #outcome = self.agent.execute_sm_with_introspection()
        r.sleep()
        
        while self.agent.get_sm().is_running() and not self.agent.get_sm().preempt_requested():
            # check that preempt has not been requested by the client
            if self._as.is_preempt_requested():
                rospy.loginfo('%s: Preempted' % self._action_name)
                self._as.set_preempted()
                self.agent.get_sm().request_preempt()
                self.success = False
                break

            #rospy.loginfo(self.agent.get_sm().get_active_states())
            userdata = self.agent.get_sm_userdata()

            
                
            # get current pose form state machine
            self._feedback = object_search_action.msg.SearchFeedback()
            self._feedback.state = userdata.state

            
            if userdata.state == 'driving' and self.send_pose == False: 
                self._feedback.goal_pose = userdata.sm_pose_data
                self.send_clouds = False
                self.send_pose = True
                self._as.publish_feedback(self._feedback)
            elif userdata.state == 'image_analysis' and self.send_clouds == False:
                self._feedback.objs = userdata.cloud
                self._feedback.labels = userdata.labels
                self.send_clouds = True
                self._as.publish_feedback(self._feedback)
            elif userdata.state != 'image_analysis' and userdata.state != 'driving':
                self.send_pose = False
                self._as.publish_feedback(self._feedback)

            r.sleep()

        if self.success:
            
            userdata = self.agent.get_sm_userdata()

            rospy.loginfo('obj desc: %s', userdata.sm_obj_desc)
            rospy.loginfo('obj found: %s ' % userdata.obj_found)
            rospy.loginfo('obj descs: %s ' % userdata.obj_descs)
            rospy.loginfo('time (sec)): %s ' % userdata.time)
            rospy.loginfo('exceeded_max_time: %s ' % userdata.exceeded_max_time)
            rospy.loginfo('exceeded_max_poses: %s ' % userdata.exceeded_max_poses)

            rospy.loginfo('%s: Succeeded' % self._action_name)


            self._result.obj_found = userdata.obj_found
            self._result.exceeded_max_time = userdata.exceeded_max_time
            self._result.exceeded_max_poses = userdata.exceeded_max_poses
            
            self._as.set_succeeded(self._result)

# # append the seeds for the fibonacci sequence
# self._feedback.sequence = []



if __name__ == '__main__':
  rospy.init_node('object_search_action')
  ObjectSearchServer(rospy.get_name())
  rospy.spin()
