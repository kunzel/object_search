#! /usr/bin/env python
import rospy
import smach
import smach_ros
import json

from nav_goals_msgs.srv import NavGoals
from nav_goals_msgs.srv import WeightedNavGoals

class UninformedSearch_Random (smach.State):

    """
    Select next search pose

    """
    
    def __init__(self, inflation_radius, polygon):

        self.inflation_radius = inflation_radius
        self.polygon = polygon
        
        smach.State.__init__(self,
                             outcomes=['succeeded', 'aborted', 'preempted'],
                             output_keys=['pose_output'])

        rospy.wait_for_service('nav_goals')
        try:
            self.nav_goals = rospy.ServiceProxy('nav_goals', NavGoals)
        except rospy.ServiceException, e:
            rospy.logerr("Service call failed: %s" % e)


    def execute(self, userdata):
        rospy.loginfo('Executing state %s', self.__class__.__name__)
        try:
            resp = self.nav_goals(1, self.inflation_radius, self.polygon)
            userdata.pose_output = resp.goals.poses[0]
        except rospy.ServiceException, e:
            rospy.logerr("Service call failed: %s" % e)
            return 'aborted'
               
        return 'succeeded'


class InformedSearch_SupportingPlanes (smach.State):

    def __init__(self, inflation_radius, polygon):

        self.inflation_radius = inflation_radius
        self.polygon = polygon
        
        smach.State.__init__(self,
                             outcomes=['succeeded', 'aborted', 'preempted'],
                             input_keys=['obj_desc','obj_list'],
                             output_keys=['pose_output'])

        rospy.wait_for_service('nav_goals')
        try:
            self.nav_goals = rospy.ServiceProxy('nav_goals', NavGoals)
        except rospy.ServiceException, e:
            rospy.logerr("Service call failed: %s" % e)

        rospy.wait_for_service('nav_goals_evaluation')
        try:
            self.nav_goals_eval = rospy.ServiceProxy('nav_goals_evaluation', WeightedNavGoals)
        except rospy.ServiceException, e:
            rospy.logerr("Service call failed: %s" % e)


    def execute(self, userdata):
        rospy.loginfo('Executing state %s', self.__class__.__name__)
        try:
            nav_goals_resp = self.nav_goals(10, self.inflation_radius, self.polygon)


            nav_goals_eval_resp = self.nav_goals_eval(json.dumps(userdata.obj_desc),
                                                      json.dumps(userdata.obj_list),
                                                      nav_goals_resp.goals)

            
            
            userdata.pose_output = nav_goals_eval_resp.sorted_goals.poses[0]
            
        except rospy.ServiceException, e:
            rospy.logerr("Service call failed: %s" % e)
            return 'aborted'
               
        return 'succeeded'


class InformedSearch_QSR (smach.State):
    pass
