#! /usr/bin/env python
import rospy
import smach
import smach_ros

from nav_goals_msgs.srv import NavGoals

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
    pass

class InformedSearch_QSR (smach.State):
    pass
