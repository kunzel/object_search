#! /usr/bin/env python
#import roslib; roslib.load_manifest('object_search')
import rospy
import smach
import smach_ros
import sys
import getopt


from nav_goals_msgs.srv import NavGoals
from geometry_msgs.msg import Polygon
from agent import KBAgent

class Searcher(KBAgent):

    """
    Definition of a search-agent.

    """
    
    def make_sm(self):
        self.sm = smach.StateMachine(outcomes=['succeeded','aborted','preempted'])

        with self.sm:
            smach.StateMachine.add('SearchMonitor', SearchMonitor(),                      
                                   { 'search_succeeded': 'succeeded',
                                     'search_failed': 'aborted',
                                     'succeeded': 'NextSearchPose', # search_in_progress
                                     'aborted':'aborted',
                                     'preempted':'preempted'})

            smach.StateMachine.add('NextSearchPose',NextSearchPose(), 
                                   {'succeeded':'GoToPose',
                                    'aborted':'aborted',
                                    'preempted':'preempted'})
            
            smach.StateMachine.add('GoToPose', GoToPose(), 
                                   {'succeeded':'Perceive',
                                    'aborted':'aborted',
                                    'preempted':'preempted'})

            smach.StateMachine.add('Perceive', Perceive(), 
                                   {'succeeded':'SearchMonitor',
                                    'aborted':'aborted',
                                    'preempted':'preempted'})

        
        return self.sm

    def set_sm_userdata(self, obj):
        """Set userdata for state machine."""
        self.init_kb()
        self.sm.userdata.obj = obj


#______________________________________________________________________________
# behavior

class SearchMonitor (smach.State):
        
    """
    Monitors the progress of the overall search. Also builds statistics.

    """

    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['search_succeeded','search_failed','succeeded', 'aborted', 'preempted'])
        self.first_call = True

    def execute(self, userdata):
        rospy.loginfo('Executing state %s', self.__class__.__name__)
        
        if self.first_call:
            # take time etc
            self.first_call = False

        return 'succeeded'


class NextSearchPose (smach.State):

    """
    Select appropriate lesson to do next.

    """
    
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['succeeded', 'aborted', 'preempted'])

        rospy.wait_for_service('nav_goals')
        try:
            self.nav_goals = rospy.ServiceProxy('nav_goals', NavGoals)
        except rospy.ServiceException, e:
            rospy.logerr("Service call failed: %s" % e)


    def execute(self, userdata):
        rospy.loginfo('Executing state %s', self.__class__.__name__)
        try:
            poly = Polygon()
            resp = self.nav_goals(1,0.7,poly)
            rospy.loginfo(resp)
        except rospy.ServiceException, e:
            rospy.logerr("Service call failed: %s" % e)
            return 'aborted'
               
        return 'succeeded'


class GoToPose (smach.State):

    """
    Go to pose.

    """
    
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['succeeded', 'aborted', 'preempted'])


    def execute(self, userdata):
        rospy.loginfo('Executing state %s', self.__class__.__name__)
               
        return 'succeeded'



    
class Perceive (smach.State):

    """
    Perceive the environemt.

    """

    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['succeeded', 'aborted', 'preempted'])

    def execute(self, userdata):
        rospy.loginfo('Executing state %s', self.__class__.__name__)
        return 'succeeded'

    
#______________________________________________________________________________
# main  

class Usage(Exception):
    def __init__(self, msg):
        self.msg = msg

def help_msg():
    return """
  Usage: searcher.py [-h] <object> 

    object        type of the object

    -h, --help for seeing this msg
"""

def main(argv=None):
    if argv is None:
        argv = sys.argv
    try:
        try:
            opts, args = getopt.getopt(argv[1:], "h", ["help"])
        except getopt.error, msg:
            raise Usage(msg)

        if ('-h','') in opts or ('--help', '') in opts: #len(args) != 2 or
            raise Usage(help_msg())

        print >>sys.stderr, args
        
        # create ros node
        rospy.init_node("searcher_node")
        
        # create an agent
        agent = Searcher()

        # set userdata: object type, ...
        agent.set_sm_userdata(args[0])
        
        # run agent's state machine with or without introspection server
        # outcome = agent.execute_sm()
        outcome = agent.execute_sm_with_introspection()
        
        rospy.spin()
        
    except Usage, err:
        print >>sys.stderr, err.msg
        print >>sys.stderr, "for help use --help"
        return 2

if __name__ == "__main__":
    sys.exit(main())

