#! /usr/bin/env python
#import roslib; roslib.load_manifest('object_search')
import rospy
import smach
import smach_ros
import sys
import getopt
import random

import actionlib
from actionlib_msgs.msg import *
from move_base_msgs.msg import *

from nav_goals_msgs.srv import NavGoals
from geometry_msgs.msg import Polygon
from geometry_msgs.msg import Pose
from agent import KBAgent

from search_methods import UninformedSearch_Random
from search_methods import InformedSearch_SupportingPlanes
from search_methods import InformedSearch_QSR


MOVE_BASE_EXEC_TIMEOUT=rospy.Duration(600.0)
MOVE_BASE_PREEMPT_TIMEOUT=rospy.Duration(10.0)


#callback that build the move_base goal, from the input data        
def move_base_goal_cb(userdata,goal):
    
    next_goal = move_base_msgs.msg.MoveBaseGoal()            
    next_goal.target_pose.header.frame_id = "/map"
    next_goal.target_pose.header.stamp = rospy.Time.now()
    next_goal.target_pose.pose = userdata.pose_input 
    
    return next_goal


class SearchAgent(KBAgent):

    """
    Definition of a search-agent.

    """
    def __init__(self, search_method):

        if search_method == 'is_sp':
            self.search_method = InformedSearch_SupportingPlanes()
        elif search_method == 'is_qsr':
            self.search_method = InformedSearch_QSR()
        else: # 'us_rand'
            poly = Polygon() # empty polygon -> consider whole map
            self.search_method = UninformedSearch_Random(0.7, poly)

        # the super().__init__() methods calls make_sm(),
        # hence self.search_method has to be set in advance
        super(SearchAgent,self).__init__()
       

    
    def make_sm(self):
        self.sm = smach.StateMachine(outcomes=['succeeded','aborted','preempted'])
        
        with self.sm:

            smach.StateMachine.add('SearchMonitor', SearchMonitor(),                      
                                   transitions={ 'search_succeeded': 'succeeded',
                                                 'search_aborted':   'aborted',
                                                 'search_in_progress': 'SearchMethod', 
                                                 'preempted':'preempted'},
                                   remapping={'obj':'sm_object',
                                              'max_poses':'sm_max_poses',
                                              'max_time':'sm_max_time'})

            
            smach.StateMachine.add('SearchMethod', self.search_method, 
                                   transitions={'succeeded':'GoToPose',
                                                'aborted':'aborted',
                                                'preempted':'preempted'},
                                   remapping={'pose_output':'sm_pose_data'})


            smach.StateMachine.add('GoToPose',
                                   smach_ros.SimpleActionState('move_base',
                                                               MoveBaseAction,
                                                               goal_cb = move_base_goal_cb,
                                                               #result_cb = move_base_result_cb,
                                                               input_keys = ['pose_input'],
                                                               exec_timeout = MOVE_BASE_EXEC_TIMEOUT,
                                                               preempt_timeout = MOVE_BASE_PREEMPT_TIMEOUT
                                                               ),
                                   transitions={'succeeded':'Perceive','aborted':'aborted','preempted':'preempted'},
                                   remapping={'pose_input':'sm_pose_data'},
                                   )
            
            # smach.StateMachine.add('GoToPose', GoToPose(), 
            #                        transitions={'succeeded':'Perceive',
            #                                     'aborted':'aborted',
            #                                     'preempted':'preempted'},
            #                        remapping={'pose_input':'sm_pose_data'})

            smach.StateMachine.add('Perceive', Perceive(), 
                                   {'succeeded':'SearchMonitor',
                                    'aborted':'aborted',
                                    'preempted':'preempted'})

        
        return self.sm

    def set_sm_userdata(self, obj):
        """Set userdata for state machine."""
        self.init_kb()
        self.sm.userdata.sm_object    = obj
        self.sm.userdata.sm_max_time  = 120
        self.sm.userdata.sm_max_poses = 10


112#______________________________________________________________________________
# behavior

class SearchMonitor (smach.State):
        
    """
    Monitors the progress of the overall search. Also builds statistics.

    """

    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['search_succeeded','search_aborted','search_in_progress','preempted'],
                             input_keys=['obj','max_time','max_poses'],
                             output_keys=['obj_found','obj_pose','searched_poses','time','exceeded_max_time','exceeded_max_poses'])
        self.first_call = True

    def execute(self, userdata):
        rospy.loginfo('Executing state %s', self.__class__.__name__)
        # rospy.loginfo('Searching for %s' % obj )
        
        userdata.obj_found = False
        userdata.obj_pose  = Pose()
        userdata.searched_poses  = []
        userdata.time = 0
        userdata.exceeded_max_time = False
        userdata.exceeded_max_poses = False
        
        if self.first_call:
            self.first_call = False
            self.start = rospy.Time.now().secs
            rospy.loginfo('start: %s', self.start)
            self.count = 0

        self.end = rospy.Time.now().secs
        time = self.end - self.start
        userdata.time = time
        
        found = int(random.uniform(1, 300))
        # check whether obj was found
        if found == 1: 
            userdata.obj_found = True
            # Todo: set pose
            userdata.obj_pose =  Pose() 

            rospy.loginfo('object found: %s', userdata.obj)
            rospy.loginfo('total time: %s', time)
            rospy.loginfo('searched poses: %s', self.count)
            return 'search_succeeded'

        # check for max_time 
        if time >= userdata.max_time:
            userdata.exceeded_max_time = True
            # also check for max_poses
            if self.count >= userdata.max_poses:
                userdata.exceeded_max_poses = True

            rospy.loginfo('total time: %s', time)
            rospy.loginfo('searched poses: %s', self.count)
            return 'search_aborted'
        
        # check for max_poses
        self.count += 1
        if self.count >  userdata.max_poses:
            userdata.exceeded_max_poses = True

            rospy.loginfo('total time: %s', time)
            rospy.loginfo('searched poses: %s', self.count - 1)
            return 'search_aborted'

        return 'search_in_progress'



# class GoToPose (smach.State):

#     """
#     Go to pose.

#     """
    
#     def __init__(self):
#         smach.State.__init__(self,
#                              outcomes=['succeeded', 'aborted', 'preempted'],
#                              input_keys=['pose_input'])


#     def execute(self, userdata):
#         rospy.loginfo('Executing state %s', self.__class__.__name__)
        
#         rospy.loginfo(userdata.pose_input)
#         return 'succeeded'



    
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
  Usage: searcher.py [-h] <object> <method>

    object        type of the object
    method        search method: uninformed, informed-planes, informed-qsr 

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

b        print >>sys.stderr, args
        
        # create ros node
        rospy.init_node("searcher_node")
        
        # create an agent
        agent = SearchAgent(args[1])

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

