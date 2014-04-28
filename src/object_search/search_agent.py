#! /usr/bin/env python
#import roslib; roslib.load_manifest('object_search')
import rospy
import smach
import smach_ros
import sys
import getopt
import random
import json

from ros_mary_tts.srv import ros_mary

from agent import Agent

from search_methods import UninformedSearch_Random
from search_methods import InformedSearch_SupportingPlanes
from search_methods import InformedSearch_ViewEvaluation

import actionlib
from actionlib_msgs.msg import *
from move_base_msgs.msg import *

from std_msgs.msg import String
from geometry_msgs.msg import Polygon
from geometry_msgs.msg import Point32
from geometry_msgs.msg import Pose
from nav_goals_msgs.srv import NavGoals

from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import JointState

from classifier_srv_definitions.srv import segment_and_classify


MOVE_BASE_EXEC_TIMEOUT=rospy.Duration(600.0)
MOVE_BASE_PREEMPT_TIMEOUT=rospy.Duration(10.0)

#callback that build the move_base goal, from the input data        
def move_base_goal_cb(userdata,goal):
    
    next_goal = move_base_msgs.msg.MoveBaseGoal()            
    next_goal.target_pose.header.frame_id = "/map"
    next_goal.target_pose.header.stamp = rospy.Time.now()
    next_goal.target_pose.pose = userdata.pose_input 
    
    return next_goal


class SearchAgent(Agent):

    """
    Definition of a search-agent.

    """
    def __init__(self):

        robot = rospy.get_param('robot', 'real')
        inf_radius = rospy.get_param('inflation_radius', '0.5')
        search_method = rospy.get_param('search_method','random')

        polygon = rospy.get_param('search_area',[])
        #polygon = [[1.2,0.5],[-4.6,0.05],[-3.5,-6.0],[1.5,-6.0]]
        rospy.loginfo('Polygon: %s', polygon)
        points = []
        for point in polygon:
            rospy.loginfo('Point: %s', point)
            points.append(Point32(float(point[0]),float(point[1]),0))

        #poly = Polygon() # empty polygon -> consider whole map
        poly = Polygon(points) 
            
        if search_method == 'support':
            self.search_method = InformedSearch_SupportingPlanes(float(inf_radius), poly)
        elif search_method == 'qsr':
            self.search_method = InformedSearch_ViewEvaluation(float(inf_radius), poly)
        else: # 'random'
            self.search_method = UninformedSearch_Random(float(inf_radius), poly)

        if robot == 'real':
            self.perception = PerceiveReal()
        else: # 'sim'
            self.perception = PerceiveSim()




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
                                   remapping={'obj_desc':'sm_obj_desc',
                                              'obj_list':'sm_obj_list',
                                              'min_objs':'sm_min_objs',
                                              'max_objs':'sm_max_objs',
                                              'max_poses':'sm_max_poses',
                                              'max_time':'sm_max_time',
                                              })

            
            smach.StateMachine.add('SearchMethod', self.search_method, 
                                   transitions={'succeeded':'GoToPose',
                                                'aborted':'aborted',
                                                'preempted':'preempted'},
                                   remapping={'obj_desc':'sm_obj_desc',
                                              'obj_list':'sm_obj_list',
                                              'max_poses':'sm_max_poses',
                                              'pose_output':'sm_pose_data',
                                              'view_list':'sm_view_list'})


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

            smach.StateMachine.add('Perceive', self.perception, 
                                   {'succeeded':'SearchMonitor',
                                    'aborted':'aborted',
                                    'preempted':'preempted'},
                                   remapping={'view_list':'sm_view_list',
                                              'obj_list':'sm_obj_list'})

        
        return self.sm

    def set_sm_userdata(self, obj_desc):
        """Set userdata for state machine."""
        rospy.loginfo('Getting parameters from server')

        #obj_desc = rospy.get_param('obj_desc','{"type" : "Bar"}')

        self.sm.userdata.sm_obj_desc  = json.loads(obj_desc)

        self.sm.userdata.state = 'pose_selection' 
        
        self.sm.userdata.sm_min_objs  = rospy.get_param('min_objs',1)
        self.sm.userdata.sm_max_objs =  rospy.get_param('max_objs',1)
        self.sm.userdata.sm_max_time  = rospy.get_param('max_time', 120)
        self.sm.userdata.sm_max_poses = rospy.get_param('max_poses', 10)

        rospy.loginfo("Search for %s", self.sm.userdata.sm_obj_desc)
        rospy.loginfo("min_objs: %s", self.sm.userdata.sm_min_objs)
        rospy.loginfo("max_objs: %s", self.sm.userdata.sm_max_objs)
        rospy.loginfo("max_time: %s", self.sm.userdata.sm_max_time)
        rospy.loginfo("max_poses: %s", self.sm.userdata.sm_max_poses)
        
        # initialize empty obj list
        self.sm.userdata.sm_obj_list  = []

    def get_sm_userdata(self):
        return self.sm.userdata

112#______________________________________________________________________________
# behavior

class SearchMonitor (smach.State):
        
    """
    Monitors the progress of the overall search. Also builds statistics.

    """

    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['search_succeeded','search_aborted','search_in_progress','preempted'],
                             input_keys=['obj_desc','obj_list','min_objs','max_objs','max_time','max_poses'],
                             output_keys=['obj_found','obj_descs','searched_poses','time','exceeded_max_time','exceeded_max_poses'])
        self.first_call = True
        self.found_objs = dict()

        self.ptu_cmd = rospy.Publisher('/ptu/cmd', JointState)

        rospy.wait_for_service('/ros_mary')

        try:
            self.mary = rospy.ServiceProxy('/ros_mary', ros_mary )
        except rospy.ServiceException, e:
            rospy.logerr("Service call failed: %s" % e)


    def execute(self, userdata):
        rospy.loginfo('Executing state %s', self.__class__.__name__)
        # rospy.loginfo('Searching for %s' % obj )
        
        userdata.obj_found = False
        userdata.obj_descs  = [] 
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

        # check for max_time, if exceeded stop immediately 
        if time >= userdata.max_time:
            # check whether we've already found enough objects
            # if yes, the search was successful
            if len(self.found_objs) >= userdata.min_objs:
                userdata.obj_found = True

                rospy.loginfo('call ros_mary')
                try:
                    mary_resp = self.mary("Object located!")
                except rospy.ServiceException, e:
                    rospy.logerr("Service call failed: %s" % e)
                
                obj_descs = []
                for key in self.found_objs:
                    rospy.loginfo('Found obj: %s', self.found_objs[key])
                    obj_descs.append(self.found_objs[key])
                    
                userdata.obj_descs = obj_descs
                    
                rospy.loginfo('searched poses: %s', self.count)

                # nevertheless set flags 
                userdata.exceeded_max_time = True
                # also check for max_poses
                if self.count >= userdata.max_poses:
                    userdata.exceeded_max_poses = True

                return 'search_succeeded'

            # if not, abort search
            userdata.exceeded_max_time = True
            # also check for max_poses
            if self.count >= userdata.max_poses:
                userdata.exceeded_max_poses = True

            rospy.loginfo('searched poses: %s', self.count)
            return 'search_aborted'
        
        # check whether obj was found at last pose
        found = False
        for obj_desc in userdata.obj_list:
            for key in userdata.obj_desc:
                # Todo: match object descriptions in different ways:
                # partial match, full match, contradicting match (one feature matches whereas another dont)
                if key in obj_desc:
                    if obj_desc.get(key) == userdata.obj_desc.get(key):
                        self.found_objs[obj_desc.get('name')] = obj_desc
                        #found = True
                        
        # stop search if we've found enough objs
        if len(self.found_objs) >= userdata.max_objs:
            userdata.obj_found = True

            rospy.loginfo('call ros_mary')
            try:
                mary_resp = self.mary("Object located!")
            except rospy.ServiceException, e:
                rospy.logerr("Service call failed: %s" % e)
                

            obj_descs = []
            for key in self.found_objs:
                rospy.loginfo('Found obj: %s', self.found_objs[key])
                obj_descs.append(self.found_objs[key])

            userdata.obj_descs = obj_descs

            rospy.loginfo('searched poses: %s', self.count)
            return 'search_succeeded'

        # check for max_poses
        self.count += 1
        if self.count >  userdata.max_poses:
            # check whether we already found enough objects
            if len(self.found_objs) >= userdata.min_objs:
                userdata.obj_found = True

                obj_descs = []
                for key in self.found_objs:
                    rospy.loginfo('Found obj: %s', self.found_objs[key])
                    obj_descs.append(self.found_objs[key])
                    
                userdata.obj_descs = obj_descs

                rospy.loginfo('searched poses: %s', self.count - 1)

                # nevertheless set flag
                userdata.exceeded_max_poses = True
                
                return 'search_succeeded'
            
            userdata.exceeded_max_poses = True

            rospy.loginfo('searched poses: %s', self.count - 1)
            return 'search_aborted'

        # back to original position
        joint_state = JointState()
        joint_state.header.frame_id = 'tessdaf'
        joint_state.name = ['pan', 'tilt']
        joint_state.position = [float(0.0),float(0.0)]
        joint_state.velocity = [float(1.0),float(1.0)]
        joint_state.effort = [float(1.0),float(1.0)]
        self.ptu_cmd.publish(joint_state)

        
        return 'search_in_progress'



class PerceiveSim (smach.State):

    """
    Perceive the environemt.

    """

    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['succeeded', 'aborted', 'preempted'],
                             input_keys=['view_list'],
                             output_keys=['state','obj_list','cloud'])

        rospy.Subscriber("semcam", String, self.callback)
        self.ptu_cmd = rospy.Publisher('/ptu/cmd', JointState)
        
        self.obj_list = []
        self.active = False
        self.first_call = False

    def callback(self,data):
        if self.active == True and self.first_call == True:
            self.first_call = False
            obj_list = json.loads(data.data)
            if len(obj_list) == 0:
                rospy.loginfo("Nothing perceived")
            for obj_desc in obj_list:
                rospy.loginfo("Perceived: %s" % obj_desc.get('name'))

            for obj in obj_list:
                self.obj_list.append(obj)

    def execute(self, userdata):
        # rospy.loginfo('Executing state %s', self.__class__.__name__)
        # self.active = True
        # self.first_call = True

        # # wait for some time to read from the topic
        # # TODO: replace with a service call
        # rospy.sleep(3)
        # userdata.obj_list = self.obj_list

        # self.active = False
        # return 'succeeded'
        rospy.loginfo('Executing state %s', self.__class__.__name__)

        self.obj_list = []

        i = 1
        for view in userdata.view_list:

            if self.preempt_requested():
                self.service_preempt()
                return 'preempted'
            
            rospy.loginfo('%i view: set PTU to %s',i,view)

            joint_state = JointState()
            joint_state.header.frame_id = 'tessdaf'
            joint_state.name = ['pan', 'tilt']
            joint_state.position = [float(view[0]),float(view[1])]
            joint_state.velocity = [float(1.0),float(1.0)]
            joint_state.effort = [float(1.0),float(1.0)]
            
            self.ptu_cmd.publish(joint_state)
            
            # wait until PTU has finished or point cloud get screwed up
            rospy.sleep(2)
            
            rospy.loginfo('%i view: receive from semantic camera',i)
            self.active = True
            self.first_call = True

            # wait for some time to read once from the topic and store it onto self.pointcloud
            # TODO: replace with a service call

            userdata.state = 'taking_image'

            rospy.sleep(3)
            
            self.active = False

            userdata.cloud = []
            userdata.state = 'image_analysis'
            
            rospy.sleep(3)
            i = i + 1

            


        userdata.obj_list = self.obj_list

        # back to original position
        joint_state = JointState()
        joint_state.header.frame_id = 'tessdaf'
        joint_state.name = ['pan', 'tilt']
        joint_state.position = [float(0.0),float(0.0)]
        joint_state.velocity = [float(1.0),float(1.0)]
        joint_state.effort = [float(1.0),float(1.0)]
            
        self.ptu_cmd.publish(joint_state)

        return 'succeeded'



class PerceiveReal (smach.State):

    """
    Perceive the environemt.

    """

    def __init__(self):
        
        smach.State.__init__(self,
                             outcomes=['succeeded', 'aborted', 'preempted'],
                             input_keys=['view_list'],
                             output_keys=['state','obj_list','bbox','cloud','labels'])

        self.obj_list = []
        self.active = False
        self.first_call = False

        rospy.Subscriber("/head_xtion/depth/points", PointCloud2, self.callback)
        #rospy.Subscriber("/camera/depth_registered/points", PointCloud2, self.callback)

        

        self.ptu_cmd = rospy.Publisher('/ptu/cmd', JointState)

        self.cluster_vis = rospy.Publisher('/cluster_vis', PointCloud2)

        rospy.wait_for_service('/classifier_service/segment_and_classify')

        try:
            self.obj_rec = rospy.ServiceProxy('/classifier_service/segment_and_classify', segment_and_classify )
        except rospy.ServiceException, e:
            rospy.logerr("Service call failed: %s" % e)



    def callback(self,data):

        if self.active == True and self.first_call == True:
            self.first_call = False

            self.pointcloud = data
            rospy.loginfo("Got pointcloud!!!")
            
    

    def execute(self, userdata):
        rospy.loginfo('Executing state %s', self.__class__.__name__)

        self.obj_list = []

        i = 1
        for view in userdata.view_list:

            rospy.loginfo('%i view: set PTU to %s',i,view)

            joint_state = JointState()

            joint_state.header.frame_id = 'tessdaf'
            joint_state.name = ['pan', 'tilt']
            joint_state.position = [float(view[0]),float(view[1])]
            joint_state.velocity = [float(1.0),float(1.0)]
            joint_state.effort = [float(1.0),float(1.0)]
            
            self.ptu_cmd.publish(joint_state)
            
            # wait until PTU has finished or point cloud get screwed up
            rospy.sleep(2)
            
            rospy.loginfo('%i view: receive point cloud',i)
            self.active = True
            self.first_call = True

            userdata.state = 'taking_image'

            # wait for some time to read once from the topic and store it onto self.pointcloud
            # TODO: replace with a service call
            rospy.sleep(2)
            self.active = False

            rospy.loginfo('%i view: call object recognition service',i)
            try:
                obj_rec_resp = self.obj_rec(self.pointcloud)
            except rospy.ServiceException, e:
                rospy.logerr("Service call failed: %s" % e)

            objects = obj_rec_resp.class_results

            if len(objects) == 0:
                rospy.loginfo("%i view: nothing perceived",i)

                userdata.cloud = obj_rec_resp.cloud
                userdata.bbox = obj_rec_resp.bbox
                userdata.obj_list = self.obj_list

            else:
                rospy.loginfo('%i view: found objects: %i', i, len(objects))

                userdata.cloud = obj_rec_resp.cloud
                userdata.bbox = obj_rec_resp.bbox
                userdata.obj_list = self.obj_list

                #vis_cloud = obj_rec_resp.cloud[0]

                #self.cluster_vis.publish(vis_cloud)



                
                labels = []
                for j in range(len(objects)):

                    obj = objects[j]
                    

                    max_idx = obj.confidence.index(max(obj.confidence))

                    obj_desc = dict()

                    obj_desc['type'] = obj.class_type[max_idx].data.strip('/')

                    labels.append(obj_desc['type'])
                    
                    rospy.loginfo('Object: %s', obj_desc['type'])
                    
                    self.obj_list.append(obj_desc)

                userdata.labels = labels
                userdata.state = 'image_analysis'
                
                rospy.sleep(20)


            i = i + 1
            
        
        # back to original position
        #joint_state = JointState()
        #joint_state.header.frame_id = 'tessdaf'
        #joint_state.name = ['pan', 'tilt']
        #joint_state.position = [float(0.0),float(0.0)]
        #joint_state.velocity = [float(1.0),float(1.0)]
        #joint_state.effort = [float(1.0),float(1.0)]
        #self.ptu_cmd.publish(joint_state)

        return 'succeeded'


    
#______________________________________________________________________________
# main  

class Usage(Exception):
    def __init__(self, msg):
        self.msg = msg

def help_msg():
    return """
  Usage: search_agent.py [-h] <object> <method>

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

        print >>sys.stderr, args
        
        # create ros node
        rospy.init_node("search_agent")
        
        # create an agent
        agent = SearchAgent()

        # set userdata: object type, ...
        agent.set_sm_userdata()
        
        # run agent's state machine with or without introspection server
        # outcome = agent.execute_sm()
        outcome = agent.execute_sm_with_introspection()

        userdata = agent.get_sm_userdata()

        rospy.loginfo('obj desc: %s', userdata.sm_obj_desc)
        rospy.loginfo('obj found: %s ' % userdata.obj_found)
        rospy.loginfo('obj descs: %s ' % userdata.obj_descs)
#        rospy.loginfo('searched_poses: %s ' % len(userdata.searched_poses))
        rospy.loginfo('time (sec)): %s ' % userdata.time)
        rospy.loginfo('exceeded_max_time: %s ' % userdata.exceeded_max_time)
        rospy.loginfo('exceeded_max_poses: %s ' % userdata.exceeded_max_poses)

        
    except Usage, err:
        print >>sys.stderr, err.msg
        print >>sys.stderr, "for help use --help"
        return 2

if __name__ == "__main__":
    sys.exit(main())

