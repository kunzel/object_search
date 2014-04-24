#! /usr/bin/env python
import rospy
import smach
import smach_ros
import json
import math
import random
import copy

from nav_goals_msgs.srv import NavGoals
from nav_goals_msgs.srv import WeightedNavGoals

from view_evaluation.srv import BestViews
from view_evaluation.srv import BestViewsVisualiser

from geometry_msgs.msg import Point
from geometry_msgs.msg import PoseWithCovarianceStamped

from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray

class UninformedSearch_Random (smach.State):

    """
    Select next search pose

    """
    
    def __init__(self, inflation_radius, polygon):

        self.inflation_radius = inflation_radius
        self.polygon = polygon
        
        smach.State.__init__(self,
                             outcomes=['succeeded', 'aborted', 'preempted'],
                             output_keys=['pose_output','view_list'])

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
            userdata.view_list = [[0.0,0.5],[0.5,0.5],[-0.5,0.5]]
            
        except rospy.ServiceException, e:
            rospy.logerr("Service call failed: %s" % e)
            return 'aborted'
               
        return 'succeeded'


class InformedSearch_SupportingPlanes (smach.State):

    def __init__(self, inflation_radius, polygon):

        self.inflation_radius = inflation_radius
        self.polygon = polygon

        self.num_of_nav_goals =    int(rospy.get_param('num_of_nav_goals', '500'))
        self.num_of_trajectories = int(rospy.get_param('num_of_trajectories', '500'))
        
        self.agenda = []
        self.index = -1
        
        smach.State.__init__(self,
                             outcomes=['succeeded', 'aborted', 'preempted'],
                             input_keys=['obj_desc','obj_list','max_poses'],
                             output_keys=['state','pose_output','view_list'])

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

        # get current robot pose
        self.first_call = True
        rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, self.amcl_cb)

        # visualizing nav goals in RVIZ
        self.pubmarker = rospy.Publisher('supporting_planes_poses', MarkerArray)
        self.marker_len = 0

    def amcl_cb(self, data):

        if self.first_call == True:
            self.current_pose = data.pose.pose
            rospy.loginfo("Got current robot pose: (%f,%f)" % (self.current_pose.position.x, self.current_pose.position.y))
            self.first_call = False
        
    def get_qsr_type(self, v4r_type):

        if v4r_type == 'mug':
            return 'Cup'
        elif v4r_type == 'bottle':
            return 'Bottle'
        elif v4r_type == 'banana':
            return 'Banana'
        elif v4r_type == 'notebook':
            return 'Laptop'
        elif v4r_type == 'keyboard':
            return 'Keyboard'
        else:
            return 'None'
        
                
            
    def execute(self, userdata):
        rospy.loginfo('Executing state %s', self.__class__.__name__)
        
        
        try:

            if len(self.agenda) == 0:
            
                nav_goals_resp = self.nav_goals(self.num_of_nav_goals, self.inflation_radius, self.polygon)

                qsr_obj_type = self.get_qsr_type(userdata.obj_desc['type'])

                nav_goals_eval_resp = self.nav_goals_eval(qsr_obj_type,
                                                          json.dumps(userdata.obj_list),
                                                          nav_goals_resp.goals)

                
                viewpoints = create_viewpoints(nav_goals_eval_resp.sorted_goals.poses,
                                               nav_goals_eval_resp.weights)

    
                vp_trajectory = plan_views(self.current_pose, viewpoints, self.num_of_trajectories, userdata.max_poses)

                vp_trajectory_weights = get_weights(vp_trajectory)

                self.delete_markers()            
                markerArray = MarkerArray()
            
#                for i in range(0,len(nav_goals_eval_resp.sorted_goals.poses)):

#                    self.agenda.append(nav_goals_eval_resp.sorted_goals.poses[i])

                for i in range(0,len(vp_trajectory)):

                    self.agenda.append(vp_trajectory[i].get_pose())
                    if i < userdata.max_poses:
                        self.create_marker(markerArray,
                                           i,
                                           vp_trajectory[i].get_pose(),
                                           #nav_goals_eval_resp.sorted_goals.poses[i],
                                           #nav_goals_eval_resp.weights,
                                           vp_trajectory_weights)


                self.marker_len =  len(markerArray.markers)
                self.pubmarker.publish(markerArray)
                



            if len(self.agenda) != 0:
                self.index += 1
                if self.index >= userdata.max_poses or self.index >= len(self.agenda):
                    return 'aborted'
                
                userdata.state = 'driving'

                userdata.view_list = [[0.0,0.5]] #,[0.5,0.5],[-0.5,0.5]]

                userdata.pose_output = self.agenda[self.index]
                rospy.loginfo("Next pose [%i]: (%s,%s)", self.index, self.agenda[self.index].position.x,self.agenda[self.index].position.y )

            
        except rospy.ServiceException, e:
            rospy.logerr("Service call failed: %s" % e)
            return 'aborted'
               
        return 'succeeded'


    def create_marker(self,markerArray, marker_id, pose, weights):
        marker1 = Marker()
        marker1.id = marker_id 
        marker1.header.frame_id = "/map"
        marker1.type = marker1.TRIANGLE_LIST
        marker1.action = marker1.ADD
        marker1.scale.x = 1
        marker1.scale.y = 1
        marker1.scale.z = 2
        marker1.color.a = 0.25

        max_idx = weights.index(max(weights))
        min_idx = weights.index(min(weights))
        
        marker1.color.r = r_func((weights[marker_id] - weights[min_idx]) / (weights[max_idx]- weights[min_idx] +1))
        marker1.color.g = g_func((weights[marker_id] - weights[min_idx]) / (weights[max_idx]- weights[min_idx] +1))
        marker1.color.b = b_func((weights[marker_id] - weights[min_idx]) / (weights[max_idx]- weights[min_idx] +1))

        #rospy.loginfo("weight: %s max: %s ratio: %s",weights[marker_id], weights[0], weights[marker_id] / weights[0])
        #rospy.loginfo("ID: %s RGB: %s %s %s", marker_id, marker1.color.r, marker1.color.g, marker1.color.b)

        marker1.pose.orientation = pose.orientation
        marker1.pose.position = pose.position
        marker1.points = [Point(0,0,0.01),Point(3,-1,0.01),Point(3,1,0.01)]
        
        markerArray.markers.append(marker1)


    def delete_markers(self):
        markerArray = MarkerArray()
        for i in range(0,self.marker_len):
            marker = Marker()
            marker.header.frame_id = "/map"
            marker.id = i
            marker.action = marker.DELETE
            markerArray.markers.append(marker)
        self.pubmarker.publish(markerArray)


class Viewpoint():

    def __init__(self, pose, weight, prob):

        self.pose = pose
        self.weight = weight
        self.prob = prob

    def get_pose(self):

        return self.pose
    
    def get_weight(self):

        return self.weight

    def get_prob(self):

        return self.prob
    

def plan_views(current_pose, viewpoints, traj_num=100, traj_len=10):

    costs = calc_costs(viewpoints)

    traj = sample_trajectories(viewpoints, traj_num, traj_len)
    
    traj_costs = evalaute_trajectories(current_pose, traj, costs)

    min_traj = get_min_traj(traj_costs, traj)

    return min_traj 

def get_weights(viewpoints):

    weights = []
    
    for vp in viewpoints:
        weights.append(vp.get_weight())
        
    return weights

def create_viewpoints(pose, weight):

    viewpoints = []

    wsum = sum(weight)
    
    for i in range(len(pose)):
        vp = Viewpoint(pose[i],weight[i],float(weight[i])/float(wsum))
        viewpoints.append(vp)
        
    return viewpoints


def calc_costs(viewpoints):

    costs = dict() 

    for i in range(len(viewpoints)):

        costs2 = dict()

        for j in range(len(viewpoints)):

            if i != j:
                p1 = viewpoints[i].get_pose()
                p2 = viewpoints[j].get_pose()

                dist = math.sqrt( (p1.position.x - p2.position.x)**2 +  (p1.position.y - p2.position.y)**2 )   
                
                costs2[viewpoints[j]] = dist

        costs[viewpoints[i]] = costs2
    
    return costs

def sample_trajectories(viewpoints, number, length):

    pop0 = []
    for i in range(len(viewpoints)):
        #if i < length:
        pop0.extend([i for x in range(int(viewpoints[i].get_weight()) - 1 ) ])
        #else:
         #   break

    if len(pop0) < length:
        return []
    
    traj_lst = []
        
    for i in range(number):

        popn = copy.deepcopy(pop0)
        
        traj = []
        for j in range(length):

            idx = random.sample(popn,1)
            popn = [x for x in popn if x != idx[0]]

            traj.append(viewpoints[idx[0]])

        traj_lst.append(traj)
            
    return traj_lst

def evalaute_trajectories(current_pose, trajectories, costs):

    traj_costs = []
    
    for traj in trajectories:
        c = 0

        p1 = current_pose 
        p2 = traj[0].get_pose()

        dist = math.sqrt( (p1.position.x - p2.position.x)**2 +  (p1.position.y - p2.position.y)**2 )   
        c += dist

        prob = 1
        
        for i in range(len(traj)-1):

            prob -= traj[i].get_prob()

            c += costs[traj[i]][traj[i+1]] * prob
            
        traj_costs.append(c)

    return traj_costs


def get_min_traj(traj_costs, trajectories):

    if len(trajectories) == 0:
        return []
    
    min_idx = traj_costs.index(min(traj_costs))

    return trajectories[min_idx]


def trapezoidal_shaped_func(a, b, c, d, x):
  min_val = min(min((x - a)/(b - a), float(1.0)), (d - x)/(d - c))
  return max(min_val, float(0.0))


def r_func(x):
    a = -0.125
    b =  0.125
    c =  0.375
    d =  0.625

    x = 1.0 - x
  
    value = trapezoidal_shaped_func(a,b,c,d,x)
    return value

def g_func(x):
    a =  0.125
    b =  0.375
    c =  0.625
    d =  0.875
    
    x = 1.0 - x
    
    value = trapezoidal_shaped_func(a,b,c,d,x)
    return value


def b_func(x):
    a =  0.375
    b =  0.625
    c =  0.875
    d =  1.125
  
    x = 1.0 - x
  
    value = trapezoidal_shaped_func(a,b,c,d,x)
    return value

        
    
class InformedSearch_ViewEvaluation (smach.State):

    def __init__(self, inflation_radius, polygon):

        self.inflation_radius = inflation_radius
        self.polygon = polygon

        self.num_of_nav_goals =    int(rospy.get_param('num_of_nav_goals', '500'))
        self.num_of_trajectories = int(rospy.get_param('num_of_trajectories', '500'))

        self.agenda = []
        self.index = -1
        
        smach.State.__init__(self,
                             outcomes=['succeeded', 'aborted', 'preempted'],
                             input_keys=['obj_desc','obj_list','max_poses'],
                             output_keys=['state','pose_output','view_list'])

        rospy.wait_for_service('nav_goals')
        try:
            self.nav_goals = rospy.ServiceProxy('nav_goals', NavGoals)
        except rospy.ServiceException, e:
            rospy.logerr("Service call failed: %s" % e)

        rospy.wait_for_service('bestViews')
        try:
            self.best_views = rospy.ServiceProxy('bestViews', BestViews)
        except rospy.ServiceException, e:
            rospy.logerr("Service call failed: %s" % e)

        rospy.wait_for_service('bestViewsVisualiser')
        try:
            self.best_views_vis = rospy.ServiceProxy('bestViewsVisualiser', BestViewsVisualiser)
        except rospy.ServiceException, e:
            rospy.logerr("Service call failed: %s" % e)

        # get current robot pose
        self.first_call = True
        rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, self.amcl_cb)
            
        # visualizing nav goals in RVIZ
        self.pubmarker = rospy.Publisher('ignore', MarkerArray)
        
        self.pubviewcones = rospy.Publisher('best_views_cones', MarkerArray)
        self.pubviewposes = rospy.Publisher('best_views_poses', MarkerArray)

        self.marker_len = 0

    def amcl_cb(self, data):
            
        if self.first_call == True:
            self.current_pose = data.pose.pose
            rospy.loginfo("Got current robot pose: (%f,%f)" % (self.current_pose.position.x, self.current_pose.position.y))
            self.first_call = False
            

    def execute(self, userdata):

        rospy.loginfo('Executing state %s', self.__class__.__name__)
        try:

            if len(self.agenda) == 0:

                nav_goals_resp = self.nav_goals(self.num_of_nav_goals, self.inflation_radius, self.polygon)

                i = 0
                #pan = [0.0,0.5,-0.5, 0.0,0.5,-0.5, 0.0,0.5,-0.5]
                #tilt = [0.25,0.25,0.25, 0.5,0.5,0.5 , 0.0,0.0,0.0]
                pan =  [0.0, 0.0]
                tilt = [0.25, 0.5]

            
                best_views_resp = self.best_views(nav_goals_resp.goals,
                                                  pan, tilt, userdata.max_poses)

                best_views_vis_resp = self.best_views_vis(nav_goals_resp.goals,
                                                          pan, tilt, userdata.max_poses)

                
                viewpoints = create_viewpoints(best_views_resp.bestPoses.poses,
                                               best_views_resp.bestPosesWeights)

                print "len(vp):", len(viewpoints)
                print best_views_resp.bestPosesWeights
                
                vp_trajectory = plan_views(self.current_pose,
                                           viewpoints,
                                           self.num_of_trajectories,
                                           userdata.max_poses)

                vp_trajectory_weights = get_weights(vp_trajectory)


                self.delete_markers()            
                markerArray = MarkerArray()


                for i in range(0,len(vp_trajectory)):

                    self.agenda.append(vp_trajectory[i].get_pose())
                    if i < userdata.max_poses:
                        self.create_marker(markerArray,
                                           i,
                                           vp_trajectory[i].get_pose(),
                                           vp_trajectory_weights)

                
                # for i in range(0,len(best_views_resp.bestPoses.poses)):
                #     self.create_marker(markerArray,
                #                        i,
                #                        best_views_resp.bestPoses.poses[i],
                #                        best_views_resp.bestPosesWeights)


                self.marker_len =  len(markerArray.markers)
                self.pubmarker.publish(markerArray)

                #self.pubviewcones.publish(best_views_vis_resp.cones3D)
                #self.pubviewposes.publish(best_views_vis_resp.cones2D)
                                                      
                rospy.loginfo("Best pose: (%s,%s)", best_views_resp.bestPoses.poses[0].position.x,best_views_resp.bestPoses.poses[0].position.y )


            if len(self.agenda) != 0:
                self.index += 1
                if self.index >= userdata.max_poses or self.index >= len(self.agenda):
                    return 'aborted'
                
                userdata.state = 'driving'

                userdata.view_list = [[0.0,0.5]] #,[0.5,0.5],[-0.5,0.5]]

                userdata.pose_output = self.agenda[self.index]
                rospy.loginfo("Next pose [%i]: (%s,%s)", self.index, self.agenda[self.index].position.x,self.agenda[self.index].position.y )

                
                # view_lst = []
                # for i in range(0,len(best_views_resp.pan_sorted)/2):
                #     view_lst.append([best_views_resp.pan_sorted[i],best_views_resp.tilt_sorted[i]])

                # userdata.view_list = view_lst #[[best_views_resp.pan_sorted[0],best_views_resp.tilt_sorted[0]]]

            
        except rospy.ServiceException, e:
            rospy.logerr("Service call failed: %s" % e)
            return 'aborted'
               
        return 'succeeded'


    def create_marker(self,markerArray, marker_id, pose, weights):
        marker1 = Marker()
        marker1.id = marker_id 
        marker1.header.frame_id = "/map"
        marker1.type = marker1.TRIANGLE_LIST
        marker1.action = marker1.ADD
        marker1.scale.x = 1
        marker1.scale.y = 1
        marker1.scale.z = 2
        marker1.color.a = 0.25

        max_idx = weights.index(max(weights))
        min_idx = weights.index(min(weights))
        
        marker1.color.r = r_func((weights[marker_id] - weights[min_idx]) / (weights[max_idx]- weights[min_idx] +1))
        marker1.color.g = g_func((weights[marker_id] - weights[min_idx]) / (weights[max_idx]- weights[min_idx] +1))
        marker1.color.b = b_func((weights[marker_id] - weights[min_idx]) / (weights[max_idx]- weights[min_idx] +1))

        marker1.pose.orientation = pose.orientation
        marker1.pose.position = pose.position
        marker1.points = [Point(0,0,0.01),Point(3,-1,0.01),Point(3,1,0.01)]
        
        markerArray.markers.append(marker1)

    
    # def create_marker(self,markerArray, marker_id, pose, weights):
    #     marker1 = Marker()
    #     marker1.id = marker_id 
    #     marker1.header.frame_id = "/map"
    #     marker1.type = marker1.TRIANGLE_LIST
    #     marker1.action = marker1.ADD
    #     marker1.scale.x = 1
    #     marker1.scale.y = 1
    #     marker1.scale.z = 2
    #     marker1.color.a = 0.25
    #     marker1.color.r = r_func(weights[marker_id] / (weights[0]+1))
    #     marker1.color.g = g_func(weights[marker_id] / (weights[0]+1))
    #     marker1.color.b = b_func(weights[marker_id] / (weights[0]+1))

    #     #rospy.loginfo("weight: %s max: %s ratio: %s",weights[marker_id], weights[0], weights[marker_id] / weights[0])
    #     #rospy.loginfo("ID: %s RGB: %s %s %s", marker_id, marker1.color.r, marker1.color.g, marker1.color.b)

    #     marker1.pose.orientation = pose.orientation
    #     marker1.pose.position = pose.position
    #     marker1.points = [Point(0,0,0.01),Point(3,-1,0.01),Point(3,1,0.01)]
        
    #     markerArray.markers.append(marker1)


    def delete_markers(self):
        markerArray = MarkerArray()
        for i in range(0,self.marker_len):
            marker = Marker()
            marker.header.frame_id = "/map"
            marker.id = i
            marker.action = marker.DELETE
            markerArray.markers.append(marker)
        self.pubmarker.publish(markerArray)


