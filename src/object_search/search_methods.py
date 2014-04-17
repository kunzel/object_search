#! /usr/bin/env python
import rospy
import smach
import smach_ros
import json
import math
import random

from nav_goals_msgs.srv import NavGoals
from nav_goals_msgs.srv import WeightedNavGoals

from view_evaluation.srv import BestViews
from view_evaluation.srv import BestViewsVisualiser

from geometry_msgs.msg import Point
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

        # visualizing nav goals in RVIZ
        self.pubmarker = rospy.Publisher('supporting_planes_poses', MarkerArray)
        self.marker_len = 0
            

    def execute(self, userdata):
        rospy.loginfo('Executing state %s', self.__class__.__name__)
        
        
        try:

            if len(self.agenda) == 0:
            
                nav_goals_resp = self.nav_goals(500, self.inflation_radius, self.polygon)


                nav_goals_eval_resp = self.nav_goals_eval(json.dumps(userdata.obj_desc),
                                                          json.dumps(userdata.obj_list),
                                                          nav_goals_resp.goals)

                self.delete_markers()            
                markerArray = MarkerArray()
            
                for i in range(0,len(nav_goals_eval_resp.sorted_goals.poses)):

                    self.agenda.append(nav_goals_eval_resp.sorted_goals.poses[i])
                    
                    if i < userdata.max_poses:
                        self.create_marker(markerArray,
                                           i,
                                           nav_goals_eval_resp.sorted_goals.poses[i],
                                           nav_goals_eval_resp.weights)


                self.marker_len =  len(markerArray.markers)
                self.pubmarker.publish(markerArray)
                



            if len(self.agenda) != 0:
                self.index += 1
                if self.index >= userdata.max_poses or self.index >= len(self.agenda):
                    return 'aborted'
                
                userdata.state = 'driving'
                userdata.view_list = [[0.0,0.5],[0.5,0.5],[-0.5,0.5]]
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
        marker1.color.r = r_func(weights[marker_id] / weights[0])
        marker1.color.g = g_func(weights[marker_id] / weights[0])
        marker1.color.b = b_func(weights[marker_id] / weights[0])

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

        
    
class InformedSearch_QSR (smach.State):
    pass

class InformedSearch_ViewEvaluation (smach.State):

    def __init__(self, inflation_radius, polygon):

        self.inflation_radius = inflation_radius
        self.polygon = polygon
        
        smach.State.__init__(self,
                             outcomes=['succeeded', 'aborted', 'preempted'],
                             input_keys=['obj_desc','obj_list'],
                             output_keys=['pose_output','view_list'])

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

        # visualizing nav goals in RVIZ
        self.pubmarker = rospy.Publisher('ignore', MarkerArray)
        
        self.pubviewcones = rospy.Publisher('best_views_cones', MarkerArray)
        self.pubviewposes = rospy.Publisher('best_views_poses', MarkerArray)

        self.marker_len = 0
            

    def execute(self, userdata):
        rospy.loginfo('Executing state %s', self.__class__.__name__)
        try:
            nav_goals_resp = self.nav_goals(50, self.inflation_radius, self.polygon)

            i = 0
            pan = [0.0,0.5,-0.5, 0.0,0.5,-0.5, 0.0,0.5,-0.5]
            tilt = [0.25,0.25,0.25, 0.5,0.5,0.5 , 0.0,0.0,0.0]
            # for view in view_list:
            #     pan.append(float(view[0]))
            #     tilt.append(float(view[1]))
            
            best_views_resp = self.best_views(nav_goals_resp.goals,
                                                  pan, tilt, 1)

            best_views_vis_resp = self.best_views_vis(nav_goals_resp.goals,
                                                      pan, tilt, 1)

            self.delete_markers()            
            markerArray = MarkerArray()
            
            for i in range(0,len(best_views_resp.bestPoses.poses)):
                self.create_marker(markerArray,
                                   i,
                                   best_views_resp.bestPoses.poses[i],
                                   best_views_resp.bestPosesWeights)


            self.marker_len =  len(markerArray.markers)
            self.pubmarker.publish(markerArray)

            self.pubviewcones.publish(best_views_vis_resp.cones3D)
            self.pubviewposes.publish(best_views_vis_resp.cones2D)
                                                      
            rospy.loginfo("Best pose: (%s,%s)", best_views_resp.bestPoses.poses[0].position.x,best_views_resp.bestPoses.poses[0].position.y )
            
            userdata.pose_output = best_views_resp.bestPoses.poses[0]

            view_lst = []
            for i in range(0,len(best_views_resp.pan_sorted)/2):
                view_lst.append([best_views_resp.pan_sorted[i],best_views_resp.tilt_sorted[i]])
                
            userdata.view_list = view_lst #[[best_views_resp.pan_sorted[0],best_views_resp.tilt_sorted[0]]] #[[0.0,0.5],[0.5,0.5],[-0.5,0.5]]

            
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
        marker1.color.r = r_func(weights[marker_id] / (weights[0]+1))
        marker1.color.g = g_func(weights[marker_id] / (weights[0]+1))
        marker1.color.b = b_func(weights[marker_id] / (weights[0]+1))

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


