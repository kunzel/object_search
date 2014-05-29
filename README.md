object_search
=============

Lightweight inference tools for hypothesizing object positions and planning viewpoints in 3D space. The object_search repository is a catkinized ROS package.

Prerequisites
-------------

* [ROS](http://www.ros.org/wiki/ROS/Installation)
* [Octomap (1.6)](http://octomap.github.io/)

Please refer to the [OctoMap mailing list](https://groups.google.com/forum/#!forum/octomap) for installation instructions: ['How to use the latest version of octomap in ROS groovy?'](https://groups.google.com/forum/#!topic/octomap/BJ2QkZPxRCY)


Getting Started (in simulation)
-------------------------------

1. Compile your catkin workspace including the [object_search](https://github.com/kunzel/object_search) package:
      
        $ catkin_make

2. Open a terminal and run:

      Start the core:
      
        $ roscore
         
      Launch the simulator: (Please note: the object does not include any objects by default)
                                          
        $ roslaunch --wait strands_morse bham_cs_morse.launch env:=cs_lg_obj_search
        
        
      Add an object in the cs_lg_obj_search.py file as follows:
      
        cup = PassiveObject('strands_sim/robots/strands_objects.blend','cup')
        cup.properties(Object = True, Type = 'Cup')
        cup.translate(0,0,1.0)

      Launch the 2D robot navigation                 
        
        $ roslaunch --wait strands_morse bham_cs_nav2d.launch
        
      Launch the octomap server and build a map. Or use a pre-build map:
      
        $ rosrun octomap_server octomap_server_node `rospack find strands_morse`/bham/maps/cs_lg_obj_search_octomap.bt
        
      Launch the object search utils
      
        $ roslauch object_search object_search.launch
        
      Run the action server:
      
        $ rosrun object_search object_search_server.py
        
      Start the search using an action client where `<object>` is the name of the object the robot is looking for, e.g. Cup: 
      
        $ rosrun object_search object_search_client.py <object>
        
        
      Start RVIZ and subscribe to a `PoseArray` named `nav_goals` and a `MarkerArray` named `/supporting_planes_poses` to visualize the viewplanning result.
      
      
      
