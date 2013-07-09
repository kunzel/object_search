object_search
=============

Lightweight inference tools for hypothesizing object positions and planning viewpoints in 3D space. The object_search repository is a catkinized ROS package.

Prerequisites
-------------

* [ROS](http://www.ros.org/wiki/ROS/Installation)
* [Octomap (1.6)](http://octomap.github.io/)

Please refer to the [OctoMap mailing list](https://groups.google.com/forum/#!forum/octomap) for installation instructions: ['How to use the latest version of octomap in ROS groovy?'](https://groups.google.com/forum/#!topic/octomap/BJ2QkZPxRCY)


Getting Started
---------------

1. Compile your catkin workspace including the [object_search](https://github.com/kunzel/object_search) package:
      
        $ catkin_make

2. Open a terminal and run:

        $ roscd object_search/maps
      
        $ rosrun object_search supporting_plane_detector tum_kitchen_octomap.bt
      
        $ octovis sp_tum_kitchen_octomap.bt
