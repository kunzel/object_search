object_search
=============

Lightweight inference tools for hypothesizing object positions and planning viewpoints in 3D space. The object_search repositoy is a catkinized ROS package.

Prerequisites
-------------

* [Octomap (1.6)](http://octomap.github.io/)

Please refer to the octomap mailing list for installation instructions: ['How to use the latest version of octomap in ROS groovy?'](https://groups.google.com/forum/#!topic/octomap/BJ2QkZPxRCY)


Getting Started
---------------

1. Compile your catkin workspace:
      
        $ catkin_make

2. Open a terminal and run:

        $ roscd object_search/maps
      
        $ rosrun object_search supporting_plane_detector tum_kitchen_octomap.bt
      
        $ octovis sp_tum_kitchen_octomap.bt
