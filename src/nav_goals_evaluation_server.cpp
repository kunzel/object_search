#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/PoseArray.h"
#include "nav_goals_msgs/WeightedNavGoals.h"

#include "octomap_msgs/GetOctomap.h"
using octomap_msgs::GetOctomap;

#include <octomap_msgs/conversions.h>
#include <octomap/octomap.h>
using namespace octomap;

OcTree* retrieve_octree()
{
  ros::NodeHandle n;
  std::string servname = "octomap_binary";
  ROS_INFO("Requesting the map from %s...", n.resolveName(servname).c_str());
  GetOctomap::Request req;
  GetOctomap::Response resp;
  while(n.ok() && !ros::service::call(servname, req, resp))
    {
      ROS_WARN("Request to %s failed; trying again...", n.resolveName(servname).c_str());
      usleep(1000000);
    }
 
  OcTree* octree = octomap_msgs::binaryMsgToMap(resp.map);
  // AbstractOccupancyOcTree* octree = NULL;
  // if (tree){
  //   octree = dynamic_cast<AbstractOccupancyOcTree*>(tree);
  // }

  if (octree){
    ROS_INFO("Map received (%zu nodes, %f m res)", octree->size(), octree->getResolution());
    return octree;
  }
  return NULL;
}

bool evaluate(nav_goals_msgs::WeightedNavGoals::Request  &req,
              nav_goals_msgs::WeightedNavGoals::Response &res)
{
  static bool first_call = true;
  static OcTree* octree = NULL;
 
  ROS_INFO("Started evaluation");
  ROS_INFO("request: obj_desc: %s, obj_list: %s", req.obj_desc.c_str(),  req.obj_list.c_str());

  // Get Octomap (only once)
  if (first_call)
    {
      first_call = false;
      
      octree = retrieve_octree();

      // ros::NodeHandle n;
      // ros::ServiceClient client = n.serviceClient<octomap_msgs::GetOctomap>("octomap_binary");
      // octomap_msgs::GetOctomap srv;

      // if (client.call(srv))
      //   {
      //     ROS_INFO("ID: %s", srv.response.map.id.c_str());
      //   }
      // else
      //   {
      //     ROS_ERROR("Failed to call service octomap_binary");
      //     return 1;
      //   }
    }
  
  // Iterate through Octomap and add weights to the poses
  // For all poses check whether is withing triangle


  // Sort weights and poses


  // Visualize result (colored triangles)

  // Send responce
  res.sorted_goals = (geometry_msgs::PoseArray) req.goals;
  
  int count = 0;
  for( std::vector<geometry_msgs::Pose>::const_iterator it = req.goals.poses.begin(); 
       it!=req.goals.poses.end(); ++it) 
    {
      ROS_INFO("%i", count++);
    }

 
  ROS_INFO("Finished evaluation. Sending back response ...");
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "nav_goals_evaluation_server");
  ros::NodeHandle n;
  ros::ServiceServer service = n.advertiseService("nav_goals_evaluation", evaluate);

  ROS_INFO("Started nav_goals_evaluation service");
  ros::spin();
  ROS_INFO("Stopped nav_goals_evaluation service");

  return 0;
}
