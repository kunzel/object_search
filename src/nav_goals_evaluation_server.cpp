#include "ros/ros.h"
#include <tf/transform_datatypes.h>
#include "std_msgs/String.h"
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/Quaternion.h"
#include "nav_goals_msgs/WeightedNavGoals.h"
#include "visualization_msgs/Marker.h"
#include <math.h> 

#include <tf/transform_datatypes.h>
#include <tf/LinearMath/Matrix3x3.h>
#include <tf/LinearMath/Quaternion.h>
#include <math.h>

#include "octomap_msgs/GetOctomap.h"
using octomap_msgs::GetOctomap;

#include <octomap_msgs/conversions.h>
#include <octomap/octomap.h>
using namespace octomap;

#include "qsr_msgs/QSRToGMM.h"
#include "qsr_msgs/Gaussian2D.h"

#include <stdio.h>
#include <stdlib.h>
#include <math.h>

using namespace qsr_msgs;

#define ANGLE_MAX_DIFF (M_PI / 8)  


int get_GMMs(std::string object, std::vector<std::string> qsr_landmark_type, std::vector< std::vector<float> > &landmark_gmm)
{
  ros::NodeHandle n;
  std::string servname = "qsr_to_gmm";
  ROS_INFO("Requesting GMMs from %s...", n.resolveName(servname).c_str());

  for( std::vector<std::string>::iterator it = qsr_landmark_type.begin(); 
       it!=qsr_landmark_type.end(); ++it)

    {

      QSRToGMM::Request req;
      req.object = object;
      req.landmark = *it;
      QSRToGMM::Response res;
      
      if (ros::service::call(servname, req, res))
        {
          std::vector<float> gmm_param;
          ROS_INFO("Got GMMs");
          for(int i = 0; i != res.weight.size(); i++) 
            {
              
              ROS_INFO("weight: %f", res.weight[i]);
              gmm_param.push_back(res.weight[i]);
              gmm_param.push_back(res.gaussian[i].mean[0]);
              gmm_param.push_back(res.gaussian[i].mean[1]);
              gmm_param.push_back(res.gaussian[i].covariance[0]);
              gmm_param.push_back(res.gaussian[i].covariance[1]);
              gmm_param.push_back(res.gaussian[i].covariance[2]);
              gmm_param.push_back(res.gaussian[i].covariance[3]);
            }

          landmark_gmm.push_back(gmm_param);
        }


      else
        {
          ROS_ERROR("Failed to call service qsr_to_gmm");
          return 1;
        }
    }

  return 0;
}


int get_landmarks(std::vector<geometry_msgs::Pose> &qsr_landmark, std::vector<std::string> &qsr_landmark_type)
{

  static ros::NodeHandle n;
  static ros::Publisher landmark_pub = n.advertise<geometry_msgs::PoseArray>( "landmarks", 0 );
  geometry_msgs::PoseArray landmark_pose_array;

  landmark_pose_array.header.frame_id = "map";
  
  ros::NodeHandle nh;
  XmlRpc::XmlRpcValue lst;

  int i = 0; 
  
  std::ostringstream os;
  os << "/qsr_landmark/id" << i << "/pose";
  std::string landmark = os.str();

  ROS_INFO("Trying to get landmark: %s", landmark.c_str());
  
  while (nh.getParam(landmark.c_str(), lst))
    {
      ROS_INFO("Landmark id%i:", i);
      //ROS_ASSERT(lst.getType() == XmlRpc::XmlRpcValue::TypeArray);
      geometry_msgs::Pose p;

      p.position.x = static_cast<double>(lst[0]);
      p.position.y = static_cast<double>(lst[1]);
      p.position.z = static_cast<double>(lst[2]);
      p.orientation.w = static_cast<double>(lst[3]);
      p.orientation.x = static_cast<double>(lst[4]);
      p.orientation.y = static_cast<double>(lst[5]);
      p.orientation.z = static_cast<double>(lst[6]);

      ROS_INFO("Got landmark pose: (%f %f %f)", p.position.x, p.position.y, p.position.z);
      qsr_landmark.push_back(p);
      landmark_pose_array.poses.push_back(p);

      std::ostringstream os_type;
      os_type << "/qsr_landmark/id" << i << "/type";
      std::string landmark_type = os_type.str();
      std::string lm_type;
      if (nh.getParam(landmark_type, lm_type))
        {
          ROS_INFO("Got landmark type: %s", lm_type.c_str());
          qsr_landmark_type.push_back(lm_type);
        }
      else
        {
          ROS_ERROR("Could not get landmark type!");
          qsr_landmark_type.push_back("");
        }
      
      std::ostringstream os;
      os << "/qsr_landmark/id" << ++i << "/pose";
      landmark = os.str();
      ROS_INFO("Trying to get landmark: %s", landmark.c_str());
    }

  landmark_pub.publish( landmark_pose_array );

  ROS_INFO("Failed to get landmark: %s", landmark.c_str());
  ROS_INFO("Number of landmarks found: %i", i);
  
  return 0;
}


struct WeightComparator
{
  const std::vector<float> & value_vector;

  WeightComparator(const std::vector<float> & val_vec):
    value_vector(val_vec) {}
  
  bool operator()(float i1, float i2)
    {
      return value_vector[i1] > value_vector[i2];
    }
};


typedef struct { double x, y; } vec;
typedef struct { int n; vec* v; } polygon_t, *polygon;
 
#define BIN_V(op, xx, yy) vec v##op(vec a,vec b){vec c;c.x=xx;c.y=yy;return c;}
#define BIN_S(op, r) double v##op(vec a, vec b){ return r; }
BIN_V(sub, a.x - b.x, a.y - b.y);
BIN_V(add, a.x + b.x, a.y + b.y);
BIN_S(dot, a.x * b.x + a.y * b.y);
BIN_S(cross, a.x * b.y - a.y * b.x);
 
/* return a + s * b */
vec vmadd(vec a, double s, vec b)
{
	vec c;
	c.x = a.x + s * b.x;
	c.y = a.y + s * b.y;
	return c;
}
 
/* check if x0->x1 edge crosses y0->y1 edge. dx = x1 - x0, dy = y1 - y0, then
   solve  x0 + a * dx == y0 + b * dy with a, b in real
   cross both sides with dx, then: (remember, cross product is a scalar)
	x0 X dx = y0 X dx + b * (dy X dx)
   similarly,
	x0 X dy + a * (dx X dy) == y0 X dy
   there is an intersection iff 0 <= a <= 1 and 0 <= b <= 1
 
   returns: 1 for intersect, -1 for not, 0 for hard to say (if the intersect
   point is too close to y0 or y1)
*/
int intersect(vec x0, vec x1, vec y0, vec y1, double tol, vec *sect)
{
	vec dx = vsub(x1, x0), dy = vsub(y1, y0);
	double d = vcross(dy, dx), a;
	if (!d) return 0; /* edges are parallel */
 
	a = (vcross(x0, dx) - vcross(y0, dx)) / d;
	if (sect)
		*sect = vmadd(y0, a, dy);
 
	if (a < -tol || a > 1 + tol) return -1;
	if (a < tol || a > 1 - tol) return 0;
 
	a = (vcross(x0, dy) - vcross(y0, dy)) / d;
	if (a < 0 || a > 1) return -1;
 
	return 1;
}
 
/* distance between x and nearest point on y0->y1 segment.  if the point
   lies outside the segment, returns infinity */
double dist(vec x, vec y0, vec y1, double tol)
{
	vec dy = vsub(y1, y0);
	vec x1, s;
	int r;
 
	x1.x = x.x + dy.y; x1.y = x.y - dy.x;
	r = intersect(x, x1, y0, y1, tol, &s);
	if (r == -1) return HUGE_VAL;
	s = vsub(s, x);
	return sqrt(vdot(s, s));
}
 
#define for_v(i, z, p) for(i = 0, z = p->v; i < p->n; i++, z++)
/* returns 1 for inside, -1 for outside, 0 for on edge */
int inside(vec v, polygon p, double tol)
{
	/* should assert p->n > 1 */
	int i, k, crosses;
	vec *pv;
	double min_x, max_x, min_y, max_y;
 
	for (i = 0; i < p->n; i++) {
		k = i + 1 % p->n;
		min_x = dist(v, p->v[i], p->v[k], tol);
		if (min_x < tol) return 0;
	}
 
	min_x = max_x = p->v[0].x;
	min_y = max_y = p->v[1].y;
 
	/* calculate extent of polygon */
	for_v(i, pv, p) {
		if (pv->x > max_x) max_x = pv->x;
		if (pv->x < min_x) min_x = pv->x;
		if (pv->y > max_y) max_y = pv->y;
		if (pv->y < min_y) min_y = pv->y;
	}
	if (v.x < min_x || v.x > max_x || v.y < min_y || v.y > max_y)
		return -1;
 
	max_x -= min_x; max_x *= 2;
	max_y -= min_y; max_y *= 2;
	max_x += max_y;
 
	vec e;
	while (1) {
		crosses = 0;
		/* pick a rand point far enough to be outside polygon */
		e.x = v.x + (1 + rand() / (RAND_MAX + 1.)) * max_x;
		e.y = v.y + (1 + rand() / (RAND_MAX + 1.)) * max_x;
 
		for (i = 0; i < p->n; i++) {
			k = (i + 1) % p->n;
			k = intersect(v, e, p->v[i], p->v[k], tol, 0);
 
			/* picked a bad point, ray got too close to vertex.
			   re-pick */
			if (!k) break;
 
			if (k == 1) crosses++;
		}
		if (i == p->n) break;
	}
	return (crosses & 1) ? 1 : -1;
}

double normal_dist_2d(double x, double y, double mean1, double var1, double mean2, double var2)
  {
    return (1.0 / (2.0 * M_PI * sqrt(var1) * sqrt(var2)) * exp(-1 * ( pow(x - mean1,2) / ( 2.0 * var1) +  pow(y - mean2,2) / ( 2.0 * var2) )));
  }

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

  if (octree){
    ROS_INFO("Map received (%zu nodes, %f m res)", octree->size(), octree->getResolution());
    return octree;
  }
  return NULL;
}

OcTree* extract_supporting_planes(OcTree* tree)
{
  OcTree* sp_tree = new OcTree(tree->getResolution());  

  int free = 0;
  int occupied = 0;
  int supported = 0;
  
  ROS_INFO("Extracting supporting planes from octomap");
  
  for(OcTree::leaf_iterator it = tree->begin_leafs(),
        end=tree->end_leafs(); it!= end; ++it)
    {
      if (tree->isNodeOccupied(*it))
        {
          occupied++;
          std::vector<point3d> normals;
          
          point3d p3d = it.getCoordinate();

          bool got_normals = tree->getNormals(p3d ,normals, true); 
          std::vector<point3d>::iterator normal_iter;
          
          point3d avg_normal (0.0, 0.0, 0.0);
          for(std::vector<point3d>::iterator normal_iter = normals.begin(), 
                end = normals.end(); normal_iter!= end; ++normal_iter)
            {
              avg_normal+= (*normal_iter);
            }
          if (normals.size() > 0) 
            {
              supported++;
              // cout << "#Normals: " << normals.size() << endl;

              avg_normal/= normals.size();       
              
              point3d z_axis ( 0.0, 0.0, 1.0);
              double angle = avg_normal.angleTo(z_axis);

              point3d coord = it.getCoordinate();

              if ( angle < ANGLE_MAX_DIFF)
                {
                  sp_tree->updateNode(coord,true);
                } 
            }  
        } 
      else 
        {
          free++;
        }
    }
  ROS_INFO("Extracted map size: %i (%i free, and %i occupied leaf nodes were discarded)", supported, free, occupied - supported);
  return sp_tree;
}


bool voxel_in_frustum(point3d p3d, geometry_msgs::Pose pose)
{
  static int id = 0;
  static ros::NodeHandle n;
  static ros::Publisher vis_pub = n.advertise<visualization_msgs::Marker>( "frustum", 0 );

  
  float theta = tf::getYaw(pose.orientation);
  
  //ROS_INFO("theta: %f", theta);

  // Origin of the triangle
  float ox = pose.position.x;
  float oy = pose.position.y;

  // Frustum
  float aax = ox + 3;
  float aay = oy - 1;

  float bbx = ox + 3;
  float bby = oy + 1;
  
  float ax = cos(theta) * (aax-ox) - sin(theta) * (aay-oy) + ox;
  float ay = sin(theta) * (aax-ox) + cos(theta) * (aay-oy) + oy; 
  
  float bx = cos(theta) * (bbx-ox) - sin(theta) * (bby-oy) + ox;
  float by = sin(theta) * (bbx-ox) + cos(theta) * (bby-oy) + oy;

  //ROS_INFO("(%f,%f) (%f,%f) (%f,%f)", ox, oy, ax, ay, bx, by);


  vec vsq[] = {	{ox,oy}, {ax,ay}, {bx,by} };
  
  // int NUM_POSES = 30;
  // id = (id + 1) % NUM_POSES; 

  // visualization_msgs::Marker marker;
  // marker.header.frame_id = "map";
  // marker.header.stamp = ros::Time();
  // marker.ns = "my_namespace";
  // marker.id = id;
  // marker.type = visualization_msgs::Marker::SPHERE;
  // marker.action = visualization_msgs::Marker::ADD;
  // marker.pose.position.x = ox;
  // marker.pose.position.y = oy;
  // marker.pose.position.z = 0.1;
  // marker.pose.orientation.x = 0.0;
  // marker.pose.orientation.y = 0.0;
  // marker.pose.orientation.z = 0.0;
  // marker.pose.orientation.w = 1.0;
  // marker.scale.x = 0.1;
  // marker.scale.y = 0.1;
  // marker.scale.z = 0.1;
  // marker.color.a = 1.0;
  // marker.color.r = 1.0;
  // marker.color.g = 0.0;
  // marker.color.b = 0.0;

  // id = (id + 1) % NUM_POSES; 
  
  // visualization_msgs::Marker marker1;
  // marker1.header.frame_id = "map";
  // marker1.header.stamp = ros::Time();
  // marker1.ns = "my_namespace";
  // marker1.id = id;
  // marker1.type = visualization_msgs::Marker::SPHERE;
  // marker1.action = visualization_msgs::Marker::ADD;
  // marker1.pose.position.x = ax;
  // marker1.pose.position.y = ay;
  // marker1.pose.position.z = 0.1;
  // marker1.pose.orientation.x = 0.0;
  // marker1.pose.orientation.y = 0.0;
  // marker1.pose.orientation.z = 0.0;
  // marker1.pose.orientation.w = 1.0;
  // marker1.scale.x = 0.1;
  // marker1.scale.y = 0.1;
  // marker1.scale.z = 0.1;
  // marker1.color.a = 1.0;
  // marker1.color.r = 0.0;
  // marker1.color.g = 1.0;
  // marker1.color.b = 0.0;


  // id = (id + 1) % NUM_POSES; 

  // visualization_msgs::Marker marker2;
  // marker2.header.frame_id = "map";
  // marker2.header.stamp = ros::Time();
  // marker2.ns = "my_namespace";
  // marker2.id = id;
  // marker2.type = visualization_msgs::Marker::SPHERE;
  // marker2.action = visualization_msgs::Marker::ADD;
  // marker2.pose.position.x = bx;
  // marker2.pose.position.y = by;
  // marker2.pose.position.z = 0.1;
  // marker2.pose.orientation.x = 0.0;
  // marker2.pose.orientation.y = 0.0;
  // marker2.pose.orientation.z = 0.0;
  // marker2.pose.orientation.w = 1.0;
  // marker2.scale.x = 0.1;
  // marker2.scale.y = 0.1;
  // marker2.scale.z = 0.1;
  // marker2.color.a = 1.0;
  // marker2.color.r = 0.0;
  // marker2.color.g = 0.0;
  // marker2.color.b = 1.0;
  
  // vis_pub.publish( marker );
  // vis_pub.publish( marker1 );
  // vis_pub.publish( marker2 );


  polygon_t frustum = { 3, vsq };
 
	vec point = { p3d.x(), p3d.y() }; 
 
  int inside_frustum = inside(point, &frustum, 1e-10);
	//ROS_INFO("%d", inside);
 
  if ( inside_frustum == 1)    
    return true;
  return false;
}

bool evaluate(nav_goals_msgs::WeightedNavGoals::Request  &req,
              nav_goals_msgs::WeightedNavGoals::Response &res)
{
  static bool first_call = true;
  static OcTree* octree = NULL;
  static OcTree* sp_octree = NULL;

  std::vector<geometry_msgs::Pose> landmark_pose;
  std::vector<std::string> landmark_type;

  std::vector< std::vector<float> > landmark_gmm;

 
  ROS_INFO("Started evaluation");
  ROS_INFO("request: obj_desc: %s, obj_list: %s", req.obj_desc.c_str(),  req.obj_list.c_str());

  
  get_landmarks(landmark_pose, landmark_type);

  get_GMMs(req.obj_desc.c_str(), landmark_type, landmark_gmm);

  // Get Octomap and extract supporting planes (only once)
  //if (first_call)
  //  {
  first_call = false;
  octree = retrieve_octree();
  sp_octree = extract_supporting_planes(octree);
  //  }
  
  // Weight voxels according to distribution (uniform, QSR) 
  std::vector<OcTreeKey> keys; 
  std::vector<point3d> coords; 
  std::vector<int> voxel_weights;

  std::vector<float> pose_weights;
  // init weights
  for( std::vector<geometry_msgs::Pose>::const_iterator it = req.goals.poses.begin(); 
       it!=req.goals.poses.end(); ++it)
    {
      pose_weights.push_back(1.0);
    } 

  int max_weight = 0;
  for(OcTree::leaf_iterator it = sp_octree->begin_leafs(),
        end=sp_octree->end_leafs(); it!= end; ++it)
    {
      if (sp_octree->isNodeOccupied(*it))
        // should always be the case after the supporting planes extraction
        {
          point3d p3d = it.getCoordinate();
          OcTreeKey key = sp_octree->coordToKey(p3d); 

          keys.push_back(key);
          coords.push_back(p3d);
          
          double x = it.getX();
          double y = it.getY();
          // if (weight > max_weight)
          //   max_weight = weight;
          // voxel_weights.push_back(weight);

          // Check for all poses whether current voxel is in frustum
          int idx = 0;
          
          for( std::vector<geometry_msgs::Pose>::const_iterator it = req.goals.poses.begin(); 
               it!=req.goals.poses.end(); ++it) 
            {
              //if ( p3d.z() >= 0.4 )//IGNORE THE GROUND PLANE 

              if ( voxel_in_frustum(p3d,*it) )
                {
                  if (landmark_pose.size() == 0) 
                    {
                      pose_weights[idx] += 1.00;
                      
                    }
                  else
                    {
                  
                      for (int l = 0; l < landmark_pose.size(); l++)
                        {
                          
                          std::vector<float> gmm = landmark_gmm[l];
                          
                          for (int i = 0; i < gmm.size(); i = i + 7)
                            {

                              float gmm_weight = gmm[i];
                              float mean_1 =  gmm[i + 1]; 
                              float mean_2 =  gmm[i + 2];;
                              float covar_1 = gmm[i + 3];
                              float covar_2 = gmm[i + 4];
                              float covar_3 = gmm[i + 6];
                              float covar_4 = gmm[i + 6];
                              
                              
                              // Rotate QSRs wrt landmark 
                              tf::Quaternion q(landmark_pose[l].orientation.x,
                                               landmark_pose[l].orientation.y,
                                               landmark_pose[l].orientation.z,
                                               landmark_pose[l].orientation.w);
                              
                              
                              tf::Matrix3x3 rot(q);
                              
                              tf::Matrix3x3 inv_rot = rot.inverse();
                              
                              tf::Matrix3x3 sigma(covar_1, covar_2, 0.0, 
                                                  covar_3, covar_4, 0.0,
                                                  0.0, 0.0, 0.0);
                              
                              // rotate covariance matrix
                              tf::Matrix3x3 rot_covar = (inv_rot * sigma) * rot;  
                              
                              tf::Vector3 vec1 = rot_covar.getRow(0);
                              tf::Vector3 vec2 = rot_covar.getRow(1);
                              
                              float covar_1_r = vec1.getX();
                              float covar_4_r = vec2.getY();
                              
                              tfScalar roll, pitch, yaw;
                              rot.getRPY(roll, pitch, yaw);
                              
                              float mean_1_r = mean_1 * cos(yaw) - mean_2 * sin(yaw);
                              float mean_2_r = mean_1 * sin(yaw) - mean_2 * cos(yaw);
                          
                              float mean_1_rt = mean_1_r + landmark_pose[l].position.x;
                              float mean_2_rt = mean_2_r + landmark_pose[l].position.y;

                              // ROS_INFO("LM(%i) LOCAL GMM(%i): (mean %f %f) (corvariance %f %f %f %f)",  l, i,
                              //          qsr_mean_1[i],
                              //          qsr_mean_2[i],
                              //          qsr_covar_1[i],
                              //          qsr_covar_2[i],
                              //          qsr_covar_3[i],
                              //          qsr_covar_4[i]);
                              // ROS_INFO("LM(%i) GLOBAL GMM(%i): (mean %f %f) (corvariance %f %f)", l, i,
                              //          mean_1_rt,
                              //          mean_2_rt,
                              //          covar_1_r,
                              //          covar_4_r);
                              
                              
                              pose_weights[idx] += 0.1 + (gmm_weight * (normal_dist_2d(x, y, 
                                                                                       mean_1_rt , covar_1_r , 
                                                                                       mean_2_rt , covar_4_r)));
                            }
                        }
                    }
                }
              idx++;
            }

        }
    }


  std::vector<int> pose_indices;
  
  for (int i = 0; i < req.goals.poses.size(); i++)
    {
      pose_indices.push_back(i);
    }
  
  // Sort pose indices wrt to the weights
  std::sort(pose_indices.begin(), pose_indices.end(), WeightComparator(pose_weights));

  // Sort weights themsselves
  std::sort(pose_weights.begin(), pose_weights.end());
  std::reverse(pose_weights.begin(), pose_weights.end());

  ROS_INFO("Sorted goal poses:");
  int i = 0;
  for( std::vector<int>::const_iterator it = pose_indices.begin(); 
       it!=pose_indices.end(); ++it) 
    {
      ROS_INFO("%i POSE (%f,%f), WEIGHT %f", i, req.goals.poses[(*it)].position.x, req.goals.poses[(*it)].position.y, pose_weights[i]);
      res.sorted_goals.poses.push_back(req.goals.poses[(*it)]);
      i++;
    }

  // Send responce
  res.weights = pose_weights;
 
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
