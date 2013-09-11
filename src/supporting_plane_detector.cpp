/*
 * Copyright (c) 2013, Lars Kunze, University of Birmingham
 * All rights reserved.
 * License: New BSD
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the University of Freiburg nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <stdlib.h>  
#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <octomap/ColorOcTree.h>
#include <string>

#define _USE_MATH_DEFINES 
#include <math.h>

#define ANGLE_MAX_DIFF (M_PI / 8) 

using namespace std;
using namespace octomap;


// double bivariate_normal_dist(double x, double y,
//                              double mean0, double var00, double var01, double var02, double var03,
//                              )


double normal_dist_2d(double x, double y, double mean1, double var1, double mean2, double var2)
  {
    return (1.0 / (2.0 * M_PI * sqrt(var1) * sqrt(var2)) * exp(-1 * ( pow(x - mean1,2) / ( 2.0 * var1) +  pow(y - mean2,2) / ( 2.0 * var2) )));
  }


float trapezoidal_shaped_func(float a, float b, float c, float d, float x)
{  
  float min = std::min(std::min((x - a)/(b - a), (float) 1.0), (d - x)/(d - c));
  return std::max(min, (float) 0.0);
}


int r_func(float x)
{
  float a = -0.125;
  float b =  0.125;
  float c =  0.375;
  float d =  0.625;

  x = 1.0 - x;
  
  float value = trapezoidal_shaped_func(a,b,c,d,x);
  
  return floor(value * 255);
}

int g_func(float x)
{
  float a =  0.125;
  float b =  0.375;
  float c =  0.625;
  float d =  0.875;

  x = 1.0 - x;
  
  float value = trapezoidal_shaped_func(a,b,c,d,x);
  
  return floor(value * 255);
}

int b_func(float x)
{
  float a =  0.375;
  float b =  0.625;
  float c =  0.875;
  float d =  1.125;
  
  x = 1.0 - x;
  
  float value = trapezoidal_shaped_func(a,b,c,d,x);
  
  return floor(value * 255);
}


void usage(char** argv){
  cout << endl;
  cout << "usage: " << argv[0] << " <octomap.bt> " << endl;
  cout << endl;
}

int main(int argc, char** argv) {
  
  if (argc != 2) 
    {
      cout << "Wrong number of arguments!" << endl;
      usage(argv);
      exit(EXIT_FAILURE);
      
    } 
  cout << endl;
  cout << "Loading map: " << argv[1]<< endl;
  cout << endl;

  OcTree* tree = new OcTree(string(argv[1]));
  // create empty tree for storing supporting planes (resolution 0.05)
  ColorOcTree* sp_tree = new ColorOcTree(0.05);  

  int free = 0;
  int occupied = 0;
  
  std::vector<OcTreeKey> keys; 
  std::vector<point3d> coords; 
  std::vector<int> weights; 
  
  for(OcTree::leaf_iterator it = tree->begin_leafs(),
        end=tree->end_leafs(); it!= end; ++it)
    {
      if (tree->isNodeOccupied(*it))
        {
          occupied++;
          std::vector<point3d> normals;
          
          point3d p3d = it.getCoordinate();
          OcTreeKey key = tree->coordToKey(p3d); 
          
          OcTreeKey neighborkey = key; 
          neighborkey[0] +=1; // neighbor in pos. x-direction 
          OcTreeNode* result = tree->search(neighborkey);

          //point3d res_p3d = result->getCoordinate();
          // if (result != NULL){ 
          //   tree->isNodeOccupied(result); 
          //   cout << "POS X NEIGHBOR OCCUPIED" << endl;
            
          // } else {
          //   cout << "POS X NEIGHBOR UNKOWN" << endl;
          // }

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
              // cout << "#Normals: " << normals.size() << endl;

              avg_normal/= normals.size();       
              
              point3d z_axis ( 0.0, 0.0, 1.0);
              double angle = avg_normal.angleTo(z_axis);

              point3d coord = it.getCoordinate();
              sp_tree->updateNode(coord,true);
              OcTreeKey sp_key = sp_tree->coordToKey(coord);
              if ( angle < ANGLE_MAX_DIFF)
                {
                  //sp_tree->setNodeColor(sp_key, 0, 255, 0);
                  keys.push_back(sp_key);
                  coords.push_back(coord);
                } 
               else 
                 {
                  sp_tree->setNodeColor(sp_key, 250, 250, 250);
                 }
            }
          
        } else 
        {
          free++;
        }
    }
  int max_weight = 0;
  for(std::vector<point3d>::iterator cit = coords.begin(), end = coords.end(); cit!= end; ++cit)
    {
      
      int weight = (int) (500 * normal_dist_2d((*cit).x(), (*cit).y(), -1.0 , 0.2, 4.7 , 0.1));
      weight += (int) (500    * normal_dist_2d((*cit).x(), (*cit).y(), 2.5 , 0.1, 4.7 , 0.2));
      weight += (int) (500    * normal_dist_2d((*cit).x(), (*cit).y(), 1 , 0.1, -2.5 , 0.3));
      if (weight > max_weight)
        max_weight = weight;

      weights.push_back(weight);
    }

  // Set color for probabilities
  int i = 0;
  for(std::vector<OcTreeKey>::iterator kit = keys.begin(), end = keys.end(); kit!= end; ++kit)
    {
      const unsigned char r = r_func((float) weights[i]/max_weight);
      const unsigned char g = g_func((float) weights[i]/max_weight);
      const unsigned char b = b_func((float) weights[i]/max_weight);
      sp_tree->setNodeColor((*kit), r, g, b);
      i++;
    }
  
  
  // cout << "FREE: " << free << endl;
  // cout << "OCCUPIED: " << occupied << endl;
  // cout << endl;
  
  sp_tree->write("sp_" + string(argv[1]) + ".ot");
  cout << endl;
  cout << "Wrote file: sp_" << string(argv[1]) << endl;
  cout << endl;
 
  cout << "Now you can use octovis to visualize: octovis sp_" << string(argv[1]) << endl;
  cout << "Hint: hit 'F'-key in viewer to see the freespace" << endl  << endl;  

}

