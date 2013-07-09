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
#include <string>

#define _USE_MATH_DEFINES 
#include <math.h>

#define ANGLE_MAX_DIFF (M_PI / 16) 

using namespace std;
using namespace octomap;

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
  OcTree* sp_tree = new OcTree(0.05);  

  int free = 0;
  int occupied = 0;
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
              avg_normal/= normals.size();       
              
              point3d z_axis ( 0.0, 0.0, 1.0);
              double angle = avg_normal.angleTo(z_axis);
              
              if ( angle < ANGLE_MAX_DIFF)  
                sp_tree->updateNode(it.getCoordinate(),true);
              
            }
        } else 
        {
          free++;
        }
    }
  
  
  // cout << "FREE: " << free << endl;
  // cout << "OCCUPIED: " << occupied << endl;
  // cout << endl;
  
  sp_tree->writeBinary("sp_" + string(argv[1]));
  cout << endl;
  cout << "Wrote file: sp_" << string(argv[1]) << endl;
  cout << endl;
 
  cout << "Now you can use octovis to visualize: octovis sp_" << string(argv[1]) << endl;
  cout << "Hint: hit 'F'-key in viewer to see the freespace" << endl  << endl;  

}
