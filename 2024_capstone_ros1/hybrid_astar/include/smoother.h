#ifndef SMOOTHER_H
#define SMOOTHER_H 

#include <cmath>
#include <vector>

#include "dynamicvoronoi.h"
#include "node3d.h"
#include "vector2d.h"
#include "helper.h"
#include "constants.h"
namespace HybridAStar {
/*!
   \brief This class takes a path object and smoothes the nodes of the path.

   It also uses the Voronoi diagram as well as the configuration space.
*/
class Smoother {
 public:
  Smoother(DynamicVoronoi& v):voronoi(v) 
  {}
  ~Smoother() {path.clear();}

  /*!
     \brief This function takes a path consisting of nodes and attempts to iteratively smooth the same using gradient descent.

     During the different interations the following cost are being calculated
     obstacleCost
     curvatureCost
     smoothnessCost
     voronoiCost
  */
  void smoothPath();

  /*!
     \brief Given a node pointer the path to the root node will be traced recursively
     \param node a 3D node, usually the goal node
     \param i a parameter for counting the number of nodes
  */
  bool tracePath(const Node3D* node, int i = 0, std::vector<Node3D> path = std::vector<Node3D>());
  bool tracePath(const Node3D* node, float x, float y ,int i = 0, std::vector<Node3D> path = std::vector<Node3D>());

  void setOrigin(float x, float y, int width_, int height_){
    width = width_;
    height = height_;
    origin_x = x;
    origin_y = y;
    max_x = origin_x + width * Constants::cellSize;
    max_y = origin_y + height * Constants::cellSize;
  }

  /// returns the path of the smoother object
  std::vector<Node3D>& getPath() {return path;}

  /// obstacleCost - pushes the path away from obstacles
  Vector2D obstacleTerm(Vector2D xi);

  /// curvatureCost - forces a maximum curvature of 1/R along the path ensuring drivability
  Vector2D curvatureTerm(Vector2D x_im2, Vector2D x_im1, Vector2D x_i, Vector2D x_ip1, Vector2D x_ip2);

  /// smoothnessCost - attempts to spread nodes equidistantly and with the same orientation
  Vector2D smoothnessTerm(Vector2D xim2, Vector2D xim1, Vector2D xi, Vector2D xip1, Vector2D xip2);

  /// voronoiCost - trade off between path length and closeness to obstaclesg
  Vector2D voronoiTerm(Vector2D xi);

  /// a boolean test, whether vector is on the grid or not
  // bool isOnGrid(Vector2D vec) {
  //   if (vec.getX() >= 0 && vec.getX() < width &&
  //       vec.getY() >= 0 && vec.getY() < height) {
  //     return true;
  //   }
  //   return false;
  // }
  bool isOnGrid(Vector2D vec) {
    if (vec.getX() >= origin_x && vec.getX() < max_x &&
        vec.getY() >= origin_y && vec.getY() < max_y) {
      return true;
    }
    return false;
  }


 private:
  /// maximum possible curvature of the non-holonomic vehicle
  float kappaMax = 1.f / (Constants::r * Constants::cellSize * 1.1);
//   /// maximum distance to obstacles that is penalized
//   float obsDMax = Constants::obsDMax;
//   /// maximum distance for obstacles to influence the voronoi field
//   float vorObsDMax = Constants::voroObsDMax;
  /// voronoi diagram describing the topology of the map
  DynamicVoronoi& voronoi;
  /// width of the map
  int width;
  /// height of the map
  int height;
  /// origin of map x,y
  float origin_x;
  float origin_y;
  float max_x;
  float max_y;
  /// path to be smoothed
  std::vector<Node3D> path;
};
}
#endif // SMOOTHER_H
