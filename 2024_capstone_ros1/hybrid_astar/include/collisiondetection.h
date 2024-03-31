#ifndef COLLISIONDETECTION_H
#define COLLISIONDETECTION_H

#include <nav_msgs/OccupancyGrid.h>

#include "constants.h"
#include "lookup.h"
#include "node2d.h"
#include "node3d.h"

namespace HybridAStar {
namespace {
inline void getConfiguration(const Node2D* node, float& x, float& y, float& t) {
  x = node->getX();
  y = node->getY();
  // avoid 2D collision checking
  t = 99;
}

inline void getConfiguration(const Node3D* node, float& x, float& y, float& t) {
  x = node->getX();
  y = node->getY();
  t = node->getT();
}

inline void getConfigurationTrailer(const Node3D* node, float& x, float& y, float& t) {
  x = node->getTrailerX();
  y = node->getTrailerY();
  t = node->getTrailerT();
}

}
/*!
   \brief The CollisionDetection class determines whether a given configuration q of the robot will result in a collision with the environment.

   It is supposed to return a boolean value that returns true for collisions and false in the case of a safe node.
*/
class CollisionDetection {
 public:
  /// Constructor
  CollisionDetection();
  ~CollisionDetection();

  void collisionReconfigure();

  /*!
     \brief evaluates whether the configuration is safe
     \return true if it is traversable, else false
  */
   template<typename T> bool isTraversable(const T* node, bool twoD = false) const { // twoD agurgment is for voronoi astar
      /* Depending on the used collision checking mechanism this needs to be adjusted
         standard: collision checking using the spatial occupancy enumeration
         other: collision checking using the 2d costmap and the navigation stack
      */
      bool cost = true;
      float x;
      float y;
      float t;
      // assign values to the configuration
      getConfiguration(node, x, y, t);


      // 2D collision test
      if (t == 99 || twoD ) {
      return !(grid->data[node->getIdx()] >= Constants::inscribed_th || grid->data[node->getIdx()] == Constants::unknown_th );
      }
      // 3D collision test
      // Ttractor Test
      cost = configurationTest(x, y, t, false);
      if ( !cost )
         return false;

      // Trailer Test
      node->calcTrailer(x,y,t);
      cost = configurationTest(x, y, t, true);

      return cost;
   }

  /*!
     \brief Calculates the cost of the robot taking a specific configuration q int the World W
     \param x the x position
     \param y the y position
     \param t the theta angle
     \return the cost of the configuration q of W(q)
     \todo needs to be implemented correctly
  */
  float configurationCost(float x, float y, float t) const {return 0;}

  /*!
     \brief Tests whether the configuration q of the robot is in C_free
     \param x the x position
     \param y the y position
     \param t the theta angle
     \param isTrailer in case of Trailer, use TrailerLookupTable
     \return true if it is in C_free, else false
  */
  bool configurationTest(float x, float y, float t, bool isTrailer) const;

  /*!
     \brief updates the grid with the world map
  */
  void updateGrid(nav_msgs::OccupancyGrid::Ptr map) {grid = map;}

 private:
  /// The occupancy grid
  nav_msgs::OccupancyGrid::Ptr grid;
  /// The collision lookup table
//   Constants::config collisionLookup[Constants::headings * Constants::positions];
  Constants::config * collisionLookup;
  Constants::config * collisionLookupTrailer;
};
}
#endif // COLLISIONDETECTION_H
