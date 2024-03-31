#ifndef PLANNER_H
#define PLANNER_H

#include <iostream>
#include <ctime>

#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include "constants.h"
#include "helper.h"
#include "collisiondetection.h"
#include "dynamicvoronoi.h"
#include "algorithm.h"
#include "node3d.h"
#include "path.h"
#include "smoother.h"
#include "visualize.h"
//#include "lookup.h"

namespace HybridAStar {
/*!
   \brief A class that creates the interface for the hybrid A* algorithm.

    It inherits from `ros::nav_core::BaseGlobalPlanner` so that it can easily be used with the ROS navigation stack
   \todo make it actually inherit from nav_core::BaseGlobalPlanner
*/
class Planner {
 public:
  /// The default constructor
  Planner();
  ~Planner();

  /*!
     \brief Initializes the collision as well as heuristic lookup table
     \todo probably removed
  */
   // void initializeLookups();

   /*!
     \brief Sets the map e.g. through a callback from a subscriber listening to map updates.
     \param map the map or occupancy grid
   */
   void setMap(const nav_msgs::OccupancyGrid::Ptr map);
   
   bool voronoiReconfigure();
   void createVoronoiDiagram(const  nav_msgs::OccupancyGrid::Ptr map);
   void collisionReconfigure();


   /*!
     \brief setStart
     \param start the start pose
  */
   void setStart(const geometry_msgs::PoseStamped& start_);
   // void setStart(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& initial);
   // void setStart_odom(const nav_msgs::Odometry::ConstPtr& initial );


   /*!
      \brief setGoal
      \param goal the goal pose
   */
   void setGoal(const geometry_msgs::PoseStamped& goal_);
   // void setGoal(const geometry_msgs::PoseStamped::ConstPtr& goal);

   /*!
      \brief The central function entry point making the necessary preparations to start the planning.
   */
   bool plan();

   // returning topics to publish
   nav_msgs::Path& getPath(bool smoothed);
   nav_msgs::Path& getPathTrailer(bool smoothed);
   visualization_msgs::MarkerArray& getPathNodes(bool smoothed);
   visualization_msgs::MarkerArray& getPathVehicles(bool smoothed);
   visualization_msgs::MarkerArray& getsubGoalMarkerArray();
   nav_msgs::OccupancyGrid& getvoroCostmap();

 private:
   /// The node handle
   Path path;
   /// The smoother used for optimizing the path
   Smoother smoother;
   /// The path smoothed and ready for the controller
   Path smoothedPath;
   /// The visualization used for search visualization
   Visualize visualization;
   /// The collission detection for testing specific configurations
   CollisionDetection configurationSpace;
   /// The voronoi diagram
   DynamicVoronoi voronoiDiagram;
   /// A pointer to the grid the planner runs on
   nav_msgs::OccupancyGrid::Ptr grid;
   /// The start pose set through RViz
   geometry_msgs::PoseStamped start;
   /// The goal pose set through RViz
   geometry_msgs::PoseStamped goal;
   /// Flags for allowing the planner to plan
   bool validStart = false;
   /// Flags for allowing the planner to plan
   bool validGoal = false;
   /// Flags for allowing the planner to plan
   bool validMap = false;
   /// A lookup table for configurations of the vehicle and their spatial occupancy enumeration -> collisiondetection.cpp
   // Constants::config collisionLookup[Constants::headings * Constants::positions];
   /// A lookup of analytical solutions (Dubin's paths)
   // float* dubinsLookup = new float [Constants::headings * Constants::headings * Constants::dubinsWidth * Constants::dubinsWidth];
   float * dubinsLookup;
   /// frame id
   std::string frame_id;

   /// memory storage 
   Node3D* nodes3D;
   int nodes3D_length;
   Node2D* nodes2D;
   int nodes2D_length;

   
};
}
#endif // PLANNER_H
