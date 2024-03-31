#ifndef PATH_H
#define PATH_H

#include <iostream>
#include <cstring>
#include <vector>

#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <visualization_msgs/MarkerArray.h>

#include "node3d.h"
#include "constants.h"
#include "helper.h"
namespace HybridAStar {
/*!
   \brief A class for tracing and visualizing the path generated by the Planner
*/
class Path {
 public:
  /// The default constructor initializing the path object and setting publishers for the same.
  Path(bool smoothed = false) {
   if (smoothed){
      this->smoothed = smoothed;
   }

    // _________________
    // TOPICS TO PUBLISH
   //  pubPath = n.advertise<nav_msgs::Path>(pathTopic, 1, true);
   //  pubPathNodes = n.advertise<visualization_msgs::MarkerArray>(pathNodesTopic, 1, true);
   //  pubPathVehicles = n.advertise<visualization_msgs::MarkerArray>(pathVehicleTopic, 1, true);

    // CONFIGURE THE CONTAINER
   
   frame_id = "map";
   path.header.frame_id = frame_id;
   pathTrailer.header.frame_id = frame_id;
  }

  //  // __________
  //  // TRACE PATH
  //  /*!
  //     \brief Given a node pointer the path to the root node will be traced recursively
  //     \param node a 3D node, usually the goal node
  //     \param i a parameter for counting the number of nodes
  //  */
  //  void tracePath(const Node3D* node, int i = 0);
  /*!
     \brief Given a node pointer the path to the root node will be traced recursively
     \param node a 3D node, usually the goal node
     \param xy gird->info.origin.position.x,y
     \param i a parameter for counting the number of nodes
  */
  void updatePath(std::vector<Node3D> &nodePath);
  /*!
     \brief Adds a segment to the path
     \param node a 3D node
     \param xy gird->info.origin.position.x,y
  */
  void addSegment(const Node3D& node);
  void addSegmentTrailer(const Node3D& node);
  /*!
     \brief Adds a node to the path
     \param node a 3D node
     \param i a parameter for counting the number of nodes
     \param xy gird->info.origin.position.x,y
  */
  void addNode(const Node3D& node, int i);
  void addNodeTrailer(const Node3D& node, int i);
  /*!
     \brief Adds a vehicle shape to the path
     \param node a 3D node
     \param i a parameter for counting the number of nodes
     \param xy gird->info.origin.position.x,y
  */
  void addVehicle(const Node3D& node, int i);
  void addVehicleTrailer(const Node3D& node, int i);
/// Clear markers in rviz by publishing action 3 markers
  int clearRviz();

  // ______________
  // PUBLISH METHODS

  /// Clears the path
  void clear();
  /// Publishes the path
//   /// Publishes the nodes of the path
//   void publishPathNodes() { pubPathNodes.publish(pathNodes); }
//   /// Publishes the vehicle along the path
//   void publishPathVehicles() { pubPathVehicles.publish(pathVehicles); }

   nav_msgs::Path& getPath() {return path;}
   nav_msgs::Path& getPathTrailer() {return pathTrailer;}
   visualization_msgs::MarkerArray& getPathNodes() {return pathNodes;}
   visualization_msgs::MarkerArray& getPathVehicles() {return pathVehicles;}
   

 private:
//   /// A handle to the ROS node
//   ros::NodeHandle n;
//   /// Publisher for the path as a spline
//   ros::Publisher pubPath;
//   ros::Publisher pubinfoPath;
//   /// Publisher for the nodes on the path
//   ros::Publisher pubPathNodes;
//   /// Publisher for the vehicle along the path
//   ros::Publisher pubPathVehicles;
  /// Path data structure for visualization
  nav_msgs::Path path;
  nav_msgs::Path pathTrailer;
  /// Nodes data structure for visualization
  visualization_msgs::MarkerArray pathNodes;
  /// Vehicle data structure for visualization
  visualization_msgs::MarkerArray pathVehicles;
  /// Value that indicates that the path is smoothed/post processed
  bool smoothed = false;
  /// frame id
  std::string frame_id = "map";
};
}
#endif // PATH_H
