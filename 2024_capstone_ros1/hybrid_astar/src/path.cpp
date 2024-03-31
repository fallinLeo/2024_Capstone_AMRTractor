#include "path.h"

using namespace HybridAStar;


//###################################################
//                                         CLEAR PATH
//###################################################

void Path::clear() {
  Node3D node;
  path.poses.clear();
  pathTrailer.poses.clear();
  pathNodes.markers.clear();
  pathVehicles.markers.clear();
}

int Path::clearRviz(){
  int id = 1;
  visualization_msgs::Marker pathNode;
  pathNode.action = 3;
  pathNode.id = id;
  pathNode.header.frame_id = frame_id;
  pathNodes.markers.push_back(pathNode);
  id++;
  visualization_msgs::Marker pathVehicle;
  pathVehicle.action = 3;
  pathVehicle.id = id;
  pathVehicle.header.frame_id = frame_id;
  pathVehicles.markers.push_back(pathVehicle);
  id++;
  return id;
}

//###################################################
//                                         TRACE PATH
//###################################################
// __________
// TRACE PATH
void Path::updatePath(std::vector<Node3D>& nodePath) {
  path.header.stamp = ros::Time::now();
  int k = clearRviz(); // k is just for visualization marker

  for (size_t i = 0; i < nodePath.size(); ++i) {
    nodePath[i].updateTrailer();
    addSegment(nodePath[i]);
    addSegmentTrailer(nodePath[i]);
    addNode(nodePath[i], k);
    k++;
    addNodeTrailer(nodePath[i],k);
    k++;
    addVehicle(nodePath[i], k);
    k++;
    addVehicleTrailer(nodePath[i],k);
    k++;
  }
  return;
}
// ___________
// ADD SEGMENT
void Path::addSegment(const Node3D& node) {
  geometry_msgs::PoseStamped vertex;
  vertex.header.frame_id = frame_id; 
  vertex.pose.position.x = node.getX() ;
  vertex.pose.position.y = node.getY() ;
  vertex.pose.position.z = 0;
  // vertex.pose.orientation.x = 0;
  // vertex.pose.orientation.y = 0;
  // vertex.pose.orientation.z = 0;
  // vertex.pose.orientation.w = 0;
  if (node.getForward()){ //forward
    vertex.pose.orientation = tf::createQuaternionMsgFromYaw(node.getT());
  }
  else{ // backward
    if(!smoothed)
      vertex.pose.orientation = tf::createQuaternionMsgFromYaw(node.getT());
    else
      vertex.pose.orientation = tf::createQuaternionMsgFromYaw(node.getT()+M_PI);
  }

  path.poses.push_back(vertex);
}

void Path::addSegmentTrailer(const Node3D& node) {
  geometry_msgs::PoseStamped vertex;
  vertex.header.frame_id = frame_id; 
  vertex.pose.position.x = node.getTrailerX() ;
  vertex.pose.position.y = node.getTrailerY() ;
  vertex.pose.position.z = 0;
  // vertex.pose.orientation.x = 0;
  // vertex.pose.orientation.y = 0;
  // vertex.pose.orientation.z = 0;
  // vertex.pose.orientation.w = 0;
  if (node.getForward()){ //forward
    vertex.pose.orientation = tf::createQuaternionMsgFromYaw(node.getTrailerT());
  }
  else{ // backward
    if(!smoothed)
      vertex.pose.orientation = tf::createQuaternionMsgFromYaw(node.getTrailerT());
    else
      vertex.pose.orientation = tf::createQuaternionMsgFromYaw(node.getTrailerT()+M_PI);
  }

  pathTrailer.poses.push_back(vertex);
}

// ________
// ADD NODE
void Path::addNode(const Node3D& node, int i) {
  visualization_msgs::Marker pathNode;


  pathNode.header.frame_id = frame_id;
  pathNode.header.stamp = ros::Time(0);
  pathNode.id = i;
  pathNode.type = visualization_msgs::Marker::SPHERE;
  pathNode.scale.x = 0.05;
  pathNode.scale.y = 0.05;
  pathNode.scale.z = 0.05;
  pathNode.color.a = 1.0;

  if (smoothed) {
    pathNode.color.r = Constants::purple.red;
    pathNode.color.g = Constants::purple.green;
    pathNode.color.b = Constants::purple.blue;
  } else {
    pathNode.color.r = Constants::pink.red;
    pathNode.color.g = Constants::pink.green;
    pathNode.color.b = Constants::pink.blue;
  }

  pathNode.pose.position.x = node.getX() ;
  pathNode.pose.position.y = node.getY() ;
  pathNodes.markers.push_back(pathNode);
}

void Path::addNodeTrailer(const Node3D& node, int i) {
  visualization_msgs::Marker pathNode;


  pathNode.header.frame_id = frame_id;
  pathNode.header.stamp = ros::Time(0);
  pathNode.id = i;
  pathNode.type = visualization_msgs::Marker::SPHERE;
  pathNode.scale.x = 0.05;
  pathNode.scale.y = 0.05;
  pathNode.scale.z = 0.05;
  pathNode.color.a = 1.0;

  if (smoothed) {
    pathNode.color.r = Constants::darkpurple.red;
    pathNode.color.g = Constants::darkpurple.green;
    pathNode.color.b = Constants::darkpurple.blue;
  } else {
    pathNode.color.r = Constants::darkpink.red;
    pathNode.color.g = Constants::darkpink.green;
    pathNode.color.b = Constants::darkpink.blue;
  }

  pathNode.pose.position.x = node.getTrailerX() ;
  pathNode.pose.position.y = node.getTrailerY() ;
  pathNodes.markers.push_back(pathNode);
}

void Path::addVehicle(const Node3D& node, int i) {
  visualization_msgs::Marker pathVehicle;


  pathVehicle.header.frame_id = frame_id;
  pathVehicle.header.stamp = ros::Time(0);
  pathVehicle.id = i;
  pathVehicle.type = visualization_msgs::Marker::CUBE;
  pathVehicle.scale.x = Constants::RL;
  pathVehicle.scale.y = Constants::RW;
  pathVehicle.scale.z = 1;
  pathVehicle.color.a = 0.2;

  if (smoothed) {
    pathVehicle.color.r = Constants::teal.red;
    pathVehicle.color.g = Constants::teal.green;
    pathVehicle.color.b = Constants::teal.blue;

  } else {
    pathVehicle.color.r = Constants::orange.red;
    pathVehicle.color.g = Constants::orange.green;
    pathVehicle.color.b = Constants::orange.blue;
  }

  pathVehicle.pose.position.x = node.getX() ;
  pathVehicle.pose.position.y = node.getY() ;
  pathVehicle.pose.orientation = tf::createQuaternionMsgFromYaw(node.getT());
  pathVehicles.markers.push_back(pathVehicle);
}

void Path::addVehicleTrailer(const Node3D& node, int i) {
  visualization_msgs::Marker pathVehicle;


  pathVehicle.header.frame_id = frame_id;
  pathVehicle.header.stamp = ros::Time(0);
  pathVehicle.id = i;
  pathVehicle.type = visualization_msgs::Marker::CUBE;
  pathVehicle.scale.x = Constants::TL;
  pathVehicle.scale.y = Constants::TW;
  pathVehicle.scale.z = 1;
  pathVehicle.color.a = 0.2;

  if (smoothed) {
    pathVehicle.color.r = Constants::blue.red;
    pathVehicle.color.g = Constants::blue.green;
    pathVehicle.color.b = Constants::blue.blue;

  } else {
    pathVehicle.color.r = Constants::red.red;
    pathVehicle.color.g = Constants::red.green;
    pathVehicle.color.b = Constants::red.blue;
  }

  pathVehicle.pose.position.x = node.getTrailerX() ;
  pathVehicle.pose.position.y = node.getTrailerY() ;
  pathVehicle.pose.orientation = tf::createQuaternionMsgFromYaw(node.getTrailerT());
  pathVehicles.markers.push_back(pathVehicle);
}


