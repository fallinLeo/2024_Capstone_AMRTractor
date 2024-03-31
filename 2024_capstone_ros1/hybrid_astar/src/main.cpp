/**
   \file main.cpp
   \brief Main entry point of the program, starts an instance of Planner
*/

//###################################################
//                      HYBRID A* ALGORITHM
//  AUTHOR:   Karl Kurzer
//  WRITTEN:  2015-03-02
//###################################################

#include <cstring>
#include <iostream>
#include <ros/ros.h>
#include <syscon_msgs/RobotState.h>
#include <cmath>
#include <dynamic_reconfigure/server.h>
#include <hybrid_astar/constantsConfig.h>
#include "constants.h"
#include "planner.h"

const unsigned char option1 = 1 << 0; // 0000 0001 
const unsigned char option2 = 1 << 1; // 0000 0010
const unsigned char option4 = 1 << 2; // 0000 0100

// 전역 변수 선언 Global Variables
geometry_msgs::PoseStamped start;
geometry_msgs::PoseStamped goal;
HybridAStar::Planner* planner;

namespace HybridAStar{

  namespace Constants{
    // Constants.h

    // ros params
    bool simul = false;  
    bool reverse = true;     
    bool voronoibased = true;  
    bool useReedsSheppShot;
    float cellSize = 0.2;
    int directions = 3; // dynamic reconfigure0


    // raw values from the 
    float rRaw = 2.0;
    float primitiveLengthRaw = 0.25;
    float dubinsShotDistanceRaw = 5;

    // robot specifications
    double RL = 0.662;   // dynamic reconfigure 4
    double RW = 0.633;   // dynamic reconfigure 4
    double WB = 0.319;   
    double TL = 1.0;    // dynamic reconfigure 4
    double TW = 0.643;  // dynamic reconfigure 4
    double RTR = 0.9;   
    double RTF = 0.025;
    double RTB = 0.1;
    double maxth = 2.198;
    double TCH = (RL - WB)/2; 
    double HCR = TL/2 - RTF;

    double bloating = 0.03;              // dynamic reconfigure 4
    double width = RW + 2 * bloating;    
    double length = RL + 2 * bloating;  
    double trailer_width = TW + 2 * bloating;
    double trailer_length = TL + 2 * bloating;
    int headings = 60;          // dynamic reconfigure4
    float r = rRaw / cellSize;   // dynamic reconfigure0
    float deltaHeadingDeg = 360 / (float)headings;
    float deltaHeadingRad = 2 * M_PI / (float)headings;
    float deltaHeadingNegRad = 2 * M_PI - deltaHeadingRad;
    float primitiveLength = primitiveLengthRaw / cellSize; // dynamic reconfigure0
    float penaltyTurning = 1.4;     // dynamic reconfigure0
    float penaltyReversing = 1.0;   // dynamic reconfigure0
    float penaltyCOD = 1.8;         // dynamic reconfigure0
    float dubinsShotDistance = dubinsShotDistanceRaw * dubinsShotDistanceRaw / cellSize;   // dynamic reconfigure0
    float dubinsStepSize = primitiveLength;
    int bbSize = std::ceil((sqrt(width * width + length* length) +1 ) / cellSize);
    int trailer_bbSize = std::ceil((sqrt(TW * TW + TL* TL) +1 ) / cellSize);
    int positionResolution = (int)(cellSize/0.1) + 1;
    int positions = positionResolution * positionResolution;
    // float voroObsDMax = 3 / cellSize;   // dynamic reconfigure0
    float voroObsDMax = 3;   // dynamic reconfigure0
    // float obsDMax  = length / cellSize; // dynamic reconfigure0
    float obsDMax  = length; // dynamic reconfigure0

    int voroSmoothWin = 5;                   // dynamic reconfigure 2
    int voroObstacle = Constants::lethal_th; // dynamic reconfigure 2

    float alpha = 0.2;          // dynamic reconfigure0
    float wObstacle = 0.05;     // dynamic reconfigure0
    float wVoronoi = 0.95;      // dynamic reconfigure0
    float wCurvature = 0.0;     // dynamic reconfigure0
    float wSmoothness = 0.15;   // dynamic reconfigure0
    int maxIterations = 250;    // dynamic reconfigure0

    void setConfig(){
      width = RW + 2 * bloating;
      length = RL + 2 * bloating;
      trailer_width = TW + 2 * bloating;
      trailer_length = TL + 2 * bloating;
      deltaHeadingDeg = 360 / (float)headings;
      deltaHeadingRad = 2 * M_PI / (float)headings;
      deltaHeadingNegRad = 2 * M_PI - deltaHeadingRad;
      dubinsStepSize = primitiveLength;
      bbSize = std::ceil((sqrt(width * width + length* length) +1 ) / cellSize);
      trailer_bbSize = std::ceil((sqrt(TW * TW + TL* TL) +1 ) / cellSize);
      obsDMax  = length ;
    }

    void setCellSize(float c){
      cellSize = c;
      r = rRaw / cellSize;   // dynamic reconfigure0
      primitiveLength = primitiveLengthRaw / cellSize; // dynamic reconfigure0
      dubinsStepSize = primitiveLength;
      dubinsShotDistance = dubinsShotDistanceRaw * dubinsShotDistanceRaw / cellSize;   // dynamic reconfigure0
      bbSize = std::ceil((sqrt(width * width + length* length) +1 ) / cellSize);
      trailer_bbSize = std::ceil((sqrt(TW * TW + TL* TL) +1 ) / cellSize);
      positionResolution = (int)(cellSize/0.1) + 1;
      positions = positionResolution * positionResolution;
    }

  }
}



// Publishers
/// A publisher publishing the start position for RViz
ros::Publisher pubStart;
/// Publisher for the path as a spline
ros::Publisher pubPath;
ros::Publisher pubPathTrailer;
/// Publisher for the nodes on the path
ros::Publisher pubPathNodes;
/// Publisher for the vehicle along the path
ros::Publisher pubPathVehicles;
/// Publisher for the path as a spline
ros::Publisher spubPath;
ros::Publisher spubPathTrailer;
/// Publisher for the nodes on the path
ros::Publisher spubPathNodes;
/// Publisher for the vehicle along the path
ros::Publisher spubPathVehicles;
/// Publisher for Voronoi diagram edges
ros::Publisher pubVoroCost;
/// Publisher for Voronoi diagram subgoals 
ros::Publisher pubSubGoal;


///  Subscribers
/// A subscriber for receiving map updates
ros::Subscriber subMap;
/// A subscriber for receiving goal updates
ros::Subscriber subGoal;
/// A subscriber for receiving start updates
ros::Subscriber subStart;
/// The path produced by the hybrid A* algorithm


// callbacks
void setStart_rviz(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& initial);
void setStart_robot(const syscon_msgs::RobotState::ConstPtr& initial);
void setGoal(const geometry_msgs::PoseStamped::ConstPtr& goal);
void setMap(const nav_msgs::OccupancyGrid::Ptr map);
bool planning();
void dynamicReconfigCb(hybrid_astar::constantsConfig &config, uint32_t level);
void cellSizeChanged(float cellSize);

//###################################################
//                                               MAIN
//###################################################
/**
   \fn main(int argc, char** argv)
   \brief Starting the program
   \param argc The standard main argument count
   \param argv The standard main argument value
   \return 0
*/
int main(int argc, char** argv) {

  ros::init(argc, argv, "a_star");
  ros::NodeHandle n;
  ros::param::get("~simul",HybridAStar::Constants::simul);
  ros::param::get("~reverse",HybridAStar::Constants::reverse);
  ros::param::get("~voronoibased",HybridAStar::Constants::voronoibased);
  ros::param::get("~useReedsSheppShot",HybridAStar::Constants::useReedsSheppShot);
  ROS_ERROR("simul : %d",HybridAStar::Constants::simul);
  ROS_ERROR("reverse : %d",HybridAStar::Constants::reverse);
  ROS_ERROR("voronoibased : %d",HybridAStar::Constants::voronoibased);
  ROS_ERROR("useReedsSheppShot : %d",HybridAStar::Constants::useReedsSheppShot);

  HybridAStar::Planner planner_;
  planner = &planner_;  

  pubStart = n.advertise<geometry_msgs::PoseStamped>("/move_base_simple/start", 1);
  // Path
  pubPath = n.advertise<nav_msgs::Path>("/plan_hybrid", 1, true);
  pubPathTrailer = n.advertise<nav_msgs::Path>("/pathTrailer", 1, true);
  pubPathNodes = n.advertise<visualization_msgs::MarkerArray>("/pathNodes", 1, true);
  pubPathVehicles = n.advertise<visualization_msgs::MarkerArray>("/pathVehicle", 1, true);
  // Smoothed Path
  spubPath = n.advertise<nav_msgs::Path>("/sPath", 1, true);
  spubPathTrailer = n.advertise<nav_msgs::Path>("/sPathTrailer", 1, true);
  spubPathNodes = n.advertise<visualization_msgs::MarkerArray>("/sPathNodes", 1, true);
  spubPathVehicles = n.advertise<visualization_msgs::MarkerArray>("/sPathVehicle", 1, true);
  // Voronoi diagram
  pubSubGoal =  n.advertise<visualization_msgs::MarkerArray>("subGoals", 1, true);
  pubVoroCost =  n.advertise<nav_msgs::OccupancyGrid>("voroCostmap", 1, true);


  // ___________________
  // TOPICS TO SUBSCRIBE
  if(HybridAStar::Constants::simul){
    subStart = n.subscribe("robot_state", 1, setStart_robot);
    // subMap = n.subscribe("move_base/global_costmap/costmap", 1, setMap);
    subMap = n.subscribe("global_costmap/costmap", 1, setMap);
  }
  else{
    subStart = n.subscribe("/initialpose", 1,setStart_rviz );
    subMap = n.subscribe("/map", 1, setMap);
  }
  subGoal = n.subscribe("/move_base_simple/goal", 1, setGoal);
  // subGoal = n.subscribe("/goal_pose", 1, setGoal);

  // __________________________
  // DYNAMIC RECONFIGURE SERVER
  dynamic_reconfigure::Server<hybrid_astar::constantsConfig> server;
  dynamic_reconfigure::Server<hybrid_astar::constantsConfig>::CallbackType f;
  f = boost::bind(&dynamicReconfigCb, _1, _2);
  server.setCallback(f);


  ros::spin();
  return 0;
}


void setStart_rviz(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& initial)
{
  start.pose.position = initial->pose.pose.position;
  start.pose.orientation = initial->pose.pose.orientation;
  start.header.frame_id = initial->header.frame_id;
  start.header.stamp = ros::Time::now();
  pubStart.publish(start);
  planner->setStart(start);
  planning();
}

void setStart_robot(const syscon_msgs::RobotState::ConstPtr& initial){
  start.pose.position.x = initial->pose.x;
  start.pose.position.y = initial->pose.y;
  start.pose.orientation = tf::createQuaternionMsgFromYaw(initial->pose.theta);
  start.header.frame_id = "map";
  start.header.stamp = ros::Time::now();
  // pubStart.publish(start);
  planner->setStart(start);
}

void setGoal(const geometry_msgs::PoseStamped::ConstPtr& end) {
  goal = *end;
  planner->setGoal(goal);
  planning();

}

void setMap(const nav_msgs::OccupancyGrid::Ptr map) {
  if(HybridAStar::Constants::cellSize!=map->info.resolution){
    cellSizeChanged(map->info.resolution);
  }
  planner->setMap(map);
  pubVoroCost.publish(planner->getvoroCostmap());
}

bool planning(){
  if(!planner->plan()){
    pubSubGoal.publish(planner->getsubGoalMarkerArray());
    return false;
  }
  // path making success
  pubPath.publish(planner->getPath(false));
  pubPathTrailer.publish(planner->getPathTrailer(false));
  pubPathNodes.publish(planner->getPathNodes(false));
  pubPathVehicles.publish(planner->getPathVehicles(false));
  spubPath.publish(planner->getPath(true));
  spubPathTrailer.publish(planner->getPathTrailer(true));
  spubPathNodes.publish(planner->getPathNodes(true));
  spubPathVehicles.publish(planner->getPathVehicles(true));
  pubSubGoal.publish(planner->getsubGoalMarkerArray());
  return true;
}

void dynamicReconfigCb(hybrid_astar::constantsConfig &c, uint32_t level) {
  HybridAStar::Constants::voroSmoothWin = c.voronoi_smooth;
  HybridAStar::Constants::voroObstacle  = ((c.voronoi_lethal == 0) ? HybridAStar::Constants::lethal_th : HybridAStar::Constants::inscribed_th);
  HybridAStar::Constants::RL = c.tractor_length;
  HybridAStar::Constants::RW = c.tractor_width;
  HybridAStar::Constants::TL = c.trailer_length;
  HybridAStar::Constants::TW = c.trailer_width;
  HybridAStar::Constants::bloating = c.robot_bloating;
  HybridAStar::Constants::headings = c.headings;
  HybridAStar::Constants::rRaw = c.robot_radius;
  HybridAStar::Constants::r = HybridAStar::Constants::rRaw / HybridAStar::Constants::cellSize;
  HybridAStar::Constants::primitiveLengthRaw = c.primitive_length;
  HybridAStar::Constants::primitiveLength = HybridAStar::Constants::primitiveLengthRaw / HybridAStar::Constants::cellSize;
  HybridAStar::Constants::penaltyTurning = c.penaltyTurning;
  HybridAStar::Constants::penaltyReversing = c.penaltyReversing;
  HybridAStar::Constants::penaltyCOD = c.penaltyCOD;
  HybridAStar::Constants::dubinsShotDistanceRaw = c.dubinsShotDistance;
  HybridAStar::Constants::dubinsShotDistance = HybridAStar::Constants::dubinsShotDistanceRaw * HybridAStar::Constants::dubinsShotDistanceRaw / HybridAStar::Constants::cellSize;
  HybridAStar::Constants::voroObsDMax = c.opti_voroObsDMax;
  HybridAStar::Constants::alpha = c.opti_alpha;
  HybridAStar::Constants::wObstacle = c.opti_wObstacle;
  HybridAStar::Constants::wVoronoi = c.opti_wVoronoi;
  HybridAStar::Constants::wCurvature = c.opti_wCurvature;
  HybridAStar::Constants::wSmoothness = c.opti_wSmoothness;
  HybridAStar::Constants::maxIterations = c.opti_maxIterations;
  HybridAStar::Constants::directions = c.directions;
  HybridAStar::Constants::setConfig();
  // ROS_INFO("LEVEL : %d, masked?: %d %d",level, level & option2, level & option4);
  if(level & option2){
    if(planner->voronoiReconfigure()){
      pubVoroCost.publish(planner->getvoroCostmap());
    }
  }
  if(level & option4){
    planner->collisionReconfigure();
    
  }
}

void cellSizeChanged(float cellSize){
  HybridAStar::Constants::setCellSize(cellSize);
  // setmap에서 Voronoi 만들고 publish한다.
  planner->collisionReconfigure();
}

