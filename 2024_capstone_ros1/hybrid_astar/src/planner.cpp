#include "planner.h"

using namespace HybridAStar;
//###################################################
//                                        CONSTRUCTOR
//###################################################
Planner::Planner() : smoother(voronoiDiagram), 
nodes3D(nullptr), nodes3D_length(0),
nodes2D(nullptr), nodes2D_length(0)

 {
  // _____
  // TODOS
  //    initializeLookups();
  // Lookup::collisionLookup(collisionLookup);
  // ___________________
  // COLLISION DETECTION
  //    CollisionDetection configurationSpace

  smoothedPath = Path(true);
  frame_id = "map";


};

Planner::~Planner(){
  if(nodes3D){
    delete [] nodes3D;
  }
  if(nodes2D){
    delete [] nodes2D;
  }
}


//###################################################
//                                       LOOKUPTABLES
//###################################################
// void Planner::initializeLookups() {
//   if (Constants::dubinsLookup) {
//     Lookup::dubinsLookup(dubinsLookup);
//   }
//   Lookup::collisionLookup(collisionLookup);
// }
void Planner::collisionReconfigure(){
  ROS_INFO_STREAM("cell Size : " << Constants::cellSize);
  ROS_INFO_STREAM("radius : " << Constants::r * Constants::cellSize);
  ROS_INFO_STREAM("headings : " << Constants::headings);
  ROS_INFO_STREAM("primitiveLength : " << Constants::primitiveLength * Constants::cellSize);
  ROS_INFO_STREAM("dubinsShotDistance Size : " << Constants::dubinsShotDistance * Constants::cellSize);
  configurationSpace.collisionReconfigure();
}

void Planner::createVoronoiDiagram(const  nav_msgs::OccupancyGrid::Ptr map){
  //create array for Voronoi diagram
  int height = map->info.height;
  int width = map->info.width;
  bool** binMap;
  binMap = new bool*[width];

  for (int x = 0; x < width; x++) { binMap[x] = new bool[height]; }
  for (int x = 0; x < width; ++x) {
    for (int y = 0; y < height; ++y) {
      binMap[x][y] = (map->data[y * width + x]>=Constants::voroObstacle || map->data[y * width + x]==Constants::unknown_th) ? true : false;
    }
  }
  ros::Time t0 = ros::Time::now();
  voronoiDiagram.initializeMap(width, height, binMap, map->info.origin.position.x, map->info.origin.position.y);
  voronoiDiagram.update();
  voronoiDiagram.visualize();
  ros::Time t1 = ros::Time::now();
  ros::Duration d(t1 - t0);
  std::cout << "created Voronoi Diagram in ms: " << d * 1000 << std::endl;
}

//###################################################
//                                                MAP
//###################################################
void Planner::setMap(const nav_msgs::OccupancyGrid::Ptr map) {
  validMap = true;
  grid = map;
  //update the configuration space with the current map
  configurationSpace.updateGrid(map);
  //create array for Voronoi diagram
  createVoronoiDiagram(map);
}

bool Planner::voronoiReconfigure(){
  if(validMap){
    createVoronoiDiagram(grid);
    return true;
  }
  else{
    return false;
  }
}

//###################################################
//                                   INITIALIZE START
//###################################################
void Planner::setStart(const geometry_msgs::PoseStamped& start_) {
  start = start_;
  validStart=true;
}

//###################################################
//                                    INITIALIZE GOAL
//###################################################
void Planner::setGoal(const geometry_msgs::PoseStamped& goal_) {
  goal = goal_;
  validGoal=true;
}

//###################################################
//                           RETURN TOPICS TO PUBLISH
//###################################################
nav_msgs::Path& Planner::getPath(bool smoothed = false) 
{
  if(smoothed){ return smoothedPath.getPath(); }
  else { return path.getPath(); }    
}

nav_msgs::Path& Planner::getPathTrailer(bool smoothed = false) 
{
  if(smoothed){ return smoothedPath.getPathTrailer(); }
  else { return path.getPathTrailer(); }    
}

visualization_msgs::MarkerArray& Planner::getPathNodes(bool smoothed = false) 
{
  if(smoothed){ return smoothedPath.getPathNodes(); }
  else { return path.getPathNodes(); }    
}
visualization_msgs::MarkerArray& Planner::getPathVehicles(bool smoothed = false)
{
  if(smoothed){ return smoothedPath.getPathVehicles(); }
  else { return path.getPathVehicles(); }    
}
visualization_msgs::MarkerArray& Planner::getsubGoalMarkerArray(){
  return voronoiDiagram.getsubGoalMarkerArray();
}
nav_msgs::OccupancyGrid& Planner::getvoroCostmap(){
  return voronoiDiagram.getvoroCostmap();
}

//###################################################
//                                      PLAN THE PATH
//###################################################
bool Planner::plan() {

  ROS_INFO_STREAM("path planning is started ");

  // if a start as well as goal are defined go ahead and plan
  if (validStart && validGoal && validMap) {
    ros::Time start_time = ros::Time::now();
    ros::Time t0, t1;
    ros::Duration d, mem_free_time, mem_alloc_time;
    // ________________________________
    // LISTS ALLOWCATED ROW MAJOR ORDER
    int width = grid->info.width;
    int height = grid->info.height;
    float grid_x = grid->info.origin.position.x;
    float grid_y = grid->info.origin.position.y;
    Node3D::updateConstants();
    int depth = Constants::headings;
    int length = width * height * depth;
    // define list pointers and initialize lists

    t0 = ros::Time::now();
    // Node3D* nodes3D = new Node3D[length]();
    // Node2D* nodes2D = new Node2D[width * height]();
    // if nodes3D empty or lenth is not same -> allocate new memory
    if( !nodes3D || nodes3D_length != length){
      if(nodes3D){
        delete [] nodes3D;
      }
      nodes3D = new Node3D[length]();
      nodes3D_length = length;
    }
    if( !nodes2D || nodes2D_length != width * height){
      if(nodes2D){
        delete [] nodes2D;
      }
      nodes2D = new Node2D[width * height]();
      nodes2D_length = width * height;
    }
    t1 = ros::Time::now();
    mem_alloc_time = ros::Duration(t1 - t0);


    visualization.set_map_origin(grid_x,grid_y);
    smoother.setOrigin(grid_x,grid_y,width, height);
    //point vectors to save pointers of dubins path subgoal
    std::vector<Node3D*> dubins_pointers;

    // _________________________
    // retrieving goal position
    float gx = (goal.pose.position.x-grid_x) / Constants::cellSize;
    float gy = (goal.pose.position.y-grid_y) / Constants::cellSize;
    float t = tf::getYaw(goal.pose.orientation);
    // set theta to a value (0,2PI]
    t = Helper::normalizeHeadingRad(t);    
    if (!(grid->info.height >= gy && gy >= 0 && grid->info.width >= gx && gx >= 0)) {
      ROS_ERROR("invalid goal x: %6.2f y: %6.2f t: %2.2f", goal.pose.position.x, goal.pose.position.y , Helper::toDeg(t));
      validGoal = false;
      return false;
    }
    const Node3D nGoal(gx, gy, t, 0, 0, 0, nullptr);
    // _________________________
    // retrieving start position
    float sx = (start.pose.position.x-grid_x) / Constants::cellSize;
    float sy = (start.pose.position.y-grid_y) / Constants::cellSize;
    t = tf::getYaw(start.pose.orientation);
    // set theta to a value (0,2PI]
    t = Helper::normalizeHeadingRad(t);
    if (!(grid->info.height >= sy && sy >= 0 && grid->info.width >= sx && sx >= 0)) {
      ROS_ERROR("invalid start x: %6.2f y: %6.2f t: %2.2f", start.pose.position.x, start.pose.position.y , Helper::toDeg(t));
      validStart = false;
      return false;
    }

    Node3D nStart(sx, sy, t, 0, 0, 0, nullptr);

    // if Voronoi based Hybrid A* enabled do A* in voronoi a* , make subgoal
    if(Constants::voronoibased){
      t0 = ros::Time::now();
      bool success = voronoiDiagram.makesubGoal(sx, sy, gx, gy, configurationSpace); // get end of subgoals (end == cloesest voronoi edge from goal)
      t1 = ros::Time::now();
      d = ros::Duration(t1 - t0);
      ROS_INFO_STREAM("VORONOI ASTAR in ms: " << d * 1000);
      if(!success)
      {
        ROS_ERROR("VORONOI ASTAR FAILED, PLEASH CHECK VORONOI DIAGRAM");
      }
    }

    // ___________________________
    // START AND TIME THE PLANNING
    t0 = ros::Time::now();

    // CLEAR THE VISUALIZATION
    visualization.clear();

    // CLEAR THE PATH
    path.clear();
    smoothedPath.clear();

    // FIND THE PATH
    Node3D* nSolution = Algorithm::hybridAStar(nStart, nGoal, nodes3D, nodes2D, width, height,
    configurationSpace, dubinsLookup, visualization, dubins_pointers, voronoiDiagram.getsubgoal());
    // TRACE THE PATH
    // smoother.tracePath(nSolution);
    bool traceResult = smoother.tracePath(nSolution, grid->info.origin.position.x, grid->info.origin.position.y);
    if (nSolution == nullptr || !traceResult){
      ROS_ERROR("hybridAStar failed to find the path");
      for(std::vector<Node3D*>::iterator iter = dubins_pointers.begin();iter !=dubins_pointers.end();iter++){
        delete [] *iter;
      }
      dubins_pointers.clear();
      return false;
    }
    else {
      ROS_INFO_STREAM("hybridAStar succeeded to find the path");
    }

    // CREATE THE UPDATED PATH
    path.updatePath(smoother.getPath());
    // path.updatePath(smoother.getPath(),0,0);
    // SMOOTH THE PATH
    t1 = ros::Time::now();
    d = ros::Duration(t1 - t0);
    ROS_INFO_STREAM("SEARCH TOTAL TIME in ms: " << d * 1000);

    t0 = ros::Time::now();
    smoother.smoothPath();
    // CREATE THE UPDATED PATH
    smoothedPath.updatePath(smoother.getPath());
    // smoothedPath.updatePath(smoother.getPath(),0, 0);
    t1 = ros::Time::now();
    d = ros::Duration(t1 - t0);
    ROS_INFO_STREAM("OPTI TOTAL TIME in ms: " << d * 1000);

    t0 = ros::Time::now();
    if (Constants::visualization)
      visualization.publishNode3DCosts(nodes3D, width, height, depth);
    if (Constants::visualization2D)
      visualization.publishNode2DCosts(nodes2D, width, height);

    // 동적할당된 모든 메모리를 free해준다
    for(std::vector<Node3D*>::iterator iter = dubins_pointers.begin();iter !=dubins_pointers.end();iter++){
      delete [] *iter;
    }
    dubins_pointers.clear();
    t1 = ros::Time::now();
    mem_free_time = ros::Duration(t1-t0);

    ROS_INFO_STREAM("MEM ALLOCATE TIME in ms: " << mem_alloc_time * 1000);
    ROS_INFO_STREAM("MEM FREE TOTAL TIME in ms: " << mem_free_time * 1000);

    ros::Time t100 = ros::Time::now();
    ros::Duration d100(t100-start_time);
    ROS_INFO_STREAM("PLANNING TOTAL TIME in ms: " << d100 * 1000);
    return true;

  } else {
    ROS_INFO_STREAM("missing goal or start");
    return false;
  }


}

