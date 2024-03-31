#include <pluginlib/class_list_macros.h>
#include "global_plugin.h"

//register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(HybridAStar::GlobalHAPlanner, nav_core::BaseGlobalPlanner)
const unsigned char option1 = 1 << 0; // 0000 0001 
const unsigned char option2 = 1 << 1; // 0000 0010
const unsigned char option4 = 1 << 2; // 0000 0100

using namespace std;

//Default Constructors
namespace HybridAStar {


namespace Constants{
    // Constants.h

    // ros params
    bool simul = false;  
    bool reverse = true;     
    bool voronoibased = true;  
    bool useReedsSheppShot = false;
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



    GlobalHAPlanner::GlobalHAPlanner () : initialized_(false)
    {}

    GlobalHAPlanner::GlobalHAPlanner(std::string name,costmap_2d::Costmap2DROS* costmap_ros) :
    initialized_(false)
    {
        initialize(name, costmap_ros);
    }

    GlobalHAPlanner::~GlobalHAPlanner(){
        if(server)
            delete server;
    }


    void GlobalHAPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros){
        if(initialized_){
            ROS_WARN("This planner has already been initialized, you can't call it twice, doing nothing");
            return;
        }
        ros::NodeHandle n("~/" + name);

        // __________________________
        // ROS PARAM
        n.param<bool>("reverse",HybridAStar::Constants::reverse, true);
        n.param<bool>("voronoibased",HybridAStar::Constants::voronoibased, true);
        n.param<bool>("useReedsSheppShot",HybridAStar::Constants::useReedsSheppShot, true);

        // __________________________
        // MAP RECEIVE
        costmap_ros_ = costmap_ros;
        costmap_ = costmap_ros_->getCostmap();
        HybridAStar::Constants::cellSize = costmap_->getResolution();
        map.info.origin.orientation.x = 0.0;
        map.info.origin.orientation.y = 0.0;
        map.info.origin.orientation.z = 0.0;
        map.info.origin.orientation.w = 1.0;

        // __________________________
        // DYNAMIC RECONFIGURE SERVER
        server = new dynamic_reconfigure::Server<hybrid_astar::constantsConfig>(ros::NodeHandle("~/" + name));
        dynamic_reconfigure::Server<hybrid_astar::constantsConfig>::CallbackType f;
        f = boost::bind(&GlobalHAPlanner::dynamicReconfigCb, this , _1, _2);

        // need c++14 over
        // dynamic_reconfigure::Server<hybrid_astar::constantsConfig>::CallbackType f =
        //         [this](auto& config, auto level){ dynamicReconfigCb(config, level); };
        server->setCallback(f);
        // __________________________
        // PUBLISHERS
        // Path
        // pubPath = n.advertise<nav_msgs::Path>("/path", 1, true);
        pubPath = n.advertise<nav_msgs::Path>("path", 1);
        pubPathTrailer = n.advertise<nav_msgs::Path>("pathTrailer", 1);
        pubPathNodes = n.advertise<visualization_msgs::MarkerArray>("pathNodes", 1, true);
        pubPathVehicles = n.advertise<visualization_msgs::MarkerArray>("pathVehicle", 1, true);
        // Smoothed Path
        spubPath = n.advertise<nav_msgs::Path>("sPath", 1, true);
        spubPathTrailer = n.advertise<nav_msgs::Path>("sPathTrailer", 1, true);
        spubPathNodes = n.advertise<visualization_msgs::MarkerArray>("sPathNodes", 1, true);
        spubPathVehicles = n.advertise<visualization_msgs::MarkerArray>("sPathVehicle", 1, true);
        // Voronoi diagram
        pubSubGoal =  n.advertise<visualization_msgs::MarkerArray>("subGoals", 1, true);
        pubVoroCost =  n.advertise<nav_msgs::OccupancyGrid>("voroCostmap", 1, true);
        
        // set changed configurations
        // change voronoi diagram costmap
        if(planner.voronoiReconfigure()){
            pubVoroCost.publish(planner.getvoroCostmap());
        }
        // change collision lookup
        planner.collisionReconfigure();
        initialized_ = true;
    }

    void GlobalHAPlanner::dynamicReconfigCb(hybrid_astar::constantsConfig &c, uint32_t level) {
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
        HybridAStar::Constants::dubinsShotDistance = HybridAStar::Constants::dubinsShotDistanceRaw * HybridAStar::Constants::dubinsShotDistanceRaw / HybridAStar::Constants::cellSize;        HybridAStar::Constants::voroObsDMax = c.opti_voroObsDMax;
        HybridAStar::Constants::alpha = c.opti_alpha;
        HybridAStar::Constants::wObstacle = c.opti_wObstacle;
        HybridAStar::Constants::wVoronoi = c.opti_wVoronoi;
        HybridAStar::Constants::wCurvature = c.opti_wCurvature;
        HybridAStar::Constants::wSmoothness = c.opti_wSmoothness;
        HybridAStar::Constants::maxIterations = c.opti_maxIterations;
        HybridAStar::Constants::directions = c.directions;
        HybridAStar::Constants::setConfig();
        // ROS_INFO("LEVEL : %d, masked?: %d %d",level, level & option2, level & option4);
        // change voronoi diagram
        if(level & option2){
            if(planner.voronoiReconfigure()){
            pubVoroCost.publish(planner.getvoroCostmap());
            }
        }
        // change collision lookup
        if(level & option4){
            planner.collisionReconfigure();
        }
    }

    void GlobalHAPlanner::cellSizeChanged(float cellSize){
        HybridAStar::Constants::setCellSize(cellSize);
        // setmap에서 Voronoi 만들고 publish한다.
        planner.collisionReconfigure();
    }


    bool GlobalHAPlanner::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,  std::vector<geometry_msgs::PoseStamped>& plan ){
        plan.clear();
        //turn cost map to nav_msgs::OccupancyGrid
        //set map to planner 
        costmap_ = costmap_ros_->getCostmap();
        unsigned char* grid = costmap_->getCharMap();
        map.info.width = costmap_->getSizeInCellsX();
        map.info.height = costmap_->getSizeInCellsY();
        map.info.origin.position.x = costmap_->getOriginX();
        map.info.origin.position.y = costmap_->getOriginY();
        if(map.info.resolution != costmap_->getResolution()){
            map.info.resolution = costmap_->getResolution();
            cellSizeChanged(costmap_->getResolution());
        }
        

        std::vector<int8_t> map_data_;
        map_data_.clear();
        for(unsigned int i=0; i<map.info.width*map.info.height; i++)
        {
            int cost = grid[i];
            if(cost==255){ // unknown
                map_data_.push_back(-1);
            }
            else if(cost==254){ // lethal
                map_data_.push_back(100);
            }
            else if(cost==253){ // inscribed
                map_data_.push_back(99);
            }
            else{ // others
                map_data_.push_back((int)(cost/25.5)); // 0~255 -> 0~100
            }
        }
        map.data = map_data_;
        mapptr = boost::make_shared<nav_msgs::OccupancyGrid>(map);
        // planner.setMap(nav_msgs::OccupancyGrid::Ptr(&map));
        planner.setMap(mapptr);
        pubVoroCost.publish(planner.getvoroCostmap());


        //set start and goal to planner
        planner.setGoal(goal);
        planner.setStart(start);

        if(!planner.plan())
            return false;
        
        pubPath.publish(planner.getPath(false));
        pubPathTrailer.publish(planner.getPathTrailer(false));
        pubPathNodes.publish(planner.getPathNodes(false));
        pubPathVehicles.publish(planner.getPathVehicles(false));
        spubPath.publish(planner.getPath(true));
        spubPathTrailer.publish(planner.getPathTrailer(true));
        spubPathNodes.publish(planner.getPathNodes(true));
        spubPathVehicles.publish(planner.getPathVehicles(true));
        pubSubGoal.publish(planner.getsubGoalMarkerArray());
        
        nav_msgs::Path& tmp = planner.getPath(true);
        std::vector<geometry_msgs::PoseStamped>::reverse_iterator tmpiter(tmp.poses.rbegin());
        for (; tmpiter != tmp.poses.rend() ; ++tmpiter)
            plan.push_back(*tmpiter);
        map_data_.clear();
        return true;
    }
};