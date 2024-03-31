/** include the libraries you need in your planner here */
/** for global path planner interface */
#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_core/base_global_planner.h>
#include <geometry_msgs/PoseStamped.h>
#include <angles/angles.h>
#include <base_local_planner/world_model.h>
#include <base_local_planner/costmap_model.h>
#include <tf/transform_datatypes.h>
#include <ros/types.h>
#include <boost/shared_ptr.hpp>
#include <dynamic_reconfigure/server.h>
#include <hybrid_astar/constantsConfig.h>

// #include "nodelet/nodelet.h"


#include <iostream>
#include "constants.h"
#include "planner.h"
#include <cmath>



using std::string;

#ifndef GLOBAL_PLANNER_CPP
#define GLOBAL_PLANNER_CPP

namespace HybridAStar {

    class GlobalHAPlanner : public nav_core::BaseGlobalPlanner {
    public:

        GlobalHAPlanner();
        GlobalHAPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros);
        ~GlobalHAPlanner();

        /** overridden classes from interface nav_core::BaseGlobalPlanner **/
        void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);
        bool makePlan(const geometry_msgs::PoseStamped& start,
                    const geometry_msgs::PoseStamped& goal,
                    std::vector<geometry_msgs::PoseStamped>& plan
                    );
        void dynamicReconfigCb(hybrid_astar::constantsConfig &c, uint32_t level);
        void cellSizeChanged(float cellSize);

    protected:
        bool initialized_;

    private:
        costmap_2d::Costmap2DROS* costmap_ros_;
        costmap_2d::Costmap2D* costmap_;

        // Publishers
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
        /// Publisher for Voronoi diagram edges in costmap
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

        Planner planner;
        
        nav_msgs::OccupancyGrid::Ptr mapptr;
        nav_msgs::OccupancyGrid map;

        dynamic_reconfigure::Server<hybrid_astar::constantsConfig> *server;



    };
};
#endif
