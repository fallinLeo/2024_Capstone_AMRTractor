#include "algorithm.h"

#include <boost/heap/binomial_heap.hpp>
#include <boost/heap/fibonacci_heap.hpp>

using namespace HybridAStar;

float aStar(Node2D& start, Node2D& goal, Node2D* nodes2D, int width, int height, CollisionDetection& configurationSpace, Visualize& visualization, Node2D& hagoal);
void updateH(Node3D& start, const Node3D& goal, Node2D* nodes2D, float* dubinsLookup, int width, int height, CollisionDetection& configurationSpace, Visualize& visualization, const Node3D& hastart);
Node3D* dubinsShot(Node3D& start, const Node3D& goal, CollisionDetection& configurationSpace,std::vector<Node3D*> &dubins_pointers);
Node3D* reedsSheppShot(Node3D& start, const Node3D& goal, CollisionDetection& configurationSpace,std::vector<Node3D*> &dubins_pointers);
bool is_forward_backward(const Node3D* pred, const Node3D* curr);

ros::Duration astar_t(0.0);
ros::Duration dubins_t(0.0);
ros::Duration node3d_t(0.0);
boost::heap::fibonacci_heap<Node2D*, boost::heap::compare<CompareNodes2>> O2d;


//###################################################
//                                        3D A*
//###################################################
Node3D* Algorithm::hybridAStar(Node3D& start,
                               const Node3D& goal,
                               Node3D* nodes3D,
                               Node2D* nodes2D,
                               int width,
                               int height,
                               CollisionDetection& configurationSpace,
                               float* dubinsLookup,
                               Visualize& visualization,
                               std::vector<Node3D*> &dubins_pointers,
                               std::vector<Node3D>& subgoals
                               ) {
                            
  // TEST IF OBSTACLE ON GOAL
  if (!configurationSpace.isTraversable(&goal)){
    ROS_INFO_STREAM("OBSTACLE ON GOAL NODE");
    return nullptr;
  }
  // TEST IF OBSTACLE ON START
  if (!configurationSpace.isTraversable(&start)){
    ROS_INFO_STREAM("OBSTACLE ON START NODE");
    return nullptr;
  }

  astar_t = ros::Duration(0.0);
  dubins_t = ros::Duration(0.0);
  node3d_t = ros::Duration(0.0);
  ros::Time t0,t1;
  t0 = ros::Time::now();
  O2d.clear();
  for (int i = 0; i < width * height; ++i) {
    nodes2D[i].reset();
  }
  for (int i = 0; i < width * height * Constants::headings; ++i) {
    nodes3D[i].reset();
  }
  t1 = ros::Time::now();
  ros::Duration d1(t1 - t0);
  ROS_INFO_STREAM("RESET NODES IN : " << d1*1000 << " [ms]" );
  
  // PREDECESSOR AND SUCCESSOR INDEX
  int iPred, iSucc, iSubg;
  float newG;
  // Number of possible directions, 3 for forward driving and an additional 3 for reversing
  int dir = Constants::reverse ? Constants::directions*2 : Constants::directions;
  // Number of iterations the algorithm has run for stopping based on Constants::iterations
  int iterations = 0;

  // VISUALIZATION DELAY
  // ros::Duration d(0.3);
  ros::Duration d(0.003);

  // OPEN LIST AS BOOST IMPLEMENTATION
  typedef boost::heap::fibonacci_heap<Node3D*, boost::heap::compare<CompareNodes3>> priorityQueue;
  priorityQueue open_set;
  open_set.clear();

  // update h value
  updateH(start, goal, nodes2D, dubinsLookup, width, height, configurationSpace, visualization, start);
  // mark start as open
  start.open();

  // push on priority queue aka open list
  open_set.push(&start);
  iPred = start.setIdx(width, height);
  nodes3D[iPred] = start;

  // NODE POINTER 
  Node3D* nPred;
  Node3D* nSucc;
  std::vector<Node3D>::iterator subgiter;
  // float max = 0.f;

  // copy x,y,theta from subgoal(dynamicvoronoi) to nodes3d(planner)
  if (Constants::voronoibased && subgoals.size()!=0 ) {
    for(subgiter = subgoals.begin(); subgiter != subgoals.end(); subgiter ++){
      //iSubg = subgiter->setIdx(width, height);
      updateH(*subgiter, goal, nodes2D, dubinsLookup, width, height, configurationSpace, visualization, start);
    }
  }
  
  // continue until open_set empty
  while (!open_set.empty()) {
    // pop node with lowest cost from priority queue
    nPred = open_set.top();
    // set index
    iPred = nPred->setIdx(width, height);
    iterations++;

    // RViz visualization
    if (Constants::visualization) {
      visualization.publishNode3DPoses(*nPred);
      visualization.publishNode3DPose(*nPred);
      if(!ros::ok())
        return nullptr;
      d.sleep();
    }

    // _____________________________
    // LAZY DELETION of rewired node
    // if there exists a pointer this node has already been expanded
    if (nodes3D[iPred].isClosed()) {
      // pop node from the open list and start with a fresh node
      open_set.pop();
      continue;
    }
    // _________________
    // EXPANSION OF NODE
    else if (nodes3D[iPred].isOpen()) {
      // add node to closed list
      nodes3D[iPred].close();
      // remove node from open list
      open_set.pop();
      // printf("POP NODE x : %5.1f y : %5.1f   g cost : %7.3f   h cost : %7.3f   t cost : %7.3f\n",
        //  nodes3D[iPred].getX(),nodes3D[iPred].getY(),nodes3D[iPred].getG(),nodes3D[iPred].getH(),nodes3D[iPred].getC());
      // _________
      // GOAL TEST
      if (*nPred == goal || iterations > Constants::iterations) {
        // DEBUG
        if (iterations > Constants::iterations){
          std::cout<<"  iterations !!"<<iterations <<std::endl;
          nPred = nullptr;
        }
        else{
          std::cout<<"  nPred is goal !!"<<iterations <<std::endl;
        }
        std::cout<<"  Expand Nodes inFoward Search : "<<iterations <<std::endl;

        return nPred;
      }
      // ____________________
      // CONTINUE WITH SEARCH
      else {
        // _______________________
        // SEARCH WITH dubinsShot ot reedsSheppShot
        t0 = ros::Time::now();
        if ((!Constants::useReedsSheppShot) && nPred->isInRange(goal) && nPred->getForward()) {
          nSucc = dubinsShot(*nPred, goal, configurationSpace,dubins_pointers);
          if (nSucc != nullptr && *nSucc == goal) {
            std::cout<<"  Expand Nodes inFoward Search : "<<iterations <<std::endl;
            std::cout<<"  2D atar duration : "<<astar_t*1000<< " [ms]" <<std::endl;
            std::cout<<"  dubins_t duration : "<<dubins_t*1000<< " [ms]" <<std::endl;
            std::cout<<"  node expansion duration : "<<node3d_t*1000<< " [ms]" <<std::endl;
            return nSucc;
          }
        }
        if (Constants::useReedsSheppShot && nPred->isInRange(goal) ) {
          nSucc = reedsSheppShot(*nPred, goal, configurationSpace,dubins_pointers);
          if (nSucc != nullptr && *nSucc == goal) {
            std::cout<<"  Expand Nodes inFoward Search : "<<iterations <<std::endl;
            std::cout<<"  2D atar duration : "<<astar_t*1000<< " [ms]" <<std::endl;
            std::cout<<"  dubins_t duration : "<<dubins_t*1000<< " [ms]" <<std::endl;
            std::cout<<"  node expansion duration : "<<node3d_t*1000<< " [ms]" <<std::endl;
            return nSucc;
          }
        }

        // SEARCH WITH subgoals of voronoi diagram
        bool valid_sub_goal = false;
        if (Constants::voronoibased && subgoals.size()!=0 ) {
          for(subgiter = subgoals.begin(); subgiter != subgoals.end(); subgiter ++){
            iSubg = subgiter->setIdx(width, height);
            Node3D subgNode = *subgiter;

            // 루프방지, 자기보다 뒤에있는 subgoal과의 연결 방지
            if (*nPred == subgNode || nPred->getH() < subgNode.getH() )
              continue;
            
            // goal에 너무 가까운 subgoal은 확장 금지
            if(subgNode.isInRange(goal, false))
              continue;

            // subgoal으로 한번 확장되었으면 확장금지(루프 방지)
            if(subgNode.isOpen() || subgNode.isClosed())
              continue;


            if( Constants::useReedsSheppShot && ( !nPred->getForward() || *nPred == start )){
              nSucc = reedsSheppShot(*nPred, subgNode, configurationSpace, dubins_pointers);
            }
            else{
              nSucc = dubinsShot(*nPred, subgNode, configurationSpace, dubins_pointers);
            }


            if (nSucc != nullptr && *nSucc == subgNode) {
              // std::cout<<"SUBGOAL SUCCESS!"<<std::endl;
              subgNode.setPred(nSucc->getPred());
              // if gcost is same as nPred
              subgNode.setG(nPred->getG());

              // if gcost is zero;
              // subgNode.setG(0.0);

              // if gcost is nPred+dubins length?
              // subgNode.setG(nSucc->getG());

              subgNode.open();
              nodes3D[iSubg] = subgNode;
              open_set.push(&nodes3D[iSubg]);
              valid_sub_goal = true;
              break;
            }
          }
        }

        t1 = ros::Time::now();
        dubins_t += t1 - t0;

        if (valid_sub_goal)
          continue;



        t0 = ros::Time::now();
        // ______________________________
        // SEARCH WITH FORWARD SIMULATION
        for (int i = 0; i < dir; i++) {
          // create possible successor
          nSucc = nPred->createSuccessor(i);
          // set index of the successor
          iSucc = nSucc->setIdx(width, height);
          // ensure successor is on grid and traversable
          if (nSucc->isOnGrid(width, height) && configurationSpace.isTraversable(nSucc)) {

            // ensure successor is not on closed list or it has the same index as the predecessor
            if (!nodes3D[iSucc].isClosed() || iPred == iSucc) {

              // calculate new G value
              nSucc->updateG();
              newG = nSucc->getG();

              // if successor not on open list or found a shorter way to the cell
              if (!nodes3D[iSucc].isOpen() || newG < nodes3D[iSucc].getG() || iPred == iSucc) {
                t1 = ros::Time::now();
                node3d_t += t1 - t0;
                // calculate H value
                updateH(*nSucc, goal, nodes2D, dubinsLookup, width, height, configurationSpace, visualization, start);
                t0 = ros::Time::now();
                // if the successor is in the same cell but the C value is larger
                if (iPred == iSucc && nSucc->getC() > nPred->getC() + Constants::tieBreaker) {
                  delete nSucc;
                  continue;
                }
                // if successor is in the same cell and the C value is lower, set predecessor to predecessor of predecessor
                else if (iPred == iSucc && nSucc->getC() <= nPred->getC() + Constants::tieBreaker) {
                  nSucc->setPred(nPred->getPred());
                }

                if (nSucc->getPred() == nSucc) {
                  std::cout << "looping";
                }

                // put successor on open list
                nSucc->open();
                nodes3D[iSucc] = *nSucc;
                open_set.push(&nodes3D[iSucc]);
                delete nSucc;
              } else { delete nSucc; }
            } else {  delete nSucc; }
          } else {  delete nSucc; }
        }

        t1 = ros::Time::now();
        node3d_t += t1 - t0;
      }
    }
  }

  if (open_set.empty()) {
    std::cout<<"Open nodes are empty!! iters :"<<iterations <<std::endl;
    return nullptr;
  }
  std::cout<<"Something gone wrong"<<iterations <<std::endl;
  return nullptr;
}

//###################################################
//                                        2D A*
//###################################################
float aStar(Node2D& start,
            Node2D& goal,
            Node2D* nodes2D,
            int width,
            int height,
            CollisionDetection& configurationSpace,
            Visualize& visualization,
            Node2D& hagoal 
                       ) {

  // PREDECESSOR AND SUCCESSOR INDEX
  int iPred, iSucc;
  float newG;

  // VISUALIZATION DELAY
  ros::Duration d(0.003);


  // update h value
  if(O2d.empty()){
    start.updateH(hagoal);
    // mark start as open
    start.open();
    // push on priority queue
    O2d.push(&start);
    iPred = start.setIdx(width);
    nodes2D[iPred] = start;
  }

  // NODE POINTER
  Node2D* nPred;
  Node2D* nSucc;

  
  // continue until open_set empty
  while (!O2d.empty()) {
    // pop node with lowest cost from priority queue
    nPred = O2d.top();
    // set index
    iPred = nPred->setIdx(width);

    // _____________________________
    // LAZY DELETION of rewired node
    // if there exists a pointer this node has already been expanded
    if (nodes2D[iPred].isClosed()) {
      // pop node from the open list and start with a fresh node
      O2d.pop();
      continue;
    }
    // _________________
    // EXPANSION OF NODE
    else if (nodes2D[iPred].isOpen()) {
      // add node to closed list
      nodes2D[iPred].close();
      nodes2D[iPred].discover();
      // printf("POP NODE x : %3d y : %3d   g cost : %7.3f   h cost : %7.3f   t cost : %7.3f\n",
      //     nodes2D[iPred].getX(),nodes2D[iPred].getY(),nodes2D[iPred].getG(),nodes2D[iPred].getH(),nodes2D[iPred].getC());

      // RViz visualization
      if (Constants::visualization2D) {
        visualization.publishNode2DPoses(*nPred);
        visualization.publishNode2DPose(*nPred);
        d.sleep();
      }

      // remove node from open list
      O2d.pop();

      // _________
      // GOAL TEST
      if (*nPred == goal) {
        return nPred->getG();
      }
      // ____________________
      // CONTINUE WITH SEARCH
      else {
        // _______________________________
        // CREATE POSSIBLE SUCCESSOR NODES
        for (int i = 0; i < Node2D::dir; i++) {
          // create possible successor
          nSucc = nPred->createSuccessor(i);
          // set index of the successor
          iSucc = nSucc->setIdx(width);

          // ensure successor is on grid ROW MAJOR
          // ensure successor is not blocked by obstacle
          // ensure successor is not on closed list
          if (nSucc->isOnGrid(width, height) &&  configurationSpace.isTraversable(nSucc) && !nodes2D[iSucc].isClosed()) {
            // calculate new G value
            nSucc->updateG();
            newG = nSucc->getG();

            // if successor not on open list or g value lower than before put it on open list
            if (!nodes2D[iSucc].isOpen() || newG < nodes2D[iSucc].getG()) {
              // calculate the H value
              nSucc->updateH(hagoal);

              // put successor on open list
              nSucc->open();
              nodes2D[iSucc] = *nSucc;
              O2d.push(&nodes2D[iSucc]);
              delete nSucc;
            } else { delete nSucc; }
          } else { delete nSucc; }
        }
      }
    }
  }

  // return large number to guide search away
  return 1000;
}

//###################################################
//                                         COST TO GO
//###################################################
void updateH(Node3D& start, const Node3D& goal, Node2D* nodes2D, float* dubinsLookup, int width, int height, CollisionDetection& configurationSpace, Visualize& visualization, const Node3D& hastart) {
  float dubinsCost = 0;
  float reedsSheppCost = 0;
  float twoDCost = 0;
  float twoDoffset = 0;

  if(start.isInRange(goal)){
    ros::Time t0 = ros::Time::now();
    // if dubins heuristic is activated calculate the shortest path
    // constrained without obstacles
    if (!Constants::reverse) {
      ompl::base::DubinsStateSpace dubinsPath(Constants::r);
      State* dbStart = (State*)dubinsPath.allocState();
      State* dbEnd = (State*)dubinsPath.allocState();
      dbStart->setXY(start.getX(), start.getY());
      dbStart->setYaw(start.getT());
      dbEnd->setXY(goal.getX(), goal.getY());
      dbEnd->setYaw(goal.getT());
      dubinsCost = dubinsPath.distance(dbStart, dbEnd);
      dubinsPath.freeState(dbStart);
      dubinsPath.freeState(dbEnd);
    }

    // if reversing is active use a
    if (Constants::reverse ) {
      ompl::base::ReedsSheppStateSpace reedsSheppPath(Constants::r);
      State* rsStart = (State*)reedsSheppPath.allocState();
      State* rsEnd = (State*)reedsSheppPath.allocState();
      rsStart->setXY(start.getX(), start.getY());
      rsStart->setYaw(start.getT());
      rsEnd->setXY(goal.getX(), goal.getY());
      rsEnd->setYaw(goal.getT());
      reedsSheppCost = reedsSheppPath.distance(rsStart, rsEnd);
      reedsSheppPath.freeState(rsStart);
      reedsSheppPath.freeState(rsEnd);
      ros::Time t1 = ros::Time::now();
      ros::Duration d(t1 - t0);
      dubins_t += d;
      //    std::cout << "calculated Reed-Sheep Heuristic in ms: " << d * 1000 << std::endl;
    }
  }
  

  // if twoD heuristic is activated determine shortest path
  // unconstrained with obstacles
  if (Constants::twoD && !nodes2D[(int)start.getY() * width + (int)start.getX()].isDiscovered()) {
    ros::Time t0 = ros::Time::now();
    // create a 2d start node
    Node2D start2d(start.getX(), start.getY(), 0, 0, nullptr);
    // create a 2d goal node
    Node2D goal2d(goal.getX(), goal.getY(), 0, 0, nullptr);
    // run 2d astar and return the cost of the cheapest path for that node
    Node2D hastart2d(hastart.getX(), hastart.getY(), 0, 0, nullptr);
    nodes2D[(int)start.getY() * width + (int)start.getX()].setG(aStar(goal2d, start2d, nodes2D, width, height, configurationSpace, visualization, hastart2d));

    ros::Time t1 = ros::Time::now();
    ros::Duration d(t1 - t0);
    astar_t += d;
    //    std::cout << "calculated 2D Heuristic in ms: " << d * 1000 << std::endl;
  }


  if (Constants::twoD) {
    // offset for same node in cell
    twoDoffset = sqrt(((start.getX() - (long)start.getX()) - (goal.getX() - (long)goal.getX())) * ((start.getX() - (long)start.getX()) - (goal.getX() - (long)goal.getX())) +
                      ((start.getY() - (long)start.getY()) - (goal.getY() - (long)goal.getY())) * ((start.getY() - (long)start.getY()) - (goal.getY() - (long)goal.getY())));
    twoDCost = nodes2D[(int)start.getY() * width + (int)start.getX()].getG() - twoDoffset;
    // printf("twoDoffset : %f gcost : %f twoDcost : %f", twoDcost, )

  }

  // return the maximum of the heuristics, making the heuristic admissable
  start.setH(std::max(reedsSheppCost, std::max(dubinsCost, twoDCost)));
}


//###################################################
//                                        DUBINS SHOT
//###################################################
Node3D* dubinsShot(Node3D& start, const Node3D& goal, CollisionDetection& configurationSpace,std::vector<Node3D*> &dubins_pointers) {
  ompl::base::DubinsStateSpace DubinsPath(Constants::r);
  State* rsStart = (State*)DubinsPath.allocState();
  State* rsEnd = (State*)DubinsPath.allocState();
  // start
  rsStart->setXY(start.getX(), start.getY());
  rsStart->setYaw(start.getT());
  // goal
  rsEnd->setXY(goal.getX(), goal.getY());
  rsEnd->setYaw(goal.getT());

  int i = 0;
  float length = DubinsPath.distance(rsStart, rsEnd);
  if (length <= Constants::dubinsStepSize){
    DubinsPath.freeState(rsStart);
    DubinsPath.freeState(rsEnd);
    return nullptr;
  }
  float prop = 1.0 / (length/Constants::dubinsStepSize);

  Node3D* dubinsNodes = new Node3D [(int)(length/Constants::dubinsStepSize)+2]; // 시작노드 포함 안함
  float p=prop;
  bool last = false;
  // for(float p=prop; p<=1.0; p+=prop){
  while(!last){
    if(p>=1.0){
      p = 1.0;
      last = true;
    }

    State* middle = (State*)DubinsPath.allocState();
    DubinsPath.interpolate(rsStart,rsEnd, p, middle);
    dubinsNodes[i].setX(middle->getX());
    dubinsNodes[i].setY(middle->getY());
    dubinsNodes[i].setT(Helper::normalizeHeadingRad(middle->getYaw()));
    DubinsPath.freeState(middle);
    // collision check
    if (configurationSpace.isTraversable(&dubinsNodes[i])) {

      // set the predecessor to the previous step
      if (i > 0) {
        dubinsNodes[i].setPred(&dubinsNodes[i - 1]);
      } else {
        dubinsNodes[i].setPred(&start);
      }
      dubinsNodes[i].setHT(dubinsNodes[i].calcHtfromPred());

      if (&dubinsNodes[i] == dubinsNodes[i].getPred()) {
        std::cout << "looping shot";
      }
    } else {
      //      std::cout << "Dubins shot collided, discarding the path" << "\n";
      // delete all nodes
      DubinsPath.freeState(rsStart);
      DubinsPath.freeState(rsEnd);
      delete [] dubinsNodes;
      return nullptr;
    }
    i++;
    p+=prop;
  }

  DubinsPath.freeState(rsStart);
  DubinsPath.freeState(rsEnd);



  dubinsNodes[i - 1].setG(start.getG()+length);
  dubins_pointers.push_back(dubinsNodes);
  // printf("DU dubins pointers push back!\n");
  // std::cout<<"dx "<<std::endl;
  // printf("goal x : %f y : %f t : %f \n",goal.getX(),goal.getY(),goal.getT());
  // printf("dsds x : %f y : %f t : %f \n",dubinsNodes[i - 1].getX(),dubinsNodes[i - 1].getY(),dubinsNodes[i - 1].getT());

  return &dubinsNodes[i - 1];
}


//###################################################
//                                    REEDSSHEPP SHOT
//###################################################
Node3D* reedsSheppShot(Node3D& start, const Node3D& goal, CollisionDetection& configurationSpace,std::vector<Node3D*> &dubins_pointers){
  ompl::base::ReedsSheppStateSpace reedsSheppPath(Constants::r);
  State* rsStart = (State*)reedsSheppPath.allocState();
  State* rsEnd = (State*)reedsSheppPath.allocState();
  // start
  rsStart->setXY(start.getX(), start.getY());
  rsStart->setYaw(start.getT());
  // goal
  rsEnd->setXY(goal.getX(), goal.getY());
  rsEnd->setYaw(goal.getT());

  int i = 0;
  float length = reedsSheppPath.distance(rsStart, rsEnd);
  if (length <= Constants::dubinsStepSize){
    reedsSheppPath.freeState(rsStart);
    reedsSheppPath.freeState(rsEnd);
    return nullptr;
  }
  float prop = 1.0 / (length/Constants::dubinsStepSize);

  Node3D* reedSheppNodes = new Node3D [(int)(length/Constants::dubinsStepSize)+2]; // 시작노드 포함 안함
  float p=prop;
  bool last = false;
  // for(float p=prop; p<=1.0; p+=prop){
  while(!last){
    if(p>=1.0){
      p = 1.0;
      last = true;
    }

    State* middle = (State*)reedsSheppPath.allocState();
    reedsSheppPath.interpolate(rsStart,rsEnd, p, middle);
    reedSheppNodes[i].setX(middle->getX());
    reedSheppNodes[i].setY(middle->getY());
    reedSheppNodes[i].setT(Helper::normalizeHeadingRad(middle->getYaw()));
    reedsSheppPath.freeState(middle);
    // collision check
    if (configurationSpace.isTraversable(&reedSheppNodes[i])) {

      // set the predecessor to the previous step
      if (i > 0) {
        reedSheppNodes[i].setPred(&reedSheppNodes[i - 1]);
        reedSheppNodes[i].setForward(is_forward_backward(&reedSheppNodes[i-1], &reedSheppNodes[i]));
      } else {
        reedSheppNodes[i].setPred(&start);
        reedSheppNodes[i].setForward(is_forward_backward(&start, &reedSheppNodes[i]));
      }
      reedSheppNodes[i].setHT(reedSheppNodes[i].calcHtfromPred());

      if (&reedSheppNodes[i] == reedSheppNodes[i].getPred()) {
        std::cout << "looping shot";
      }
    } else {
      //      std::cout << "Dubins shot collided, discarding the path" << "\n";
      // delete all nodes
      reedsSheppPath.freeState(rsStart);
      reedsSheppPath.freeState(rsEnd);
      delete [] reedSheppNodes;
      return nullptr;
    }
    i++;
    p+=prop;
  }



  reedsSheppPath.freeState(rsStart);
  reedsSheppPath.freeState(rsEnd);
  //  std::cout << "Dubins shot connected, returning the path" << "\n";
  if ( start.getPred() == nullptr && i >= 2){ // 바로 rscurve가 생성되는 경우 예외처리
    reedSheppNodes[0].setPrim(reedSheppNodes[1].getPrim());
    reedSheppNodes[0].setForward(reedSheppNodes[1].getForward());
  }

  reedSheppNodes[i - 1].setG(start.getG()+length);
  dubins_pointers.push_back(reedSheppNodes);
  // printf("RS dubins pointers push back!\n");
  // std::cout<<"rs "<<std::endl;
  // printf("goal x : %f y : %f t : %f \n",goal.getX(),goal.getY(),goal.getT());
  // printf("rsrs x : %f y : %f t : %f \n",reedSheppNodes[i - 1].getX(),reedSheppNodes[i - 1].getY(),reedSheppNodes[i - 1].getT());

  return &reedSheppNodes[i - 1];
}


// return 1 if forwad return 4 if backward -> node.prim
// it will be used in path.cpp/addSegment/directionlist
bool is_forward_backward(const Node3D* pred, const Node3D* curr){
  float x1 = ( curr->getX() - pred->getX() );
  float y1 = ( curr->getY() - pred->getY() );
  float x2 = cos(curr->getT());
  float y2 = sin(curr->getT());
  return (( x1*x2 + y1*y2 )>0 );
}