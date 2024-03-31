#include "dynamicvoronoi.h"

#include <math.h>
#include <iostream>

using namespace HybridAStar;

DynamicVoronoi::DynamicVoronoi()
  {
  sqrt2 = sqrt(2.0);
  data = NULL;
  gridMap = NULL;
}

DynamicVoronoi::~DynamicVoronoi() {
  if (data) {
    for (int x=0; x<sizeX; x++) delete[] data[x];
    delete[] data;
  }
  if (gridMap) {
    for (int x=0; x<sizeX; x++) delete[] gridMap[x];
    delete[] gridMap;
  }
}

void DynamicVoronoi::initializeEmpty(int _sizeX, int _sizeY, bool initGridMap) {
  sizeX = _sizeX;
  sizeY = _sizeY;
  if (data) {
    for (int x=0; x<sizeX; x++) delete[] data[x];
    delete[] data;
  }
  data = new dataCell*[sizeX];
  for (int x=0; x<sizeX; x++) data[x] = new dataCell[sizeY];

  if (initGridMap) {
    if (gridMap) {
      for (int x=0; x<sizeX; x++) delete[] gridMap[x];
      delete[] gridMap;
    }
    gridMap = new bool*[sizeX];
    for (int x=0; x<sizeX; x++) gridMap[x] = new bool[sizeY];
  }
  
  dataCell c;
  c.dist = INFINITY;
  c.sqdist = INT_MAX;
  c.edgesqdist = INT_MAX;
  c.edgedist = INFINITY;
  c.obstX = invalidObstData;
  c.obstY = invalidObstData;
  c.voronoi = free;
  c.queueing = fwNotQueued;
  c.needsRaise = false;
  c.edgeX = invalidObstData;
  c.edgeY = invalidObstData;

  for (int x=0; x<sizeX; x++)
    for (int y=0; y<sizeY; y++) data[x][y] = c;

  if (initGridMap) {
    for (int x=0; x<sizeX; x++) 
      for (int y=0; y<sizeY; y++) gridMap[x][y] = 0;
  }
}

void DynamicVoronoi::initializeMap(int _sizeX, int _sizeY, bool** _gridMap, float _originX, float _originY) {
  initializeEmpty(_sizeX, _sizeY, true);
  if(gridMap){ //delete before map
    for (int x=0; x<sizeX; x++) delete[] gridMap[x];
    delete[] gridMap;
  }
  gridMap = _gridMap;
  originX = _originX;
  originY = _originY;

  for (int x=0; x<sizeX; x++) {
    for (int y=0; y<sizeY; y++) {
      if (gridMap[x][y]) {
        dataCell c = data[x][y];
        if (!isOccupied(x,y,c)) {
          
          bool isSurrounded = true;
          for (int dx=-1; dx<=1; dx++) {
            int nx = x+dx;
            if (nx<=0 || nx>=sizeX-1) continue;
            for (int dy=-1; dy<=1; dy++) {
              if (dx==0 && dy==0) continue;
              int ny = y+dy;
              if (ny<=0 || ny>=sizeY-1) continue;

              if (!gridMap[nx][ny]) {
                isSurrounded = false;
                break;
              }
            }
          }
          if (isSurrounded) {
            c.obstX = x;
            c.obstY = y;
            c.sqdist = 0;
            c.dist=0;
            c.voronoi=occupied;
            c.queueing = fwProcessed;
            data[x][y] = c;
          } else setObstacle(x,y);
        }
      }
    }
  }
}

void DynamicVoronoi::occupyCell(int x, int y) {
  gridMap[x][y] = 1;
  setObstacle(x,y);
}
void DynamicVoronoi::clearCell(int x, int y) {
  gridMap[x][y] = 0;
  removeObstacle(x,y);
}

void DynamicVoronoi::setObstacle(int x, int y) {
  dataCell c = data[x][y];
  if(isOccupied(x,y,c)) return;
  
  addList.push_back(INTPOINT(x,y));
  c.obstX = x;
  c.obstY = y;
  data[x][y] = c;
}

void DynamicVoronoi::removeObstacle(int x, int y) {
  dataCell c = data[x][y];
  if(isOccupied(x,y,c) == false) return;

  removeList.push_back(INTPOINT(x,y));
  c.obstX = invalidObstData;
  c.obstY  = invalidObstData;    
  c.queueing = bwQueued;
  data[x][y] = c;
}

void DynamicVoronoi::exchangeObstacles(const std::vector<INTPOINT>& points) {

  for (unsigned int i=0; i<lastObstacles.size(); i++) {
    int x = lastObstacles[i].x;
    int y = lastObstacles[i].y;

    bool v = gridMap[x][y];
    if (v) continue;
    removeObstacle(x,y);
  }  

  lastObstacles.clear();
  lastObstacles.reserve(points.size());

  for (unsigned int i=0; i<points.size(); i++) {
    int x = points[i].x;
    int y = points[i].y;
    bool v = gridMap[x][y];
    if (v) continue;
    setObstacle(x,y);
    lastObstacles.push_back(points[i]);
  }  
}

void DynamicVoronoi::update(bool updateRealDist) {

  commitAndColorize(updateRealDist);

  while (!open.empty()) {
    INTPOINT p = open.pop();
    int x = p.x;
    int y = p.y;
    dataCell c = data[x][y];

    if(c.queueing==fwProcessed) continue; 

    if (c.needsRaise) {
      // RAISE 
      for (int dx=-1; dx<=1; dx++) {
        int nx = x+dx;
        if (nx<=0 || nx>=sizeX-1) continue;
        for (int dy=-1; dy<=1; dy++) {
          if (dx==0 && dy==0) continue;
          int ny = y+dy;
          if (ny<=0 || ny>=sizeY-1) continue;
          dataCell nc = data[nx][ny];
          if (nc.obstX!=invalidObstData && !nc.needsRaise) {
            if(!isOccupied(nc.obstX,nc.obstY,data[nc.obstX][nc.obstY])) {
              open.push(nc.sqdist, INTPOINT(nx,ny));
              nc.queueing = fwQueued;
              nc.needsRaise = true;
              nc.obstX = invalidObstData;
              nc.obstY = invalidObstData;
              if (updateRealDist) nc.dist = INFINITY;
              nc.sqdist = INT_MAX;
              data[nx][ny] = nc;
            } else {
              if(nc.queueing != fwQueued){
                open.push(nc.sqdist, INTPOINT(nx,ny));
                nc.queueing = fwQueued;
                data[nx][ny] = nc;
              }
            }      
          }
        }
      }
      c.needsRaise = false;
      c.queueing = bwProcessed;
      data[x][y] = c;
    }
    else if (c.obstX != invalidObstData && isOccupied(c.obstX,c.obstY,data[c.obstX][c.obstY])) {

      // LOWER
      c.queueing = fwProcessed;
      c.voronoi = occupied;

      for (int dx=-1; dx<=1; dx++) {
        int nx = x+dx;
        if (nx<=0 || nx>=sizeX-1) continue;
        for (int dy=-1; dy<=1; dy++) {
          if (dx==0 && dy==0) continue;
          int ny = y+dy;
          if (ny<=0 || ny>=sizeY-1) continue;
          dataCell nc = data[nx][ny];
          if(!nc.needsRaise) {
            int distx = nx-c.obstX;
            int disty = ny-c.obstY;
            int newSqDistance = distx*distx + disty*disty;		
            bool overwrite =  (newSqDistance < nc.sqdist);
            if(!overwrite && newSqDistance==nc.sqdist) { 
              if (nc.obstX == invalidObstData || isOccupied(nc.obstX,nc.obstY,data[nc.obstX][nc.obstY])==false) overwrite = true;
            }
            if (overwrite) {
              open.push(newSqDistance, INTPOINT(nx,ny));
              nc.queueing = fwQueued;
              if (updateRealDist) {
                nc.dist = sqrt((double) newSqDistance);
              }
              nc.sqdist = newSqDistance;
              nc.obstX = c.obstX;
              nc.obstY = c.obstY;
            } else { 
              checkVoro(x,y,nx,ny,c,nc);
            }
            data[nx][ny] = nc;
          }
        }
      }
    }
    data[x][y] = c;
  }


  while (!closest.empty()){
  // for(int j = 0;j<100;j++){
    INTPOINT p = closest.pop();
    int x = p.x;
    int y = p.y;
    dataCell c = data[x][y];
    for (int dx=-1; dx<=1; dx++) {
      int nx = x+dx;
      if (nx<=0 || nx>=sizeX-1) continue;
      for (int dy=-1; dy<=1; dy++) {
        if (dx==0 && dy==0) continue;
        int ny = y+dy;
        if (ny<=0 || ny>=sizeY-1) continue;
        dataCell nc = data[nx][ny];
        if(nc.edgesqdist>c.edgesqdist){
          int distx = nx-c.edgeX;
          int disty = ny-c.edgeY;
          int newSqDistance = distx*distx + disty*disty;	
          if (nc.edgesqdist > newSqDistance){
            nc.edgeX = c.edgeX;
            nc.edgeY = c.edgeY;
            nc.edgesqdist = newSqDistance;
            nc.edgedist = sqrt((double)newSqDistance);
            closest.push(nc.edgesqdist,INTPOINT(nx,ny));
          }
        }
        data[nx][ny] = nc;
      }
    }
    data[x][y] = c;
  }
}

float DynamicVoronoi::getDistance( int x, int y ) const {
  if( (x>0) && (x<sizeX) && (y>0) && (y<sizeY)) return data[x][y].dist; 
  else return -INFINITY;
}

float DynamicVoronoi::getDistanceEdge( int x, int y ) const {
  if( (x>0) && (x<sizeX) && (y>0) && (y<sizeY)) return data[x][y].edgedist; 
  else return -INFINITY;
}

bool DynamicVoronoi::isVoronoi( int x, int y ) const {
  dataCell c = data[x][y];
  return (c.voronoi==free || c.voronoi==voronoiKeep);
}

void DynamicVoronoi::commitAndColorize(bool updateRealDist) {
  // ADD NEW OBSTACLES
  for (unsigned int i=0; i<addList.size(); i++) {
    INTPOINT p = addList[i];
    int x = p.x;
    int y = p.y;
    dataCell c = data[x][y];

    if(c.queueing != fwQueued){
      if (updateRealDist) c.dist = 0;
      c.sqdist = 0;
      c.obstX = x;
      c.obstY = y;
      c.queueing = fwQueued;
      c.voronoi = occupied;
      data[x][y] = c;
      open.push(0, INTPOINT(x,y));
    }
  }

  // REMOVE OLD OBSTACLES
  for (unsigned int i=0; i<removeList.size(); i++) {
    INTPOINT p = removeList[i];
    int x = p.x;
    int y = p.y;
    dataCell c = data[x][y];

    if (isOccupied(x,y,c)==true) continue; // obstacle was removed and reinserted
    open.push(0, INTPOINT(x,y));
    if (updateRealDist) c.dist  = INFINITY;
    c.sqdist = INT_MAX;
    c.needsRaise = true;
    data[x][y] = c;
  }
  removeList.clear();
  addList.clear();
}

void DynamicVoronoi::checkVoro(int x, int y, int nx, int ny, dataCell& c, dataCell& nc) {
  if ((c.sqdist>1 || nc.sqdist>1) && nc.obstX!=invalidObstData) { 
    if (abs(c.obstX-nc.obstX) > Constants::voroSmoothWin || abs(c.obstY-nc.obstY) > Constants::voroSmoothWin ) {
      //compute dist from x,y to obstacle of nx,ny	 
      int dxy_x = x-nc.obstX;
      int dxy_y = y-nc.obstY;
      int sqdxy = dxy_x*dxy_x + dxy_y*dxy_y;
      int stability_xy = sqdxy - c.sqdist;
      if (sqdxy - c.sqdist<0) return;

      //compute dist from nx,ny to obstacle of x,y
      int dnxy_x = nx - c.obstX;
      int dnxy_y = ny - c.obstY;
      int sqdnxy = dnxy_x*dnxy_x + dnxy_y*dnxy_y;
      int stability_nxy = sqdnxy - nc.sqdist;
      if (sqdnxy - nc.sqdist <0) return;

      //which cell is added to the Voronoi diagram?
      if(stability_xy <= stability_nxy && c.sqdist>2) {
        if (c.voronoi != free) {
          c.voronoi = free;
          c.edgesqdist = 0;
          c.edgedist = 0.0;
          c.edgeX = x;
          c.edgeY = y;
          reviveVoroNeighbors(x,y);
          pruneQueue.push(INTPOINT(x,y));
          closest.push(0,INTPOINT(x,y));
        }
      }
      if(stability_nxy <= stability_xy && nc.sqdist>2) {
        if (nc.voronoi != free) {
          nc.voronoi = free;
          nc.edgesqdist = 0;
          nc.edgedist = 0.0;
          nc.edgeX = nx;
          nc.edgeY = ny;
          reviveVoroNeighbors(nx,ny);
          pruneQueue.push(INTPOINT(nx,ny));
          closest.push(0,INTPOINT(nx,ny));
        }
      }
    }
  }
}

void DynamicVoronoi::reviveVoroNeighbors(int &x, int &y) {
  for (int dx=-1; dx<=1; dx++) {
    int nx = x+dx;
    if (nx<=0 || nx>=sizeX-1) continue;
    for (int dy=-1; dy<=1; dy++) {
      if (dx==0 && dy==0) continue;
      int ny = y+dy;
      if (ny<=0 || ny>=sizeY-1) continue;
      dataCell nc = data[nx][ny];
      if (nc.sqdist != INT_MAX && !nc.needsRaise && (nc.voronoi == voronoiKeep || nc.voronoi == voronoiPrune)) {
        nc.voronoi = free;
        data[nx][ny] = nc;
        pruneQueue.push(INTPOINT(nx,ny));
      }
    }
  }
}

bool DynamicVoronoi::isOccupied(int x, int y) const {
  dataCell c = data[x][y];
  return (c.obstX==x && c.obstY==y);
}

bool DynamicVoronoi::isOccupied(int &x, int &y, dataCell &c) { 
  return (c.obstX==x && c.obstY==y);
}

void DynamicVoronoi::visualize(const char *filename) {
// make cost map and save voro edges to voroList
  voroList.clear();
  voroCostmap.data.clear();
  voroCostmap.header.frame_id = "map";
  voroCostmap.header.stamp = ros::Time();
  voroCostmap.info.resolution = Constants::cellSize;
  voroCostmap.info.width  = sizeX;
  voroCostmap.info.height = sizeY;
  voroCostmap.info.origin.position.x = originX;
  voroCostmap.info.origin.position.y = originY;
  voroCostmap.info.origin.orientation.x = 0.0;
  voroCostmap.info.origin.orientation.y = 0.0;
  voroCostmap.info.origin.orientation.z = 0.0;
  voroCostmap.info.origin.orientation.w = 1.0;
  voroCostmap.data.assign(sizeX*sizeY,0);
  for(int y = sizeY-1; y >=0; y--){      
    for(int x = 0; x<sizeX; x++){	
      if (isVoronoi(x,y)){
        voroCostmap.data[y * sizeX + x] = 100;
        voroList.push_back(INTPOINT(x,y));

      }
      else if(isOccupied(x,y))
      {
        voroCostmap.data[y * sizeX + x] = 0;
      }
      else{
        int cost = (int) (data[x][y].edgedist / Constants::voroObsDMax * 100.0);
        if ( cost > 100 )
          cost = 100;
        voroCostmap.data[y * sizeX + x] = cost;
      }
    }
  }
}

void DynamicVoronoi::prune() {
  // filler
  while(!pruneQueue.empty()) {
    INTPOINT p = pruneQueue.front();
    pruneQueue.pop();
    int x = p.x;
    int y = p.y;

    if (data[x][y].voronoi==occupied) continue;
    if (data[x][y].voronoi==freeQueued) continue;

    data[x][y].voronoi = freeQueued;
    open.push(data[x][y].sqdist, p);

    /* tl t tr
       l c r
       bl b br */

    dataCell tr,tl,br,bl;
    tr = data[x+1][y+1];
    tl = data[x-1][y+1];
    br = data[x+1][y-1];
    bl = data[x-1][y-1];

    dataCell r,b,t,l;
    r = data[x+1][y];
    l = data[x-1][y];
    t = data[x][y+1];
    b = data[x][y-1];

    if (x+2<sizeX && r.voronoi==occupied) { 
      // fill to the right
      if (tr.voronoi!=occupied && br.voronoi!=occupied && data[x+2][y].voronoi!=occupied) {
        r.voronoi = freeQueued;
        open.push(r.sqdist, INTPOINT(x+1,y));
        data[x+1][y] = r;
      }
    } 
    if (x-2>=0 && l.voronoi==occupied) { 
      // fill to the left
      if (tl.voronoi!=occupied && bl.voronoi!=occupied && data[x-2][y].voronoi!=occupied) {
        l.voronoi = freeQueued;
        open.push(l.sqdist, INTPOINT(x-1,y));
        data[x-1][y] = l;
      }
    } 
    if (y+2<sizeY && t.voronoi==occupied) { 
      // fill to the top
      if (tr.voronoi!=occupied && tl.voronoi!=occupied && data[x][y+2].voronoi!=occupied) {
        t.voronoi = freeQueued;
        open.push(t.sqdist, INTPOINT(x,y+1));
        data[x][y+1] = t;
      }
    } 
    if (y-2>=0 && b.voronoi==occupied) { 
      // fill to the bottom
      if (br.voronoi!=occupied && bl.voronoi!=occupied && data[x][y-2].voronoi!=occupied) {
        b.voronoi = freeQueued;
        open.push(b.sqdist, INTPOINT(x,y-1));
        data[x][y-1] = b;
      }
    } 
  }


  while(!open.empty()) {
    INTPOINT p = open.pop();
    dataCell c = data[p.x][p.y];
    int v = c.voronoi;
    if (v!=freeQueued && v!=voronoiRetry) { // || v>free || v==voronoiPrune || v==voronoiKeep) {
      //      assert(v!=retry);
      continue;
    }

    markerMatchResult r = markerMatch(p.x,p.y);
    if (r==pruned) c.voronoi = voronoiPrune;
    else if (r==keep) c.voronoi = voronoiKeep;
    else { // r==retry
      c.voronoi = voronoiRetry;
      //      printf("RETRY %d %d\n", x, sizeY-1-y);
      pruneQueue.push(p);
    }
    data[p.x][p.y] = c;

    if (open.empty()) {
      while (!pruneQueue.empty()) {
        INTPOINT p = pruneQueue.front();
        pruneQueue.pop();
        open.push(data[p.x][p.y].sqdist, p);
      }
    }
  }
  //  printf("match: %d\nnomat: %d\n", matchCount, noMatchCount);
}

DynamicVoronoi::markerMatchResult DynamicVoronoi::markerMatch(int x, int y) {
  // implementation of connectivity patterns
  bool f[8];

  int nx, ny;
  int dx, dy;

  int i=0;
  int count=0;
  //  int obstacleCount=0;
  int voroCount=0;
  int voroCountFour=0;

  for (dy=1; dy>=-1; dy--) {
    ny = y+dy;
    for (dx=-1; dx<=1; dx++) {
      if (dx || dy) {
        nx = x+dx;
        dataCell nc = data[nx][ny];
        int v = nc.voronoi;
        bool b = (v<=free && v!=voronoiPrune); 
        //	if (v==occupied) obstacleCount++;
        f[i] = b;
        if (b) {
          voroCount++;
          if (!(dx && dy)) voroCountFour++;
        }
        if (b && !(dx && dy) ) count++;
        //	if (v<=free && !(dx && dy)) voroCount++;
        i++;
      }
    }
  }
  if (voroCount<3 && voroCountFour==1 && (f[1] || f[3] || f[4] || f[6])) {
    //    assert(voroCount<2);
    //    if (voroCount>=2) printf("voro>2 %d %d\n", x, y);
    return keep;
  }

  // 4-connected
  if ((!f[0] && f[1] && f[3]) || (!f[2] && f[1] && f[4]) || (!f[5] && f[3] && f[6]) || (!f[7] && f[6] && f[4])) return keep;
  if ((f[3] && f[4] && !f[1] && !f[6]) || (f[1] && f[6] && !f[3] && !f[4])) return keep;
  


  // keep voro cells inside of blocks and retry later
  if (voroCount>=5 && voroCountFour>=3 && data[x][y].voronoi!=voronoiRetry) {
    return retry;
  }

  return pruned;
}

const Node3D* DynamicVoronoi::voronoiAstar(int sx, int sy, int gx, int gy, CollisionDetection& configurationSpace)
{
  int ssqdist = 0, gsqdist = 0;
  
  INTPOINT startp, goalp;
  std::vector<INTPOINT>::iterator iter = voroList.begin();
  ssqdist = (iter->x-sx)*(iter->x-sx) + (iter->y-sy)*(iter->y-sy);
  gsqdist = (iter->x-gx)*(iter->x-gx) + (iter->y-gy)*(iter->y-gy);
  startp = *iter;
  goalp = *iter;
  iter++;
  for(;iter!=voroList.end();iter++){
    if (ssqdist > (iter->x-sx)*(iter->x-sx) + (iter->y-sy)*(iter->y-sy)){
      ssqdist = (iter->x-sx)*(iter->x-sx) + (iter->y-sy)*(iter->y-sy);
      startp = * iter;
    }

    if (gsqdist > (iter->x-gx)*(iter->x-gx) + (iter->y-gy)*(iter->y-gy)){
      gsqdist = (iter->x-gx)*(iter->x-gx) + (iter->y-gy)*(iter->y-gy);
      goalp = * iter;
    }
  }
  Node3D start(startp.x, startp.y, 0,0,0,0,nullptr);
  Node3D goal(goalp.x, goalp.y, 0,0,0,0,nullptr);
  boost::heap::fibonacci_heap<Node3D*, boost::heap::compare<CompareNodes3>> O;

  int iPred, iSucc;
  float newG;
  // push on priority queue
  iPred = start.setIdx( start.getY()*sizeX + start.getX() );
  nodes3D[iPred] = start;
  nodes3D[iPred].updateH_euclid(goal);

  // mark start as open
  nodes3D[iPred].open();

  O.push((&nodes3D[iPred]));
  // NODE POINTER
  Node3D* nPred;
  Node3D* nSucc;
  int iteration = 0;
  // continue until O empty
  while (!O.empty()) {
    iteration++;
    // pop node with lowest cost from priority queue
    nPred = O.top();
    // set index
    iPred = nPred->getIdx();
    // _____________________________
    // LAZY DELETION of rewired node
    // if there exists a pointer this node has already been expanded
    if (nodes3D[iPred].isClosed()) {
      // pop node from the open list and start with a fresh node
      O.pop();
      continue;
    }
    // _________________
    // EXPANSION OF NODE
    else if (nodes3D[iPred].isOpen()) {
      // add node to closed list
      nodes3D[iPred].close();
      // remove node from open list
      O.pop();

      // _________
      // GOAL TEST
      if (*nPred == goal) {
        return nPred;
      }
      // ____________________
      // CONTINUE WITH SEARCH
      else {
        // _______________________________
        // CREATE POSSIBLE SUCCESSOR NODES
        for (int dx=-1; dx<=1; dx++) {
          int nx = nPred->getX()+dx;
          if (nx<0 || nx>sizeX-1) continue;
          for (int dy=-1; dy<=1; dy++) {
            if (dx==0 && dy==0) continue;
            int ny = nPred->getY()+dy;
            if (ny<0 || ny>sizeY-1) continue;
            //if it not voronoi cell -> pass;
            if (!isVoronoi(nx,ny)) continue;
            // create possible successor
            nSucc = new Node3D(nx,ny,0,0,nPred->getG(),0,nPred);
            iSucc = nSucc->setIdx(nSucc->getY()*sizeX + nSucc->getX());
            if(!configurationSpace.isTraversable(nSucc, true)){
              delete nSucc;
              continue;
            }
            // set index of the successor
            

            // ensure successor is on grid ROW MAJOR
            // ensure successor is not blocked by obstacle
            // ensure successor is not on closed list
            if (!nodes3D[iSucc].isClosed()) {
              // calculate new G value
              
              newG = nSucc->getG()+sqrt(dx*dx + dy*dy);
              nSucc->setG(newG);

              // if successor not on open list or g value lower than before put it on open list
              if (!nodes3D[iSucc].isOpen() || newG < nodes3D[iSucc].getG()) {
                // calculate the H value
                nSucc->updateH_euclid(goal);

                // put successor on open list
                nSucc->open();
                nodes3D[iSucc] = *nSucc;
                O.push(&nodes3D[iSucc]);
                delete nSucc;
              } else { delete nSucc; }
            } else { delete nSucc; }
          }
        }
      }
    }
  }
  return nullptr;
}

/*
	Function to calculate yaw/heading of nodes

	x_1: x coordinate of parent node
	y_1: y coordinate of parent node
	x_2: x coordinate of current node
	y_2: y coordinate of current node

*/
float calc_yaw(float x_1, float y_1, float x_2, float y_2) {

	if(y_2 - y_1 > 0 && x_2 - x_1 > 0) {
		return asin((y_2 - y_1)/hypot(x_2 - x_1, y_2 - y_1));
	}

	if(y_2 - y_1 > 0 && x_2 - x_1 < 0) {
		return (3.14 - asin((y_2 - y_1)/hypot(x_2 - x_1, y_2 - y_1)));
	}

	if(y_2 - y_1 < 0 && x_2 - x_1 > 0) {
		return asin((y_2 - y_1)/hypot(x_2 - x_1, y_2 - y_1));
	}

	if(y_2 - y_1 < 0 && x_2 - x_1 < 0) {
		return (-3.14 - asin((y_2 - y_1)/hypot(x_2 - x_1, y_2 - y_1)));
	}

	if(y_2 - y_1 == 0) {
		if(x_2 > x_1) {
			return 0.0;
		} else {
			return 3.14;
		}
	}

	if(x_2 - x_1 == 0) {
		if(y_2 > y_1) {
			return 1.57;
		} else {
			return -1.57;
		}
	}
}

bool DynamicVoronoi::makesubGoal(int sx, int sy, int gx, int gy, CollisionDetection& configurationSpace){
  nodes3D = new Node3D[sizeX*sizeY]();
  const Node3D* goal = voronoiAstar(sx, sy, gx, gy, configurationSpace);
  if (goal==nullptr)
    return false;

  int ichild = goal->getIdx();

  Node3D* child = &(nodes3D[ichild]);
  Node3D* parent;
  int id = 0;
  const Node3D* tmp;
  subGoalMarkerArray.markers.clear();
  visualization_msgs::Marker subNode;
  subNode.header.frame_id = "map";
  subNode.header.stamp = ros::Time();
  subNode.ns = "subNodes";
  subNode.id = id++;
  subNode.type = visualization_msgs::Marker::ARROW;
  subNode.action = visualization_msgs::Marker::DELETEALL;
  subNode.pose.position.x = 0;
  subNode.pose.position.y = 0;
  subNode.pose.position.z = 0.1;
  subNode.pose.orientation.x = 0.0;
  subNode.pose.orientation.y = 0.0;
  subNode.pose.orientation.z = 0.0;
  subNode.pose.orientation.w = 1.0;
  subNode.scale.x = Constants::cellSize;
  subNode.scale.y = 0.04;
  subNode.scale.z = 0.04;
  subNode.color.a = 0.7; // Don't forget to set the alpha!
  subNode.color.r = 1.0;
  subNode.color.g = 0.7;
  subNode.color.b = 0.8;
  subGoalMarkerArray.markers.push_back(subNode);
  subNode.action = visualization_msgs::Marker::ADD;

  subGoalnodes.clear();
  subGoalnodes.push_back(*child);
  while((tmp=child->getPred())!=nullptr){
    subNode.id=id++;
    parent = &(nodes3D[tmp->getIdx()]);
    float t = calc_yaw(parent->getX(),parent->getY(), child->getX(), child->getY());
    parent->setT( Helper::normalizeHeadingRad(t));
    subNode.pose.position.x = parent->getX()*Constants::cellSize + originX;
    subNode.pose.position.y = parent->getY()*Constants::cellSize + originY;
    subNode.pose.orientation = tf::createQuaternionMsgFromYaw(parent->getT());
    child = parent;
    subGoalMarkerArray.markers.push_back(subNode);
    subGoalnodes.push_back(*parent);
  }
  subNode.id=id++;

  ichild = goal->getIdx();
  tmp = nodes3D[ichild].getPred();
  if(tmp!=nullptr){ //todo when start and goal is same
    float t = nodes3D[tmp->getIdx()].getT();
    nodes3D[ichild].setT(t);
    subGoalnodes[0].setT(t);
  }
  subNode.pose.position.x = goal->getX()*Constants::cellSize + originX;
  subNode.pose.position.y = goal->getY()*Constants::cellSize + originY;
  subNode.pose.orientation = tf::createQuaternionMsgFromYaw(goal->getT());
  subGoalMarkerArray.markers.push_back(subNode);

  delete [] nodes3D;

  return true;
}


