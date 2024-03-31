#include "node3d.h"

using namespace HybridAStar;

int Node3D::dir;
float Node3D::ds;
std::vector<float> Node3D::dx;
std::vector<float> Node3D::dy;
std::vector<float> Node3D::dt;

//###################################################
//                                         IS ON GRID
//###################################################
bool Node3D::isOnGrid(const int width, const int height) {
  Node3D::updateTrailer();
  return x >= 0 && x < width && y >= 0 && y < height 
  && (int)(t / Constants::deltaHeadingRad) >= 0 
  && (int)(t / Constants::deltaHeadingRad) < Constants::headings
  && trailer_x >= 0 && trailer_x < width && trailer_y >= 0 && trailer_y < height 
  && (int)(trailer_t / Constants::deltaHeadingRad) >= 0 
  && (int)(trailer_t / Constants::deltaHeadingRad) < Constants::headings;
}


//###################################################
//                                        IS IN RANGE
//###################################################
bool Node3D::isInRange(const Node3D& goal, bool useRandom) const { // rand defalut true
float dx,dy;
  if(useRandom){
    int random = rand() % 10 + 1;
    dx = std::abs(x - goal.x) / random;
    dy = std::abs(y - goal.y) / random;
  }
  else{
    dx = std::abs(x - goal.x) ;
    dy = std::abs(y - goal.y) ;
  }
  return (dx * dx) + (dy * dy) < Constants::dubinsShotDistance;
}

//###################################################
//                                   CREATE SUCCESSOR
//###################################################
Node3D* Node3D::createSuccessor(const int i) {
  float xSucc;
  float ySucc;
  float tSucc;
  float htSucc;
  bool f;
  // calculate successor positions forward
  if (i < Node3D::dir) {
    xSucc = x + Node3D::dx[i] * cos(t) - Node3D::dy[i] * sin(t);
    ySucc = y + Node3D::dx[i] * sin(t) + Node3D::dy[i] * cos(t);
    tSucc = Helper::normalizeHeadingRad(t + Node3D::dt[i]);
    htSucc = Helper::normalizeHeadingRad(Node3D::dt[i]-Node3D::ds *sin(ht)+ht);
    f = true;
  }
  // backwards
  else {
    xSucc = x - Node3D::dx[i - Node3D::dir] * cos(t) + Node3D::dy[i - Node3D::dir] * sin(t);
    ySucc = y - Node3D::dx[i - Node3D::dir] * sin(t) - Node3D::dy[i - Node3D::dir] * cos(t);
    tSucc = Helper::normalizeHeadingRad(t + Node3D::dt[i - Node3D::dir]);
    htSucc = Helper::normalizeHeadingRad(Node3D::dt[i - Node3D::dir]+ Node3D::ds *sin(ht)+ht);
    f=false;
  }

  return new Node3D(xSucc, ySucc, tSucc, htSucc, g, 0, this, i, f);
}

float Node3D::calcHtfromPred() const {
  float dt = t - pred->t;
  if (forward){
    return Helper::normalizeHeadingRad(dt- Node3D::ds *sin(pred->ht)+pred->ht);
  }
  else{
    return Helper::normalizeHeadingRad(dt+ Node3D::ds *sin(pred->ht)+pred->ht);
  }
}


//###################################################
//                                      MOVEMENT COST
//###################################################
void Node3D::updateG() {
  // forward driving
  if (forward) {
    // penalize turning
    if (pred->prim != prim) {
      // penalize change of direction
      if (!pred->forward) {
        g += dx[0] * Constants::penaltyTurning * Constants::penaltyCOD;
      } else {
        g += dx[0] * Constants::penaltyTurning;
      }
    } else {
      g += dx[0];
    }
  }
  // reverse driving
  else {
    // penalize turning and reversing
    if (pred->prim != prim) {
      // penalize change of direction
      if (forward) {
        g += dx[0] * Constants::penaltyTurning * Constants::penaltyReversing * Constants::penaltyCOD;
      } else {
        g += dx[0] * Constants::penaltyTurning * Constants::penaltyReversing;
      }
    } else {
      g += dx[0] * Constants::penaltyReversing;
    }
  }
}

//###################################################
//                                 3D NODE COMPARISON
//###################################################
bool Node3D::operator == (const Node3D& rhs) const {
  return (int)x == (int)rhs.x &&
         (int)y == (int)rhs.y &&
         (std::abs(t - rhs.t) <= Constants::deltaHeadingRad ||
          std::abs(t - rhs.t) >= Constants::deltaHeadingNegRad);
}

// Node3D& Node3D::operator = (const Node3D& rhs){
//   this->x = rhs.x;
//   this->y = rhs.y;
//   this->t = rhs.t;
//   this->g = rhs.g;
//   this->h = rhs.h;
//   this->pred = rhs.pred;
//   this->o = rhs.o;
//   this->c = rhs.c;
//   this->idx = rhs.idx;
//   this->prim = rhs.prim;
//   return *this;
// }
