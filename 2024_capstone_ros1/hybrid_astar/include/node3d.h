#ifndef NODE3D_H
#define NODE3D_H

#include <cmath>

#include "constants.h"
#include "helper.h"
namespace HybridAStar {
/*!
   \brief A three dimensional node class that is at the heart of the algorithm.

   Each node has a unique configuration (x, y, theta) in the configuration space C.
*/
class Node3D {
 public:

  /// The default constructor for 3D array initialization
  Node3D(): Node3D(0, 0, 0, 0, 0, 0, nullptr) {}
  /// Constructor for a node with the given arguments
  Node3D(float x, float y, float t, float ht, float g, float h, const Node3D* pred, int prim = 0, bool forward=true) {
    this->x = x;
    this->y = y;
    this->t = t;
    this->ht = ht;
    this->g = g;
    this->h = h;
    this->pred = pred;
    this->o = false;
    this->c = false;
    this->idx = -1;
    this->prim = prim;
    this->forward = forward;
    updateTrailer();
  }

  // GETTER METHODS
  /// get the x position
  float getX() const { return x; }
  /// get the y position
  float getY() const { return y; }
  /// get the heading theta
  float getT() const { return t; }
  /// get the hitch angle
  float getHT() const {return ht; }
  /// get trailer's center x
  float getTrailerX() const {return trailer_x; }
  /// get trailer's center y
  float getTrailerY() const {return trailer_y; }
  /// get trailer's heading
  float getTrailerT() const {return trailer_t; }

  /// get the cost-so-far (real value)
  float getG() const { return g; }
  /// get the cost-to-come (heuristic value)
  float getH() const { return h; }
  /// get the total estimated cost
  float getC() const { return g + h; }
  /// get the index of the node in the 3D array
  int getIdx() const { return idx; }
  /// get the number associated with the motion primitive of the node
  int getPrim() const { return prim; }
  /// get the motion direction of node
  bool getForward() const { return forward; }
  /// determine whether the node is open
  bool isOpen() const { return o; }
  /// determine whether the node is closed
  bool isClosed() const { return c; }
  /// determine whether the node is open
  const Node3D* getPred() const { return pred; }

  // SETTER METHODS
  /// set the x position
  void setX(const float& x) { this->x = x; }
  /// set the y position
  void setY(const float& y) { this->y = y; }
  /// set the heading theta
  void setT(const float& t) { this->t = t; }
  /// set the hitch angle
  void setHT(const float& ht) { this->ht = ht; }
  /// set the cost-so-far (real value)
  void setG(const float& g) { this->g = g; }
  /// set the cost-to-come (heuristic value)
  void setH(const float& h) { this->h = h; }
  /// set the cost-to-come (heuristic value)
  void setPrim(const int& prim) { this->prim = prim; }
  /// the motion direction of node
  void setForward(const bool& forward) { this->forward = forward; }
  /// set and get the index of the node in the 3D grid
  int setIdx(int width, int height) { this->idx = (int)(t / Constants::deltaHeadingRad) * width * height + (int)(y) * width + (int)(x); return idx;}
  int setIdx(int idx) { this->idx = idx; return idx;}
  int calcIdx(int width, int height) const { return (int)(t / Constants::deltaHeadingRad) * width * height + (int)(y) * width + (int)(x); }
  float calctrailer_t() { return Helper::normalizeHeadingRad(this->t - this->ht);}
  /// calculate and set trailer's t,x,y
  void updateTrailer() {
    trailer_t = Helper::normalizeHeadingRad(t - ht);
    trailer_x = x - Constants::TCH * cos(t) - Constants::HCR * cos(trailer_t);
    trailer_y = y - Constants::TCH * sin(t) - Constants::HCR * sin(trailer_t);
  }
  void calcTrailer(float& tx, float& ty, float& tt ) const {
    tt = Helper::normalizeHeadingRad(t - ht);
    tx = x - Constants::TCH * cos(t) - Constants::HCR * cos(trailer_t);
    ty = y - Constants::TCH * sin(t) - Constants::HCR * sin(trailer_t);
  }
  // calculate ht form pred node
  float calcHtfromPred() const;


  /// open the node
  void open() { o = true; c = false;}
  /// close the node
  void close() { c = true; o = false; }
  /// set a pointer to the predecessor of the node
  void setPred(const Node3D* pred) { this->pred = pred; }
  /// set the node neither open nor closed
  void reset() { c = false; o = false;}

  // UPDATE METHODS
  /// Updates the cost-so-far for the node x' coming from its predecessor. It also discovers the node.
  void updateG();

  // These two are for voronoi astar
  void updateH_euclid(const Node3D& goal) { h = movementCost(goal); }  
  float movementCost(const Node3D& pred) const { return sqrt((x - pred.x) * (x - pred.x) + (y - pred.y) * (y - pred.y)); }


  // CUSTOM OPERATORS
  /// Custom operator to compare nodes. Nodes are equal if their x and y position as well as heading is similar.
  bool operator == (const Node3D& rhs) const;
  // Node3D& operator = (const Node3D& rhs);

  // RANGE CHECKING
  /// Determines whether it is appropriate to find a analytical solution.
  bool isInRange(const Node3D& goal, bool useRandom = true) const;

  // GRID CHECKING
  /// Validity check to test, whether the node is in the 3D array.
  bool isOnGrid(const int width, const int height) ;

  // SUCCESSOR CREATION
  /// Creates a successor in the continous space.
  Node3D* createSuccessor(const int i);

  // CONSTANT VALUES
  /// Number of possible directions
  static int dir;
  /// Possible movements in the x direction
  static std::vector<float> dx;
  /// Possible movements in the y direction
  static std::vector<float> dy;
  /// Possible movements regarding heading theta
  static std::vector<float> dt;
  /// for calculate hitch angle
  static float ds;

  /// if constant values updated
  static void updateConstants(){
    if (Constants::directions % 2)
      dir = Constants::directions;
    else
      dir = Constants::directions + 1;
    float radMax = Constants::primitiveLength / Constants::r;
    float dr_, dx_, dy_,dtrad_;
    ds = Constants::primitiveLength / Constants::RTR * Constants::cellSize;
    dy.clear();
    dx.clear();
    dt.clear();
    int halfd = (dir-1)/2;
    for(int i=0; i<dir; i++){
      if(i==halfd){
        dtrad_ = 0;
        dx_ = Constants::primitiveLength;
        dy_ = 0;
      }
      else{
        dtrad_ = -radMax/halfd * (i-halfd);
        dr_ = Constants::primitiveLength / dtrad_;
        dx_ = abs(dr_*sin(dtrad_));
        dy_ = dr_*(1-cos(dtrad_));
      }
      dy.push_back(dy_);
      dx.push_back(dx_);
      dt.push_back(dtrad_);
    }
  }

 private:
  /// the x position
  float x;
  /// the y position
  float y;
  /// the heading theta
  float t;
  /// hitch angle
  float ht;
  /// the trailer's heading theta
  float trailer_t;
  /// the trailer's x
  float trailer_x;
  /// the trailer's y
  float trailer_y;
  /// the cost-so-far
  float g;
  /// the cost-to-go
  float h;
  /// the index of the node in the 3D array
  int idx;
  /// the open value
  bool o;
  /// the closed value
  bool c;
  /// the motion primitive of the node
  int prim;
  /// the motion direction of node
  bool forward;
  /// the predecessor pointer
  const Node3D* pred;
};

//###################################################
//                                    NODE COMPARISON
//###################################################
/*!
   \brief A structure to sort nodes in a heap structure
*/
struct CompareNodes3 {
  /// Sorting 3D nodes by increasing C value - the total estimated cost
  bool operator()(const Node3D* lhs, const Node3D* rhs) const {
    return lhs->getC() > rhs->getC();
  }
};

}
#endif // NODE3D_H
