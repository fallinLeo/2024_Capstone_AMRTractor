#ifndef CONSTANTS
#define CONSTANTS
/*!
   \file constants.h
   \brief This is a collection of constants that are used throughout the project.
   \todo All constants need to be checked and documented
*/

////###################################################
////                                               INFO
////###################################################
//    HEADING => 0 - 359 degrees, 0 being north pointing towards positive Y
//    X-COORDINATE => designating the width of the grid
//    Y-COORDINATE => designating the height of the grid

#include <cmath>
#include <vector>

/*!
    \brief The namespace that wraps the entire project
    \namespace HybridAStar
*/

namespace HybridAStar {
/*!
    \brief The namespace that wraps constants.h
    \namespace Constants
*/
namespace Constants {
// _________________
// CONFIG FLAGS

/// A flag for additional debugging output via `std::cout`
static const bool coutDEBUG = false;
/// A flag for the visualization of 3D nodes (true = on; false = off)
static const bool visualization = false ;
/// A flag for the visualization of 2D nodes (true = on; false = off)
static const bool visualization2D = false ;

/*!
  recommend set
  if reverse is not okay
  reverse:false / dubinsShot:true / reedsSheppShot:false / dubins:true
  if reverse is okay
  reverse:true / dubinsShot:false / reedsSheppShot:true / dubins:false
*/
/// A flag to toggle reversing (true = on; false = off)
extern bool reverse;
/// recommand when reedsSheppShot is on, dubinsShot should be off
/// A flag to toggle the connection of the path via Dubin's shot (true = on; false = off)
// extern bool dubinsShot;
// /// A flag to toggle the connection of the path via ReedShepp's shot (true = on; false = off) 
// extern bool reedsSheppShot;
// /// A flag to toggle the Dubin's heuristic, this should be false, if reversing is enabled (true = on; false = off)
// /// when is off it uses Reeds-Shep
// extern bool dubins;
extern bool useReedsSheppShot;
/*!
   \var static const bool dubinsLookup
   \brief A flag to toggle the Dubin's heuristic via lookup, potentially speeding up the search by a lot
   \todo not yet functional
*/
// static const bool dubinsLookup = false && dubins;
/// A flag to toggle the 2D heuristic (true = on; false = off)
static const bool twoD = true;
/// use Voronoi-based Hybrid A*?
extern bool voronoibased;
/// directions in forward search (max of motion primitives)
extern int directions;

// _________________
// GENERAL CONSTANTS

/// [#] --- Limits the maximum search depth of the algorithm, possibly terminating without the solution
static const int iterations = 100000000;
/// [m] --- Uniformly adds a padding around the vehicle
extern double bloating;
/// [m] --- The width of the vehicle(Tractor)
extern double width;
/// [m] --- The cell size of the 2D grid of the world
extern float cellSize;
/// [m] --- The length of the vehicle(Tractor)
extern double length;
/// [m] --- The minimum turning radius of the vehicle
extern float r;
/// [m] --- The width of the vehicle(Trailer)
extern double trailer_width;
/// [m] --- The length of the vehicle(Trailer)
extern double trailer_length;

// <robot specifications> 
/// [m] --- Tractor Length (same as length)
extern double RL;
/// [m] --- Tractor Width (same as width)
extern double RW;
/// [m] --- Tractor Wheelbase
extern double WB;
/// [m] --- Trailer Length
extern double TL;
/// [m] --- Trailer Width
extern double TW;
/// [m] --- Hitch Joint to Trailer Rear-Axle Distance
extern double RTR;
/// [m] --- Trailer Front Overhang
extern double RTF;
/// [m] --- Trailer Rear Overhang
extern double RTB;
/// [c*M_PI] --- Maximum Hitch Angle
extern double maxth;
/// [m] --- The distance between tractor's center to hitch
extern double TCH;
/// [m] --- The distance between hitch to trailer's center
extern double HCR;
/// [m] -- The bounding box size length and width to precompute all possible headings for trailer
extern int trailer_bbSize;

/// [#] --- The number of discretizations in heading
extern int headings;
/// [°] --- The discretization value of the heading (goal condition)
extern float deltaHeadingDeg;
/// [c*M_PI] --- The discretization value of heading (goal condition)
extern float deltaHeadingRad;
/// [c*M_PI] --- The heading part of the goal condition
extern float deltaHeadingNegRad;
/// [#] --- threshold to determine obstacle in map  [0~100]
extern int voroObstacle;
static const int lethal_th = 100;
static const int inscribed_th = 99;
static const int unknown_th = -1;


/*!
  \brief [m] --- The tie breaker breaks ties between nodes expanded in the same cell


  As the cost-so-far are bigger than the cost-to-come it is reasonbale to believe that the algorithm would prefer the predecessor rather than the successor.
  This would lead to the fact that the successor would never be placed and the the one cell could only expand one node. The tieBreaker artificially increases the cost of the predecessor
  to allow the successor being placed in the same cell.
*/
static const float tieBreaker = 0.05;
/// [m] --- primitive Length x / cellSize  -> x will be length in meter
// extern float primitiveLength = 0.11 / cellSize;
extern float primitiveLength;


// _________________________________
// HEURISTIC CONSTANTS + REEDS-SHEPP

/// [#] --- A factor to ensure admissibility of the holonomic with obstacles heuristic
static const float factor2D = sqrt(5) / sqrt(2) + 1;
/// [#] --- A movement cost penalty for turning (choosing non straight motion primitives)
extern float penaltyTurning;
/// [#] --- A movement cost penalty for reversing (choosing motion primitives > 2)
extern float penaltyReversing;
/// [#] --- A movement cost penalty for change of direction (changing from primitives < 3 to primitives > 2)
extern float penaltyCOD;
/// [m] --- The distance to the goal when the analytical solution (Dubin's shot) first triggers 비교할 때 루트 안함 (이유 불명)
extern float dubinsShotDistance;
/// [m] --- The step size for the analytical solution (Dubin's shot) primarily relevant for collision checking
extern float dubinsStepSize ;



// ____________________________________
// DUBINS LOOKUP SPECIFIC + REEDS-SHEPP

/// [m] --- The width of the dubinsArea / 2 for the analytical solution (Dubin's shot)
static const int dubinsWidth = 15;
/// [m] --- The area of the lookup for the analytical solution (Dubin's shot)
static const int dubinsArea = dubinsWidth * dubinsWidth;


// _________________________
// COLLISION LOOKUP SPECIFIC

/// [m] -- The bounding box size length and width to precompute all possible headings
extern int bbSize; 
/// [#] --- The sqrt of the number of discrete positions per cell
extern int positionResolution;
/// [#] --- The number of discrete positions per cell
extern int positions;
/// A structure describing the relative position of the occupied cell based on the center of the vehicle
struct relPos {
  /// the x position relative to the center
  int x;
  /// the y position relative to the center
  int y;
};
/// A structure capturing the lookup for each theta configuration
struct config {
  /// the number of cells occupied by this configuration of the vehicle
  int length;
  /*!
     \var relPos pos[64]
     \brief The maximum number of occupied cells
     \todo needs to be dynamic
  */
  // relPos pos[300];
  std::vector<relPos> pos;
};

// _________________
// SMOOTHER SPECIFIC
/// [m] --- 이것보다 가까우면 obstacle optimization 실시
/// maximum distance to obstacles that is penalized
extern float obsDMax;
/// maximum distance for obstacles to influence the voronoi field
/// [m] --- 이것보다 가까우면 voronoi optimization 실시
extern float voroObsDMax;
/// [cell] --- Voronoi branch purning param , 작을수록 diagrarm 작게 나누고, 클수록 diagram 큰 덩이로 나눔 최소는 1
extern int voroSmoothWin;

/// falloff rate for the voronoi field, 그리고 전체적인 optimization강도?에도 영향 미침
extern float alpha;
/// weight for the obstacle term
extern float wObstacle;
/// weight for the voronoi term
extern float wVoronoi;
/// weight for the curvature term
extern float wCurvature;
/// weight for the smoothness term
extern float wSmoothness;
/// max iteration for optimization
extern int maxIterations;


// ____________________________________________
// COLOR DEFINITIONS FOR VISUALIZATION PURPOSES
/// A structure to express colors in RGB values
struct color {
  /// the red portion of the color
  float red;
  /// the green portion of the color
  float green;
  /// the blue portion of the color
  float blue;
};
/// A definition for a color used for visualization
static constexpr color teal = {102.f / 255.f, 217.f / 255.f, 239.f / 255.f};
/// A definition for a color used for visualization
static constexpr color green = {166.f / 255.f, 226.f / 255.f, 46.f / 255.f};
/// A definition for a color used for visualization
static constexpr color orange = {253.f / 255.f, 151.f / 255.f, 31.f / 255.f};
/// A definition for a color used for visualization
static constexpr color pink = {249.f / 255.f, 38.f / 255.f, 114.f / 255.f};
/// A definition for a color used for visualization
static constexpr color purple = {174.f / 255.f, 129.f / 255.f, 255.f / 255.f};
/// A definition for a color used for visualization
static constexpr color red = {255.f / 255.f, 74.f / 255.f, 74.f / 255.f};
/// A definition for a color used for visualization
static constexpr color blue = {103.f / 255.f, 107.f / 255.f, 240.f / 255.f};
/// A definition for a color used for visualization
static constexpr color darkpink = {150.f / 255.f, 0.f / 255.f, 0.f / 255.f};
/// A definition for a color used for visualization
static constexpr color darkpurple = {124.f / 255.f, 79.f / 255.f, 200.f / 255.f};

// ___________
// PURPOSE OF THIS PLANNING
extern bool simul;


}
}

#endif // CONSTANTS

