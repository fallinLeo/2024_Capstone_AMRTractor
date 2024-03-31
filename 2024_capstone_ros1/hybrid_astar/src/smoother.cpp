#include "smoother.h"
using namespace HybridAStar;
//###################################################
//                                     CUSP DETECTION
//###################################################
inline bool isCusp(const std::vector<Node3D>& path, int i) {
  bool revim2 = ! path[i - 2].getForward();
  bool revim1 = !path[i - 1].getForward();
  bool revi   = !path[i].getForward();
  bool revip1 = !path[i + 1].getForward();
  bool revip2 = !path[i + 2].getForward();

  return (revim2 != revim1 || revim1 != revi || revi != revip1 || revip1 != revip2);
}
//###################################################
//                                SMOOTHING ALGORITHM
//###################################################
void Smoother::smoothPath() {
  float totalWeight = Constants::wSmoothness + Constants::wCurvature + Constants::wVoronoi + Constants::wObstacle;
  if(totalWeight == 0){
    return;
  }
  // load the current voronoi diagram into the smoother

  // current number of iterations of the gradient descent smoother
  int iterations = 0;
  // the lenght of the path in number of nodes
  int pathLength = 0;

  // path objects with all nodes oldPath the original, newPath the resulting smoothed path
  pathLength = path.size();

  std::vector<Node3D> newPath = path;

  // descent along the gradient untill the maximum number of iterations has been reached
  while (iterations < Constants::maxIterations) {

    // choose the first three nodes of the path
    for (int i = 2; i < pathLength - 2; ++i) {

      Vector2D xim2(newPath[i - 2].getX(), newPath[i - 2].getY());
      Vector2D xim1(newPath[i - 1].getX(), newPath[i - 1].getY());
      Vector2D xi(newPath[i].getX(), newPath[i].getY());
      Vector2D xip1(newPath[i + 1].getX(), newPath[i + 1].getY());
      Vector2D xip2(newPath[i + 2].getX(), newPath[i + 2].getY());
      Vector2D correction;


      // the following points shall not be smoothed
      // keep these points fixed if they are a cusp point or adjacent to one
      if (isCusp(newPath, i)) { continue; }

      correction = correction - obstacleTerm(xi);
      if (!isOnGrid(xi + correction)) { continue; }

      correction = correction - voronoiTerm(xi);
      if (!isOnGrid(xi + correction)) { continue; }

      // ensure that it is on the grid
      correction = correction - smoothnessTerm(xim2, xim1, xi, xip1, xip2);
      if (!isOnGrid(xi + correction)) { continue; }

      // ensure that it is on the grid
      correction = correction - curvatureTerm(xim2, xim1, xi, xip1, xip2);
      if (!isOnGrid(xi + correction)) { continue; }

      // ensure that it is on the grid

      xi = xi + Constants::alpha * correction/totalWeight;
      // xi = xi + correction/totalWeight;
      newPath[i].setX(xi.getX());
      newPath[i].setY(xi.getY());
    }

    iterations++;
  }

  // calc yaw
  for (int i = 1; i < pathLength; ++i) {

    Vector2D xim1(newPath[i - 1].getX(), newPath[i - 1].getY());
    Vector2D xi(newPath[i].getX(), newPath[i].getY());
    Vector2D Dxi = xi - xim1;
    newPath[i - 1].setT(Helper::normalizeHeadingRad(std::atan2(Dxi.getY(), Dxi.getX()))+M_PI);

  }
  if (pathLength > 1)
    newPath[pathLength-1].setT(newPath[pathLength-2].getT());
  path = newPath;
}

bool Smoother::tracePath(const Node3D* node, int i, std::vector<Node3D> path) {
  if (node == nullptr) {
    this->path = path;
    return true;
  }
  i++;
  path.push_back(*node);
  tracePath(node->getPred(), i, path);
  return true;
}

bool Smoother::tracePath(const Node3D* node, float x, float y, int i, std::vector<Node3D> path) {
  if (node == nullptr) {
    this->path = path;
    return true;
  }
  if ( i > Constants::iterations * 2){
      printf("tracePath iteration over! %d\n!", i);
    // this->path = path;
    path.clear();
    return false;
  }
  i++;
  Node3D n = *node;
  n.setX(node->getX()*Constants::cellSize + x);
  n.setY(node->getY()*Constants::cellSize + y);
  path.push_back(n);
  return tracePath( node->getPred(),x , y, i, path);
}



//###################################################
//                                      OBSTACLE TERM
//###################################################
Vector2D Smoother::obstacleTerm(Vector2D xi) {
  int x = (xi.getX() - origin_x) / Constants::cellSize;
  int y = (xi.getY() - origin_y) / Constants::cellSize;
  Vector2D gradient;
  // the distance to the closest obstacle from the current node
  float obsDst = voronoi.getDistance( x , y) * Constants::cellSize;
  // the vector determining where the obstacle is
  // if the node is within the map
  if (x < width && x >= 0 && y < height && y >= 0) {
    Vector2D obsVct((x - voronoi.data[x][y].obstX) * Constants::cellSize,
                    (y - voronoi.data[x][y].obstY) * Constants::cellSize);

    // the closest obstacle is closer than desired correct the path for that
    if (obsDst < Constants::obsDMax) {
      gradient = Constants::wObstacle * 2 * (obsDst - Constants::obsDMax) * obsVct / obsDst;
      // printf("obst %f %f\n",gradient.getX(),gradient.getY());
      return gradient;
    }
  }
  return gradient;
}

//###################################################
//                                       VORONOI TERM
//###################################################
Vector2D Smoother::voronoiTerm(Vector2D xi) {
  int x = (xi.getX() - origin_x) / Constants::cellSize;
  int y = (xi.getY() - origin_y) / Constants::cellSize;
  Vector2D gradient;
  //    Constants::alpha > 0 = falloff rate
  //    dObs(x,y) = distance to nearest obstacle
  //    dEge(x,y) = distance to nearest edge of the GVD
  //    dObsMax   = maximum distance for the cost to be applicable
  // distance to the closest obstacle
  float obsDst = voronoi.getDistance(x, y)* Constants::cellSize;
  // distance to the closest voronoi edge
  float edgDst = voronoi.getDistanceEdge(x, y)* Constants::cellSize; 
  // the vector determining where the obstacle is
  Vector2D obsVct((x - voronoi.data[x][y].obstX)* Constants::cellSize,
                    (y - voronoi.data[x][y].obstY)* Constants::cellSize);
  // the vector determining where the voronoi edge is
  Vector2D edgVct((x - voronoi.data[x][y].edgeX)* Constants::cellSize,
                    (y - voronoi.data[x][y].edgeY)* Constants::cellSize);
  //calculate the distance to the closest obstacle from the current node
  //obsDist =  voronoiDiagram.getDistance(node->getX(),node->getY())

  if (obsDst < Constants::voroObsDMax) {
    //calculate the distance to the closest GVD edge from the current node
    // the node is away from the optimal free space area
    if (edgDst > 0) {
      Vector2D PobsDst_Pxi = obsVct / obsDst;
      Vector2D PedgDst_Pxi = edgVct / edgDst;
      float PvorPtn_PedgDst = Constants::alpha * obsDst * std::pow(obsDst - Constants::voroObsDMax, 2) / (std::pow(Constants::voroObsDMax, 2)
                              * (obsDst + Constants::alpha) * std::pow(edgDst + obsDst, 2));

      float PvorPtn_PobsDst = (Constants::alpha * edgDst * (obsDst - Constants::voroObsDMax) * ((edgDst + 2 * Constants::voroObsDMax + Constants::alpha)
                               * obsDst + (Constants::voroObsDMax + 2 * Constants::alpha) * edgDst + Constants::alpha * Constants::voroObsDMax))
                              / (std::pow(Constants::voroObsDMax, 2) * std::pow(obsDst + Constants::alpha, 2) * std::pow(obsDst + edgDst, 2));
      gradient = Constants::wVoronoi * ( PvorPtn_PobsDst * PobsDst_Pxi + PvorPtn_PedgDst * PedgDst_Pxi );
      // printf("voro %f %f\n",gradient.getX(),gradient.getY());
    }
  }
  return gradient;
}

//###################################################
//                                     CURVATURE TERM
//###################################################
Vector2D Smoother::curvatureTerm(Vector2D x_im2, Vector2D x_im1, Vector2D x_i, Vector2D x_ip1, Vector2D x_ip2) {
  Vector2D gradient;
  // the vectors between the nodes
  const Vector2D& delta_x_im1 = x_im1 - x_im2;
  const Vector2D& delta_x_i = x_i - x_im1;
  const Vector2D& delta_x_ip1 = x_ip1 - x_i;
  const Vector2D& delta_x_ip2 = x_ip2 - x_ip1;

  // ensure that the absolute values are not null
  if (delta_x_im1.length() > 0 && delta_x_i.length() > 0 && delta_x_ip1.length() > 0 && delta_x_ip2.length() > 0) {
    // the angular change at the node
    auto compute_kappa = [](const Vector2D& delta_x_0, const Vector2D& delta_x_1, float& delta_phi, float& kappa) {
        delta_phi = std::acos(Helper::clamp(delta_x_0.dot(delta_x_1) / (delta_x_0.length() * delta_x_1.length()), -1, 1));
        kappa = delta_phi / delta_x_0.length();
    };
    float delta_phi_im1, kappa_im1;
    compute_kappa(delta_x_im1, delta_x_i, delta_phi_im1, kappa_im1);
    float delta_phi_i, kappa_i;
    compute_kappa(delta_x_i, delta_x_ip1, delta_phi_i, kappa_i);
    float delta_phi_ip1, kappa_ip1;
    compute_kappa(delta_x_ip1, delta_x_ip2, delta_phi_ip1, kappa_ip1);

    // if the curvature is smaller then the maximum do nothing
    if (kappa_i <= kappaMax) {
      Vector2D zeros;
      return zeros;
    } else {
      auto compute_d_delta_phi = [](const float delta_phi){
          return -1. / std::sqrt(1. - std::pow(std::cos(delta_phi), 2));
      };

      const float& d_delta_phi_im1 = compute_d_delta_phi(delta_phi_im1);
      const Vector2D& d_cos_delta_phi_im1 = delta_x_im1.ort(delta_x_i) / (delta_x_im1.length() * delta_x_i.length());
      const Vector2D& d_kappa_im1 = 1. / delta_x_im1.length() * d_delta_phi_im1 * d_cos_delta_phi_im1;
      const Vector2D& kim1 = 2. * (kappa_im1 - kappaMax) * d_kappa_im1;

      const float& d_delta_phi_i = compute_d_delta_phi(delta_phi_i);
      const Vector2D& d_cos_delta_phi_i = delta_x_ip1.ort(delta_x_i) / (delta_x_ip1.length() * delta_x_i.length()) -
                                          delta_x_i.ort(delta_x_ip1) / (delta_x_i.length() * delta_x_ip1.length());
      const Vector2D& d_kappa_i = 1. / delta_x_i.length() * d_delta_phi_i * d_cos_delta_phi_i -
                                  delta_phi_i / std::pow(delta_x_i.length(), 3) * delta_x_i;
      const Vector2D& ki = 2. * (kappa_i - kappaMax) * d_kappa_i;

      const float& d_delta_phi_ip1 = compute_d_delta_phi(delta_phi_ip1);
      const Vector2D& d_cos_delta_phi_ip1 = -delta_x_ip2.ort(delta_x_ip1) / (delta_x_ip2.length() * delta_x_ip1.length());
      const Vector2D& d_kappa_ip1 = 1. / delta_x_ip1.length() * d_delta_phi_ip1 * d_cos_delta_phi_ip1 +
                                    delta_phi_ip1 / std::pow(delta_x_ip1.length(), 3) * delta_x_ip1;
      const Vector2D& kip1 = 2. * (kappa_ip1 - kappaMax) * d_kappa_ip1;

      // calculate the gradient
      gradient = Constants::wCurvature * (0.25 * kim1 + 0.5 * ki + 0.25 * kip1);

      if (std::isnan(gradient.getX()) || std::isnan(gradient.getY())) {
        // std::cout << "nan values in curvature term" << std::endl;
        Vector2D zeros;
        return zeros;
      }
      if (std::isinf(gradient.getX()) || std::isinf(gradient.getY())) {
        std::cout << "inf values in curvature term" << std::endl;
        Vector2D zeros;
        return zeros;
      }

      // return gradient of 0
      else {
        return gradient;
      }
    }
  }
  // return gradient of 0
  else {
    std::cout << "abs values not larger than 0" << std::endl;
    Vector2D zeros;
    return zeros;
  }
}

//###################################################
//                                    SMOOTHNESS TERM
//###################################################
Vector2D Smoother::smoothnessTerm(Vector2D xim2, Vector2D xim1, Vector2D xi, Vector2D xip1, Vector2D xip2) {
  Vector2D gradient = Constants::wSmoothness * (xim2 - 4 * xim1 + 6 * xi - 4 * xip1 + xip2);
  // Vector2D tmp1 = xi-xip1;
  // Vector2D tmp2 = xi-xim1;
  // Vector2D gradient = Constants::wSmoothness * 2 * ( xip1 - 2*xi + xim1 ) * ( tmp1/tmp1.length() - 2 + tmp2/tmp2.length() );
  // printf("smoo %f %f\n",gradient.getX(),gradient.getY());
  return gradient;
}

