#include "Point.hpp"
#include "Cluster.hpp"
#include <math.h>
#include <vector>

using namespace std;

Cluster::Cluster(int id_cluster, Point point)
    : id_cluster(id_cluster),
    circleCenter(point)
  {
    this->points.push_back(point);
    point.setIDCluster(id_cluster);
  };

void Cluster::addPoint(Point point){
  points.push_back(point);
      //TODO set circleCenter form extrapolation of points
  }

bool Cluster::removePoint(int id_point){
  int total_points = points.size();
  points.erase(points.begin() + id_point);
  return true;
}

Point Cluster::getCircleCenter(){
  return circleCenter;
}

Point Cluster::getPoint(int index){
  return points[index];
}

int Cluster::getTotalNBPoints(){
  return points.size();
}

Point Cluster::getLastAddedPoint(){
  return points[points.size()];
}

int Cluster::getID(){
  return id_cluster;
}
