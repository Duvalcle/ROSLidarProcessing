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
  int size = points.size();
  int median = size/2;
  //Update de center of the cluster.
  // To adapt for further version
  circleCenter = points[median];
  rayon = points[0].getDistance(points[points.size()-1]);
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
  return points[points.size()-1];
}

int Cluster::getID(){
  return id_cluster;
}

double Cluster::getRayon(){
  return rayon;
}
