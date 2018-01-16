#include "Point.hpp"
#include "Cluster.hpp"
#include <math.h>
#include <vector>

using namespace std;

/* Cluster Constructor : store Points into a vector of Point */
Cluster::Cluster(int id_cluster, Point point)
    : id_cluster(id_cluster),
    circleCenter(point)
  {
    this->points.push_back(point); //init the vector with the first point
    point.setIDCluster(id_cluster); //init the id of the cluster
  };

/* Method which add point to the manipulated cluster*/
void Cluster::addPoint(Point point){
  points.push_back(point); // add the point to the vector
  int size = points.size(); // updating the size of the cluster
  int median = size/2; // getting the index of the median index
  //Update de center of the cluster.
  // To adapt for further versions (here we only use median)
  circleCenter = points[median]; // we estimate the center of the cluster with the median
  ray = points[0].getDistance(points[points.size()-1]); // updating the size of the oject considering the new point
  }

/* Method which remove point to the manipulated cluster */
/* we do not update the other data of the class by choice */
bool Cluster::removePoint(int id_point){
  points.erase(points.begin() + id_point);
  return true;
}

// getter on the median of the cluster
Point Cluster::getCircleCenter(){
  return circleCenter;
}

// getter on a point of the cluster
Point Cluster::getPoint(int index){
  return points[index];
}

// return the size of the cluster (ie : the size of the attribute vector "points")
int Cluster::getTotalNBPoints(){
  return points.size();
}

// return  the last added point to this cluster
Point Cluster::getLastAddedPoint(){
  return points[points.size()-1];
}

// return the id of the cluster
int Cluster::getID(){
  return id_cluster;
}

void Cluster::resetID(int id){
  for(int i = 0; i < points.size() ; i++){
    points[i].setIDCluster(id);
  }
}

// return the width of the cluster
double Cluster::getRay(){
  return ray;
}
