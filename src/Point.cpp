#include "Point.hpp"
#include <math.h>
using namespace std;

/**
 * Constructor of a point.
 * Cartesian expression of point coordinates
 */
Point::Point(float x, float y, int id){
  this-> x = x; // coordinates
  this-> y = y; // coordinates
  this->id_cluster = id; //not really used for now, but it is still there
}

//attribute x getter
float Point::getX(){
  return x;
}

//attribute y getter
float Point::getY(){
  return y;
}

//calculate distance between an arg point and the current working point
double Point::getDistance(Point point){
  return sqrt(pow((this->x - point.x),2)+pow((this->y - point.y),2));
}

//getter on id_cluster
int Point::getIDCluster(){
  return id_cluster;
}

//setter on id_cluster
void Point::setIDCluster(int id){
  this->id_cluster = id;
}
