#include "Point.hpp"
#include <math.h>
using namespace std;


Point::Point(float x, float y, int id){
  this-> x = x;
  this-> y = y;
  this->id_cluster = id;
}

float Point::getX(){
  return x;
}

float Point::getY(){
  return y;
}

double Point::getDistance(Point point){
  return sqrt(pow((this->x - point.x),2)+pow((this->y - point.y),2));
}

int Point::getIDCluster(){
  return id_cluster;
}

void Point::setIDCluster(int id){
  this->id_cluster = id;
}

/*std::string Point::to_String(){
  string result = "("<< this->x << " ; " << this->y <<" )"
  return result;
}*/
