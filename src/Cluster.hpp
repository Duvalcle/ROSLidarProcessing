#ifndef __CLUSTERH__
#define __CLUSTERH__
// #include <vector>
// #include <typeinfo>
#include "Point.hpp"
#include <vector>
using namespace std;


class Cluster
{
private:
  vector<Point> points; // Contains points, caracterise the Cluster
  Point circleCenter; // Contain the median point of the bellow vector
  int id_cluster; // id of the cluster
  float ray; //Whidth of the cluster (euclidian)
public:
  Cluster(int id_cluster, Point point); // Cluster Constructor : store Points into a vector of Point
  void addPoint(Point point); // Method which add point to the manipulated cluster
  bool removePoint(int id_point); // Method which remove point to the manipulated cluster
  Point getCircleCenter(); // getter on the median of the cluster
  Point getPoint(int index); // getter on a point of the cluster
  int getTotalNBPoints(); // return the size of the cluster (ie : the size of the attribute vector "points")
  Point getLastAddedPoint(); // return  the last added point to this cluster
  int getID(); // return the id of the cluster
  void resetID(int id); 
  double getRay(); // return the width of the cluster
};
#endif
