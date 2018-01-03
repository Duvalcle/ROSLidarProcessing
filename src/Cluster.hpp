#ifndef __CLUSTERH__
#define __CLUSTERH__
// #include <vector>
// #include <typeinfo>
#include "Point.hpp"
#include <vector>
using namespace std;


class Cluster
{
public:
  Cluster(int id_cluster, Point point);
  void addPoint(Point point);
  bool removePoint(int id_point);
  Point getCircleCenter();
  Point getPoint(int index);
  int getTotalNBPoints();
  Point getLastAddedPoint();
  int getID();
  double getRayon();

private:
	vector<Point> points;
	Point circleCenter;
	int id_cluster;
  double rayon;
};
#endif
