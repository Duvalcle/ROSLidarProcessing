#ifndef __POINTH__
#define __POINTH__
class Point
{
private:
	float x,y; // coordinates
  int id_cluster; //if 0 : no interresting cluster
public:
  Point(float x, float y, int id);
  float getX(); // attribute x getter
  float getY(); // attribute y getter
  double getDistance(Point point); // calculate distance between an arg point and the current working point
  int getIDCluster(); // getter on id_cluster
  void setIDCluster(int id); // setter on id_cluster
};
#endif
