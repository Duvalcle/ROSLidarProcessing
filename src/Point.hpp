#ifndef __POINTH__
#define __POINTH__
class Point
{
private:
	float x,y;
  int id_cluster; //if 0 : no interresting cluster
public:
  Point(float x, float y, int id);
  float getX();
  float getY();
  float getDistance(Point point);
  int getIDCluster();
  void setIDCluster(int id);
};
#endif
