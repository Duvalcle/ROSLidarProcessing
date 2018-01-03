#include "ros/ros.h"
#include "std_msgs/String.h"
#include <string>
//#include <urg_c/urg_sensor.h>
#include <urg_node/Status.h>
#include <urg_node/urg_node_driver.h>
#include <typeinfo>
#include <vector>
#include <math.h>
//#include "Point.h"

#include "Cluster.hpp"


using namespace std;


//Pour la table de jeu, sizeCutX = 2050 et sizeCutY = 3050
void changementRepere(vector<Cluster> data,const sensor_msgs::LaserScan& msg, float sizeCutX, float sizeCutY){
  //string chaine;
	double dist;
  float epsilon = 0.05; //tolerance for distance to integrate in the cluster
  int lastCluster = 0;//id of the last used cluster
  // ROS_INFO_STREAM("range min : "<<msg.range_min);
  // ROS_INFO_STREAM("angle_increment : "<<msg.angle_increment);
  // ROS_INFO_STREAM("float type range : "<<typeid(msg.ranges[0]).name());
  float angle = msg.angle_min;
  int rows = msg.ranges.size();
  Cluster cluster0 = Cluster(0, Point(0,0,0));
	data.push_back(cluster0);
	Point pointNull = Point(0,0,0);
	//sizeof(msg.ranges)/sizeof(msg.ranges[0])
  for (int i = 0; i < rows ; i++){
    // Transformation repère
		//ROS_INFO_STREAM("Test : "<< i << " Rows : " << rows);
    angle = angle +  (float) msg.angle_increment;
    float x = msg.ranges[i] * std::cos(angle);// + sizeCutX/2 - 0.050; // reposition du repère
    float y = msg.ranges[i] * std::sin(angle); // reposition du repère
		//ROS_INFO_STREAM("OK1");

    // Si point en dehors map, supprimer (mise à 0 dans cluster0)
    if (x > sizeCutX || x < -sizeCutX || y < -sizeCutX || y > sizeCutY){
      //tableau_donnees[i][0] = 0;
      //tableau_donnees[i][1] = 0;
      data[0].addPoint(pointNull);
			//ROS_INFO_STREAM("OK2");
    }
    else{
      Point current = Point(x,y,0);
			Point oldPoint = data[lastCluster].getLastAddedPoint();
      dist = current.getDistance(oldPoint);
			//ROS_INFO_STREAM("dist : "<<dist);
			//ROS_INFO_STREAM("Current : " << current.getX() <<" " << current.getY()<< " Old : "<< oldPoint.getX() <<" " << oldPoint.getY());
      if (dist>epsilon){
				// adding in new cluster
				lastCluster++;
				Cluster newCluster = Cluster(lastCluster, current);
				data.push_back(newCluster);
				ROS_INFO_STREAM("NEW CLUSTER : " << dist);
				//ROS_INFO_STREAM("OK3");
      }
      else{
        //adding in the last cluster
        data[lastCluster].addPoint(current);
				//ROS_INFO_STREAM("ADD To cluster : "<<lastCluster);

				//ROS_INFO_STREAM("OK4");
        //data.push_back(Point(x,y,0));

      }
      //tableau_donnees[i][0] = x;
      //tableau_donnees[i][1] = y;
    }

  }
	int nbCluster = data.size();
	ROS_INFO_STREAM("___________________Nb_cluster : " << nbCluster);
	for (int j=0; j<nbCluster; j++){
		 Point centre = data[j].getCircleCenter();
		 ROS_INFO_STREAM("Centre : "<< centre.getX()*1000 << " ; " <<centre.getY()*1000 <<
		 								" CSize : "<<data[j].getTotalNBPoints() << " Rayon : " << data[j].getRayon());
	}
	data.erase(data.begin());
	bool changement = true;
	while(changement){
		int k = 0;
		changement = false;
		while (k < data.size()){
			if (data[k].getTotalNBPoints() < 6){
				data.erase(data.begin() + k);
				changement = true;
				break;
			}
			k++;
		}
	}
	nbCluster = data.size();
	ROS_INFO_STREAM("____ELIM SURPLUS____Nb_cluster : " << nbCluster);
	for (int j=0; j<nbCluster; j++){
		 Point centre = data[j].getCircleCenter();
		 ROS_INFO_STREAM("Centre : "<< centre.getX()*1000 << " ; " <<centre.getY()*1000 <<
										" CSize : "<<data[j].getTotalNBPoints() << " Rayon : " << data[j].getRayon());
	}

	data.clear();
}

//process à chaque message
void scanCallBack(const sensor_msgs::LaserScan& msg)
{
  //int rows = (sizeof msg.ranges)/(sizeof msg.ranges[0]);
  int rows = msg.ranges.size();
  //Cluster cluster0 = Cluster(0);
  vector<Cluster> data;
  float tableau_donnees[rows][2];
  changementRepere(data, msg, 0.5,0.5);
  //for (int i = 0; i < rows ; i++){
    //  ROS_INFO_STREAM(i << " : x = ");// << data.at(i));// << " y = " << tableau_donnees[i][1]);
	//}
	data.clear();
  //clustering(tableau_donnees, rows);
  /*for (int i = 0 ; i<rows; i++){
    if (tableau_donnees[i][0] != 0 && tableau_donnees[i][1] != 0){
      ROS_INFO_STREAM("x : " << tableau_donnees[i][0] << " y : " << tableau_donnees[i][0]);
    }

  }*/
  //if (tableau_donnees)
  // ROS_INFO_STREAM("I heard: " << msg.header.frame_id);
  //ROS_INFO_STREAM(" rows : " << rows);
}

int main(int argc, char **argv){
  ros::init(argc, argv, "hokuyo_proceessing_Robotik_UTT");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("scan",1000, scanCallBack);
  ros::spin();
}
//float linearRegression()
//publishing image.MSG
/*void publishingIMG(){
    laser_geometry::LaserProjection projector_;

     sensor_msgs::PointCloud cloud;
     projector_.projectLaser(*scan_in, cloud);


 }
}

void clustering(float tableau_donnees[][2], int rows){
  //Calcul distance 2 points
  double dist;
  list<float[2]> clusterNull;
  list<float[2]> cluster1;
  list<float[2]> cluster2;
  int Currentcluster = 1;

  if (tableau_donnees[0][0] == 0 && tableau_donnees[0][1] == 0)
    clusterNull.push_back(tableau_donnees[0]);
  else
    cluster1.push_back(tableau_donnees[0]);
  for (int i = 0 ; i<rows-1 ; i++){
    if (tableau_donnees[i+1][0] == 0 && tableau_donnees[i+1][1] == 0){
      clusterNull.push_back(tableau_donnees[i+1]);
    }
    else {
    dist = sqrt(pow((tableau_donnees[i][0]-tableau_donnees[i+1][0]),2)+pow((tableau_donnees[i][1]-tableau_donnees[i+1][1]),2));
    if (dist>1){ //condition de chgmt de cluster
      if (Currentcluster == 1)
        Currentcluster = 2;
      if (Currentcluster == 2)
        Currentcluster = 1;
    }

    if (Currentcluster == 1)
      cluster1.push_back(tableau_donnees[i+1]);
    if (Currentcluster == 2)
      cluster2.push_back(tableau_donnees[i+1]);
    }
  }
}
*/
