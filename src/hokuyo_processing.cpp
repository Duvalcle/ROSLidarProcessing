#include "ros/ros.h"
#include "std_msgs/String.h"
#include <string>
#include <unistd.h>
#include <fcntl.h>    /* For O_RDWR */
#include <errno.h>
#include <string.h>
#include <termios.h>
#include <stdio.h>
#include <urg_node/Status.h>
#include <urg_node/urg_node_driver.h>
#include <typeinfo>
#include <vector>
#include <math.h>

#include "Cluster.hpp"



#define SIZE_BUF 2
#if defined _BSD_SOURCE || defined _SVID_SOURCE
 #define __USE_MISC     1
#endif

using namespace std;

int fd; //File descriptor to open serial
// prototype for linking serial.c Library
extern "C"{
  int set_interface_attribs(int fd, int speed);
  void set_mincount(int fd, int mcount);
}

//Pour la table de jeu, sizeCutX = 2050 et sizeCutY = 3050
void changementRepere(vector<Cluster> data,const sensor_msgs::LaserScan& msg, float sizeCutX, float sizeCutY){
  //string chaine;
	double dist;
  float epsilon = 0.05; //tolerance for distance to integrate in the cluster
  int lastCluster = 0;//id of the last used cluster

  float angle = msg.angle_min;
  int rows = msg.ranges.size();
  Cluster cluster0 = Cluster(0, Point(0,0,0));
	data.push_back(cluster0);
	Point pointNull = Point(0,0,0); //usefull for first distance calculation
  for (int i = 0; i < rows ; i++){
    // Transformation repère
    angle = angle +  (float) msg.angle_increment;
    float x = msg.ranges[i] * std::cos(angle);// + sizeCutX/2 - 0.050; // reposition du repère
    float y = msg.ranges[i] * std::sin(angle); // reposition du repère

    // Si point en dehors map, supprimer (mise à 0 dans cluster0)
    if (x > sizeCutX || x < -sizeCutX || y < -sizeCutX || y > sizeCutY){
      data[0].addPoint(pointNull);
    }
    else{
      Point current = Point(x,y,0);
			Point oldPoint = data[lastCluster].getLastAddedPoint();
      dist = current.getDistance(oldPoint);
      if (dist>epsilon){
				// adding in new cluster
				lastCluster++;
				Cluster newCluster = Cluster(lastCluster, current);
				data.push_back(newCluster);
				ROS_INFO_STREAM("NEW CLUSTER : " << dist);
      }
      else{
        data[lastCluster].addPoint(current);
      }
    }
  }
	// int nbCluster = data.size();
	// ROS_INFO_STREAM("___________________Nb_cluster : " << nbCluster);
	// for (int j=0; j<nbCluster; j++){
	// 	 Point centre = data[j].getCircleCenter();
	// 	 ROS_INFO_STREAM("Centre : "<< centre.getX()*1000 << " ; " <<centre.getY()*1000 <<
	// 	 								" CSize : "<<data[j].getTotalNBPoints() << " Rayon : " << data[j].getRayon());
	// }

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
	int nbCluster = data.size();
	int wlen;
	std::stringstream sstm;
	ROS_INFO_STREAM("____ELIM SURPLUS____Nb_cluster : " << nbCluster);
	for (int j=0; j<nbCluster; j++){
		 Point centre = data[j].getCircleCenter();
		sstm << "Centre : "<< centre.getX() << " ; " <<centre.getY() <<" CSize : "<< data[j].getTotalNBPoints() << " Rayon : " << data[j].getRayon() << "\n";
		 std::string stringcopy = sstm.str();
		 ROS_INFO_STREAM(stringcopy);
		wlen = write(fd,stringcopy.c_str(),stringcopy.size());
		if (wlen!=stringcopy.size()){
			printf("error from writing : %d, %d\n",wlen,errno);
		}
		tcdrain(fd); // delay for output
		sstm.str("");
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
	data.clear();
}

int main(int argc, char **argv){
	// *** Etablissement liaison série *** //
	char portname[] = "/dev/ttyACM1"; //Port ACM1 le zigbee doit être connecté après l'Hokuyo(ttyACM0) à la raspberry
 	int wlen; //File Descriptor and writen legnth
	if (fd < 0) {
		printf("Error opening %s: %s\n", portname, strerror(errno));
		return -1;
	}
	else if (fd > 0) {
		printf("open successfully \n");
	}
	set_interface_attribs(fd, B115200);
	set_mincount(fd, 0); /* set to pure timed read */

 	fd = open(portname, O_RDWR | O_NOCTTY | O_SYNC);
	wlen = write(fd,"INIT ICI\n",9);
	if (wlen!=9){
		printf("error from writing : %d, %d\n",wlen,errno);
	}
	tcdrain(fd); // delay for output
	// *** ROS init and launching *** //
	ros::init(argc, argv, "hokuyo_proceessing_Robotik_UTT");
	ros::NodeHandle n;
	ros::Subscriber sub = n.subscribe("scan",1000, scanCallBack);
	ros::spin();
}
