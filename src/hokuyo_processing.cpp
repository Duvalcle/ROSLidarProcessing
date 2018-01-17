#include "ros/ros.h"
#include "std_msgs/String.h"
#include <string>
#include <fcntl.h>    /* For open function (serial) */
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

/*** prototype for linking serial.c Library ***/
extern "C"{
  int set_interface_attribs(int fd, int speed);
  void set_mincount(int fd, int mcount);
}


/**
 * Changing axes from polar to cartesian and clustering in vector<Cluster> data
 * @param *data pointer to a vector of class Cluster. Should be empty when calling function
 *        msg message from topic scan published by the hokuyo and urg_node
 *        Settings for robotics cup, sizeCutX = 2050 et sizeCutY = 3050
 */
void axesChangingAndClustering(vector<Cluster> *data,const sensor_msgs::LaserScan& msg, float sizeCutX, float sizeCutY){
	double dist; // variable registering distance between 2 points
  float epsilon = 0.05; //tolerance for distance to integrate in the cluster (in meters)
  int currentCluster = 0; //id of the last used cluster

  float angle = msg.angle_min; // contain permanent current angle
  int rows = msg.ranges.size(); // number of captured mesures
  Point nullPoint = Point(0,0,0); //usefull for first distance calculation
  Cluster cluster0 = Cluster(0, nullPoint); // setting a new Cluster and adding the origin to be able to calculate distances
	data->push_back(cluster0); //Adding cluster0 to data
  for (int i = 0; i < rows ; i++){
    // Axes transformation from polar to cartesian
    // Here you can reposition the center of the axis to fit the place of the Lidar on the table
    float x = msg.ranges[i] * std::cos(angle); // x transformation
    float y = msg.ranges[i] * std::sin(angle); // y transformation
    angle = angle + (float) msg.angle_increment; //updating current angle

    // If the point is outside the definite map, it is transformed to a nullPoint and adding to Cluster 0
    if (x > sizeCutX || x < -sizeCutX || y < -sizeCutX || y > sizeCutY){
      data->operator[](0).addPoint(nullPoint);// "operator[]" is because we manipulate a pointer
    }
    else{
      Point currentPoint = Point(x,y,currentCluster); // creating a point from calculated {x,y}, set ID cluster to currentCluster by default
			Point oldPoint = data->operator[](currentCluster).getLastAddedPoint(); // getting last point seen
      dist = currentPoint.getDistance(oldPoint);// calculate distance between previous point and current
      // if the distance is two high between 2 points it means that the 2 points do not belong to the same object
      if (dist<epsilon){
        //adding the point to the current cluster. This point is a part of the current seen object
        data->operator[](currentCluster).addPoint(currentPoint);
      }
      else{
        // adding in a new cluster (ie : creating an object)
				currentCluster++; // updating the number of the current cluster
        currentPoint.setIDCluster(currentCluster); // updating the id of the cluster for the point
				Cluster newCluster = Cluster(currentCluster, currentPoint); // Creating a new Cluster with the current point added in it
				data->push_back(newCluster); // adding the new Cluster to the structured vector data
				//ROS_INFO_STREAM("NEW CLUSTER : " << dist); // printing log to check if all is alright
      }
    }
  }
}

/**
 * Function designed for writing values in serial port to communicate over zigbee
 */
void serialWrite(Point point, float ray){
  float x = point.getX();
  float y = point.getY();
  int id = point.getIDCluster(); //Cluster Id it belongs to
  int wlen; // serial write length
  //Union struct which allows easy transformations
  //float union
  union float_bytes {
       float val;
       unsigned char bytes[sizeof(float)];
  } dataF;
  //int union
  union int_bytes {
         int val;
         unsigned char bytes[sizeof(int)];
  } dataI;
  //Init the written string in the serial port
  char finalChar[8];
  //This is where you can modify the data to be send
  dataI.val = id; //update dataI to take the ID
  //finalChar[0] = dataI.bytes[1];
  finalChar[0] = dataI.bytes[0]; //Keeping just the LSByte : no more than 256 clusters in our application
  dataF.val = x; //update dataF to take x
  finalChar[1] = dataF.bytes[3]; //2 most significant bytes for x
  finalChar[2] = dataF.bytes[2];
  dataF.val = y; //update dataF to take y
  finalChar[3] = dataF.bytes[3];//2 most significant bytes for y
  finalChar[4] = dataF.bytes[2];
  dataF.val = ray; //update dataF to take ray
  finalChar[5] = dataF.bytes[3];//2 most significant bytes for the size of the object
  finalChar[6] = dataF.bytes[2];
  finalChar[7] = '\n';

  wlen = write(fd, finalChar, 8); //writing to the serial device
  tcdrain(fd); // delay for output
  ROS_INFO("END transmition");
}


void readSerial(){

}
/**
 * callBack is called whenever a message has benn posted to the topic scan
 * it calls "axesChangingAndClustering"; print LOG, suppress parasites clusters
 * and send points throw serial to Zigbee module
 */
void scanCallBack(const sensor_msgs::LaserScan& msg)
{
  vector<Cluster> *data = new vector<Cluster>; // creating the structure of the data wich will be processed
  axesChangingAndClustering(data, msg, 0.5,0.5); // calling previous function
  int nbCluster = data->size(); //Checking the number of objects which has been detected
  //ROS_INFO_STREAM("___________________Nb_cluster : " << nbCluster);
  for (int j=0; j<nbCluster; j++){
     Point centre = data->operator[](j).getCircleCenter(); // get the center of theses clusters
    // ROS_INFO_STREAM("Centre : "<< centre.getX()*1000 << " ; " <<centre.getY()*1000 <<
    //                " CSize : " << data->operator[](j).getTotalNBPoints() << " Rayon : " << data->operator[](j).getRay());
  }

  // Getting rid of extra cluster
  data->erase(data->begin()); // Delete Cluster 0 (origin points only)
  // Loop designed to suppress extra clusters with noise and a to small number of points contained
  bool flag = true;
  //while it exists one cluster containing 5 points or less, we suppress it
  while(flag){
		int k = 0;
		flag = false;
		while (k < data->size()){ // list all the clusters
      //we've fixed the limit to 6 elements : above 6 we keep the cluster, under we delete it
			if (data->operator[](k).getTotalNBPoints() < 6){
				data->erase(data->begin() + k); // deletion of the cluster k
				flag = true; // flag to true, we've made a change of cluster number, we loop again
				break;
			}
			k++;
		}
	}
	nbCluster = data->size(); // update the number of objects detected
  //update the ID of points in the cluster
  for (int i = 0 ; i < nbCluster ; i++){
    for (int j = 0 ;  j < data->operator[](i).getTotalNBPoints() ; j++){
      data->operator[](i).resetID(i+1);
    }
  }

  int wlen; // serial write length
	//ROS_INFO_STREAM("____No More Extras____Nb_cluster : " << nbCluster);
  //Transmitting infos to Log and serial
	for (int k=0; k<nbCluster; k++){
		 Point centre = data->operator[](k).getCircleCenter(); //getting the centre of the cluster
     //ROS_INFO_STREAM("ID : "<< centre.getIDCluster() << "Centre : " << centre.getX() << " ; " << centre.getY() <<  " Ray : " << data->operator[](j).getRay()); //" CSize : " << data->operator[](j).getTotalNBPoints() <<
     float ray = data->operator[](k).getRay();
     serialWrite(centre, ray);
	}

	data->clear(); //cleaning the vector for the next callback
}

int main(int argc, char **argv){
  ros::init(argc, argv, "hokuyo_processing_node");
	ros::NodeHandle n;
  // *** Etablissement liaison série *** //
	char portname[] = "/dev/ttyACM2"; //Port ACM1 le zigbee doit être connecté après l'Hokuyo(ttyACM0) à la raspberry
 	int wlen; //File Descriptor and writen legnth
	if (fd < 0) {
		ROS_INFO("Error opening ", strerror(errno));
		return -1;
	}
	else if (fd > 0) {
	//	ROS_INFO_STREAM("Serial Port " << portname << "open successfully \n");
	}
	set_interface_attribs(fd, B115200);
	set_mincount(fd, 0); /* set to pure timed read */

	fd = open(portname, O_RDWR | O_NOCTTY | O_SYNC);
	//wlen = write(fd,"INIT ICI\n",9);
	// if (wlen!=9){
	// 	ROS_INFO_STREAM("Serial : error from writing : " << wlen << ": " << errno);
	// }
	//tcdrain(fd); // delay for output
	//*** ROS init and launching *** //
  Point point = Point(0.001,0.002,5);
  //serialWrite(point, 0.003);

  //subscribe to topic scan, allow buffering 1000msg before ignoring them, calling scanCallBack function when a message is received
	ros::Subscriber sub = n.subscribe("scan",1000, scanCallBack);
	ros::spin();
}
