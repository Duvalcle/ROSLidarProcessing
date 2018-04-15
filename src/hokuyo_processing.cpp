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

#define ENNEMI_POS			25 //mode for the broadcast over zigbee (cf to protocol docummentation Robotik)
#define TRACKING_TOLERENCE	50 //cm tolerence before loosing track of an obstacle, calculated from center
#define RAY_ROBOT_MIN		80 //mm
#define RAY_ROBOT_MAX		100 //mm

#define ID_REF					0
#define ID_FRIEND_BIG_ROBOT		1
#define ID_FRIEND_LITTLE_ROBOT	2
#define ID_ENNEMI_ROBOT_1		3
#define ID_ENNEMI_ROBOT_2		4
#define ID_STRUCTURE			5
#define ID_UNKNOWN				6

#define SIZE_BUF 2
#if defined _BSD_SOURCE || defined _SVID_SOURCE
 #define __USE_MISC     1
#endif

using namespace std;

int fd; //File descriptor to open serial
vector<Cluster> *previousData = new vector<Cluster>; // Previuos frame of the data curently processed

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
      }
    }
  }
}

/**
 * Function designed for writing values in serial port to communicate over zigbee
 */
void serialWrite(Point point, float ray){
	int wlen; //Variable to control written data over Serial
	uint8_t finalChar[7]; // Init Byte Array which will be send over serial
	int x = (int)(point.getX()*1000); // Cast x position from Float to int (in millimeters)
 	int y = (int)(point.getY()*1000); // Cast y position from Float to int (in millimeters)
 	int R = (int)(ray*1000); // Cast Ray from Float to int (in millimeters)
 	int id = point.getIDCluster(); //Cluster Id it belongs to
	//Composition of the frame. Some bytes are written befor others to avoid overwrite
	finalChar[0] = ENNEMI_POS;
	finalChar[1] = id;
	finalChar[3] = x;
	finalChar[2] = (uint16_t) x>>8; //Cast to take the second byte of the x position int
	finalChar[5] = y;
	finalChar[4] = (uint16_t) y>>8; //Cast to take the second byte of the y position int
	finalChar[7] = R;
	finalChar[6] = (uint16_t) R>>8;//Cast to take the second byte of Ray the int
	ROS_INFO_STREAM("[SerialWrite] Mode "<< ENNEMI_POS << "\t Index : "<< id <<"\t x : "<< x << "\t y : " << y << "\t R : "<<R); //What is going to be send
	wlen = write(fd, finalChar, 7); //writing to the serial device
	if (wlen!=7){
		ROS_INFO_STREAM("[SerialWrite] error from writing : " << wlen << ": " << errno);
	}
	tcdrain(fd); // delay for output
}

void beautifyClusters(vector<Cluster> *data){
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
}

/** 
 * Dealing IDs
 * 		1 : Friend Big Robot
 * 		2 : Friend Little Robot
 * 		3 : Ennemi Robot 1
 * 		4 : Ennemi Robot 2
 * 		5 : Structure Obstacle
 * 		6 : Unkown Object 
 */
void classifier(vector<Cluster> *data){
	int nbClusterCurrent = data->size(); //Checking the number of objects which has been detected
	int nbClusterPrevious = previousData->size(); //Checking the number of objects which has been detected
	int previousID;
	Point reference = Point(0,0,0);
	Cluster currentCluster = data->operator[](0);

	if (nbClusterPrevious == 0){
		//current data is the only data available, tracking off, creating IDs
		//list the elements of the vector
		for (int i = 0; i<nbClusterCurrent; i++){
			//parcours de la liste ancienne
			currentCluster = data->operator[](i);
			Point currentCenter = currentCluster.getCircleCenter();		
			//Si rayon obstacle environ égal à size robot :
			if (currentCluster.getRay() >= RAY_ROBOT_MIN && currentCluster.getRay() <= RAY_ROBOT_MAX){
				// si obstacle loin (supérieur à 1.50m : robot ami
				if (currentCenter.getDistance(reference) > 1500){
					currentCluster.resetID(ID_FRIEND_BIG_ROBOT);
				}
				// Sinon robot ennemi
				else if(currentCenter.getX() > 0 && currentCenter.getY()){
					if(currentCenter.getIDCluster() == ID_ENNEMI_ROBOT_1){
						currentCluster.resetID(ID_ENNEMI_ROBOT_2);
					}
					else{
						currentCluster.resetID(ID_ENNEMI_ROBOT_1);
					}
				}
				else{
					currentCluster.resetID(ID_UNKNOWN);
				}	 
			}
			else{
				// Sinon Object inconnu
				currentCluster.resetID(ID_UNKNOWN);
			}
		}
		
		
		//Identification of our 1st robot :
		
	}
	else{
		//parcours de la liste current
		for (int i = 0; i<nbClusterCurrent; i++){
			//parcours de la liste ancienne
			Point currentCenter = data->operator[](i).getCircleCenter();
			for (int j = 0; j<nbClusterPrevious; j++){
				Point previousCenter = data->operator[](j).getCircleCenter(); // get the center of theses clusters
				if (previousCenter.getDistance(currentCenter) < TRACKING_TOLERENCE){
					data->operator[](i).resetID(data->operator[](j).getID());
					break;
				}	
			}		
		}
	}
}


/**
 * callBack is called whenever a message has benn posted to the topic scan
 * it calls "axesChangingAndClustering"; print LOG, suppress parasites clusters
 * and send points throw serial to Zigbee module
 */
void scanCallBack(const sensor_msgs::LaserScan& msg)
{
	float sizeCutX = 0.5;
	float sizeCutY = 0.5;
	vector<Cluster> *data = new vector<Cluster>; // creating the structure of the data wich will be processed
	axesChangingAndClustering(data, msg, sizeCutX,sizeCutY); // calling previous function
	int nbCluster = data->size(); //Checking the number of objects which has been detected

	for (int j=0; j<nbCluster; j++){
		Point centre = data->operator[](j).getCircleCenter(); // get the center of theses clusters
	}
	beautifyClusters(data);
	nbCluster = data->size(); // update the number of objects detected
	//update the ID of points in the cluster
	for (int i = 0 ; i < nbCluster ; i++){
		for (int j = 0 ;  j < data->operator[](i).getTotalNBPoints() ; j++){
			data->operator[](i).resetID(i+1);
		}
	}
	/**
	* Section to uncomment to send every center Points
	*
	*/
	for (int k=0; k<nbCluster; k++){
	 	Point centre = data->operator[](k).getCircleCenter(); //getting the centre of the cluster
		ROS_INFO_STREAM("ID : "<< centre.getIDCluster() << "Centre : " << centre.getX() << " ; " << centre.getY() <<  " Ray : " << data->operator[](k).getRay()); //" CSize : " << data->operator[](j).getTotalNBPoints() <<
		float ray = data->operator[](k).getRay();
		serialWrite(centre, ray);
		usleep(25000);
	 }

	/**
	* Section to comment to send every center Points
	* and not only the nearest cluster
	*/
	/*Point reference = Point(0,0,0);
	float mini = 50000; // To be sure that the distance will be inferior than this number
	Cluster clusterMin = data->operator[](0);
	//Calculate nearest object from reference
	for (int k=0; k<nbCluster; k++){
		Cluster currentCluster = data->operator[](k);
		Point centre = currentCluster.getCircleCenter(); //getting the centre of the cluster
		float distance = centre.getDistance(reference); //Calculate distance between the object and reference
		//Calculate min function
		if (distance<mini){
			mini = distance; // refresh minimum
			clusterMin = currentCluster; // register the nearest object
		}
	}
	
	float ray = clusterMin.getRay();
	Point centre = clusterMin.getCircleCenter(); // Get the center of the nearest object
	ROS_INFO_STREAM("ID : "<< centre.getIDCluster() << "Centre : " << centre.getX() << " ; " << centre.getY() <<  " Ray : " << ray << "Distance : " << mini); //" CSize : " << data->operator[](j).getTotalNBPoints() <<
	serialWrite(centre, ray); //Write to the arduino in serial
	usleep(25000); // Delay for Arduino node to understand the frame
*/
	//*** End of section ***//
	previousData = data;
	data->clear(); //cleaning the vector for the next callback
}

int main(int argc, char **argv){
  ros::init(argc, argv, "hokuyo_processing_node");
	ros::NodeHandle n;
  // *** Etablissement liaison série *** //
	char portname[] = "/dev/ttyUSB0"; //nom de port à changer en fonction du driver utilisé par la carte Arduino
 	int wlen; //File Descriptor and writen legnth
	fd = open(portname, O_RDWR | O_NOCTTY | O_SYNC);
	if (fd < 0) {
		ROS_INFO_STREAM("Error opening " << strerror(errno));
		return -1;
	}
	else if (fd > 0) {
		ROS_INFO_STREAM("Serial Port " << portname << "open successfully \n");
	}
	set_interface_attribs(fd, B57600); // Set baud rate according to Robotik's spec
	set_mincount(fd, 0); /* set to pure timed read */
	//*** ROS launching *** //
  //subscribe to topic scan, allow buffering 1000msg before ignoring them, calling scanCallBack function when a message is received
	ros::Subscriber sub = n.subscribe("scan",1, scanCallBack);
	ros::spin();
}
