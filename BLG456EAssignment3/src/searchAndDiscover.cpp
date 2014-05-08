/*

Okay. Here is my thought process:

Since the sense-and-react (assignemnt1) approach is not sufficient for
a reasonable map exploration, we need to "guide" the robot to unexplored areas.
To do that, we need to use map information.

What I had in mind was dividing the map into regions of 10x10 with a total 
of 1444 regions given that map size is 384x384 (map size is read from metadata). 
Using the 2D coordinates, I saved the regions' center's coordinates into 
Node class. My plan was using these center coordinates for robots destinations. 
A region would be classified as discovered if ~20 pixels are discovered ( > 50 ).

I successfully divided the map into regions and output the discovered regions
using ROS_WARN(). You can wander around using teleop and see how that works.

----------------------------------------------------------------------------

I used the sample assignment 2 code for getting the robot to a certain point.
However, unlike 2nd assignment, the path is not obstacle-free. My initial goal 
point is the center of the map, which is assigned after the map is analyzed.
I tried to use the laser scan as well for avoiding obstacles, however I seem
to have failed. The robot bumps the walls. Right now the code that makes
the robot move autonomously executes due to AUTO being #defined. Simply remove
it to take control using teleop to see how exploring regions work in roslaunch
terminal.

Even though I failed getting the robot from point A to point B without bumping
into obstacles, the main idea behind the homework was dividing the map into
regions and selecting the closest region to the robot using its center coordinates
(x and y in the class definition). Then, after exploring the first and probably the
obstacle free region, moving onto adjacent regions. While doing that, the robot
is supposed to avoid bumping obstacles. This task alone is path planning and
hard to implement, but I got the big picture here. 

*/

#include "ros/ros.h"
#include "nav_msgs/OccupancyGrid.h"
#include <tf/transform_listener.h>
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"
#include <sstream>
#include <vector>
#include <queue>
#include <iostream>
#include <list>
#include <cmath>

#define REGION_WIDTH	38	// 100 regions if the map is ~380x380
#define REGION_HEIGHT	38

#define CLOSENESS_THRESH 0.1 // 0.1m = 10cm - want to get this close to waypoint
#define LINEAR_VEL 2 //will go this fast in forward direction
#define ANGULAR_VEL 0.5 //will turn this fast
#define TURN_THRESH 0.2 // will be facing within 0.2 radians before will go forward

#define PI 3.141592653589793

#define AUTO	// for autonomous action. otherwise -> teleop

using namespace std;

class Node {
public:
	Node(int x, int y, int id) : x(x), y(y), id(id) { discovered = false; }

	void addAdjNode(Node* n) 		{ adjList.push_back(n); }
	list<Node*> getAdj() const		{ return adjList; 		}
	void setDisc(bool what) 		{ discovered = what; 	}
	bool DISC() const				{ return discovered;	}
	int X() const 					{ return x;				}
	int Y()	const 					{ return y;				}
	int ID() const					{ return id;			}
private:
	bool discovered;		// is the region discovered?
	int x, y;				// coordinates of the Node
	list<Node*> adjList;	// list of adjacent nodes
							// that have direct connection
	int id;
};

typedef std::list<Node*> lN; 
typedef list<Node*>::iterator lNi; 

//////////////////	GLOBAL VARIABLES

int mapW, mapH;

list<Node*> regions;	// map sampling
bool initd = false;		// samples are not initalized
ros::Publisher movement_publisher;			// velocity publisher	
boost::shared_ptr<tf::TransformListener> tf_listener_; //smart pointer would be better than naked
tf::TransformListener* tf_listener = NULL;	// transform listener

double waypoint_x, waypoint_y;
bool onTheWay = false;

////////////////// 	END OF GLOBALS

int coord2Index(int x, int y){
	return y*mapH + x;
}

double distance(double x1, double x2, double y1, double y2){
	return sqrt(pow((x1-x2),2) + pow((y1-y2),2));
}




void doSampling(vector<signed char, allocator<signed char> > map){

	// divide the map into regions
	int count = 0;
	for(size_t i=0; i< map.size(); ++i ){
		int x = i % mapW;
		int y = i / mapW;
		if(x != 0 && y != 0){
			if(x % REGION_WIDTH == 0 && y % REGION_HEIGHT == 0){
				count++;	// push the center of the region as X and Y
				regions.push_back(new Node(x-REGION_WIDTH/2, y-REGION_HEIGHT/2, count));
			}
		}
	}

	// print the resulting regions after sampling
	cout << endl << "Sampling completed with a total of " << regions.size() << " regions\n";
	cout << "==================================================" << endl;
	
	for(lNi it = regions.begin() ; it != regions.end() ; ++it){
		//int center = (*it)->X() + (*it)->Y() * MAP_HEIGHT;
		//int regionSize = REGION_HEIGHT*REGION_WIDTH;
		int regionBegin = coord2Index((*it)->X()-REGION_WIDTH/2, (*it)->Y()-REGION_HEIGHT/2);
		int regionEnd = coord2Index((*it)->X()+REGION_WIDTH/2, (*it)->Y()+REGION_HEIGHT/2);
		cout << "Region " << (*it)->ID() << " (" 
			<< (*it)->X() << "," << (*it)->Y()
			<< "):\tBegin: " << regionBegin 
			<< "\tEnd: " << regionEnd << endl;
	}

	onTheWay = true;
	initd = true;
	return;
}

bool isDiscovered(Node* n, vector<signed char, allocator<signed char> > map){

	int count = 0;
	//int center = n->X() + n->Y() * MAP_WIDTH;
	int regionSize = REGION_HEIGHT*REGION_WIDTH;
	int regionBegin = coord2Index(n->X()-REGION_WIDTH/2, n->Y()-REGION_HEIGHT/2);
	//int regionEnd = coord2Index(n->X()+REGION_WIDTH/2, n->Y()+REGION_HEIGHT/2);;

	// scan the region and count discovered pixels
	for(int j = 0 ; j < REGION_HEIGHT ; j++){
		for(int i = regionBegin ; i < regionBegin + REGION_WIDTH-1 ; ++i){
			if(map[i + j*mapW] > 50)
				count++;	
		}
	}

	// if 1/5 of the region is discovered, return true
	//if (count > regionSize*1/10 ) cout << "count(" << n->ID() << "): " << count << endl;
	return count > regionSize*1/5 ? true : false;
}

void analyzeRegions(vector<signed char, allocator<signed char> > map){

	// iterate through regions, print the discovered regions
	for(lNi it = regions.begin() ; it != regions.end() ; ++it){
		if( isDiscovered(*it, map) ){		
			(*it)->setDisc(true);
		}
	}
}

void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg){

	// process map info
 	vector<signed char , allocator<signed char> > map =  msg.get()->data;
	int unknownArea=0;
	int allArea=0;
 	int discoveredArea=0;

	// read map size
	mapW = msg.get()->info.width;
	mapH = msg.get()->info.height;

 	for(size_t i=0; i< map.size(); ++i ){
		allArea++;
		/*cout << "(" << ((float)(i%mapW))/38.4 << ", "
					<< ((float)(i/mapW)) << ")" << endl;*/	

		if(map[i] >50)
			discoveredArea++;
		if(map[i]<0)
			unknownArea++; 
		if(map[i]==0 && false){
			cout << "(" << ((float)(i%mapW))/38.4 -5 << ", "
						<< ((float)(i/mapW))/38.4 -5 << ") is black." << endl;	
		}
		
	}



	// if the sampling is not done, do it (executed only once)
	if(initd == false)
		doSampling(map);
	
	// analyze regions
	analyzeRegions(map);

	// output	
	cout << "Map W: " << mapW << "\tMap H: " << mapH << "\tRes: " << msg.get()->info.resolution << endl;
	ROS_WARN("Discovered obstacle area: %d",discoveredArea); //Success Measure
	ROS_WARN("Unknown area: %d",unknownArea); 
	ROS_WARN("All area: %d",allArea); 
	ROS_WARN("Total Regions: %d",(int)regions.size());
	ROS_WARN(" ");
	for(lNi it = regions.begin() ; it != regions.end() ; ++it){
		if((*it)->DISC() == true)
			ROS_WARN("REGION %d IS DISCOVERED", (*it)->ID());
	}
	ROS_WARN("Pose: x=%f,y=%f,theta=%f",msg->info.origin.position.x,
			msg->info.origin.position.y,
			2*asin(msg->info.origin.orientation.z)*(msg->info.origin.orientation.w<0?-1:1));

	//waypoint_x = msg->info.origin.position.x;
	//waypoint_y = msg->info.origin.position.y;
	waypoint_x = 1.5;
	waypoint_y = -0.5;
	

	// print robot transform
	tf::StampedTransform transform;
	try{
	
		// read and print current position and current goal position
		(*tf_listener).lookupTransform("/map", "/base_link", ros::Time(0), transform);
		double x,y,z;
		x = transform.getOrigin().x();	
		y = transform.getOrigin().y();	
		z = transform.getOrigin().z();		
		
		cout << "******* Transform ************\n";
		cout << "X: " << x << "\tY: " << y << "\tZ: " << z 
			<< "\tAngle: " << transform.getRotation().getAngle() << endl;
			
		double map_x, map_y, map_z;
		map_x = x;
		map_y = y;
		
		unsigned int grid_x = (unsigned int)((map_x - msg->info.origin.position.x) / msg->info.resolution);
		unsigned int grid_y = (unsigned int)((map_y - msg->info.origin.position.y) / msg->info.resolution);
		cout << "*****  OccupancyGrid ********\n";
		cout << "Grid X: " << grid_x << "\tGrid Y: " << grid_y << endl;
		
	}catch(tf::TransformException ex){
		ROS_ERROR("%s", ex.what());
	}
	

}

void laserCallback(const sensor_msgs::LaserScan::ConstPtr& scan){

	#ifdef AUTO
	// TAKE CAUTION, IF THERE'S OBSTACLE AVOID IT
	geometry_msgs::Twist motion;
	bool obstacle = false;

	
	//movement_publisher.publish(motion);

	////////////////////////////////////////////////////////////////////////
	
	// IF THERE IS A GOAL POINT, ACT.
	if(onTheWay == true){
		cout << "on the way " << endl;
	
		double robot_x,robot_y,robot_orientation;
		ros::Rate rate(100); //100 times a second; should be fast enough!
		robot_x=robot_y=std::numeric_limits<double>::infinity(); 

		// 0.1m = 10cm - want to get this close 
		while (distance(waypoint_x, robot_x, waypoint_y, robot_y) > CLOSENESS_THRESH){ 
			if (!ros::ok()){
				std::cout<<"Interrupted while route following."<<std::endl;
				return;
			}
			rate.sleep();

			//GET ROBOT LOCATION AND ORIENTATION
			tf::StampedTransform robot_transform;
			try{
				tf_listener_->lookupTransform("/odom","/base_link",ros::Time(0),robot_transform);
				robot_x = robot_transform.getOrigin().x();
				robot_y = robot_transform.getOrigin().y();
				robot_orientation = robot_transform.getRotation().getAngle()*
									robot_transform.getRotation().getAxis().getZ();

			//DEBUF INFO 		std::cout<<"angle: "<<robot_transform.getRotation().getAngle()<<" axis Z: "<<robot_transform.getRotation().getAxis().getZ()<<std::endl;
			}catch (tf::TransformException ex){
				continue;
			}

			//DEBUG INFO         std::cout<<"   robot_x="<<robot_ex<<   "    robot_y="<<robot_y<<std::endl;
			//DEBUG INFO         std::cout<<"waypoint_x="<<waypoint_x<<" waypoint_y="<<waypoint_y<<std::endl;

			//GET ANGLE OF VECTOR FROM ROBOT TO TARGET
			double direction_vector_angle = atan2(waypoint_y-robot_y,waypoint_x-robot_x);

			//LOOK AT DIFFERENCE BETWEEN THIS ANGLE AND CURRENT ANGLE
			double angle_diff=direction_vector_angle-robot_orientation;
			while(angle_diff<-PI) angle_diff+=2*PI; //normalise to between -pi and pi 
			while(angle_diff>PI) angle_diff-=2*PI; //normalise to between -pi and pi 

			//DEBUG INFO          std::cout<<direction_vector_angle<<"(atan2("<<waypoint_y-robot_y<<","<<waypoint_x-robot_x<<"))-"<<robot_orientation<<"="<<angle_diff<<std::endl;

			//MAKE A DECISION
			geometry_msgs::Twist cmd;
			if(angle_diff>TURN_THRESH){
				cmd.linear.x=0;
				cmd.angular.z=ANGULAR_VEL;
			//DEBUG INFO              std::cout<<"ANTICLOCKWISE"<<std::endl;
			}else if(angle_diff<-TURN_THRESH){
				cmd.linear.x=0;
				cmd.angular.z=-ANGULAR_VEL;
			//DEBUG INFO              std::cout<<"CLOCKWISE"<<std::endl;
			}else{
				cmd.linear.x=LINEAR_VEL;
				cmd.angular.z=0;
			//DEBUG INFO              std::cout<<"STRAIGHT"<<std::endl;
			}

			//Laser has 270 degree of scanning capacity. Range 76 stands for 30 degrees.
			/*for(int j=0; j<76; j++){

				//Checking whether the 30 degree is obstacle-free or not.
				if(scan->ranges[(4*76)+j] <= 2){
					obstacle = true;
					break;
				}
			}
	
			if(obstacle == true){
				cmd.linear.x = 0.0;	
				cmd.angular.z = -1.0;
			}*/
			movement_publisher.publish(cmd);
			
		}

		// the point is reached, so do not act anymore
		onTheWay = false;
	}
	#endif



}

int main(int argc, char **argv){
  	ros::init(argc, argv, "searchAndDiscover");
  	ros::NodeHandle n;
  	ros::Subscriber map_reader =n.subscribe("map",3,mapCallback);
	movement_publisher = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);
	ros::Subscriber sub = n.subscribe("/base_scan/scan", 1, laserCallback);

	// sort transform listener
	tf_listener = new tf::TransformListener();	
	tf_listener_ = boost::shared_ptr<tf::TransformListener>(new tf::TransformListener());
  	ros::spin();
	delete tf_listener;
  	return 0;
}

