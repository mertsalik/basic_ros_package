#include "ros/ros.h"
#include "nav_msgs/OccupancyGrid.h"
#include <tf/transform_listener.h>
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"

#include <sstream>
#include <iostream>
#include <iomanip>

#include <list>
#include <vector>
#include <queue>

#include <cmath>
#include <ctime>
#include <cstdlib>
#include <cassert>

#include "Node.cpp"
#include "Box2DHelper.hpp"

#define CLOSENESS_THRESH 0.1 // 0.1m = 10cm - want to get this close to waypoint
#define LINEAR_VEL 1.0 //will go this fast in forward direction
#define ANGULAR_VEL 0.5 //will turn this fast
#define TURN_THRESH 0.2 // will be facing within 0.2 radians before will go forward
#define GOAL_THRESH 0.2 // closeness threshold for goal point

#define PI 3.141592653589793

#define RRTe	// RRT \ PRM
#define RRT_VERTICE_COUNT 200
#define DELTA_Q 100

using namespace std;

//////////////////	GLOBAL VARIABLES

ros::Publisher movement_publisher;			// velocity publisher	
boost::shared_ptr<tf::TransformListener> tf_listener_; //smart pointer would be better than naked
tf::TransformListener* tf_listener = NULL;	// transform listener

double waypoint_x = 0, waypoint_y = 0;
double goal_x = 2.75, goal_y = 3.8;
Node* goalNode = NULL;
list<Node*> path;

bool reached = false;
bool obstacle = false;


////// BOX2D VARIABLES

RosWorld *ros_world;

///// RRT VARIABLES

list<Node*> RRT_graph;
bool isGraphBuilt = false;

////////////////// 	END OF GLOBALS

double distance(const double x1, const double x2, const double y1, const double y2){
    return sqrt(pow((x1-x2),2) + pow((y1-y2),2));
}

Node* nearestVertex(list<Node*>& g, Node*& n){
    double dist = 1000000000;
    list<Node*>::iterator nearest;
    for(list<Node*>::iterator it = g.begin() ; it != g.end() ; ++it){
        if( distance(n->x(), (*it)->x(), n->y(), (*it)->y()) < dist){
            nearest = it;
            dist = distance(n->x(), (*it)->x(), n->y(), (*it)->y());
        }
    }
    
    return *nearest;
    
}

bool isColliding(Node* n1, Node* n2){
    return false;
}

Node* new_conf(Node* n, Node* r){
    if(isColliding(n, r)){
        Node* collisionFree;
        
        return collisionFree;
    }
    else
        return r;
}

void build_RRT_graph(double init_x, double init_y){
    srand((unsigned)time(NULL));
    
    // This algorithm is derived from the pseudocode at the following link
    // http://en.wikipedia.org/wiki/Rapidly_exploring_random_tree#Algorithm

    // initialize the graph with the initial coordinates of the robot
    RRT_graph.push_back(new Node(init_x, init_y, 0));

    // Randomly sample RRT_VERTICE_COUNT vertices (might not be collision-free)
    // collision detection will be done later
    for (unsigned i=1; i<=RRT_VERTICE_COUNT ; ++i){
        
        // get a random x, y
        double x = ((double)(rand()%2000 - 1000))/100;
        double y = ((double)(rand()%2000 - 1000))/100;
        Node* randNode = new Node(x, y, i);
        //cout << " Rand Node " << i << " (" << x << "," << y << ")" <<endl;
        
        // find the nearest node in the graph to the random node
        Node* nearest = nearestVertex(RRT_graph, randNode);
        //cout << " Nearest node in Tree: " << nearest->id() << endl;
        
        // check for collisions and find a suitable point that is 
        // collision free for adding to the tree
        Node* newNode = new_conf(nearest, randNode);
        
        // construct the edge between nodes
        nearest->addAdj(newNode);
        newNode->addAdj(nearest);
        
        // add the new node to the tree
        RRT_graph.push_back(newNode);
        
        //cout << endl;
    }
    
    cout << "\n\n****RRT Graph(" << RRT_graph.size() <<") is Initialized****\n" << endl;
}

void printPath(){
    
    cout << "\n*******   PATH   ***********\n\n";
    cout << fixed << setprecision(3);
    for(list<Node*>::const_iterator it = path.begin() ; it != path.end() ; ++it){
        cout << "Node ID: " << (*it)->id() << "\tX: " << (*it)->x() 
                                           << "\tY: " << (*it)->y() << endl;

    }
    cout << endl << endl;
}

void DFS(list<Node*> l, Node* n){
    //cout << "DFS CALLED" << endl;
    n->setVisited(); 
    //cout << n->id() << " visited" << endl;
    for(list<Node*>::const_iterator it = n->adjs().begin() ; it != n->adjs().end() ; ++it){
        if((*it)->visited() == false){
            (*it)->setParent(n);
            //cout << "  " << (*it)->id() << " setting parent to " << n->id() << endl;
            DFS(n->adjs(), (*it));
        }
        else{
            ;//cout << "  " << (*it)->id() << " already visited" << endl;
        }
    }
}

void initPath(){
    //cout << "PATH INITIALIZING" << endl;
    // find the node in the tree closest to the destination coordinates
    double d = 1000000000;
    for(list<Node*>::const_iterator it = RRT_graph.begin() ; it != RRT_graph.end() ; ++it){
        if( distance((*it)->x(), goal_x, (*it)->y(), goal_y) < d ){
            d = distance((*it)->x(), goal_x, (*it)->y(), goal_y);
            goalNode = (*it);
            
        }
    }
    cout << "Goal node found: " << goalNode->id() << " X: " << goalNode->x()
                                                 <<  " Y: " << goalNode->y() << endl;
    
    // Depth-First Search, set the parent of each node for building a path
    DFS(RRT_graph, RRT_graph.front());
    
    // initialize path list
    for(Node* n = goalNode ; n != RRT_graph.front() ; n = n->parent()){
        path.push_front(n);
    }
    path.push_front(RRT_graph.front());
    
}

void traverseRRT(){
    
    // robot variables
    double robot_x,robot_y,robot_orientation;
    ros::Rate rate(10); //100 times a second; should be fast enough!
    robot_x=robot_y=std::numeric_limits<double>::infinity(); 
    
    for(list<Node*>::iterator it = path.begin() ; it != path.end() ; ++it){
        waypoint_x = (*it)->x();
        waypoint_y = (*it)->y();
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
                robot_orientation = robot_transform.getRotation().getAngle() *
                                    robot_transform.getRotation().getAxis().getZ();

            //DEBUF INFO 		std::cout<<"angle: "<<robot_transform.getRotation().getAngle()<<" axis Z: "<<robot_transform.getRotation().getAxis().getZ()<<std::endl;
            }catch (tf::TransformException ex){
                return;
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
            //DEBUG INFO                                    
                //std::cout<<"ANTICLOCKWISE"<<std::endl;
            }else if(angle_diff<-TURN_THRESH){
                cmd.linear.x=0;
                cmd.angular.z=-ANGULAR_VEL;
            //DEBUG INFO     
                //std::cout<<"CLOCKWISE"<<std::endl;
            }else{
                cmd.linear.x=LINEAR_VEL;
                cmd.angular.z=0;
            //DEBUG INFO      
                //std::cout<<"STRAIGHT"<<std::endl;
            }


            movement_publisher.publish(cmd);
            //reached = false;
        }
        cout << "\n\nWAYPOINT REACHED!!\t" 
               "ID: " << (*it)->id() << " X: " << (*it)->x() 
                                    << " Y: " << (*it)->y()<< endl << endl;
    }
    
    reached = true;
    
    geometry_msgs::Twist stop;
    stop.linear.x=0;
    stop.angular.z=0;
    movement_publisher.publish(stop);
    

        
}


void print(){
    cout << "\nRRT GRAPH with size: " << RRT_graph.size() << endl;
    cout << fixed << setprecision(3);
    for(list<Node*>::const_iterator it = RRT_graph.begin() ; it != RRT_graph.end() ; ++it){
        cout << "Node ID: " << (*it)->id() << "\tX: " << (*it)->x() 
                                           << "\tY: " << (*it)->y() << endl;
        
        cout << "\tAdjacency List:" << endl;
        (*it)->printAdj();
        
        cout << endl;
    }
    cout << endl << endl;
}

void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg){

    // print robot transform
    tf::StampedTransform transform;
    try{

        if(!isGraphBuilt){
            // originally, we were goint to push the robot x,y coordinates
            // as parameters, however the first callback somehow produces
            // incorrect x,y values, therefore we 'assume' the correct
            // coordinates as x:0,05 and y:0,001
            srand((unsigned)time(NULL));
    
            build_RRT_graph(0.05, 0.001);
            print();
            isGraphBuilt = true;
            initPath();
            printPath();
            traverseRRT();
        
        }
        
        // read and print current position and current goal position
        (*tf_listener).lookupTransform("/map", "/base_link", ros::Time(0), transform);
        double x,y;
        x = transform.getOrigin().x();	
        y = transform.getOrigin().y();	
        
        cout << "\n\n******* Transform ************" << endl;
        cout << "X: " << x << "\tY: " << y << endl;

        cout << "GOAL X: " << goal_x << "\tGOAL Y:" << goal_y << endl;
        cout << "Distance to Actual Goal: " << distance(goal_x, x, goal_y, y) << endl << endl;
        
        cout << "Node X: " << path.back()->x() << " Node Y: " << path.back()->y() << endl;
        cout << "Distance to Goal Node: " << distance(x, goalNode->x(), 
                                                     y, goalNode->y()) << endl << endl;
        
        cout << "WP X: " << waypoint_x << "\tWP Y: " << waypoint_y << endl;
        cout << "Distance to Way Point: " << distance(waypoint_x, x, waypoint_y, y) << endl;
        cout << "***************************************\n" << endl;

        if(reached == true){
            cout << "GOAL REACHED! " << endl;
            cout << "Goal Node: ID: " << goalNode->id() << " X: " << goalNode->x() 
                                                        << " Y: " << goalNode->y() << endl;            
        }
        
        
        
    }catch(tf::TransformException ex){
            ROS_ERROR("%s", ex.what());
    }
    
	

}

void initBox2D(){
  ros_world = new RosWorld();
}

int main(int argc, char **argv){
    
    
    ros::init(argc, argv, "searchAndDiscover");
    ros::NodeHandle n;
    initBox2D();
    ros::Subscriber map_reader =n.subscribe("map",3,mapCallback);
    movement_publisher = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);

    // sort transform listener
    tf_listener = new tf::TransformListener();	
    tf_listener_ = boost::shared_ptr<tf::TransformListener>(new tf::TransformListener());
    ros::spin();
    delete tf_listener;
    return 0;
}

