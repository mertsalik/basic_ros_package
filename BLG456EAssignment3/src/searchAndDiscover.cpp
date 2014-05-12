#include "ros/ros.h"
#include "nav_msgs/OccupancyGrid.h"
#include <tf/transform_listener.h>
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"

#include <sstream>
#include <iostream>
#include <iomanip>
#include <fstream>

#include <list>
#include <vector>
#include <queue>
#include <map>

#include <cmath>
#include <ctime>
#include <cstdlib>
#include <cassert>

#include <unistd.h>
#include <sys/types.h>
#include <pwd.h>

#include "Node.cpp"
#include "Box2DHelper.hpp"

#define CLOSENESS_THRESH 0.1 // 0.1m = 10cm - want to get this close to waypoint
#define LINEAR_VEL 1.0 //will go this fast in forward direction
#define ANGULAR_VEL 0.5 //will turn this fast
#define TURN_THRESH 0.2 // will be facing within 0.2 radians before will go forward
#define GOAL_THRESH 0.2 // closeness threshold for goal point

#define PI 3.141592653589793

#define PRM	// RRT \ PRM
#define VERTICE_COUNT 600
#define PRM_NEIGHBOR_COUNT 10
#define MAX_VALUE 100000

using namespace std;

//////////////////	GLOBAL VARIABLES

ros::Publisher movement_publisher;			// velocity publisher	
boost::shared_ptr<tf::TransformListener> tf_listener_; //smart pointer would be better than naked
tf::TransformListener* tf_listener = NULL;	// transform listener

// path variables
double waypoint_x = 0, waypoint_y = 0;
double goal_x = -8.5, goal_y = 4;
Node* goalNode = NULL;
list<Node*> path;
bool reached = false;

// Box2D Variables
RosWorld *ros_world;

// RRT variables
list<Node*> RRT_graph;
bool isGraphBuilt = false;

// PRM variables
list<Node*> PRM_graph;

// directory variables
struct passwd *pw = getpwuid(getuid());
const char *homedir = pw->pw_dir;

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

    if(ros_world!=NULL){

      Point* p1 = new Point(n1->x(),n1->y());
      Point* p2 = new Point(n2->x(),n2->y());

      Point intersection_point(0,0);
      bool result = ros_world->checkRayCast(p1,p2,intersection_point);

      return result;
    }
	else{
            ROS_ERROR("RosWorld is not initlialized!");
            return false;
    }
}

Node* new_conf(Node* n, Node* r){
	// if there is no collision, return the random node
	// else, find the collision free point in the direction of vector(r - n)
    if(isColliding(n, r)){
        Point* p1 = new Point(n->x(),n->y());
        Point* p2 = new Point(r->x(),r->y());

        Point* p3 = ros_world->getNonCollidingPoint(p1,p2); 
        return new Node(p3->getX(),p3->getY(),r->id());
    }
    else
        return r;
}

void build_RRT_graph(double init_x, double init_y){
    // This algorithm is derived from the pseudocode at the following link
    // http://en.wikipedia.org/wiki/Rapidly_exploring_random_tree#Algorithm

    // initialize the graph with the initial coordinates of the robot
    RRT_graph.push_back(new Node(init_x, init_y, 0));

    // Randomly sample RRT_VERTICE_COUNT vertices
    for (unsigned i=1; i<=VERTICE_COUNT ; ++i){

        // get a random x, y
        double x = ((double)(rand()%2000 - 1000))/100;
        double y = ((double)(rand()%2000 - 1000))/100;
        Node* randNode = new Node(x, y, i);

        // make sure the randomly acquired node is collision free
        Point* randPoint = new Point(randNode->x(),randNode->y());
        while(!ros_world->pointAvailable(randPoint)){
                double x = ((double)(rand()%2000 - 1000))/100;
                double y = ((double)(rand()%2000 - 1000))/100;
                randNode = new Node(x, y, i);
                randPoint = new Point(randNode->x(),randNode->y());
        }

        // find the nearest node in the graph to the random node
        Node* nearest = nearestVertex(RRT_graph, randNode);

        // check for collisions and find a suitable point that is 
        // collision free for adding to the tree
        Node* newNode = new_conf(nearest, randNode);

        // construct the edge between nodes
        nearest->addAdj(newNode);
        newNode->addAdj(nearest);

        // add the new node to the tree
        RRT_graph.push_back(newNode);

    }
    
    cout << "\n\n****RRT Graph with " << RRT_graph.size() <<" nodes is Initialized****\n" << endl;
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
    n->setVisited(); 	// mark current node as visited
	
	// for every adjacent node to the current node
    for(list<Node*>::const_iterator it = n->adjs().begin() ; it != n->adjs().end() ; ++it){
        if((*it)->visited() == false){	// if they are not visited
            (*it)->setParent(n);		// set their parent to current node
            DFS(n->adjs(), (*it));		// recursively iterate through them
        }
    }
}

void initPath(){

    // find the closest node to the destination coordinates in the tree  
    double d = 1000000000;
    for(list<Node*>::const_iterator it = RRT_graph.begin() ; it != RRT_graph.end() ; ++it){
        if( distance((*it)->x(), goal_x, (*it)->y(), goal_y) < d ){
            d = distance((*it)->x(), goal_x, (*it)->y(), goal_y);
            goalNode = (*it);
            
        }
    }
    cout << "Goal node found: " << goalNode->id() << " X: " << goalNode->x()
                                                 <<  " Y: " << goalNode->y() << endl;
    
    // Depth-First Search, set the parent of each node for building a path afterwards
    DFS(RRT_graph, RRT_graph.front());
    
    // initialize path list
    for(Node* n = goalNode ; n != RRT_graph.front() ; n = n->parent())
        path.push_front(n);
    
	// lastly, push the initial node to the front of the path
    path.push_front(RRT_graph.front());
    
}

void traversePath(){
    
    // robot variables
    double robot_x,robot_y,robot_orientation;
    ros::Rate rate(1000); //100 times a second; should be fast enough!
    robot_x=robot_y=std::numeric_limits<double>::infinity(); 
    
	// iterate through waypoints in path list
    for(list<Node*>::iterator it = path.begin() ; it != path.end() ; ++it){
        waypoint_x = (*it)->x();
        waypoint_y = (*it)->y();
		
        // 0.1m = 10cm - want to get this close 
		// get the robot to each waypoint
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
        }
        cout << "\n\nWAYPOINT REACHED!!\t" 
               "ID: " << (*it)->id() << " X: " << (*it)->x() 
                                    << " Y: " << (*it)->y()<< endl << endl;
    }
    
    reached = true;
    
	// make robot stop once the goal is reached
    geometry_msgs::Twist stop;
    stop.linear.x=0;
    stop.angular.z=0;
    movement_publisher.publish(stop);
    

        
}

void getClosestNeighbors(list<Node*>& g, Node*& n, list<Node*>& neighbors)
{
    map<double, Node*> m;
    for(list<Node*>::iterator it = g.begin() ; it != g.end() ; ++it){
        m.insert(pair<double, Node*>(distance(n->x(), (*it)->x(), n->y(), (*it)->y()), *it));
    }
    
    m.erase(m.begin());
    
    if(m.size()>=PRM_NEIGHBOR_COUNT)
    {
        for(int i=0; i<PRM_NEIGHBOR_COUNT; i++)
        {
            neighbors.push_back((*m.begin()).second );
            m.erase(m.begin());
        }
    }
}

void build_PRM_graph(double init_x, double init_y){

    // initialize the graph with the initial coordinates of the robot
    PRM_graph.push_back(new Node(init_x, init_y, 0));
    PRM_graph.front()->setValue(0);
    
    // creating random points on the configuration space
    unsigned i = 1;
    do{
        // get a random x, y
        double x = ((double)(rand()%2000 - 1000))/100;
        double y = ((double)(rand()%2000 - 1000))/100;
        Node* randNode = new Node(x, y, i);

        // make sure the randomly acquired node is collision free
        Point* randPoint = new Point(randNode->x(),randNode->y());
        while(!ros_world->pointAvailable(randPoint)){
            double x = ((double)(rand()%2000 - 1000))/100;
            double y = ((double)(rand()%2000 - 1000))/100;
            randNode = new Node(x, y, i);
            randPoint = new Point(randNode->x(),randNode->y());
        }
       
        // add rand node to the graph
        PRM_graph.push_back(randNode);
        
        // id increase
        i++;
        
    }while(PRM_graph.size()<=VERTICE_COUNT);
    

    // setting goal values as node
    PRM_graph.push_back(new Node(goal_x, goal_y, i));

    for(list<Node*>::iterator q = PRM_graph.begin() ; q != PRM_graph.end() ; ++q)
    {
        // find each node's closest neighbors
        list<Node*> neighbors; 
        getClosestNeighbors(PRM_graph, *q, neighbors);
        
        for(list<Node*>::iterator it = neighbors.begin() ; it != neighbors.end() ; ++it)
        {
            
            // check the edge between the node and its each neighbor and path between them
            if(!((*q)->includesAdj(*it)) && !isColliding(*q, *it))
            {
                // create graph undirectedly
                (*q)->addAdj(*it);
                (*it)->addAdj(*q);
           
            }
        }
       
    }
   	
}

void Dijkstra(){
    
    // entire graph is copied to Q
    list<Node*> Q = PRM_graph;
    
    Node* u;
    while(Q.size()>0)
    {             
        u = Q.front();
        for(list<Node*>::iterator it = Q.begin() ; it != Q.end() ; ++it)
        {
            // u has the minimum value all the time
            if(u->value() > (*it)->value())
            {
                u = *it;
            }
        }
        
        Q.remove(u);

        if(u->value() == MAX_VALUE)
        {
            break;
        }
		
        // scanning u's neighbors to set parent and new value
        for(list<Node*>::const_iterator v = (u->adjs()).begin() ; v != (u->adjs()).end() ; ++v)
        {
                                  
            double alt = u->value() + distance(u->x(), (*v)->x(), u->y(), (*v)->y());
            if(alt < (*v)->value())
            {
                (*v)->setValue(alt);
                (*v)->setParent(u);
            }

        }
    }

}

void init_PRM_path()
{
    Node* temp;
    // if goal node does not have parent 
    if(PRM_graph.back()->parent() == NULL)
    {	
        // searching entire graph to find the closest node to goal which have connection to start node
        for(list<Node*>::iterator q = PRM_graph.begin() ; q != PRM_graph.end() ; ++q)
        {
  
            double dist = MAX_VALUE;
            if((*q)->parent() != NULL)
            {
 
                if(distance((*q)->x(),PRM_graph.back()->x(),(*q)->y(),PRM_graph.back()->y()) < dist)
                {
                   dist = distance((*q)->x(),PRM_graph.back()->x(),(*q)->y(),PRM_graph.back()->y());
                   temp = *q;
                }
            }                      
        }

      // temp is the temporary goal node
      PRM_graph.push_back(temp);                 
    }
    
    // at the back of the graph there is the goal node
    for(Node* n = PRM_graph.back() ; n != PRM_graph.front() ; n = n->parent()){
        path.push_front(n);
    }
    path.push_front(PRM_graph.front());

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

void exportGraph(const list<Node*> graph){
    
    ofstream file;
    string dir = homedir;
    dir += "/graph.txt";
    file.open(dir.c_str());
    
    // set all nodes to unvisited
    for(list<Node*>::const_iterator it = graph.begin() ; it != graph.end() ; ++it)
        (*it)->resetVisited();
    

    file.open(dir.c_str());
    if(file.is_open()){
        cout << "\nGRAPH OUTPUT FILE CREATED!" << endl << endl;
        file << graph.size() << endl;
        for(list<Node*>::const_iterator it = graph.begin(); it != graph.end() ; ++it){
            (*it)->setVisited();
            file << (*it)->x() << " " << (*it)->y() << endl;
            
            // avoid redundancy by not printing already visited 
            unsigned count = 0;
            for(list<Node*>::const_iterator ite = (*it)->adjs().begin() ;
                                        ite != (*it)->adjs().end() ; ++ite){
                if((*ite)->visited() == false) count++;
            }
            
            file << count << endl;
            for(list<Node*>::const_iterator ite = (*it)->adjs().begin() ;
                                        ite != (*it)->adjs().end() ; ++ite){
                if((*ite)->visited() == false)
                    file << (*ite)->x() << " " << (*ite)->y() << endl;
            }
            file << endl;
        }


        file.close();
    }
    else{
            cout << "\nERROR OPENING OUTPUT FILE" << endl << endl;
    }
    
	
    dir.clear();
    dir = homedir;
    dir += "/path.txt";
    file.open(dir.c_str());
    if(file.is_open()){
        cout << "\nPATH OUTPUT FILE CREATED!" << endl << endl;
        file << path.size() << endl;		
        for(list<Node*>::const_iterator it = path.begin(); it != path.end() ; ++it)
            file << (*it)->x() << " " << (*it)->y() << endl;

        file.close();
    }

    return;
}

void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg){

    // print robot transform
    tf::StampedTransform transform;
    try{
#ifdef RRT
        if(!isGraphBuilt){

            isGraphBuilt = true;
            // originally, we were goint to push the robot x,y coordinates
            // as parameters, however the first callback somehow produces
            // incorrect x,y values, therefore we 'assume' the correct
            // coordinates as x:0,05 and y:0,001
    
            build_RRT_graph(0.05, 0.001);
            initPath();
            exportGraph(RRT_graph);
            printPath();
            traversePath();	
        
        }
#elif defined(PRM)
        if(!isGraphBuilt){

            isGraphBuilt = true;
            // originally, we were goint to push the robot x,y coordinates
            // as parameters, however the first callback somehow produces
            // incorrect x,y values, therefore we 'assume' the correct
            // coordinates as x:0,05 and y:0,001

            build_PRM_graph(0.05, 0.001);
            Dijkstra();
            init_PRM_path();
            exportGraph(PRM_graph);
            printPath();
            traversePath();	

        }   
#endif 
        // read and print current position and current goal position
        (*tf_listener).lookupTransform("/map", "/base_link", ros::Time(0), transform);
        double x,y;
        x = transform.getOrigin().x();	
        y = transform.getOrigin().y();	
        
        cout << "\n\n******* Transform ************" << endl;
        cout << "X: " << x << "\tY: " << y << endl;

        cout << "GOAL X: " << goal_x << "\tGOAL Y:" << goal_y << endl;
        cout << "Distance to Actual Goal: " << distance(goal_x, x, goal_y, y) << endl << endl;
        
        if(goalNode != NULL){   // if PRM is running, don't print goalNode
            cout << "Node X: " << path.back()->x() << " Node Y: " << path.back()->y() << endl;
            cout << "Distance to Goal Node: " << distance(x, goalNode->x(),         
                                                     y, goalNode->y()) << endl << endl;
        }
        cout << "WP X: " << waypoint_x << "\tWP Y: " << waypoint_y << endl;
        cout << "Distance to Way Point: " << distance(waypoint_x, x, waypoint_y, y) << endl;
        cout << "***************************************\n" << endl;

        if(reached == true){
            cout << "GOAL REACHED! " << endl;
            if (goalNode != NULL){  // print goalNode only in RRT
            cout << "Goal Node: ID: " << goalNode->id() << " X: " << goalNode->x() 
                                             << " Y: " << goalNode->y() << endl;            
            }
            printPath();	        
        }
        
        
        
    }catch(tf::TransformException ex){
            ROS_ERROR("%s", ex.what());
    }
    
	

}

void initBox2D(){
  ros_world = new RosWorld();
}

int main(int argc, char **argv){
    
    srand((unsigned)time(NULL));
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

