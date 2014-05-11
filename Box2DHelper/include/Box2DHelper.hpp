#ifndef _BOX2DHELPER_
#define _BOX2DHELPER_

#include <Box2D/Box2D.h>

#include <iostream>
#include <vector>
#include <string>
#include <sstream>

#define HOR_BLOCK_WIDTH 5.0
#define HOR_BLOCK_HEIGHT 1.0
#define VER_BLOCK_WIDTH 1.0
#define VER_BLOCK_HEIGHT 5.0

#define B1_X 4
#define B1_Y 2
#define B2_X 0
#define B2_Y -5
#define B3_X -3
#define B3_Y 0
#define B4_X -7
#define B4_Y 5



std::string stringify(double x)
{
    std::ostringstream o;
    if (!(o << x)){
        std::cout << "BAD DOUBLE CONVERSION!" ;
        return "";
    }
    else{
        return o.str();
    }
}

class Point{
private:
    double x,y;
public:
    double getX(){return x;};
    double getY(){return y;}
    
    void setX(double _x){ x= _x;};
    void setY(double _y){ y= _y;};
    
    Point(int _x, int _y){
        x=(double)_x;
        y=(double)_y;
    };
    
    Point(double _x, double _y){
        x=_x;
        y=_y;
    };
    
    std::string toString(){
        return stringify(x) + "," + stringify(y);
    }
};

class Obstacle{
private:
    int id;
    Point* center;
    Point* bottomLeft;
    Point* bottomRight;
    Point* topRight;
    Point* topLeft;
public:
    Point* getCenter(){return center;};
    Obstacle(Point* point, bool isHorizontal);
    
    Point* getBL(){ return bottomLeft;};
    Point* getBR(){ return bottomRight;};
    Point* getTR(){ return topRight;};
    Point* getTL(){ return topLeft;};
    
    std::string toString();
};

class RosWorld{
private:
    std::vector<Obstacle>* objects;
    
    // BOX2D
    b2World *m_world;
    b2Vec2 gravity;
    b2Vec2 lower;
    b2Vec2 upper;
    b2BodyDef myBodyDef;
    b2FixtureDef myFixtureDef;
    
    bool checkRayCast(b2Vec2 p1, b2Vec2 p2, b2Vec2 &output);
    
    b2Body* createObject(Point* bl,Point* br, Point* tl, Point* tr, Point* center);
    
public:
    RosWorld();
    bool checkRayCast(Point* p1, Point* p2, Point &output);
};

#endif