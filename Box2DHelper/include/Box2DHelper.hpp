#ifndef _BOX2DHELPER_
#define _BOX2DHELPER_


#include <Box2D/Box2D.h>

#include <iostream>
#include <vector>
#include <string>
#include <sstream>

#define HOR_BLOCK_WIDTH 6.0
#define HOR_BLOCK_HEIGHT 2.0
#define VER_BLOCK_WIDTH 2.0
#define VER_BLOCK_HEIGHT 6.0


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
    Obstacle(Point* point, bool isHorizontal){
        center = new Point(point->getX(),point->getY());
        int width,height;
        if (isHorizontal){
            width = HOR_BLOCK_WIDTH;
            height = HOR_BLOCK_HEIGHT;
        }else{
            width = VER_BLOCK_WIDTH;
            height = VER_BLOCK_HEIGHT;
        }
        
        bottomLeft = new Point(
                               (double)center->getX() - ((double)width/2),
                               (double)center->getY() - ((double)height/2)
                               );
        bottomRight = new Point(
                                (double)center->getX() + ((double)width/2),
                                (double)center->getY() - ((double)height/2)
                                );
        topLeft = new Point(
                            (double)center->getX() - ((double)width/2),
                            (double)center->getY() + ((double)height/2)
                            );
        topRight = new Point(
                             (double)center->getX() + ((double)width/2),
                             (double)center->getY() + ((double)height/2)
                             );
    };
    
    Point* getBL(){ return bottomLeft;};
    Point* getBR(){ return bottomRight;};
    Point* getTR(){ return topRight;};
    Point* getTL(){ return topLeft;};
    
    std::string toString(){
        std::string result;
        result += "Block Points, ";
        result += " BL: " + bottomLeft->toString();
        result += " BR: " + bottomRight->toString();
        result += " TR: " + topRight->toString();
        result += " TL: " + topLeft->toString();
        return result;
    }
};

class RosWorld{
private:
    std::vector<Obstacle*> objects;
    int map_type;
    // BOX2D
    b2World *m_world;
    b2Vec2 gravity;
    b2Vec2 lower;
    b2Vec2 upper;
    b2BodyDef myBodyDef;
    b2FixtureDef myFixtureDef;

    // [Obsolete]
    bool checkRayCast(b2Vec2 p1, b2Vec2 p2, b2Vec2 &output);
    
    b2Body* createObject(Point* bl,Point* br, Point* tl, Point* tr, Point* center);
    
    bool onAnyObject(b2Vec2 &p);

public:
    RosWorld(int map_type);
    
    bool pointAvailable(Point* p);
    
    bool checkRayCast(Point* p1, Point* p2, Point &output);
    
    Point* getNonCollidingPoint(Point* p1, Point* p2);
    
    void printShapes();    
};


#endif