#include "Box2DHelper.hpp"

void RosWorld::printShapes(){
        for (b2Body* b = m_world->GetBodyList(); b; b = b->GetNext()) {
            for (b2Fixture* f = b->GetFixtureList(); f; f = f->GetNext()) {
                b2Shape::Type shapeType = f->GetType();
                if ( shapeType == b2Shape::e_polygon )
                {
                    b2PolygonShape* pshape = (b2PolygonShape*)f->GetShape();
                    for(int i=0;i<4;i++){
                        b2Vec2 vertex = pshape->GetVertex(i);
                        //std::cout << "i:" << i << "   ";
                        //std::cout << "x:" << vertex.x << " y:" << vertex.y << std::endl;
                    }
                }
            }
            b2Vec2 w_center = b->GetWorldCenter();
            //std::cout << "w_x:" << w_center.x << " w_y:" << w_center.y << std::endl;
        }
    }; 

RosWorld::RosWorld(int map_type){
	this->map_type = map_type;
        gravity.Set(0,0);
        m_world = new b2World(gravity);
        upper.Set(-10,-10);
        lower.Set(10,10);
	
	std::vector<Point*> obstacles;
	std::vector<bool> directions;
	switch(this->map_type){
	    case 3:
		break;
	    case 2:
		break;
	    case 1:
	    default:
		obstacles.push_back(new Point(4,2));
		directions.push_back(true);
		obstacles.push_back(new Point(0,-5));
		directions.push_back(true);
		obstacles.push_back(new Point(-3,0));
		directions.push_back(false);
		obstacles.push_back(new Point(-7,5));
		directions.push_back(false);
		break;
	}
	
	for(int i=0; i<obstacles.size(); i++){
	    Point* temp = obstacles[i];
	    Obstacle* block = new Obstacle(temp, directions[i]);
	    objects.push_back(block);
	}
	
        /*
        Obstacle* block1 = new Obstacle(new Point(B1_X,B1_Y), true);
        Obstacle* block2 = new Obstacle(new Point(B2_X,B2_Y), true);
        Obstacle* block3 = new Obstacle(new Point(B3_X,B3_Y), false);
        Obstacle* block4 = new Obstacle(new Point(B4_X,B4_Y), false);
        
        objects.push_back(block1);
        objects.push_back(block2);
        objects.push_back(block3);
        objects.push_back(block4);
        */
	
	for(int i=0; i<objects.size(); i++){
	    Obstacle* block = objects[i];
	    b2Body* _b1 = createObject(block->getBL(),block->getBR(),block->getTL(),block->getTR(),block->getCenter());
	}
	/*
        b2Body* _b1 = createObject(block1->getBL(),block1->getBR(),block1->getTL(),block1->getTR(),block1->getCenter());
        b2Body* _b2 = createObject(block2->getBL(),block2->getBR(),block2->getTL(),block2->getTR(),block2->getCenter());
        b2Body* _b3 = createObject(block3->getBL(),block3->getBR(),block3->getTL(),block3->getTR(),block3->getCenter());
        b2Body* _b4 = createObject(block4->getBL(),block4->getBR(),block4->getTL(),block4->getTR(),block4->getCenter());
	*/
    };
    
bool RosWorld::pointAvailable(Point* p){
        b2Vec2 checkPoint(p->getX(),p->getY());
        return !onAnyObject(checkPoint);
    };

bool RosWorld::checkRayCast(Point* p1, Point* p2, Point &output){
        Point* temp;
        double x_diff = p2->getX() - p1->getX();
        double y_diff = p2->getY() - p1->getY();
        
        for(int i=10;i>0;i--){
            temp = new Point((x_diff * 0.10 * i) + p1->getX() , (y_diff * 0.10 * i) + p1->getY());
            //std::cout << i <<") "  << "Checking : " << temp->toString() << std::endl;
            bool available = pointAvailable(temp);
            if(!available){
                //std::cout << "COLLISION DETECTED" << std::endl;
                return true;
            }
        }
        
        return false;
    };
   
Point* RosWorld::getNonCollidingPoint(Point* p1, Point* p2){
        Point output(0,0);
        bool result;
        Point* temp;
        double x_diff = p2->getX() - p1->getX();
        double y_diff = p2->getY() - p1->getY();
        
        for(int i=9;i>=0;i--){
            temp = new Point( (x_diff * 0.10 * i) + p1->getX() , (y_diff * 0.10 * i) + p1->getY());
            result = checkRayCast(p1,temp,output);
            if(!result)
                return temp;
        }
        return temp;
    };
    
bool RosWorld::onAnyObject(b2Vec2 &p){
        
        for (int i=0; i<objects.size(); i++){
            double left = objects[i]->getBL()->getX();
            double right = objects[i]->getBR()->getX();
            double top = objects[i]->getTL()->getY();
            double bottom = objects[i]->getBL()->getY();
            
            if((left <= p.x && right >= p.x) && (bottom<=p.y && top>=p.y) ){
                return true;
            }
        }
        
        if(p.x <= -9 || p.x >= 9 || p.y >= 9 || p.y <= -9)
	  return true;
	else
	  return false;
    };
    
b2Body* RosWorld::createObject(Point* bl,Point* br, Point* tl, Point* tr, Point* center){
        b2FixtureDef myFixtureDef;
        myBodyDef.type = b2_dynamicBody;
        
        b2Vec2 vertices[5];
        vertices[0].Set( tl->getX(), tl->getY());
        vertices[1].Set( tr->getX(), tr->getY());
        vertices[2].Set( br->getX(), br->getY());
        vertices[3].Set( bl->getX(), bl->getY());
        vertices[4].Set( tl->getX(), tl->getY());
        
        b2PolygonShape polygonShape;
        polygonShape.Set(vertices, 4);
        
        myFixtureDef.shape = &polygonShape; //change the shape of the fixture
        myBodyDef.position.Set(center->getX(), center->getY()); //in the middle
        b2Body* dynamicBody2 = m_world->CreateBody(&myBodyDef);
        dynamicBody2->CreateFixture(&myFixtureDef); //add a fixture to the body
        
        return dynamicBody2;
        
    };
    
// [Obsolete]
bool RosWorld::checkRayCast(b2Vec2 p1, b2Vec2 p2, b2Vec2 &output){
        
        int32 childIndex = 0;
        
        b2RayCastInput input;
        input.p1 = p1;
        input.p2 = p2;
        input.maxFraction = 1;
        
        //check every fixture of every body to find closest
        float closestFraction = 1; //start with end of line as p2
        b2Vec2 intersectionNormal(0,0);
        for (b2Body* b = m_world->GetBodyList(); b; b = b->GetNext()) {
            for (b2Fixture* f = b->GetFixtureList(); f; f = f->GetNext()) {
                //std::cout << "fixture" << std::endl;
                b2RayCastOutput output;
                if ( ! f->RayCast( &output, input, childIndex ) )
                    continue;
                if ( output.fraction < closestFraction ) {
                    //std::cout << "Oups!" << std::endl;
                    closestFraction = output.fraction;
                    intersectionNormal = output.normal;
                }
            }
        }
        
        b2Vec2 intersectionPoint = p1 + closestFraction * (p2 - p1);
        //std::cout << "ip x:" << intersectionPoint.x << " y:" << intersectionPoint.y << std::endl;
        
        output = intersectionPoint;
        
        if ( closestFraction == 1 )
            return false; //ray hit nothing so we can finish here
        if ( closestFraction == 0 )
            return false;
        
        // else
        return true;
    };