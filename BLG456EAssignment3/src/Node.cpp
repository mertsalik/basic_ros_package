#include <list>
#include <iostream>

class Node{
public:
    // constructors & destructors
    Node(double x, double y, unsigned id) : x_(x), y_(y), 
                                            id_(id), visited_(false),
                                            parent_(NULL)    {}
    
    // setters & getters
    const std::list<Node*>& adjs() const  { return adjList;   }
    double x() const                { return x_;        }
    double y() const                { return y_;        }
    unsigned id() const             { return id_;       }
    bool visited() const            { return visited_;  }
    Node* parent() const            { return parent_;   }
    
    void setVisited()               { visited_ = true;  }
    void setParent(Node* n)         { parent_ = n;      }
    
    
    // member functions
    void addAdj(Node* n) { 
        //std::cout << "Node " << id_ << " : Pushing back Node w/ ID: " << n->id() << std::endl;
        adjList.push_back(n); 
    }
    
    void printAdj() const{
        for(std::list<Node*>::const_iterator it = adjList.begin(); it != adjList.end() ; ++it){
            std::cout << "\t" << (*it)->id() <<  " " << std::endl;
        }
    }
    
    
private:
    std::list<Node*> adjList;
    double x_, y_;
    unsigned id_;
    bool visited_;
    Node* parent_;
};

