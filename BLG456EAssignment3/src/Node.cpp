#include <list>
#include <iostream>

class Node{
public:
    // constructors & destructors
    Node(double x, double y, unsigned id) : x_(x), y_(y), 
                                            id_(id), visited_(false),
                                            parent_(NULL), value_(100000) {}
    
    // setters & getters
    const std::list<Node*>& adjs() const  { return adjList;   }
    double x() const                { return x_;        }
    double y() const                { return y_;        }
    unsigned id() const             { return id_;       }
    bool visited() const            { return visited_;  }
    Node* parent() const            { return parent_;   }
    double value() const            { return value_;	}
    
    void setVisited()               { visited_ = true;  }
    void resetVisited()             { visited_ = false; }
    void setParent(Node* n)         { parent_ = n;      }
    void setValue(double v)         { value_ = v;       }
    
    // member functions
    void addAdj(Node* n) { adjList.push_back(n); }
    
    void printAdj() const{
        for(std::list<Node*>::const_iterator it = adjList.begin(); it != adjList.end() ; ++it){
            std::cout << "\t" << (*it)->id() <<  " " << std::endl;
        }
    }
    
    bool includesAdj(Node* n){
        bool include_flag = false;
        for(std::list<Node*>::iterator it = adjList.begin() ; it != adjList.end() ; ++it){
            if(*it == n){
                include_flag = true;
                break;
            }
        }
        return include_flag;
    }
    
    
private:
    std::list<Node*> adjList;
    double x_, y_;
    unsigned id_;
    bool visited_;
    Node* parent_;
    double value_;
};

