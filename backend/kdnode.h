#ifndef KDNODE_H
#define KDNODE_H

#include "point.h"

class KDNode {
public:
    point p;
    KDNode* left;
    KDNode* right;
    int height;
    
    KDNode(int x, int y) : p(x, y), left(nullptr), right(nullptr) {}
    KDNode(const point& point) 
        : p(point), left(nullptr), right(nullptr), height(1) {}
};

#endif
