#ifndef POINT_H
#define POINT_H

#include <cmath>

using namespace std;

struct point {
    int x;
    int y;
    point() : x(0), y(0) {}  // Default constructor
    point(int x, int y) : x(x), y(y) {}
    
    bool operator==(const point& other) const {
        return x == other.x && y == other.y;
    }

    // Calculate squared Euclidean distance to another point
    double distanceSquared(const point& other) const {
        double dx = x - other.x;
        double dy = y - other.y;
        return dx * dx + dy * dy;
    }
    
    // Calculate Euclidean distance to another point
    double distance(const point& other) const {
        return sqrt(distanceSquared(other));
    }
    
    // Calculate squared distance to coordinates
    double distanceSquared(int px, int py) const {
        double dx = x - px;
        double dy = y - py;
        return dx * dx + dy * dy;
    }
    
    // Calculate distance to coordinates
    double distance(int px, int py) const {
        return sqrt(distanceSquared(px, py));
    }
};

#endif // POINT_H
