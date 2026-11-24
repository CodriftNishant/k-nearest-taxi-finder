#ifndef DYNAMIC_KD_TREE_H
#define DYNAMIC_KD_TREE_H

#include "kdnode.h"
#include "point.h"
#include <iostream>
#include <vector>
#include <algorithm>
#include <cmath>
#include <queue>
#include <utility>
using namespace std;



// Using Red-Black balance strategy (height difference <= 2x)



class DynamicKDTree {
private:
    KDNode* root;
    // Using Red-Black balance strategy throughout
    
    // Height calculation
    int getHeight(KDNode* node) {
        if (!node) return 0;
        return node->height;
    }
    
    void updateHeight(KDNode* node) {
        if (!node) return;
        node->height = 1 + max(getHeight(node->left), getHeight(node->right));
    }
    
    // Balance factor calculation
    int getBalanceFactor(KDNode* node) {
        if (!node) return 0;
        return getHeight(node->left) - getHeight(node->right);
    }
    
    // Check if node is balanced
    bool isBalanced(KDNode* node) {
        if (!node) return true;
        
        int leftHeight = getHeight(node->left);
        int rightHeight = getHeight(node->right);
        int diff = abs(leftHeight - rightHeight);
        
        // Red-Black balance: allow factor of 2, but handle single-child case
        if (!node->left || !node->right) {
            return diff <= 1;  // Single child: revert to stricter balance
        }
        return diff <= 1 || (rightHeight > 0 && leftHeight <= 2 * rightHeight && rightHeight <= 2 * leftHeight);
    }
    
    // Super key comparison for x:y ordering
    bool compareXY(const point& a, const point& b) {
        if (a.x != b.x) return a.x < b.x;
        return a.y < b.y;
    }
    
    // Super key comparison for y:x ordering
    bool compareYX(const point& a, const point& b) {
        if (a.y != b.y) return a.y < b.y;
        return a.x < b.x;
    }
    
    // Get comparator based on depth (even = x:y, odd = y:x)
    bool compare(const point& a, const point& b, int depth) {
        return (depth % 2 == 0) ? compareXY(a, b) : compareYX(a, b);
    }
    
    // Search for point
    KDNode* search(KDNode* node, const point& p, int depth) {
        if (!node) return nullptr;
        
        if (node->p == p) return node;
        
        bool goLeft = compare(p, node->p, depth);
        return search(goLeft ? node->left : node->right, p, depth + 1);
    }
    
    // Rebuild subtree from all nodes in it - O(n log n)
    KDNode* rebuild(KDNode* node, int depth) {
        if (!node) return nullptr;
        
        vector<point> points;
        collectpoints(node, points);
        
        // Delete old tree
        deleteTree(node);
        
        // Rebuild from sorted points
        return buildBalanced(points, depth, 0, points.size() - 1);
    }
    
    // Collect all points from subtree
    void collectpoints(KDNode* node, vector<point>& points) {
        if (!node) return;
        collectpoints(node->left, points);
        points.push_back(node->p);
        collectpoints(node->right, points);
    }
    
    // Build balanced tree from sorted points
    KDNode* buildBalanced(vector<point>& points, int depth, int start, int end) {
        if (start > end) return nullptr;
        
        // Sort points by current dimension
        sort(points.begin() + start, points.begin() + end + 1,
             [depth, this](const point& a, const point& b) {
                 return compare(a, b, depth);
             });
        
        int mid = (start + end) / 2;
        KDNode* node = new KDNode(points[mid]);
        
        node->left = buildBalanced(points, depth + 1, start, mid - 1);
        node->right = buildBalanced(points, depth + 1, mid + 1, end);
        
        updateHeight(node);
        return node;
    }
    
    // Comparator functions for BuildKDTree algorithm (adapted from kdtree.h)
    bool comp_xy_points(int a, int b, const vector<point>& data) {
        if (data[a].x == data[b].x) {
            return data[a].y < data[b].y;
        }
        return data[a].x < data[b].x;
    }
    
    bool comp_yx_points(int a, int b, const vector<point>& data) {
        if (data[a].y == data[b].y) {
            return data[a].x < data[b].x;
        }
        return data[a].y < data[b].y;
    }
    
    // BuildKDTree algorithm adapted from kdtree.h for initial construction
    KDNode* buildKDTreeFromVector(vector<int>& xy_superKey_temp, vector<int>& yx_superKey_temp, 
                                  bool div_x, const vector<point>& data) {
        int len = xy_superKey_temp.size();
        if (len == 0) return nullptr;
        
        int mid = len / 2;
        int idx;
        KDNode* node;
        
        if (div_x) {
            idx = xy_superKey_temp[mid];
            node = new KDNode(data[idx].x, data[idx].y);
            
            vector<int> left_xy_superKey(xy_superKey_temp.begin(), xy_superKey_temp.begin() + mid);
            vector<int> left_yx_superKey;
            left_yx_superKey.reserve(mid);
            for (int i = 0; i < len; i++) {
                if (comp_xy_points(yx_superKey_temp[i], idx, data) && yx_superKey_temp[i] != idx) {
                    left_yx_superKey.push_back(yx_superKey_temp[i]);
                }
            }
            node->left = buildKDTreeFromVector(left_xy_superKey, left_yx_superKey, !div_x, data);
            
            vector<int> right_xy_superKey(xy_superKey_temp.begin() + mid + 1, xy_superKey_temp.end());
            vector<int> right_yx_superKey;
            right_yx_superKey.reserve(mid);
            for (int i = 0; i < len; i++) {
                if (!comp_xy_points(yx_superKey_temp[i], idx, data) && yx_superKey_temp[i] != idx) {
                    right_yx_superKey.push_back(yx_superKey_temp[i]);
                }
            }
            node->right = buildKDTreeFromVector(right_xy_superKey, right_yx_superKey, !div_x, data);
            
        } else {
            idx = yx_superKey_temp[mid];
            node = new KDNode(data[idx].x, data[idx].y);
            
            vector<int> left_yx_superKey(yx_superKey_temp.begin(), yx_superKey_temp.begin() + mid);
            vector<int> left_xy_superKey;
            left_xy_superKey.reserve(mid);
            for (int i = 0; i < len; i++) {
                if (comp_yx_points(xy_superKey_temp[i], idx, data) && xy_superKey_temp[i] != idx) {
                    left_xy_superKey.push_back(xy_superKey_temp[i]);
                }
            }
            node->left = buildKDTreeFromVector(left_xy_superKey, left_yx_superKey, !div_x, data);
            
            vector<int> right_yx_superKey(yx_superKey_temp.begin() + mid + 1, yx_superKey_temp.end());
            vector<int> right_xy_superKey;
            right_xy_superKey.reserve(mid);
            for (int i = 0; i < len; i++) {
                if (!comp_yx_points(xy_superKey_temp[i], idx, data) && xy_superKey_temp[i] != idx) {
                    right_xy_superKey.push_back(xy_superKey_temp[i]);
                }
            }
            node->right = buildKDTreeFromVector(right_xy_superKey, right_yx_superKey, !div_x, data);
        }
        
        updateHeight(node);
        return node;
    }
    
    // Insert with rebalancing
    KDNode* insertRecursive(KDNode* node, const point& p, int depth, bool& needRebalance) {
        if (!node) {
            needRebalance = false;
            return new KDNode(p);
        }
        
        // point already exists
        if (node->p == p) {
            needRebalance = false;
            return node;
        }
        
        bool goLeft = compare(p, node->p, depth);
        
        if (goLeft) {
            node->left = insertRecursive(node->left, p, depth + 1, needRebalance);
        } else {
            node->right = insertRecursive(node->right, p, depth + 1, needRebalance);
        }
        
        updateHeight(node);
        
        // Check balance and rebalance if needed
        if (!isBalanced(node)) {
            node = rebuild(node, depth);
            needRebalance = true;
        }
        
        return node;
    }
    
    // Find minimum node in dimension
    KDNode* findMin(KDNode* node, int dim, int depth) {
        if (!node) return nullptr;
        
        if (depth % 2 == dim % 2) {
            // We're at the splitting dimension
            if (!node->left) return node;
            return findMin(node->left, dim, depth + 1);
        }
        
        // Need to search both subtrees
        KDNode* minNode = node;
        
        KDNode* leftMin = findMin(node->left, dim, depth + 1);
        if (leftMin && compare(leftMin->p, minNode->p, dim)) {
            minNode = leftMin;
        }
        
        KDNode* rightMin = findMin(node->right, dim, depth + 1);
        if (rightMin && compare(rightMin->p, minNode->p, dim)) {
            minNode = rightMin;
        }
        
        return minNode;
    }
    
    // Find maximum node in dimension
    KDNode* findMax(KDNode* node, int dim, int depth) {
        if (!node) return nullptr;
        
        if (depth % 2 == dim % 2) {
            // We're at the splitting dimension
            if (!node->right) return node;
            return findMax(node->right, dim, depth + 1);
        }
        
        // Need to search both subtrees
        KDNode* maxNode = node;
        
        KDNode* rightMax = findMax(node->right, dim, depth + 1);
        if (rightMax && compare(maxNode->p, rightMax->p, dim)) {
            maxNode = rightMax;
        }
        
        KDNode* leftMax = findMax(node->left, dim, depth + 1);
        if (leftMax && compare(maxNode->p, leftMax->p, dim)) {
            maxNode = leftMax;
        }
        
        return maxNode;
    }
    
    // Delete with rebalancing
    KDNode* deleteRecursive(KDNode* node, const point& p, int depth, bool& found) {
        if (!node) {
            found = false;
            return nullptr;
        }
        
        if (node->p == p) {
            found = true;
            
            // Case 1: Leaf node
            if (!node->left && !node->right) {
                delete node;
                return nullptr;
            }
            
            // Case 2: One child
            if (!node->left || !node->right) {
                KDNode* child = node->left ? node->left : node->right;
                delete node;
                return child;
            }
            
            // Case 3: Two children - find predecessor/successor from higher subtree
            int dim = depth % 2;
            KDNode* replacement = nullptr;
            
            if (getHeight(node->right) >= getHeight(node->left)) {
                replacement = findMin(node->right, dim, depth + 1);
                point tempP = replacement->p;
                node->right = deleteRecursive(node->right, tempP, depth + 1, found);
                node->p = tempP;
            } else {
                replacement = findMax(node->left, dim, depth + 1);
                point tempP = replacement->p;
                node->left = deleteRecursive(node->left, tempP, depth + 1, found);
                node->p = tempP;
            }
            
        } else {
            bool goLeft = compare(p, node->p, depth);
            if (goLeft) {
                node->left = deleteRecursive(node->left, p, depth + 1, found);
            } else {
                node->right = deleteRecursive(node->right, p, depth + 1, found);
            }
        }
        
        if (!node) return nullptr;
        
        updateHeight(node);
        
        // Rebalance if needed
        if (!isBalanced(node)) {
            node = rebuild(node, depth);
        }
        
        return node;
    }
    
    // Clean up tree
    void deleteTree(KDNode* node) {
        if (!node) return;
        deleteTree(node->left);
        deleteTree(node->right);
        delete node;
    }
    
    // KNN search structures
    struct NodeDist {
        KDNode* node;
        double dist;
        NodeDist(KDNode* n, double d) : node(n), dist(d) {}
        bool operator<(const NodeDist& other) const {
            return dist < other.dist;
        }
    };
    
    void knnHelper(KDNode* node, const point& query, int depth,
                   priority_queue<NodeDist>& pq, int k) {
        if (!node) return;
        
        double dist = sqrt(pow(node->p.x - query.x, 2) + pow(node->p.y - query.y, 2));
        
        if ((int)pq.size() < k) {
            pq.push(NodeDist(node, dist));
        } else if (dist < pq.top().dist) {
            pq.pop();
            pq.push(NodeDist(node, dist));
        }
        
        int diff = (depth % 2 == 0) ? (query.x - node->p.x) : (query.y - node->p.y);
        
        KDNode* nearChild = (diff < 0) ? node->left : node->right;
        KDNode* farChild = (diff < 0) ? node->right : node->left;
        
        knnHelper(nearChild, query, depth + 1, pq, k);
        
        if ((int)pq.size() < k || diff * diff < pq.top().dist) {
            knnHelper(farChild, query, depth + 1, pq, k);
        }
    }
    
public:
    DynamicKDTree()
        : root(nullptr) {}
    
    // Constructor that builds tree from initial vector using BuildKDTree algorithm
    DynamicKDTree(const vector<point>& initialPoints)
        : root(nullptr) {
        buildFromVector(initialPoints);
    }
    
    ~DynamicKDTree() {
        deleteTree(root);
    }
    
    // Build initial tree from external vector using BuildKDTree algorithm from kdtree.h
    void buildFromVector(const vector<point>& points) {
        // Clear existing tree
        if (root) {
            deleteTree(root);
            root = nullptr;
        }
        
        if (points.empty()) return;
        
        int n = points.size();
        vector<int> xy_superKey(n);
        vector<int> yx_superKey(n);
        
        // Initialize indices
        for (int i = 0; i < n; i++) {
            xy_superKey[i] = i;
            yx_superKey[i] = i;
        }
        
        // Sort indices using the BuildKDTree comparators
        sort(xy_superKey.begin(), xy_superKey.end(), 
             [&points, this](int a, int b) { return comp_xy_points(a, b, points); });
        sort(yx_superKey.begin(), yx_superKey.end(), 
             [&points, this](int a, int b) { return comp_yx_points(a, b, points); });
        
        // Build tree using the BuildKDTree algorithm
        root = buildKDTreeFromVector(xy_superKey, yx_superKey, true, points);
    }
    
    // Insert point - O(n log n) including rebalancing
    void insert(const point& p) {
        bool needRebalance = false;
        root = insertRecursive(root, p, 0, needRebalance);
    }
    
    // Delete point - O(n log n) including rebalancing
    bool deletePoint(const point& p) {
        bool found = false;
        root = deleteRecursive(root, p, 0, found);
        return found;
    }
    
    // Search for point
    bool search(const point& p) {
        return search(root, p, 0) != nullptr;
    }
    
    // K-nearest neighbors - O(log n) average, O(n) worst case
    vector<point> kNearestNeighbors(const point& query, int k) {
        vector<point> result;
        if (!root || k <= 0) return result;
        
        priority_queue<NodeDist> pq;
        knnHelper(root, query, 0, pq, k);
        
        while (!pq.empty()) {
            result.push_back(pq.top().node->p);
            pq.pop();
        }
        
        reverse(result.begin(), result.end());
        return result;
    }
    
    // Get tree height
    int getHeight() {
        return getHeight(root);
    }
    
    // Get number of nodes
    int size() {
        int count = 0;
        countNodes(root, count);
        return count;
    }
    
    void countNodes(KDNode* node, int& count) {
        if (!node) return;
        count++;
        countNodes(node->left, count);
        countNodes(node->right, count);
    }
    
    // Inorder traversal
    void inorder() {
        inorderHelper(root);
        cout << endl;
    }
    
    void inorderHelper(KDNode* node) {
        if (!node) return;
        inorderHelper(node->left);
        cout << "(" << node->p.x << "," << node->p.y << ") ";
        inorderHelper(node->right);
    }
    
    // Get all points in the tree
    void getAllPoints(vector<point>& points) {
        getAllPointsHelper(root, points);
    }
    
    void getAllPointsHelper(KDNode* node, vector<point>& points) {
        if (!node) return;
        points.push_back(node->p);
        getAllPointsHelper(node->left, points);
        getAllPointsHelper(node->right, points);
    }
};

#endif
