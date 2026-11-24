#ifndef GRAPH_H
#define GRAPH_H

#include <iostream>
#include <unordered_map>
#include <vector>
#include <random>
#include <algorithm>
#include <utility>
#include <climits>
#include <queue>
#include <set>

using namespace std;

// Hash function for pair<int, int>
struct PairHash {
    size_t operator()(const pair<int, int>& p) const {
        // Combine hashes using a standard method
        size_t h1 = hash<int>{}(p.first);
        size_t h2 = hash<int>{}(p.second);
        return h1 ^ (h2 << 1);
    }
};

class GridGraph {
private:
    unordered_map<pair<int, int>, vector<pair<int, int>>, PairHash> adjacencyList;
    const int MIN_COORD = -100;
    const int MAX_COORD = 100;
    mt19937 rng;

    // Check if coordinates are within bounds
    bool isValid(int x, int y) const {
        return x >= MIN_COORD && x <= MAX_COORD && y >= MIN_COORD && y <= MAX_COORD;
    }

    // Get all valid neighbors of a node
    vector<pair<int, int>> getValidNeighbors(int x, int y) const {
        vector<pair<int, int>> neighbors;
        // Top, Bottom, Left, Right
        vector<pair<int, int>> directions = {{0, 1}, {0, -1}, {-1, 0}, {1, 0}};

        for (const auto& dir : directions) {
            int nx = x + dir.first;
            int ny = y + dir.second;
            if (isValid(nx, ny)) {
                neighbors.push_back({nx, ny});
            }
        }
        return neighbors;
    }

    // Add bidirectional edge
    void addEdge(const pair<int, int>& node1, const pair<int, int>& node2) {
        adjacencyList[node1].push_back(node2);
        adjacencyList[node2].push_back(node1);
    }

    // Check if edge already exists
    bool hasEdge(const pair<int, int>& node1, const pair<int, int>& node2) const {
        auto it = adjacencyList.find(node1);
        if (it == adjacencyList.end()) return false;

        const auto& neighbors = it->second;
        return find(neighbors.begin(), neighbors.end(), node2) != neighbors.end();
    }

public:
    GridGraph() : rng(random_device{}()) {}

    // Build sparse grid graph (for testing)
    void buildGraph() {
        const int GRID_SPACING = 5; // Only create nodes every 5 units for sparsity
        
        // First, initialize sparse grid nodes
        for (int x = MIN_COORD; x <= MAX_COORD; x += GRID_SPACING) {
            for (int y = MIN_COORD; y <= MAX_COORD; y += GRID_SPACING) {
                adjacencyList[{x, y}] = vector<pair<int, int>>();
            }
        }

        // For each node, connect to nearby nodes with reduced probability
        for (int x = MIN_COORD; x <= MAX_COORD; x += GRID_SPACING) {
            for (int y = MIN_COORD; y <= MAX_COORD; y += GRID_SPACING) {
                pair<int, int> currentNode = {x, y};

                // Get valid neighbors (only those on the sparse grid)
                vector<pair<int, int>> validNeighbors;
                vector<pair<int, int>> directions = {
                    {GRID_SPACING, 0}, {-GRID_SPACING, 0}, 
                    {0, GRID_SPACING}, {0, -GRID_SPACING}
                };
                
                for (const auto& dir : directions) {
                    int nx = x + dir.first;
                    int ny = y + dir.second;
                    if (isValid(nx, ny) && adjacencyList.find({nx, ny}) != adjacencyList.end()) {
                        validNeighbors.push_back({nx, ny});
                    }
                }

                if (validNeighbors.empty()) continue;

                // Check existing connections
                auto it = adjacencyList.find(currentNode);
                int existingConnections = (it != adjacencyList.end()) ? it->second.size() : 0;

                // Sparse graph: only 1-2 connections per node typically
                if (existingConnections >= 2) {
                    continue; // Already well connected
                }

                // Shuffle neighbors for randomness
                shuffle(validNeighbors.begin(), validNeighbors.end(), rng);

                // Determine connections: 1-2 per node for sparsity
                uniform_int_distribution<int> dist(1, 2);
                int targetConnections = dist(rng);
                int maxNewConnections = min(targetConnections - existingConnections, (int)validNeighbors.size());
                
                if (maxNewConnections <= 0) continue;

                // Add edges to selected neighbors
                int connectionsAdded = 0;
                for (size_t i = 0; i < validNeighbors.size() && connectionsAdded < maxNewConnections; ++i) {
                    const auto& neighbor = validNeighbors[i];

                    if (!hasEdge(currentNode, neighbor)) {
                        addEdge(currentNode, neighbor);
                        connectionsAdded++;
                    }
                }
            }
        }
    }

    // Build a sparse graph connecting specific nodes (for taxi routing)
    void buildSparseGraph(const vector<pair<int,int>>& taxi_locations, pair<int,int> pickup) {
        // Create paths from pickup to each taxi using Manhattan routing
        for(const auto& taxi : taxi_locations) {
            createManhattanPath(pickup, taxi);
        }
    }

    // Create Manhattan path between two points
    void createManhattanPath(pair<int,int> from, pair<int,int> to) {
        int x = from.first;
        int y = from.second;

        while(x != to.first || y != to.second) {
            pair<int,int> current = {x, y};

            // Ensure node exists in adjacency list
            if(adjacencyList.find(current) == adjacencyList.end()) {
                adjacencyList[current] = vector<pair<int,int>>();
            }

            // Move horizontally first, then vertically
            if(x < to.first) {
                pair<int,int> next = {x+1, y};
                if(!hasEdge(current, next)) addEdge(current, next);
                x++;
            } else if(x > to.first) {
                pair<int,int> next = {x-1, y};
                if(!hasEdge(current, next)) addEdge(current, next);
                x--;
            } else if(y < to.second) {
                pair<int,int> next = {x, y+1};
                if(!hasEdge(current, next)) addEdge(current, next);
                y++;
            } else if(y > to.second) {
                pair<int,int> next = {x, y-1};
                if(!hasEdge(current, next)) addEdge(current, next);
                y--;
            }
        }
    }

    // Dijkstra's algorithm to find shortest path and return the path
    vector<pair<int, int>> dijkstraPath(pair<int, int> start, pair<int, int> end) {
        vector<pair<int, int>> path;
        if(start == end) {
            path.push_back(start);
            return path;
        }

        unordered_map<pair<int, int>, int, PairHash> distances;
        unordered_map<pair<int, int>, pair<int, int>, PairHash> previous;
        priority_queue<pair<int, pair<int, int>>,
                           vector<pair<int, pair<int, int>>>,
                           greater<pair<int, pair<int, int>>>> pq;

        distances[start] = 0;
        pq.push({0, start});

        while(!pq.empty()) {
            int currentDist = pq.top().first;
            pair<int, int> currentNode = pq.top().second;
            pq.pop();

            // Found the destination
            if(currentNode == end) {
                // Reconstruct path
                pair<int, int> current = end;
                while(current != start) {
                    path.push_back(current);
                    current = previous[current];
                }
                path.push_back(start);
                reverse(path.begin(), path.end());
                return path;
            }

            // Skip if we've already found a better path
            if(distances.count(currentNode) && currentDist > distances[currentNode]) {
                continue;
            }

            // Explore neighbors
            auto it = adjacencyList.find(currentNode);
            if(it == adjacencyList.end()) continue;

            for(const auto& neighbor : it->second) {
                int newDist = currentDist + 1;

                if(!distances.count(neighbor) || newDist < distances[neighbor]) {
                    distances[neighbor] = newDist;
                    previous[neighbor] = currentNode;
                    pq.push({newDist, neighbor});
                }
            }
        }

        // If no path found, return Manhattan path as fallback
        int x = start.first, y = start.second;
        path.push_back({x, y});
        
        while(x != end.first || y != end.second) {
            if(x < end.first) x++;
            else if(x > end.first) x--;
            else if(y < end.second) y++;
            else if(y > end.second) y--;
            path.push_back({x, y});
        }
        
        return path;
    }

    // Dijkstra's algorithm to find shortest path distance
    int dijkstra(pair<int, int> start, pair<int, int> end) {
        if(start == end) return 0;

        unordered_map<pair<int, int>, int, PairHash> distances;
        priority_queue<pair<int, pair<int, int>>,
                           vector<pair<int, pair<int, int>>>,
                           greater<pair<int, pair<int, int>>>> pq;

        distances[start] = 0;
        pq.push({0, start});

        while(!pq.empty()) {
            int currentDist = pq.top().first;
            pair<int, int> currentNode = pq.top().second;
            pq.pop();

            // Found the destination
            if(currentNode == end) {
                return currentDist;
            }

            // Skip if we've already found a better path
            if(distances.count(currentNode) && currentDist > distances[currentNode]) {
                continue;
            }

            // Explore neighbors
            auto it = adjacencyList.find(currentNode);
            if(it == adjacencyList.end()) continue;

            for(const auto& neighbor : it->second) {
                int newDist = currentDist + 1; // Each edge has weight 1 (unit distance)

                if(!distances.count(neighbor) || newDist < distances[neighbor]) {
                    distances[neighbor] = newDist;
                    pq.push({newDist, neighbor});
                }
            }
        }

        // If no path found, return Manhattan distance as approximation
        return abs(end.first - start.first) + abs(end.second - start.second);
    }

    int getNodeCount() const {
        return adjacencyList.size();
    }

    void printNeighbors(int x, int y) const {
        pair<int, int> node = {x, y};
        auto it = adjacencyList.find(node);

        cout << "Node (" << x << ", " << y << ") has "
                  << (it != adjacencyList.end() ? it->second.size() : 0)
                  << " neighbors: ";

        if (it != adjacencyList.end() && !it->second.empty()) {
            for (size_t i = 0; i < it->second.size(); ++i) {
                const auto& neighbor = it->second[i];
                cout << "(" << neighbor.first << ", " << neighbor.second << ")";
                if (i < it->second.size() - 1) cout << ", ";
            }
        } else {
            cout << "(none)";
        }
        cout << endl;
    }

    // Get all edges in the graph for visualization
    vector<pair<pair<int,int>, pair<int,int>>> getAllEdges() const {
        vector<pair<pair<int,int>, pair<int,int>>> edges;
        set<pair<pair<int,int>, pair<int,int>>> seenEdges;
        
        for (const auto& entry : adjacencyList) {
            const auto& node = entry.first;
            for (const auto& neighbor : entry.second) {
                // Create edge pair (ensure consistent ordering to avoid duplicates)
                auto edge = (node < neighbor) ? make_pair(node, neighbor) : make_pair(neighbor, node);
                
                if (seenEdges.find(edge) == seenEdges.end()) {
                    edges.push_back(edge);
                    seenEdges.insert(edge);
                }
            }
        }
        return edges;
    }

    // Get statistics about the graph
    void printStatistics() const {
        int minDegree = INT_MAX, maxDegree = 0;
        double avgDegree = 0.0;

        for (const auto& entry : adjacencyList) {
            int degree = entry.second.size();
            minDegree = min(minDegree, degree);
            maxDegree = max(maxDegree, degree);
            avgDegree += degree;
        }

        avgDegree /= adjacencyList.size();

        cout << "\nGraph Statistics:" << endl;
        cout << "Minimum degree: " << minDegree << endl;
        cout << "Maximum degree: " << maxDegree << endl;
        cout << "Average degree: " << avgDegree << endl;
    }
};

#endif // GRAPH_H

