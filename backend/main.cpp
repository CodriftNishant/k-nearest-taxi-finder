#include <iostream>
#include <vector>
#include <algorithm>
#include <utility>
#include <cstdlib>
#include <ctime>
#include <iomanip>
#include <cmath>
#include <fstream>
#include "dynamic_kd_tree.h"
#include "graph.h"

using namespace std;



int main(int argc, char* argv[]) {
    int n;
    
    // Check if running in API mode (called from Node.js server)
    bool apiMode = (argc == 3 || argc == 5);
    bool bookingMode = (argc == 5);

    if (apiMode) {
        int qx, qy, taxiX, taxiY;
        int k = 5; // Always find 5 nearest taxis
        
        if (bookingMode) {
            // Booking Mode: Accept pickup and taxi coordinates
            qx = atoi(argv[1]);
            qy = atoi(argv[2]);
            taxiX = atoi(argv[3]);
            taxiY = atoi(argv[4]);
        } else {
            // Find Mode: Accept pickup coordinates as command-line arguments
            qx = atoi(argv[1]);
            qy = atoi(argv[2]);
        }

        // Load or generate taxi locations
        vector<point> taxiPoints;
        const char* TAXI_STATE_FILE = "taxi_state.txt";
        
        // Try to load existing taxi positions from file
        ifstream inFile(TAXI_STATE_FILE);
        if (inFile.is_open()) {
            int x, y;
            while (inFile >> x >> y) {
                taxiPoints.push_back(point(x, y));
            }
            inFile.close();
            n = taxiPoints.size();
        }
        
        // If no saved state, generate initial taxis
        if (taxiPoints.empty()) {
            srand(42);
            n = 50;
            for (int i = 0; i < n; i++) {
                int x = rand() % 100;
                int y = rand() % 100;
                taxiPoints.push_back(point(x, y));
            }
            
            // Save initial state
            ofstream outFile(TAXI_STATE_FILE);
            for (const auto& p : taxiPoints) {
                outFile << p.x << " " << p.y << "\n";
            }
            outFile.close();
        }

        // Build Dynamic KD-Tree from taxi locations
        DynamicKDTree kdtree(taxiPoints);
        
        // Find k nearest taxis
        point query(qx, qy);
        vector<point> nearest = kdtree.kNearestNeighbors(query, k);

        // Create road network graph for pathfinding
        GridGraph roadNetwork;
        vector<pair<int,int>> taxiLocations;
        for(const auto& taxi : nearest) {
            taxiLocations.push_back({taxi.x, taxi.y});
        }
        
        // Build full random road network in the visible area
        // First, find the bounds of all relevant points
        int minX = qx, maxX = qx, minY = qy, maxY = qy;
        for(const auto& loc : taxiLocations) {
            minX = min(minX, loc.first);
            maxX = max(maxX, loc.first);
            minY = min(minY, loc.second);
            maxY = max(maxY, loc.second);
        }
        
        // Expand bounds significantly to fill the entire visible area
        // Calculate the range and expand by 50% on each side
        int rangeX = maxX - minX;
        int rangeY = maxY - minY;
        int expandX = max(rangeX / 2, 15); // At least 15 units
        int expandY = max(rangeY / 2, 15);
        
        minX -= expandX; maxX += expandX;
        minY -= expandY; maxY += expandY;
        
        // Build a full random graph in this area
        for(int x = minX; x <= maxX; x++) {
            for(int y = minY; y <= maxY; y++) {
                // For each point, randomly connect to 1-4 neighbors
                vector<pair<int,int>> neighbors;
                if(x > minX) neighbors.push_back({x-1, y});
                if(x < maxX) neighbors.push_back({x+1, y});
                if(y > minY) neighbors.push_back({x, y-1});
                if(y < maxY) neighbors.push_back({x, y+1});
                
                if(!neighbors.empty()) {
                    // Randomly select 1-4 neighbors to connect
                    shuffle(neighbors.begin(), neighbors.end(), mt19937(random_device{}()));
                    int numConnections = 1 + (rand() % min(3, (int)neighbors.size()));
                    for(int i = 0; i < numConnections; i++) {
                        roadNetwork.createManhattanPath({x, y}, neighbors[i]);
                    }
                }
            }
        }
        
        // Also ensure paths exist from taxis to pickup
        roadNetwork.buildSparseGraph(taxiLocations, {qx, qy});

        // Output JSON format (to stdout for Node.js to capture)
        if (bookingMode) {
            // Booking Mode: Use graph pathfinding for realistic distance
            int distance = roadNetwork.dijkstra({taxiX, taxiY}, {qx, qy});
            double time = distance * 2.0; // 2 minutes per unit
            
            // Update KD-tree: Remove taxi from old position and add to new position
            point oldPos(taxiX, taxiY);
            point newPos(qx, qy);
            kdtree.deletePoint(oldPos);
            kdtree.insert(newPos);
            
            // Save updated taxi positions to file for persistence
            ofstream outFile(TAXI_STATE_FILE);
            vector<point> allTaxis;
            kdtree.getAllPoints(allTaxis);
            for (const auto& p : allTaxis) {
                outFile << p.x << " " << p.y << "\n";
            }
            outFile.close();
            
            cout << "{";
            cout << "\"success\":true,";
            cout << "\"movedFrom\":{\"x\":" << taxiX << ",\"y\":" << taxiY << "},";
            cout << "\"movedTo\":{\"x\":" << qx << ",\"y\":" << qy << "},";
            cout << "\"distance\":" << fixed << setprecision(2) << distance << ",";
            cout << "\"time\":" << fixed << setprecision(2) << time << ",";
            cout << "\"treeHeight\":" << kdtree.getHeight() << ",";
            cout << "\"treeSize\":" << kdtree.size();
            cout << "}" << endl;
        } else if (nearest.size() > 0) {
            // Find Mode: Calculate graph distances and re-sort by road distance
            
            // Structure to hold taxi info with graph distance
            struct TaxiInfo {
                point node;
                double euclideanDist;
                int graphDist;
                vector<pair<int,int>> path;
            };
            
            vector<TaxiInfo> taxiInfos;
            int taxiCount = min((int)nearest.size(), 5);
            
            // Calculate graph distances for all taxis
            for (int i = 0; i < taxiCount; i++) {
                TaxiInfo info;
                info.node = nearest[i];
                info.euclideanDist = sqrt(nearest[i].distanceSquared(qx, qy));
                info.path = roadNetwork.dijkstraPath({nearest[i].x, nearest[i].y}, {qx, qy});
                info.graphDist = info.path.size() - 1;
                taxiInfos.push_back(info);
            }
            
            // Sort by graph distance (shortest road distance first)
            sort(taxiInfos.begin(), taxiInfos.end(), [](const TaxiInfo& a, const TaxiInfo& b) {
                return a.graphDist < b.graphDist;
            });
            
            cout << "{\"pickup\":{\"x\":" << qx << ",\"y\":" << qy << "},";
            
            // Output road network edges for visualization
            cout << "\"roadNetwork\":[";
            vector<pair<pair<int,int>, pair<int,int>>> edges = roadNetwork.getAllEdges();
            for (size_t i = 0; i < edges.size(); i++) {
                cout << "{\"from\":{\"x\":" << edges[i].first.first << ",\"y\":" << edges[i].first.second << "},";
                cout << "\"to\":{\"x\":" << edges[i].second.first << ",\"y\":" << edges[i].second.second << "}}";
                if (i < edges.size() - 1) cout << ",";
            }
            cout << "],";
            
            // Output all nearest taxis sorted by road distance
            cout << "\"nearestTaxis\":[";
            for (size_t i = 0; i < taxiInfos.size(); i++) {
                double estimatedTime = taxiInfos[i].graphDist * 2.0;
                
                cout << "{";
                cout << "\"rank\":" << (i + 1) << ",";
                cout << "\"location\":{\"x\":" << taxiInfos[i].node.x << ",\"y\":" << taxiInfos[i].node.y << "},";
                cout << "\"euclideanDistance\":" << fixed << setprecision(2) << taxiInfos[i].euclideanDist << ",";
                cout << "\"graphDistance\":" << taxiInfos[i].graphDist << ",";
                cout << "\"estimatedTime\":" << fixed << setprecision(2) << estimatedTime << ",";
                
                // Output the path coordinates
                cout << "\"path\":[";
                for(size_t j = 0; j < taxiInfos[i].path.size(); j++) {
                    cout << "{\"x\":" << taxiInfos[i].path[j].first << ",\"y\":" << taxiInfos[i].path[j].second << "}";
                    if(j < taxiInfos[i].path.size() - 1) cout << ",";
                }
                cout << "]";
                
                cout << "}";
                if (i < taxiInfos.size() - 1) cout << ",";
            }
            cout << "],";
            
            // Output the selected taxi (shortest road distance - first in sorted list)
            const TaxiInfo& selectedTaxi = taxiInfos[0];
            
            cout << "\"nearestTaxi\":{";
            cout << "\"location\":{\"x\":" << selectedTaxi.node.x << ",\"y\":" << selectedTaxi.node.y << "},";
            cout << "\"euclideanDistance\":" << fixed << setprecision(2) << selectedTaxi.euclideanDist << ",";
            cout << "\"graphDistance\":" << selectedTaxi.graphDist << ",";
            cout << "\"estimatedTime\":" << fixed << setprecision(2) << (selectedTaxi.graphDist * 2.0) << ",";
            
            // Output the selected path
            cout << "\"path\":[";
            for(size_t j = 0; j < selectedTaxi.path.size(); j++) {
                cout << "{\"x\":" << selectedTaxi.path[j].first << ",\"y\":" << selectedTaxi.path[j].second << "}";
                if(j < selectedTaxi.path.size() - 1) cout << ",";
            }
            cout << "]";
            
            cout << "}}" << endl;
        } else {
            cout << "{\"pickup\":{\"x\":" << qx << ",\"y\":" << qy << "},\"error\":\"No taxis available\"}" << endl;
        }
    } else {
        // Interactive Console Mode (original behavior)
        srand(time(0));

        cin >> n;
        cout << "Generating " << n << " random taxi locations:" << endl;
        vector<point> taxiPoints;
        for (int i = 0; i < n; i++) {
            int x = rand() % 100;
            int y = rand() % 100;
            taxiPoints.push_back(point(x, y));
            cout << "Taxi " << i << ": (" << x << ", " << y << ")" << endl;
        }

        cout << "\nBuilding Dynamic KD-Tree..." << endl;
        DynamicKDTree kdtree(taxiPoints);

        cout << "\nDynamic KD-Tree built successfully!" << endl;
        cout << "Tree height: " << kdtree.getHeight() << endl;
        cout << "Tree size: " << kdtree.size() << endl;
        cout << "Inorder traversal: ";
        kdtree.inorder();
        cout << endl;

        // Test k-nearest neighbors search
        int qx, qy, k;
        cout << "\n--- K-Nearest Neighbors Search ---" << endl;
        cout << "Enter query point coordinates (x y): ";
        cin >> qx >> qy;
        cout << "Enter number of nearest neighbors to find (k): ";
        cin >> k;

        cout << "\nSearching for " << k << " nearest taxis to point (" << qx << ", " << qy << ")..." << endl;
        
        // Use Dynamic KD-Tree search
        point query(qx, qy);
        vector<point> nearest = kdtree.kNearestNeighbors(query, k);

        cout << "\nFound " << nearest.size() << " nearest taxi(s):" << endl;
        for (int i = 0; i < nearest.size(); i++) {
            double dist = sqrt(nearest[i].distanceSquared(qx, qy));
            cout << i + 1 << ". Taxi at (" << nearest[i].x << ", " << nearest[i].y 
                 << ") - Distance: " << dist << endl;
        }
    }

    return 0;
}
