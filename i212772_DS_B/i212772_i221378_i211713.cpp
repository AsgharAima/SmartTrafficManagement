#include<conio.h>
#include <iostream>
#include <string>
#include <fstream>
#include <sstream>
#include <climits>
#include <cstdlib>

using namespace std;

// Forward declarations
class CityTrafficNetwork;
class Vehicle;
class TrafficSignal;
class TrafficSignalManager;
class VehicleRoutingSystem;
class CongestionMonitor;
class EmergencyRouter;

// Basic structures
struct RoadNode {
    string destination;
    int travelTime;
    bool isBlocked;
    RoadNode* next;

    RoadNode(const string& dest, int time)
        : destination(dest), travelTime(time), isBlocked(false), next(nullptr) {}
};

struct IntersectionNode {
    string name;
    RoadNode* roads;
    IntersectionNode* next;

    IntersectionNode(const string& n)
        : name(n), roads(nullptr), next(nullptr) {}
};

struct PathNode {
    string intersection;
    PathNode* next;
    int travelTime;

    PathNode(const string& loc, int time)
        : intersection(loc), next(nullptr), travelTime(time) {}
};

// Observer interface
class TrafficObserver {
public:
    virtual void onEmergencyVehicle(const string& roadId) = 0;
    virtual void onEmergencyVehicleCleared(const string& roadId) = 0;
    virtual void updateTrafficDensity(const string& roadId, int vehicleCount) = 0;
    virtual ~TrafficObserver() = default;
};

// Helper functions
void displayHeader(const string& title) {
    cout << "\n================================================" << endl;
    cout << "=== " << title << " ===" << endl;
    cout << "================================================" << endl;
}

void displaySeparator() {
    cout << "\n------------------------------------------------" << endl;
}

bool isNumber(const string& str) {
    for (char c : str) {
        if (!isdigit(c)) return false;
    }
    return true;
}

// City Traffic Network Class
class CityTrafficNetwork {
private:
    IntersectionNode* intersections;
    int numIntersections;
    int numRoads;

public:
    CityTrafficNetwork()
        : intersections(nullptr), numIntersections(0), numRoads(0) {
        cout << "Initializing City Traffic Network..." << endl;
    }

    IntersectionNode* getIntersections() const { return intersections; }

    IntersectionNode* findIntersection(const string& intersection) {
        IntersectionNode* current = intersections;
        while (current) {
            if (current->name == intersection) {
                return current;
            }
            current = current->next;
        }
        return nullptr;
    }

    void addIntersection(const string& intersection) {
        if (findIntersection(intersection)) {
            return;
        }

        IntersectionNode* newNode = new IntersectionNode(intersection);
        newNode->next = intersections;
        intersections = newNode;
        numIntersections++;
       
    }

    void addRoad(const string& from, const string& to, int travelTime) {
        if (travelTime <= 0) {
            cout << "Invalid travel time for road " << from << "->" << to << endl;
            return;
        }

        IntersectionNode* fromIntersection = findIntersection(from);
        if (!fromIntersection) {
            addIntersection(from);
            fromIntersection = findIntersection(from);
        }

        if (!findIntersection(to)) {
            addIntersection(to);
        }

        // Check if road already exists
        RoadNode* existingRoad = fromIntersection->roads;
        while (existingRoad) {
            if (existingRoad->destination == to) {
                existingRoad->travelTime = travelTime;
                cout << "Updated road: " << from << "->" << to
                    << " (Time: " << travelTime << "s)" << endl;
                return;
            }
            existingRoad = existingRoad->next;
        }

        // Add new road
        RoadNode* newRoad = new RoadNode(to, travelTime);
        newRoad->next = fromIntersection->roads;
        fromIntersection->roads = newRoad;
        numRoads++;
        
    }

    bool updateRoadStatus(const string& from, const string& to, bool isBlocked) {
        IntersectionNode* fromIntersection = findIntersection(from);
        if (!fromIntersection) {
            cout << "Intersection " << from << " not found" << endl;
            return false;
        }

        RoadNode* current = fromIntersection->roads;
        while (current) {
            if (current->destination == to) {
                current->isBlocked = isBlocked;
                cout << "Updated road status: " << from << "->" << to
                    << " [" << (isBlocked ? "BLOCKED" : "OPEN") << "]" << endl;
                return true;
            }
            current = current->next;
        }

        cout << "Road " << from << "->" << to << " not found" << endl;
        return false;
    }

    void displayNetwork() const {
        displayHeader("Network Status");
        cout << "Total Intersections: " << numIntersections << endl;
        cout << "Total Roads: " << numRoads << endl;

        IntersectionNode* current = intersections;
        while (current) {
            cout << "\nIntersection " << current->name << " connected to:" << endl;
            RoadNode* road = current->roads;
            while (road) {
                cout << "  → " << road->destination
                    << " (Time: " << road->travelTime << "s"
                    << (road->isBlocked ? ", BLOCKED" : "")
                    << ")" << endl;
                road = road->next;
            }
            current = current->next;
        }
    }

    bool loadFromCSV(const string& filename) {
        ifstream file(filename);
        if (!file.is_open()) {
            cout << "Failed to open file: " << filename << endl;
            return false;
        }

        string line;
        getline(file, line); // Skip header

        while (getline(file, line)) {
            stringstream ss(line);
            string from, to, timeStr;

            getline(ss, from, ',');
            getline(ss, to, ',');
            getline(ss, timeStr, ',');

            if (!from.empty() && !to.empty() && !timeStr.empty() && isNumber(timeStr)) {
                int travelTime = stoi(timeStr);
                addRoad(from, to, travelTime);
                addRoad(to, from, travelTime); // Add reverse direction
            }
        }

        file.close();
        return true;
    }

    ~CityTrafficNetwork() {
        while (intersections) {
            IntersectionNode* currentIntersection = intersections;
            intersections = intersections->next;

            while (currentIntersection->roads) {
                RoadNode* currentRoad = currentIntersection->roads;
                currentIntersection->roads = currentIntersection->roads->next;
                delete currentRoad;
            }
            delete currentIntersection;
        }
        cout << "City Traffic Network destroyed" << endl;
    }
};

class Vehicle {
    friend class VehicleRoutingSystem;
    friend class CongestionMonitor;
    friend class EmergencyRouter;
    friend class DisruptionManager;

private:
    struct VehicleState {
        string currentIntersection;
        string nextIntersection;
        int remainingTimeToNext;
        string currentRoad;

        VehicleState(const string& start)
            : currentIntersection(start), nextIntersection(""),
            remainingTimeToNext(0), currentRoad("") {}
    };

    string id;
    string startIntersection;
    string endIntersection;
    PathNode* currentPath;
    bool isEmergency;
    Vehicle* next;
    VehicleState state;

public:
    Vehicle(const string& vehicleId, const string& start, const string& end, bool emergency = false)
        : id(vehicleId), startIntersection(start), endIntersection(end),
        currentPath(nullptr), isEmergency(emergency), next(nullptr), state(start) {
        
    }

    string getId() const { return id; }
    string getStart() const { return startIntersection; }
    string getEnd() const { return endIntersection; }
    bool isEmergencyVehicle() const { return isEmergency; }
    string getCurrentRoad() const { return state.currentRoad; }
    string getCurrentIntersection() const { return state.currentIntersection; }
    string getNextIntersection() const { return state.nextIntersection; }
    int getRemainingTime() const { return state.remainingTimeToNext; }

    void updatePath(PathNode* newPath) {
        while (currentPath) {
            PathNode* temp = currentPath;
            currentPath = currentPath->next;
            delete temp;
        }
        currentPath = newPath;
    }

    void updatePosition(const string& current, const string& next, int remainingTime) {
        state.currentIntersection = current;
        state.nextIntersection = next;
        state.remainingTimeToNext = remainingTime;
        if (!next.empty()) {
            state.currentRoad = current + "->" + next;
        }
        else {
            state.currentRoad = "";
        }
    }

    bool hasReachedDestination() const {
        return state.currentIntersection == endIntersection;
    }

    void advanceAlongPath() {
        if (state.remainingTimeToNext > 0) {
            state.remainingTimeToNext--;
            return;
        }

        if (currentPath && currentPath->next) {
            PathNode* completed = currentPath;
            currentPath = currentPath->next;
            updatePosition(state.nextIntersection, currentPath->intersection, currentPath->travelTime);
            delete completed;
        }
        else if (currentPath) {
            delete currentPath;
            currentPath = nullptr;
            updatePosition(state.nextIntersection, "", 0);
        }
    }

    void displayPath() const {
        cout << "Vehicle " << id << " (" << (isEmergency ? "Emergency" : "Regular") << "):" << endl;
        cout << "  Current Location: " << state.currentIntersection;
        if (!state.nextIntersection.empty()) {
            cout << " -> " << state.nextIntersection
                << " (Remaining Time: " << state.remainingTimeToNext << "s)";
        }
        cout << endl;

        if (currentPath) {
            cout << "  Remaining Path: ";
            PathNode* current = currentPath;
            while (current) {
                cout << current->intersection;
                if (current->next) cout << " -> ";
                current = current->next;
            }
            cout << endl;
        }
    }

    ~Vehicle() {
        while (currentPath) {
            PathNode* temp = currentPath;
            currentPath = currentPath->next;
            delete temp;
        }
    }
};


// Vehicle Class

// Vehicle Routing System Class


// Congestion Monitor Class
class CongestionMonitor {
private:
    struct CongestionNode {
        string roadId;
        int vehicleCount;
        float congestionLevel;
        bool isEmergencyRoute;
        CongestionNode* next;

        CongestionNode(const string& id)
            : roadId(id), vehicleCount(0), congestionLevel(0.0f),
            isEmergencyRoute(false), next(nullptr) {}
    };

    static const int TABLE_SIZE = 101;  // Prime number for better distribution
    CongestionNode* table[TABLE_SIZE];

    int hash(const string& str) {
        int hash = 0;
        for (char c : str) {
            hash = (hash * 31 + c) % TABLE_SIZE;
        }
        return hash;
    }

public:
    CongestionMonitor() {
        for (int i = 0; i < TABLE_SIZE; i++) {
            table[i] = nullptr;
        }
    }

    void updateRoadStatus(const string& roadId, int vehicles) {
        int index = hash(roadId);
        CongestionNode* current = table[index];

        while (current) {
            if (current->roadId == roadId) {
                current->vehicleCount = vehicles;
                // Calculate congestion level (assuming max capacity of 20 vehicles per road)
                current->congestionLevel = static_cast<float>(vehicles) / 20.0f;
                if (current->congestionLevel > 1.0f) current->congestionLevel = 1.0f;
                return;
            }
            current = current->next;
        }

        // Road not found, add new entry
        CongestionNode* newNode = new CongestionNode(roadId);
        newNode->vehicleCount = vehicles;
        newNode->congestionLevel = static_cast<float>(vehicles) / 20.0f;
        newNode->next = table[index];
        table[index] = newNode;
    }

    float getCongestionLevel(const string& roadId) {
        int index = hash(roadId);
        CongestionNode* current = table[index];

        while (current) {
            if (current->roadId == roadId) {
                return current->congestionLevel;
            }
            current = current->next;
        }
        return 0.0f;
    }

    void markEmergencyRoute(const string& roadId, bool isEmergency) {
        int index = hash(roadId);
        CongestionNode* current = table[index];

        while (current) {
            if (current->roadId == roadId) {
                current->isEmergencyRoute = isEmergency;
                cout << "Marked road " << roadId << " as "
                    << (isEmergency ? "emergency route" : "normal route") << endl;
                return;
            }
            current = current->next;
        }

        CongestionNode* newNode = new CongestionNode(roadId);
        newNode->isEmergencyRoute = isEmergency;
        newNode->next = table[index];
        table[index] = newNode;
    }

    void displayCongestionLevels() const {
        displayHeader("Congestion Status");

        for (int i = 0; i < TABLE_SIZE; i++) {
            CongestionNode* current = table[i];
            while (current) {
                cout << current->roadId << ": ";
                int bars = static_cast<int>(current->congestionLevel * 20.0f);
                cout << "[";
                for (int j = 0; j < 20; j++) {
                    if (j < bars) {
                        if (current->isEmergencyRoute) {
                            cout << "E";  // Emergency route
                        }
                        else {
                            cout << "#";  // Normal congestion
                        }
                    }
                    else {
                        cout << " ";
                    }
                }
                cout << "] " << (current->congestionLevel * 100.0f) << "% ";
                if (current->isEmergencyRoute) {
                    cout << "(Emergency Route)";
                }
                cout << endl;
                current = current->next;
            }
        }
    }

    void identifyCongestedRoads(CityTrafficNetwork& network) {
        cout << "\nCongested Roads (>70% capacity):" << endl;
        for (int i = 0; i < TABLE_SIZE; i++) {
            CongestionNode* current = table[i];
            while (current) {
                if (current->congestionLevel > 0.7f) {
                    cout << "Severe congestion on " << current->roadId
                        << " (" << (current->congestionLevel * 100.0f) << "%)" << endl;
                }
                current = current->next;
            }
        }
    }

    ~CongestionMonitor() {
        for (int i = 0; i < TABLE_SIZE; i++) {
            while (table[i]) {
                CongestionNode* temp = table[i];
                table[i] = table[i]->next;
                delete temp;
            }
        }
    }
};

class EmergencyRouter {
private:
    struct AStarNode {
        string intersection;
        float gScore;  // Cost from start
        float hScore;  // Heuristic to goal
        float fScore;  // Total score
        AStarNode* parent;
        AStarNode* next;

        AStarNode(const string& id)
            : intersection(id), gScore(INT_MAX), hScore(0),
            fScore(INT_MAX), parent(nullptr), next(nullptr) {}
    };

    class PriorityQueue {
    private:
        AStarNode* head;

    public:
        PriorityQueue() : head(nullptr) {}

        void push(AStarNode* node) {
            if (!head || node->fScore < head->fScore) {
                node->next = head;
                head = node;
                return;
            }

            AStarNode* current = head;
            while (current->next && current->next->fScore <= node->fScore) {
                current = current->next;
            }
            node->next = current->next;
            current->next = node;
        }

        AStarNode* pop() {
            if (!head) return nullptr;
            AStarNode* node = head;
            head = head->next;
            node->next = nullptr;
            return node;
        }

        bool isEmpty() const { return head == nullptr; }

        void clear() {
            while (head) {
                AStarNode* temp = head;
                head = head->next;
                delete temp;
            }
        }

        ~PriorityQueue() {
            clear();
        }
    };

    float calculateHeuristic(const string& current, const string& goal,
        CityTrafficNetwork& network) {
        // Simple heuristic: use base travel time as estimate
        IntersectionNode* currentNode = network.findIntersection(current);
        if (!currentNode) return INT_MAX;

        RoadNode* road = currentNode->roads;
        while (road) {
            if (road->destination == goal) {
                return static_cast<float>(road->travelTime);
            }
            road = road->next;
        }
        return 1.0f;  // Default estimate if no direct connection
    }

public:
    PathNode* findEmergencyRoute(const string& start, const string& end,
        CityTrafficNetwork& network, CongestionMonitor& congestion) {
        PriorityQueue openSet;
        AStarNode* allNodes = nullptr;  // Keep track of all created nodes for cleanup

        // Initialize start node
        AStarNode* startNode = new AStarNode(start);
        startNode->gScore = 0;
        startNode->hScore = calculateHeuristic(start, end, network);
        startNode->fScore = startNode->gScore + startNode->hScore;
        openSet.push(startNode);
        allNodes = startNode;

        while (!openSet.isEmpty()) {
            AStarNode* current = openSet.pop();

            if (current->intersection == end) {
                // Construct path
                PathNode* path = nullptr;
                AStarNode* pathNode = current;
                while (pathNode) {
                    PathNode* newNode = new PathNode(pathNode->intersection,
                        pathNode->gScore - (pathNode->parent ? pathNode->parent->gScore : 0));
                    newNode->next = path;
                    path = newNode;
                    pathNode = pathNode->parent;
                }

                // Mark emergency route in congestion monitor
                pathNode = current;
                while (pathNode && pathNode->parent) {
                    string roadId = pathNode->parent->intersection + "->" +
                        pathNode->intersection;
                    congestion.markEmergencyRoute(roadId, true);
                    pathNode = pathNode->parent;
                }

                // Cleanup
                while (allNodes) {
                    AStarNode* temp = allNodes;
                    allNodes = allNodes->next;
                    delete temp;
                }

                return path;
            }

            IntersectionNode* intersection = network.findIntersection(current->intersection);
            if (intersection) {
                RoadNode* road = intersection->roads;
                while (road) {
                    if (!road->isBlocked) {
                        // Calculate new cost including congestion factor
                        float congestionFactor = 1.0f + congestion.getCongestionLevel(
                            current->intersection + "->" + road->destination);
                        float newCost = current->gScore + (road->travelTime * congestionFactor);

                        // Find or create neighbor node
                        AStarNode* neighbor = allNodes;
                        while (neighbor && neighbor->intersection != road->destination) {
                            neighbor = neighbor->next;
                        }
                        if (!neighbor) {
                            neighbor = new AStarNode(road->destination);
                            neighbor->next = allNodes;
                            allNodes = neighbor;
                        }

                        if (newCost < neighbor->gScore) {
                            neighbor->parent = current;
                            neighbor->gScore = newCost;
                            neighbor->hScore = calculateHeuristic(
                                road->destination, end, network);
                            neighbor->fScore = neighbor->gScore + neighbor->hScore;
                            openSet.push(neighbor);
                        }
                    }
                    road = road->next;
                }
            }
        }

        // Cleanup if no path found
        while (allNodes) {
            AStarNode* temp = allNodes;
            allNodes = allNodes->next;
            delete temp;
        }

        cout << "No valid emergency route found from " << start << " to " << end << endl;
        return nullptr;
    }
};

class VehicleRoutingSystem {
private:
    CityTrafficNetwork& network;
    Vehicle* vehicles;
    TrafficObserver* observer;


    PathNode* findShortestPath(const string& start, const string& end) {
        struct DistanceNode {
            string intersection;
            int distance;
            string previousIntersection;
            bool visited;
            DistanceNode* next;

            DistanceNode(const string& name)
                : intersection(name), distance(INT_MAX),
                previousIntersection(""), visited(false), next(nullptr) {}
        };

        // Initialize distances
        DistanceNode* distances = nullptr;
        IntersectionNode* current = network.getIntersections();
        while (current) {
            DistanceNode* newNode = new DistanceNode(current->name);
            if (current->name == start) {
                newNode->distance = 0;
            }
            newNode->next = distances;
            distances = newNode;
            current = current->next;
        }

        // Find shortest path using Dijkstra's algorithm
        bool pathFound = false;
        while (!pathFound) {
            // Find minimum distance node
            DistanceNode* minNode = nullptr;
            DistanceNode* current = distances;
            int minDistance = INT_MAX;

            while (current) {
                if (!current->visited && current->distance < minDistance) {
                    minDistance = current->distance;
                    minNode = current;
                }
                current = current->next;
            }

            if (!minNode) break;
            minNode->visited = true;

            if (minNode->intersection == end) {
                pathFound = true;
                continue;
            }

            // Update neighbors
            IntersectionNode* currentIntersection = network.findIntersection(minNode->intersection);
            if (!currentIntersection) continue;

            RoadNode* road = currentIntersection->roads;
            while (road) {
                if (!road->isBlocked) {
                    DistanceNode* neighbor = distances;
                    while (neighbor) {
                        if (neighbor->intersection == road->destination) {
                            int newDistance = minNode->distance + road->travelTime;
                            if (newDistance < neighbor->distance) {
                                neighbor->distance = newDistance;
                                neighbor->previousIntersection = minNode->intersection;
                            }
                            break;
                        }
                        neighbor = neighbor->next;
                    }
                }
                road = road->next;
            }
        }

        // Construct path
        PathNode* path = nullptr;
        string currentIntersection = end;

        while (!currentIntersection.empty()) {
            DistanceNode* currentDist = distances;
            while (currentDist && currentDist->intersection != currentIntersection) {
                currentDist = currentDist->next;
            }

            if (!currentDist) break;

            int segmentTime = currentDist->distance;
            if (!currentDist->previousIntersection.empty()) {
                DistanceNode* prevDist = distances;
                while (prevDist && prevDist->intersection != currentDist->previousIntersection) {
                    prevDist = prevDist->next;
                }
                if (prevDist) {
                    segmentTime -= prevDist->distance;
                }
            }

            PathNode* newNode = new PathNode(currentIntersection, segmentTime);
            newNode->next = path;
            path = newNode;

            if (currentIntersection == start) break;
            currentIntersection = currentDist->previousIntersection;
        }

        // Cleanup
        while (distances) {
            DistanceNode* temp = distances;
            distances = distances->next;
            delete temp;
        }

        return path;
    }

public:

    CongestionMonitor congestion;        // Now this is defined
    EmergencyRouter emergencyRouter;

    VehicleRoutingSystem(CityTrafficNetwork& net)
        : network(net), vehicles(nullptr), observer(nullptr) {}

    void setObserver(TrafficObserver* obs) {
        observer = obs;
    }

    bool loadVehiclesFromCSV(const std::string& filename) {
        std::ifstream file(filename);
        if (!file.is_open()) {
            std::cout << "Failed to open vehicles file: " << filename << std::endl;
            return false;
        }

        std::cout << "Loading vehicles..." << std::endl;
        int loadedCount = 0;

        std::string line;
        std::getline(file, line); // Skip header

        while (std::getline(file, line)) {
            if (line.empty() || line.find_first_not_of(" \t\r\n,") == std::string::npos) {
                continue;  // Skip empty or blank lines
            }

            std::stringstream ss(line);
            std::string id, start, end;

            if (std::getline(ss, id, ',') &&
                std::getline(ss, start, ',') &&
                std::getline(ss, end, ',')) {

                // Basic trim
                id.erase(0, id.find_first_not_of(" \t\r\n"));
                id.erase(id.find_last_not_of(" \t\r\n") + 1);
                start.erase(0, start.find_first_not_of(" \t\r\n"));
                start.erase(start.find_last_not_of(" \t\r\n") + 1);
                end.erase(0, end.find_first_not_of(" \t\r\n"));
                end.erase(end.find_last_not_of(" \t\r\n") + 1);

                if (!id.empty() && !start.empty() && !end.empty()) {
                    Vehicle* newVehicle = new Vehicle(id, start, end, false);
                    newVehicle->next = vehicles;
                    vehicles = newVehicle;
                    loadedCount++;
                }
            }
        }

        file.close();
        std::cout << "Loaded " << loadedCount << " vehicles" << std::endl;
        return true;
    }

    // Add this method to VehicleRoutingSystem class
    void updateVehiclesCSV(const std::string& filename) {
        // First create a list of current vehicles
        std::ofstream file(filename);
        if (!file.is_open()) {
            std::cout << "Failed to update vehicles file" << std::endl;
            return;
        }

        // Write header
        file << "VehicleID,StartIntersection,EndIntersection" << std::endl;

        // Write current vehicles
        Vehicle* current = vehicles;
        while (current) {
            file << current->getId() << ","
                << current->getStart() << ","
                << current->getEnd() << std::endl;
            current = current->next;
        }

        file.close();
    }

    // Modify the removeVehicle method:
    void removeVehicle(const std::string& id) {
        Vehicle* current = vehicles;
        Vehicle* prev = nullptr;

        while (current) {
            if (current->getId() == id) {
                if (prev) {
                    prev->next = current->next;
                }
                else {
                    vehicles = current->next;
                }

                if (current->isEmergencyVehicle() && observer) {
                    observer->onEmergencyVehicleCleared(current->getCurrentRoad());
                }

                std::cout << "Removed vehicle " << id;
                if (current->hasReachedDestination()) {
                    std::cout << " (Destination reached: "
                        << current->getCurrentIntersection() << ")" << std::endl;
                }
                else {
                    std::cout << " (Removed at: "
                        << current->getCurrentIntersection() << ")" << std::endl;
                }

                delete current;

                // Update the CSV file after removing the vehicle
                updateVehiclesCSV("vehicles.csv");
                return;
            }
            prev = current;
            current = current->next;
        }

        std::cout << "Vehicle " << id << " not found" << std::endl;
    }

    Vehicle* getVehicles() const { return vehicles; }

    void addVehicle(const std::string& id, const std::string& start, const std::string& end, bool isEmergency = false) {
        Vehicle* newVehicle = new Vehicle(id, start, end, isEmergency);
        newVehicle->updatePath(findShortestPath(start, end));

        newVehicle->next = vehicles;
        vehicles = newVehicle;

        if (isEmergency && observer) {
            observer->onEmergencyVehicle(start + "->" + end);
        }

        // Append the new vehicle to the CSV file
        std::ofstream file("vehicles.csv", std::ios::app);  // Open in append mode
        if (file.is_open()) {
            file << id << "," << start << "," << end << std::endl;
            file.close();
        }
        else {
            std::cout << "Warning: Could not update vehicles.csv file" << std::endl;
        }
    }

    void updateAllVehiclePositions() {
        // First count vehicles on each road
        std::string currentRoad;
        Vehicle* current = vehicles;

        // Clear previous vehicle counts
        while (current) {
            if (!current->hasReachedDestination()) {
                currentRoad = current->getCurrentRoad();
                if (!currentRoad.empty() && observer) {
                    // Notify signal manager about vehicle presence
                    observer->updateTrafficDensity(currentRoad, 1);
                }
            }
            current = current->next;
        }

        // Now update positions
        current = vehicles;
        while (current) {
            if (!current->hasReachedDestination()) {
                currentRoad = current->getCurrentRoad();

                // Update vehicle position
                current->advanceAlongPath();

                if (current->isEmergencyVehicle() && observer) {
                    observer->onEmergencyVehicle(currentRoad);
                }
            }
            current = current->next;
        }
    }

    void recalculateAllRoutes() {
        Vehicle* current = vehicles;
        while (current) {
            if (!current->hasReachedDestination()) {
                current->updatePath(findShortestPath(
                    current->getCurrentIntersection(),
                    current->getEnd()
                ));
            }
            current = current->next;
        }
    }

    void displayAllVehicles() const {
        // Store previous vehicle states using existing structures
        static std::string lastDisplayState = "";
        std::string currentState = "";

        Vehicle* current = vehicles;
        int activeVehicles = 0;

        while (current) {
            if (!current->hasReachedDestination()) {
                currentState += "Vehicle " + current->getId() + " (" +
                    (current->isEmergencyVehicle() ? "Emergency" : "Regular") + "):\n" +
                    "    Current Location: " + current->getCurrentIntersection() + "\n";
                activeVehicles++;
            }
            current = current->next;
        }

        // Only display if state has changed
        if (currentState != lastDisplayState) {
            std::cout << "\nActive Vehicles: " << activeVehicles << std::endl;
            std::cout << currentState;
            lastDisplayState = currentState;
        }
    }

    void cleanupFinishedVehicles() {
        Vehicle* current = vehicles;
        Vehicle* prev = nullptr;

        while (current) {
            if (current->hasReachedDestination()) {
                Vehicle* toDelete = current;

                if (prev) {
                    prev->next = current->next;
                    current = current->next;
                }
                else {
                    vehicles = current->next;
                    current = vehicles;
                }

                // Clear emergency status if needed
                if (toDelete->isEmergencyVehicle() && observer) {
                    observer->onEmergencyVehicleCleared(toDelete->getCurrentRoad());
                }

                cout << "Vehicle " << toDelete->getId()
                    << " has reached its destination: "
                    << toDelete->getCurrentIntersection() << endl;

                delete toDelete;
            }
            else {
                prev = current;
                current = current->next;
            }
        }
    }

    // Add to VehicleRoutingSystem class

    void recalculateRoute(Vehicle* vehicle) {
        if (!vehicle || vehicle->hasReachedDestination()) return;

        // Store current position
        string currentPos = vehicle->getCurrentIntersection();

        // Find new route from current position to destination
        PathNode* newPath = vehicle->isEmergencyVehicle() ?
            emergencyRouter.findEmergencyRoute(currentPos, vehicle->getEnd(), network, congestion) :
            findShortestPath(currentPos, vehicle->getEnd());

        if (newPath) {
            vehicle->updatePath(newPath);
            cout << "Recalculated route for vehicle " << vehicle->getId()
                << " from " << currentPos << " to " << vehicle->getEnd() << endl;
        }
        else {
            cout << "Warning: No alternative route found for vehicle "
                << vehicle->getId() << endl;
        }
    }

    ~VehicleRoutingSystem() {
        while (vehicles) {
            Vehicle* temp = vehicles;
            vehicles = vehicles->next;
            delete temp;
        }
    }
};

// Traffic Signal Class
class TrafficSignal {
private:
    struct RoadQueue {
        string roadId;
        int vehicleCount;
        int waitTime;
        bool hasEmergencyVehicle;
        RoadQueue* next;

        RoadQueue(const string& id)
            : roadId(id), vehicleCount(0), waitTime(0),
            hasEmergencyVehicle(false), next(nullptr) {}
    };

    struct SignalPhase {
        string phaseId;
        RoadQueue* activeRoads;
        int duration;
        SignalPhase* next;

        SignalPhase(const string& id)
            : phaseId(id), activeRoads(nullptr), duration(30), next(nullptr) {}
    };

    string intersectionId;
    SignalPhase* currentPhase;
    SignalPhase* phases;
    int baseGreenTime;
    int minGreenTime;
    int maxGreenTime;
    bool emergencyMode;

public:
    TrafficSignal(const string& id, int baseTime = 30, int minTime = 10, int maxTime = 90)
        : intersectionId(id), currentPhase(nullptr), phases(nullptr),
        baseGreenTime(baseTime), minGreenTime(minTime), maxGreenTime(maxTime),
        emergencyMode(false) {
        cout << "Created traffic signal at intersection " << id << endl;
    }

    string getIntersectionId() const { return intersectionId; }

    void addPhase(const string& id) {
        SignalPhase* newPhase = new SignalPhase(id);
        if (!phases) {
            phases = newPhase;
            currentPhase = newPhase;
        }
        else {
            SignalPhase* last = phases;
            while (last->next) last = last->next;
            last->next = newPhase;
        }
        cout << "Added phase " << id << " to intersection " << intersectionId << endl;
    }

    void addRoadToPhase(const string& phaseId, const string& roadId) {
        SignalPhase* phase = phases;
        while (phase) {
            if (phase->phaseId == phaseId) {
                RoadQueue* newRoad = new RoadQueue(roadId);
                newRoad->next = phase->activeRoads;
                phase->activeRoads = newRoad;
                cout << "Added road " << roadId << " to phase " << phaseId << endl;
                return;
            }
            phase = phase->next;
        }
        cout << "Phase " << phaseId << " not found" << endl;
    }

    

    void signalEmergencyVehicle(const string& roadId) {
        SignalPhase* phase = phases;
        while (phase) {
            RoadQueue* road = phase->activeRoads;
            while (road) {
                if (road->roadId == roadId) {
                    road->hasEmergencyVehicle = true;
                    emergencyMode = true;
                    if (phase != currentPhase) {
                        switchToPhase(phase->phaseId);
                    }
                    cout << "Emergency vehicle detected on road " << roadId << endl;
                    return;
                }
                road = road->next;
            }
            phase = phase->next;
        }
    }

    void clearEmergencyStatus(const string& roadId) {
        bool anyEmergencyVehicle = false;
        SignalPhase* phase = phases;

        while (phase) {
            RoadQueue* road = phase->activeRoads;
            while (road) {
                if (road->roadId == roadId) {
                    road->hasEmergencyVehicle = false;
                }
                if (road->hasEmergencyVehicle) {
                    anyEmergencyVehicle = true;
                }
                road = road->next;
            }
            phase = phase->next;
        }

        emergencyMode = anyEmergencyVehicle;
        if (!emergencyMode) {
            cout << "Emergency mode cleared for intersection " << intersectionId << endl;
        }
    }

    int calculatePhaseDuration() {
        if (!currentPhase) return baseGreenTime;

        int maxVehicles = 0;
        int totalWaitTime = 0;
        RoadQueue* road = currentPhase->activeRoads;

        while (road) {
            if (road->vehicleCount > maxVehicles) {
                maxVehicles = road->vehicleCount;
            }
            totalWaitTime += road->waitTime;
            road = road->next;
        }

        int duration = baseGreenTime;
        if (emergencyMode) {
            duration = maxGreenTime;  // Maximum time for emergency vehicles
        }
        else {
            // Adjust based on traffic conditions
            duration += (maxVehicles / 5) * 5;  // Add 5 seconds for every 5 vehicles
            duration += (totalWaitTime / 30) * 5;  // Add 5 seconds for every 30 seconds of wait time

            if (duration < minGreenTime) duration = minGreenTime;
            if (duration > maxGreenTime) duration = maxGreenTime;
        }

        return duration;
    }

    void switchToPhase(const string& phaseId) {
        SignalPhase* phase = phases;
        while (phase && phase->phaseId != phaseId) {
            phase = phase->next;
        }

        if (phase) {
            currentPhase = phase;
            currentPhase->duration = calculatePhaseDuration();

            // Reset wait times for new active roads
            RoadQueue* road = currentPhase->activeRoads;
            while (road) {
                road->waitTime = 0;
                road = road->next;
            }

            cout << "Switched to phase " << phaseId
                << " (Duration: " << currentPhase->duration << "s)" << endl;
        }
    }

    void updateWaitTimes() {
        SignalPhase* phase = phases;
        while (phase) {
            if (phase != currentPhase) {
                RoadQueue* road = phase->activeRoads;
                while (road) {
                    road->waitTime++;
                    road = road->next;
                }
            }
            phase = phase->next;
        }
    }

    void updateSignalState() {
        if (!currentPhase) {
            if (phases) {
                switchToPhase(phases->phaseId);
            }
            return;
        }

        updateWaitTimes();
        currentPhase->duration--;

        if (currentPhase->duration <= 0) {
            SignalPhase* nextPhase = currentPhase->next;
            if (!nextPhase) nextPhase = phases;  // Cycle back to first phase
            switchToPhase(nextPhase->phaseId);
        }
    }

    bool hasActiveTraffic() const {
        std::cout << "Checking traffic for signal at " << intersectionId << std::endl;

        SignalPhase* currentPhase = phases;
        while (currentPhase) {
            RoadQueue* road = currentPhase->activeRoads;
            while (road) {
                std::cout << "  Road " << road->roadId
                    << " has " << road->vehicleCount << " vehicles" << std::endl;
                if (road->vehicleCount > 0 || road->hasEmergencyVehicle) {
                    return true;
                }
                road = road->next;
            }
            currentPhase = currentPhase->next;
        }
        return false;
    }

    void updateVehicleCount(const std::string& roadId, int count) {
        SignalPhase* phase = phases;
        bool updated = false;

        while (phase) {
            RoadQueue* road = phase->activeRoads;
            while (road) {
                if (road->roadId == roadId) {
                    road->vehicleCount = count;
                    updated = true;
                    std::cout << "Updated vehicle count for road " << roadId
                        << " to " << count << " at signal " << intersectionId << std::endl;
                }
                road = road->next;
            }
            phase = phase->next;
        }

        if (!updated) {
            std::cout << "No matching road " << roadId << " found at signal "
                << intersectionId << std::endl;
        }
    }

    void displayStatus() const {
        cout << "\nTraffic Signal at Intersection " << intersectionId << endl;
        cout << "Emergency Mode: " << (emergencyMode ? "ON" : "OFF") << endl;

        if (currentPhase) {
            cout << "Current Phase: " << currentPhase->phaseId
                << " (Remaining: " << currentPhase->duration << "s)" << endl;

            RoadQueue* road = currentPhase->activeRoads;
            while (road) {
                cout << "  Road " << road->roadId << ":"
                    << " Vehicles=" << road->vehicleCount
                    << " Wait=" << road->waitTime << "s"
                    << (road->hasEmergencyVehicle ? " [EMERGENCY]" : "")
                    << endl;
                road = road->next;
            }
        }
    }

    ~TrafficSignal() {
        while (phases) {
            SignalPhase* currentPhase = phases;
            phases = phases->next;

            while (currentPhase->activeRoads) {
                RoadQueue* currentRoad = currentPhase->activeRoads;
                currentPhase->activeRoads = currentPhase->activeRoads->next;
                delete currentRoad;
            }

            delete currentPhase;
        }
    }
};

class TrafficSignalManager : public TrafficObserver {
private:
    struct SignalNode {
        TrafficSignal* signal;
        SignalNode* next;
        SignalNode(TrafficSignal* s) : signal(s), next(nullptr) {}
    };

    SignalNode* signals;
    CityTrafficNetwork& network;

public:
    bool loadSignalsFromCSV(const std::string& filename) {
        std::cout << "Starting to load traffic signals..." << std::endl;
        std::ifstream file(filename);
        if (!file.is_open()) {
            std::cout << "Failed to open traffic signals file: " << filename << std::endl;
            return false;
        }

        int signalCount = 0;
        std::string line;
        std::getline(file, line); // Skip header

        while (std::getline(file, line)) {
            std::stringstream ss(line);
            std::string intersection, greenTimeStr;

            if (std::getline(ss, intersection, ',') &&
                std::getline(ss, greenTimeStr, ',')) {

                int greenTime = std::stoi(greenTimeStr);
                TrafficSignal* signal = new TrafficSignal(intersection, greenTime);

                // Add a default phase
                signal->addPhase("Phase1");

                // Find all connected roads from the network
                IntersectionNode* node = network.findIntersection(intersection);
                if (node) {
                    RoadNode* road = node->roads;
                    while (road) {
                        std::string roadId = intersection + "->" + road->destination;
                        signal->addRoadToPhase("Phase1", roadId);
                        std::cout << "Added road " << roadId << " to signal at " << intersection << std::endl;
                        road = road->next;
                    }
                }

                // Add to signal list
                SignalNode* newNode = new SignalNode(signal);
                newNode->next = signals;
                signals = newNode;
                signalCount++;
            }
        }

        file.close();
        std::cout << "Loaded " << signalCount << " traffic signals" << std::endl;
        return true;
    }

    void updateAllSignals() {
        SignalNode* current = signals;
        while (current) {
            current->signal->updateSignalState();
            current = current->next;
        }
    }

    void updateTrafficDensity(const std::string& roadId, int vehicleCount) override {
        std::cout << "Updating traffic density for road " << roadId
            << " with " << vehicleCount << " vehicles" << std::endl;

        SignalNode* current = signals;
        while (current) {
            current->signal->updateVehicleCount(roadId, vehicleCount);
            current = current->next;
        }
    }

    void displayStatus() const {
        displayHeader("Traffic Signal System Status");

        SignalNode* current = signals;
        int activeCount = 0;

        if (!signals) {
            std::cout << "No traffic signals initialized!" << std::endl;
            return;
        }

        while (current) {
            std::cout << "\nChecking signal at intersection: "
                << current->signal->getIntersectionId() << std::endl;

            if (current->signal->hasActiveTraffic()) {
                current->signal->displayStatus();
                activeCount++;
            }
            current = current->next;
        }

        std::cout << "\nTotal Active Signals: " << activeCount << std::endl;
        if (activeCount == 0) {
            std::cout << "No active traffic signals at the moment." << std::endl;

            // Debug information
            current = signals;
            std::cout << "\nDebug Information:" << std::endl;
            while (current) {
                TrafficSignal* signal = current->signal;
                std::cout << "Signal at " << signal->getIntersectionId()
                    << " exists but inactive" << std::endl;
                current = current->next;
            }
        }
    }

    void onEmergencyVehicle(const std::string& roadId) override {
        SignalNode* current = signals;
        while (current) {
            current->signal->signalEmergencyVehicle(roadId);
            current = current->next;
        }
    }

    void onEmergencyVehicleCleared(const std::string& roadId) override {
        SignalNode* current = signals;
        while (current) {
            current->signal->clearEmergencyStatus(roadId);
            current = current->next;
        }
    }


    TrafficSignalManager(CityTrafficNetwork& net) : signals(nullptr), network(net) {
        cout << "Initializing Traffic Signal Manager" << endl;
    }


    ~TrafficSignalManager() {
        while (signals) {
            SignalNode* temp = signals;
            signals = signals->next;
            delete temp->signal;
            delete temp;
        }
        cout << "Traffic Signal Manager destroyed" << endl;
    }
};

class SmartRouter {
private:
    struct RouteState {
        string intersection;
        int totalTime;
        string previousIntersection;
        bool visited;

        RouteState()
            : intersection(""), totalTime(INT_MAX),
            previousIntersection(""), visited(false) {}
    };

    CityTrafficNetwork& network;
    CongestionMonitor& congestion;

public:
    SmartRouter(CityTrafficNetwork& net, CongestionMonitor& cong)
        : network(net), congestion(cong) {}

    PathNode* findOptimalRoute(const string& start, const string& end) {
        // Initialize route states for all intersections
        RouteState* states = nullptr;
        int numStates = 0;

        // Count and create states for all intersections
        IntersectionNode* current = network.getIntersections();
        while (current) {
            RouteState* newState = new RouteState[numStates + 1];
            for (int i = 0; i < numStates; i++) {
                newState[i] = states[i];
            }
            delete[] states;
            states = newState;

            states[numStates].intersection = current->name;
            if (current->name == start) {
                states[numStates].totalTime = 0;
            }
            numStates++;
            current = current->next;
        }

        // Dynamic programming approach to find optimal path
        bool pathFound = false;
        while (!pathFound) {
            // Find unvisited intersection with minimum total time
            RouteState* minState = nullptr;
            for (int i = 0; i < numStates; i++) {
                if (!states[i].visited &&
                    (!minState || states[i].totalTime < minState->totalTime)) {
                    minState = &states[i];
                }
            }

            if (!minState || minState->totalTime == INT_MAX) break;
            minState->visited = true;

            // Check if we've reached the destination
            if (minState->intersection == end) {
                pathFound = true;
                continue;
            }

            // Update distances to adjacent intersections
            IntersectionNode* currentNode = network.findIntersection(minState->intersection);
            if (!currentNode) continue;

            RoadNode* road = currentNode->roads;
            while (road) {
                if (!road->isBlocked) {
                    // Calculate congestion-adjusted travel time
                    string roadId = minState->intersection + "->" + road->destination;
                    float congestionFactor = 1.0f + congestion.getCongestionLevel(roadId);
                    int adjustedTime = static_cast<int>(road->travelTime * congestionFactor);

                    // Find the state for this destination
                    RouteState* destState = nullptr;
                    for (int i = 0; i < numStates; i++) {
                        if (states[i].intersection == road->destination) {
                            destState = &states[i];
                            break;
                        }
                    }

                    if (destState) {
                        int newTotalTime = minState->totalTime + adjustedTime;
                        if (newTotalTime < destState->totalTime) {
                            destState->totalTime = newTotalTime;
                            destState->previousIntersection = minState->intersection;
                        }
                    }
                }
                road = road->next;
            }
        }

        // Construct the path from the computed states
        PathNode* path = nullptr;
        string currentIntersection = end;

        while (!currentIntersection.empty()) {
            // Find the current state
            RouteState* currentState = nullptr;
            for (int i = 0; i < numStates; i++) {
                if (states[i].intersection == currentIntersection) {
                    currentState = &states[i];
                    break;
                }
            }

            if (!currentState) break;

            // Calculate segment time
            int segmentTime = currentState->totalTime;
            if (!currentState->previousIntersection.empty()) {
                for (int i = 0; i < numStates; i++) {
                    if (states[i].intersection == currentState->previousIntersection) {
                        segmentTime -= states[i].totalTime;
                        break;
                    }
                }
            }

            // Create new path node
            PathNode* newNode = new PathNode(currentIntersection, segmentTime);
            newNode->next = path;
            path = newNode;

            if (currentIntersection == start) break;
            currentIntersection = currentState->previousIntersection;
        }

        delete[] states;
        return path;
    }

    void displayRoute(const string& vehicleId, PathNode* path) {
        cout << "\nOptimal route for vehicle " << vehicleId << ":" << endl;
        cout << "----------------------------------------" << endl;

        PathNode* current = path;
        int totalTime = 0;

        while (current) {
            // Display current intersection
            cout << current->intersection;

            if (current->next) {
                // Calculate congestion for this segment
                string roadId = current->intersection + "->" +
                    current->next->intersection;
                float congLevel = congestion.getCongestionLevel(roadId);

                cout << " --(" << current->travelTime << "s, ";
                // Display congestion level with visual indicator
                cout << "Congestion: ";
                int congestionBars = static_cast<int>(congestion.getCongestionLevel(roadId) * 10);
                cout << "[";
                for (int i = 0; i < 10; i++) {
                    cout << (i < congestionBars ? "#" : " ");
                }
                cout << "])--> ";

                totalTime += current->travelTime;
            }

            current = current->next;
        }

        cout << "\nEstimated total travel time: " << totalTime << " seconds" << endl;
        cout << "----------------------------------------" << endl;
    }
};


class DisruptionManager {
private:
    struct DisruptionEvent {
        string roadStart;
        string roadEnd;
        string status; // "Blocked", "Clear", "Under Repair"
        int duration;       // Time remaining for temporary disruptions
        DisruptionEvent* next;

        DisruptionEvent(const string& start, const string& end,
            const string& st, int dur = 0)
            : roadStart(start), roadEnd(end), status(st),
            duration(dur), next(nullptr) {}
    };

    CityTrafficNetwork& network;
    VehicleRoutingSystem& routingSystem;
    DisruptionEvent* activeDisruptions;
    int totalDisruptions;
    int activeBlockages;
    int underRepairCount;

public:
    DisruptionManager(CityTrafficNetwork& net, VehicleRoutingSystem& routing)
        : network(net), routingSystem(routing),
        activeDisruptions(nullptr), totalDisruptions(0),
        activeBlockages(0), underRepairCount(0) {
        cout << "Initializing Disruption Management System..." << endl;
    }

    bool loadDisruptions(const string& filename) {
        ifstream file(filename);
        if (!file.is_open()) {
            cout << "Failed to open disruptions file: " << filename << endl;
            return false;
        }

        string line;
        getline(file, line); // Skip header

        while (getline(file, line)) {
            stringstream ss(line);
            string intersection1, intersection2, status;

            getline(ss, intersection1, ',');
            getline(ss, intersection2, ',');
            getline(ss, status, ',');

            if (!intersection1.empty() && !intersection2.empty() && !status.empty()) {
                addDisruption(intersection1, intersection2, status);
            }
        }

        file.close();
        return true;
    }

    void addDisruption(const string& start, const string& end,
        const string& status, int duration = 0) {
        // Create new disruption event
        DisruptionEvent* newDisruption = new DisruptionEvent(start, end, status, duration);
        newDisruption->next = activeDisruptions;
        activeDisruptions = newDisruption;

        // Update network status
        network.updateRoadStatus(start, end, status != "Clear");
        network.updateRoadStatus(end, start, status != "Clear"); // Update both directions

        // Update counters
        totalDisruptions++;
        if (status == "Blocked") activeBlockages++;
        if (status == "Under Repair") underRepairCount++;

        // Trigger route recalculation for affected vehicles
        recalculateAffectedRoutes(start, end);

        cout << "\nNew disruption added: " << start << " -> " << end
            << " (" << status << ")" << endl;
    }

    void recalculateAffectedRoutes(const string& start, const string& end) {
        Vehicle* current = routingSystem.getVehicles();
        int affectedVehicles = 0;

        while (current) {
            bool needsRecalculation = false;
            PathNode* route = current->currentPath;

            // Check if current route passes through disrupted road
            while (route && route->next) {
                if ((route->intersection == start && route->next->intersection == end) ||
                    (route->intersection == end && route->next->intersection == start)) {
                    needsRecalculation = true;
                    break;
                }
                route = route->next;
            }

            if (needsRecalculation) {
                routingSystem.recalculateRoute(current);
                affectedVehicles++;
            }

            current = current->next;
        }

        if (affectedVehicles > 0) {
            cout << "Recalculated routes for " << affectedVehicles
                << " affected vehicles" << endl;
        }
    }

    void updateDisruptionStates() {
        DisruptionEvent* current = activeDisruptions;
        DisruptionEvent* prev = nullptr;

        while (current) {
            // Update duration for temporary disruptions
            if (current->duration > 0) {
                current->duration--;

                // Check if disruption has ended
                if (current->duration == 0) {
                    // Clear the disruption
                    network.updateRoadStatus(current->roadStart, current->roadEnd, false);
                    network.updateRoadStatus(current->roadEnd, current->roadStart, false);

                    // Update counters
                    if (current->status == "Blocked") activeBlockages--;
                    if (current->status == "Under Repair") underRepairCount--;

                    // Remove from list
                    if (prev) {
                        prev->next = current->next;
                    }
                    else {
                        activeDisruptions = current->next;
                    }

                    DisruptionEvent* toDelete = current;
                    current = current->next;
                    delete toDelete;

                    // Recalculate routes for all vehicles
                    routingSystem.recalculateAllRoutes();
                    continue;
                }
            }

            prev = current;
            current = current->next;
        }
    }

    void displayDisruptionStatus() const {
        displayHeader("Traffic Disruption Status");
        cout << "Total Disruptions: " << totalDisruptions << endl;
        cout << "Active Blockages: " << activeBlockages << endl;
        cout << "Roads Under Repair: " << underRepairCount << endl;

        DisruptionEvent* current = activeDisruptions;
        while (current) {
            cout << "\nDisruption: " << current->roadStart << " -> "
                << current->roadEnd << endl;
            cout << "Status: " << current->status;
            if (current->duration > 0) {
                cout << " (Duration: " << current->duration << "s remaining)";
            }
            cout << endl;
            current = current->next;
        }
    }

    void monitorSystemPerformance() {
        // Calculate system performance metrics
        int totalActiveVehicles = 0;
        int stuckVehicles = 0;
        int reroutedVehicles = 0;

        Vehicle* current = routingSystem.getVehicles();
        while (current) {
            if (!current->hasReachedDestination()) {
                totalActiveVehicles++;
                if (current->getRemainingTime() > 100) { // Arbitrary threshold for "stuck"
                    stuckVehicles++;
                }
            }
            current = current->next;
        }

        cout << "\nSystem Performance Under Disruptions:" << endl;
        cout << "----------------------------------------" << endl;
        cout << "Active Vehicles: " << totalActiveVehicles << endl;
        cout << "Potentially Stuck Vehicles: " << stuckVehicles << endl;
        cout << "Network Connectivity: "
            << (1.0f - (float)activeBlockages / totalDisruptions) * 100
            << "%" << endl;
    }

    ~DisruptionManager() {
        while (activeDisruptions) {
            DisruptionEvent* temp = activeDisruptions;
            activeDisruptions = activeDisruptions->next;
            delete temp;
        }
    }
};

class TrafficSimulation {
private:
    CityTrafficNetwork network;
    VehicleRoutingSystem routingSystem;
    TrafficSignalManager signalManager;
    DisruptionManager disruptionManager;
    SmartRouter router;
    bool isRunning;
    int timeStep;
    int simulationSpeed; // Steps per second

    void displayMenu() {
        displayHeader("Traffic Management System");
        cout << "\nSimulation Time: " << timeStep << "s"
            << " | Speed: " << simulationSpeed << "x" << endl;
        cout << "\nCommands:" << endl;
        cout << "1. Start/Stop Simulation" << endl;
        cout << "2. Add Vehicle" << endl;
        cout << "3. Add Emergency Vehicle" << endl;
        cout << "4. Remove Vehicle" << endl;
        cout << "5. Add Road Disruption" << endl;
        cout << "6. View Network Status" << endl;
        cout << "7. View Vehicle Status" << endl;
        cout << "8. View Traffic Signals" << endl;
        cout << "9. View Disruptions" << endl;
        cout << "10. Adjust Simulation Speed" << endl;
        cout << "11. Save Current State" << endl;
        cout << "12. Load State" << endl;
        cout << "0. Exit" << endl;
        cout << "\nEnter command: ";
    }

    void handleUserInput(const string& input) {
        if (input == "1") {
            isRunning = !isRunning;
            cout << "Simulation " << (isRunning ? "started" : "stopped") << endl;
        }
        else if (input == "2") {
            addNewVehicle(false);
        }
        else if (input == "3") {
            addNewVehicle(true);
        }
        else if (input == "4") {
            removeExistingVehicle();
        }
        else if (input == "5") {
            addNewDisruption();
        }
        else if (input == "6") {
            network.displayNetwork();
            routingSystem.congestion.displayCongestionLevels();
        }
        else if (input == "7") {
            routingSystem.displayAllVehicles();
        }
        else if (input == "8") {
            signalManager.displayStatus();
        }
        else if (input == "9") {
            disruptionManager.displayDisruptionStatus();
            disruptionManager.monitorSystemPerformance();
        }
        else if (input == "10") {
            adjustSimulationSpeed();
        }
        else if (input == "11") {
            saveSimulationState();
        }
        else if (input == "12") {
            loadSimulationState();
        }
    }

    void addNewVehicle(bool isEmergency) {
        string id, start, end;

        cout << "Enter vehicle ID: ";
        cin >> id;
        cout << "Enter start intersection: ";
        cin >> start;
        cout << "Enter destination intersection: ";
        cin >> end;

        if (network.findIntersection(start) && network.findIntersection(end)) {
            routingSystem.addVehicle(id, start, end, isEmergency);
            cout << (isEmergency ? "Emergency" : "Regular")
                << " vehicle added successfully" << endl;
        }
        else {
            cout << "Invalid intersections specified" << endl;
        }
    }

    void removeExistingVehicle() {
        string id;
        cout << "Enter vehicle ID to remove: ";
        cin >> id;
        routingSystem.removeVehicle(id);
    }

    void addNewDisruption() {
        string start, end, status;
        int duration;

        cout << "Enter start intersection: ";
        cin >> start;
        cout << "Enter end intersection: ";
        cin >> end;
        cout << "Enter status (Blocked/Clear/Under Repair): ";
        cin >> status;
        cout << "Enter duration (0 for permanent): ";
        cin >> duration;

        if (network.findIntersection(start) && network.findIntersection(end)) {
            disruptionManager.addDisruption(start, end, status, duration);
        }
        else {
            cout << "Invalid intersections specified" << endl;
        }
    }

    void adjustSimulationSpeed() {
        cout << "Enter new simulation speed (1-10): ";
        cin >> simulationSpeed;
        if (simulationSpeed < 1) simulationSpeed = 1;
        if (simulationSpeed > 10) simulationSpeed = 10;
    }

    void saveSimulationState() {
        string filename;
        cout << "Enter filename to save state: ";
        cin >> filename;

        ofstream file(filename);
        if (file.is_open()) {
            file << timeStep << "\n";
            file << simulationSpeed << "\n";
            // Add more state saving logic here
            file.close();
            cout << "Simulation state saved to " << filename << endl;
        }
        else {
            cout << "Failed to save simulation state" << endl;
        }
    }

    void loadSimulationState() {
        string filename;
        cout << "Enter filename to load state: ";
        cin >> filename;

        ifstream file(filename);
        if (file.is_open()) {
            file >> timeStep;
            file >> simulationSpeed;
            // Add more state loading logic here
            file.close();
            cout << "Simulation state loaded from " << filename << endl;
        }
        else {
            cout << "Failed to load simulation state" << endl;
        }
    }

    void updateSimulation() {
        if (!isRunning) return;

        // Update all system components
        routingSystem.updateAllVehiclePositions();
        signalManager.updateAllSignals();
        disruptionManager.updateDisruptionStates();

        // Clean up finished vehicles
        routingSystem.cleanupFinishedVehicles();

        // Increment time
        timeStep++;
    }


    bool loadInitialData() {
        bool success = true;

        // Load network
        if (!network.loadFromCSV("road_network.csv")) {
            cout << "Failed to load road network" << endl;
            success = false;
        }

        // Load signals
        if (!signalManager.loadSignalsFromCSV("traffic_signals.csv")) {
            cout << "Failed to load traffic signals" << endl;
            success = false;
        }

        // Load vehicles
        if (!routingSystem.loadVehiclesFromCSV("vehicles.csv")) {
            cout << "Failed to load vehicles" << endl;
            success = false;
        }

        // Load disruptions
        if (!disruptionManager.loadDisruptions("road_closures.csv")) {
            cout << "Failed to load road closures" << endl;
            success = false;
        }

        return success;
    }

public:
    TrafficSimulation()
        : network(),
        routingSystem(network),
        signalManager(network),
        disruptionManager(network, routingSystem),
        router(network, routingSystem.congestion),
        isRunning(false),
        timeStep(0),
        simulationSpeed(1) {
    }

    bool initialize() {
        if (!loadInitialData()) {
            cout << "Failed to initialize simulation data" << endl;
            return false;
        }

        routingSystem.setObserver(&signalManager);
        cout << "Simulation initialized successfully" << endl;
        return true;
    }
    

    void run() {
        std::string input;
        int displayInterval = 5; // Display update every 5 steps

        while (true) {
            displayMenu();
            std::getline(std::cin, input);

            if (input == "0") break;

            handleUserInput(input);

            if (isRunning) {
                updateSimulation();
                if (timeStep % displayInterval == 0) {
                    std::cout << "\033[2J\033[1;1H"; // Clear screen (works on most terminals)
                    routingSystem.displayAllVehicles();
                }
            }
        }
    }
};

// Main function
int main() {
    TrafficSimulation simulation;

    if (!simulation.initialize()) {
        cout << "Failed to initialize simulation" << endl;
        return 1;
    }

    simulation.run();
    return 0;
}
