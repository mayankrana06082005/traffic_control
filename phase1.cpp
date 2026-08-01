#include <iostream>
#include <vector>
#include <string>
#include <unordered_map>
#include <queue>
#include <cmath>
#include <thread>
#include <chrono>
#include <mutex>
#include <algorithm>
#include <memory>
#include <iomanip>

using namespace std;

// ============================================================================
// GLOBAL CONSTANTS & ENUMS
// ============================================================================
const double SIM_TIME_STEP = 0.1; // Delta t in seconds

enum class LightState {
    RED,
    GREEN,
    YELLOW
};

// ============================================================================
// FORWARD DECLARATIONS
// ============================================================================
class Node;
class Edge;
class Vehicle;
class TrafficLight;

// ============================================================================
// INTERFACES (Simulating Header Files .h)
// ============================================================================

// --- TrafficLight ---
// Controls the flow of traffic for a specific incoming edge at an intersection.
class TrafficLight {
private:
    LightState state;
    double timer;
    double green_duration;
    double yellow_duration;
    double red_duration;

public:
    TrafficLight(double g = 30.0, double y = 5.0, double r = 35.0);
    
    void update(double dt);
    LightState getState() const;
    void forceState(LightState newState);
    string getStateString() const;
};

// --- Node ---
// Represents an intersection in the traffic graph.
class Node {
public:
    string id;
    double x, y;
    
    unordered_map<string, shared_ptr<TrafficLight>> incoming_lights;
    vector<shared_ptr<Edge>> outgoing_edges;

    Node(string _id, double _x = 0.0, double _y = 0.0);

    void addIncomingLight(string edge_id, shared_ptr<TrafficLight> light);
    void addOutgoingEdge(shared_ptr<Edge> edge);
    void updateLights(double dt);
};

// --- Edge ---
// Represents a directed road segment connecting two nodes.
class Edge {
public:
    string id;
    shared_ptr<Node> start_node;
    shared_ptr<Node> end_node;
    double length;
    double speed_limit;
    
    vector<shared_ptr<Vehicle>> vehicles_on_edge;

    Edge(string _id, shared_ptr<Node> _start, shared_ptr<Node> _end, double _length = 500.0, double _speed = 15.0);
        
    void addVehicle(shared_ptr<Vehicle> v);
    void removeVehicle(shared_ptr<Vehicle> v);
    void sortVehicles();
};

// --- Vehicle ---
// Represents a single car moving through the network.
class Vehicle : public std::enable_shared_from_this<Vehicle> {
public:
    string id;
    shared_ptr<Edge> current_edge;
    double position; 
    double speed;
    double desired_speed;
    
    shared_ptr<Node> destination;
    vector<shared_ptr<Edge>> route;
    int current_route_index;

    Vehicle(string _id, shared_ptr<Node> _dest);

    void update(double dt);
};

// --- TrafficNetwork ---
// The central simulation engine that orchestrates all updates.
class TrafficNetwork {
public:
    unordered_map<string, shared_ptr<Node>> nodes;
    unordered_map<string, shared_ptr<Edge>> edges;
    vector<shared_ptr<Vehicle>> active_vehicles;
    double sim_time = 0.0;

    void addNode(shared_ptr<Node> n);
    void addEdge(shared_ptr<Edge> e);
    void spawnVehicle(shared_ptr<Vehicle> v, shared_ptr<Edge> start_edge);
    
    void step(double dt);
    void printState();
};

// ============================================================================
// IMPLEMENTATIONS (Simulating Source Files .cpp)
// ============================================================================

// --- TrafficLight Implementation ---
TrafficLight::TrafficLight(double g, double y, double r)
    : state(LightState::RED), timer(0.0), green_duration(g), yellow_duration(y), red_duration(r) {}

void TrafficLight::update(double dt) {
    timer += dt;
    switch (state) {
        case LightState::GREEN:
            if (timer >= green_duration) {
                state = LightState::YELLOW;
                timer = 0.0;
            }
            break;
        case LightState::YELLOW:
            if (timer >= yellow_duration) {
                state = LightState::RED;
                timer = 0.0;
            }
            break;
        case LightState::RED:
            if (timer >= red_duration) {
                state = LightState::GREEN;
                timer = 0.0;
            }
            break;
    }
}

LightState TrafficLight::getState() const { return state; }
void TrafficLight::forceState(LightState newState) { state = newState; timer = 0.0; }
string TrafficLight::getStateString() const {
    if (state == LightState::RED) return "RED";
    if (state == LightState::YELLOW) return "YELLOW";
    return "GREEN";
}

// --- Node Implementation ---
Node::Node(string _id, double _x, double _y) : id(_id), x(_x), y(_y) {}

void Node::addIncomingLight(string edge_id, shared_ptr<TrafficLight> light) {
    incoming_lights[edge_id] = light;
}

void Node::addOutgoingEdge(shared_ptr<Edge> edge) {
    outgoing_edges.push_back(edge);
}

void Node::updateLights(double dt) {
    for (auto& pair : incoming_lights) {
        pair.second->update(dt);
    }
}

// --- Edge Implementation ---
Edge::Edge(string _id, shared_ptr<Node> _start, shared_ptr<Node> _end, double _length, double _speed)
    : id(_id), start_node(_start), end_node(_end), length(_length), speed_limit(_speed) {}
    
void Edge::addVehicle(shared_ptr<Vehicle> v) {
    vehicles_on_edge.push_back(v);
    sortVehicles();
}

void Edge::removeVehicle(shared_ptr<Vehicle> v) {
    vehicles_on_edge.erase(remove(vehicles_on_edge.begin(), vehicles_on_edge.end(), v), vehicles_on_edge.end());
}

void Edge::sortVehicles() {
    sort(vehicles_on_edge.begin(), vehicles_on_edge.end(), [](const shared_ptr<Vehicle>& a, const shared_ptr<Vehicle>& b) {
        return a->position > b->position;
    });
}

// --- Vehicle Implementation ---
Vehicle::Vehicle(string _id, shared_ptr<Node> _dest) 
    : id(_id), destination(_dest), position(0.0), speed(0.0), current_route_index(0) {
    desired_speed = 15.0; 
}

void Vehicle::update(double dt) {
    if (!current_edge) return;

    double distance_to_end = current_edge->length - position;
    
    // Phase 1 Dummy Physics Logic
    if (distance_to_end < 20.0 && current_route_index < route.size()) {
        shared_ptr<Node> next_node = current_edge->end_node;
        
        if (next_node->incoming_lights.count(current_edge->id)) {
            auto light = next_node->incoming_lights[current_edge->id];
            if (light->getState() == LightState::RED) {
                speed = 0.0; // Stop
            } else {
                speed = desired_speed; // Go
            }
        } else {
             speed = desired_speed; 
        }
    } else {
        speed = desired_speed; 
    }

    position += speed * dt;

    // Node Transfer Logic
    if (position >= current_edge->length) {
        current_route_index++;
        if (current_route_index < route.size()) {
            double overshoot = position - current_edge->length;
            shared_ptr<Edge> next_edge = route[current_route_index];
            
            current_edge->removeVehicle(shared_from_this());
            current_edge = next_edge;
            position = overshoot;
            current_edge->addVehicle(shared_from_this());
        } else {
            current_edge->removeVehicle(shared_from_this());
            current_edge = nullptr; // Reached destination
        }
    }
}

// --- TrafficNetwork Implementation ---
void TrafficNetwork::addNode(shared_ptr<Node> n) { nodes[n->id] = n; }

void TrafficNetwork::addEdge(shared_ptr<Edge> e) { 
    edges[e->id] = e; 
    e->start_node->addOutgoingEdge(e);
    
    auto light = make_shared<TrafficLight>();
    e->end_node->addIncomingLight(e->id, light);
}

void TrafficNetwork::spawnVehicle(shared_ptr<Vehicle> v, shared_ptr<Edge> start_edge) {
    v->current_edge = start_edge;
    v->position = 0.0;
    active_vehicles.push_back(v);
    start_edge->addVehicle(v);
}

void TrafficNetwork::step(double dt) {
    for (auto& pair : nodes) {
        pair.second->updateLights(dt);
    }

    auto it = active_vehicles.begin();
    while (it != active_vehicles.end()) {
        (*it)->update(dt);
        if (!(*it)->current_edge) {
            it = active_vehicles.erase(it);
        } else {
            ++it;
        }
    }
    
    for (auto& pair: edges) {
        pair.second->sortVehicles();
    }
    sim_time += dt;
}

void TrafficNetwork::printState() {
    cout << "\n--- Time: " << fixed << setprecision(1) << sim_time << "s ---" << endl;
    for (auto& epair : edges) {
        auto edge = epair.second;
        cout << "Edge " << edge->id << " (" << edge->vehicles_on_edge.size() << " cars): ";
        
        if (edge->end_node->incoming_lights.count(edge->id)) {
             cout << "[Light: " << edge->end_node->incoming_lights[edge->id]->getStateString() << "] ";
        }
        for (auto& v : edge->vehicles_on_edge) {
            cout << " V" << v->id << "(@ " << setprecision(1) << v->position << "m) ";
        }
        cout << endl;
    }
}

// ============================================================================
// MAIN EXECUTION
// ============================================================================
int main() {
    TrafficNetwork sim;

    auto n1 = make_shared<Node>("N1");
    auto n2 = make_shared<Node>("N2"); 
    auto n3 = make_shared<Node>("N3");
    auto n4 = make_shared<Node>("N4");

    sim.addNode(n1); sim.addNode(n2); sim.addNode(n3); sim.addNode(n4);

    auto e1 = make_shared<Edge>("E1_W2E", n1, n2, 200.0);
    auto e2 = make_shared<Edge>("E2_S2N", n4, n2, 200.0);
    auto e3 = make_shared<Edge>("E3_E2E", n2, n3, 200.0);

    sim.addEdge(e1); sim.addEdge(e2); sim.addEdge(e3);

    n2->incoming_lights["E1_W2E"]->forceState(LightState::GREEN);
    n2->incoming_lights["E2_S2N"] = make_shared<TrafficLight>(30.0, 5.0, 35.0);
    n2->incoming_lights["E2_S2N"]->forceState(LightState::RED);

    auto v1 = make_shared<Vehicle>("1", n3);
    v1->route = {e1, e3}; 
    sim.spawnVehicle(v1, e1);
    
    auto v2 = make_shared<Vehicle>("2", n2); 
    v2->route = {e2};
    sim.spawnVehicle(v2, e2);

    cout << "Starting Organized Phase 1 Simulation..." << endl;
    
    for (int i = 0; i < 300; ++i) { 
        sim.step(SIM_TIME_STEP);
        if (i % 10 == 0) {
            sim.printState();
        }
    }

    return 0;
}