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
    
    int getWaitingVehicleCount() const;
};

// --- Vehicle ---
// Represents a single car moving through the network.
class Vehicle : public std::enable_shared_from_this<Vehicle> {
public:
    string id;
    shared_ptr<Edge> current_edge;
    double position; 
    double speed;
    
    double current_accel; 
    string action_state;  

    double desired_speed;
    double max_accel;
    double comfort_brake;
    double safe_time_gap;
    double min_gap;
    double length;
    
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

int Edge::getWaitingVehicleCount() const {
    int count = 0;
    for (const auto& v : vehicles_on_edge) {
        if (v->speed < 0.5) count++; // Consider speed < 0.5m/s as waiting in queue
    }
    return count;
}

// --- Vehicle Implementation ---
Vehicle::Vehicle(string _id, shared_ptr<Node> _dest) 
    : id(_id), destination(_dest), position(0.0), speed(0.0), current_route_index(0) {
    current_accel = 0.0;
    action_state = "Spawned";
    
    desired_speed = 15.0;  // 15 m/s (~33 mph)
    max_accel = 2.0;       // 2.0 m/s^2
    comfort_brake = 1.5;   // 1.5 m/s^2
    safe_time_gap = 1.5;   // 1.5 seconds following distance
    min_gap = 2.0;         // 2.0 meters minimum bumper-to-bumper gap
    length = 5.0;          // 5.0 meters physical vehicle length
}

void Vehicle::update(double dt) {
    if (!current_edge) return;

    double s = 10000.0; // Arbitrarily large distance (assumes free road initially)
    double delta_v = 0.0;
    
    action_state = "Free Flow";
    
    // 1. Find physical leader vehicle on the same edge
    auto& vehs = current_edge->vehicles_on_edge;
    for (size_t i = 0; i < vehs.size(); ++i) {
        if (vehs[i]->id == this->id) {
            if (i > 0) { // i=0 is the car closest to the intersection. If i>0, vehs[i-1] is ahead of us.
                auto leader = vehs[i-1];
                // Gap is distance between front of my car and back of leader car
                s = (leader->position - leader->length) - this->position;
                delta_v = this->speed - leader->speed;
                action_state = "Following V" + leader->id;
            }
            break;
        }
    }

    // 2. Handle Traffic Lights via "Virtual Vehicle" Trick
    double distance_to_end = current_edge->length - this->position;
    bool stop_at_light = false;
    
    shared_ptr<Node> next_node = current_edge->end_node;
    if (next_node->incoming_lights.count(current_edge->id)) {
        auto light = next_node->incoming_lights[current_edge->id];
        // Stop for RED or YELLOW lights
        if (light->getState() != LightState::GREEN) { 
            stop_at_light = true;
        }
    }

    // If stopping at a light, and the light is closer than the physical leader, the light line becomes the new target
    if (stop_at_light && distance_to_end < s) {
        s = distance_to_end;
        delta_v = this->speed - 0.0; // Virtual vehicle is completely stationary
        action_state = "Braking for Light";
    }

    // Check if completely queued/stopped for logging clarity
    if (this->speed < 0.5 && s < 10.0) {
        action_state = "Queued";
    }

    // 3. IDM (Intelligent Driver Model) Equation Calculation
    if (s < 0.1) s = 0.1; // Prevent division by zero safely
    
    // Calculate desired gap (s*)
    double s_star = min_gap + (this->speed * safe_time_gap) + 
                    (this->speed * delta_v) / (2.0 * sqrt(max_accel * comfort_brake));
    if (s_star < min_gap) s_star = min_gap; // Drivers never desire less than physical minimum gap

    // Final acceleration formula
    double acceleration = max_accel * (1.0 - pow(this->speed / desired_speed, 4) - pow(s_star / s, 2));
    this->current_accel = acceleration; // Store telemetry
    
    // 4. Euler Integration for movement
    this->speed += acceleration * dt;
    if (this->speed < 0.0) this->speed = 0.0; // Vehicles cannot reverse
    
    this->position += this->speed * dt;

    // 5. Node Transfer Logic (Crossing Intersections)
    if (this->position >= current_edge->length) {
        current_route_index++;
        if (current_route_index < route.size()) {
            double overshoot = this->position - current_edge->length;
            shared_ptr<Edge> next_edge = route[current_route_index];
            
            current_edge->removeVehicle(shared_from_this());
            current_edge = next_edge;
            this->position = overshoot;
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
    cout << "\n================================================================================" << endl;
    cout << "[ SIMULATION TIME: " << fixed << setprecision(1) << setw(5) << sim_time << " s ] NETWORK STATUS" << endl;
    cout << "--------------------------------------------------------------------------------" << endl;
    
    for (auto& epair : edges) {
        auto edge = epair.second;
        int waiting = edge->getWaitingVehicleCount();
        int total = edge->vehicles_on_edge.size();
        
        string light_str = "NONE ";
        if (edge->end_node->incoming_lights.count(edge->id)) {
             light_str = edge->end_node->incoming_lights[edge->id]->getStateString();
        }

        // Formatted Edge Header
        cout << left << setw(10) << ("[" + edge->id + "]") 
             << " | Light: " << setw(6) << light_str 
             << " | Queue: " << waiting << "/" << total << " cars" << endl;
             
        // Detailed Vehicle Telemetry
        for (auto& v : edge->vehicles_on_edge) {
            cout << "    -> V" << left << setw(3) << v->id 
                 << "| Pos: " << right << setw(5) << fixed << setprecision(1) << v->position << " m "
                 << "| Vel: " << setw(4) << v->speed << " m/s "
                 << "| Acc: " << setw(5) << showpos << v->current_accel << noshowpos << " m/s^2 "
                 << "| State: " << v->action_state << endl;
        }
    }
    cout << "================================================================================" << endl;
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

    // Spawn a trailing car behind V1 to demonstrate the Car-Following physics
    auto v3 = make_shared<Vehicle>("3", n3);
    v3->route = {e1, e3};
    sim.spawnVehicle(v3, e1);
    v3->position = -40.0; // Start 40 meters behind v1 to show smooth following

    cout << "Starting Phase 2 Simulation (IDM Physics Active)..." << endl;
    
    for (int i = 0; i < 300; ++i) { 
        sim.step(SIM_TIME_STEP);
        if (i % 10 == 0) {
            sim.printState();
        }
    }

    return 0;
}