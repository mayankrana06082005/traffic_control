#include <iostream>
#include <vector>
#include <string>
#include <memory>
#include <cmath>
#include <iomanip>
#include <fstream>
#include <sstream>
#include <map>
#include <algorithm>

using namespace std;

// ==========================================
// INTERFACES & FORWARD DECLARATIONS
// ==========================================
class Edge;
class Node;

class Vehicle {
public:
    int id;
    double position;
    double speed;
    double acceleration;
    string state_desc;
    Edge* current_edge;

    // IDM Parameters
    double v0 = 15.0; // Desired speed (m/s)
    double s0 = 2.0;  // Minimum gap (m)
    double T = 1.5;   // Safe time headway (s)
    double a = 1.0;   // Max acceleration (m/s^2)
    double b = 1.5;   // Comfortable deceleration (m/s^2)
    double vehicle_length = 4.0; 

    Vehicle(int id, Edge* start_edge, double initial_pos = 0.0);
    void update(double dt, Vehicle* leader, bool facing_red_light, double distance_to_light);
};

class TrafficLight {
public:
    bool is_green;
    TrafficLight() : is_green(false) {}
    void setGreen(bool state) { is_green = state; }
};

class Edge {
public:
    int id;
    double length;
    Node* to_node;
    vector<shared_ptr<Vehicle>> vehicles;
    TrafficLight light;

    Edge(int id, double length, Node* to_node) : id(id), length(length), to_node(to_node) {}
    void addVehicle(shared_ptr<Vehicle> v);
    int getWaitingVehicleCount();
};

class Node {
public:
    int id;
    vector<Edge*> incoming_edges;
    vector<Edge*> outgoing_edges;

    Node(int id) : id(id) {}
    void addIncoming(Edge* e) { incoming_edges.push_back(e); }
    void addOutgoing(Edge* e) { outgoing_edges.push_back(e); }
};

class AdaptiveAIController {
public:
    string latest_action;
    void optimizeIntersections(vector<shared_ptr<Node>>& nodes);
};

// ==========================================
// IMPLEMENTATIONS
// ==========================================

Vehicle::Vehicle(int id, Edge* start_edge, double initial_pos) 
    : id(id), position(initial_pos), speed(0.0), acceleration(0.0), state_desc("Spawned"), current_edge(start_edge) {}

void Vehicle::update(double dt, Vehicle* leader, bool facing_red_light, double distance_to_light) {
    double s_star = 0.0;
    double delta_v = 0.0;
    double actual_gap = 9999.0; // arbitrarily large if no leader

    // Determine the most critical obstacle (either the car ahead, or a red light)
    bool tracking_light = false;
    
    if (leader != nullptr) {
        actual_gap = (leader->position - this->position) - vehicle_length;
        delta_v = this->speed - leader->speed;
        state_desc = "Following V" + to_string(leader->id);
    }
    
    // If there's a red light, and it's closer than the car ahead, brake for the light
    if (facing_red_light && distance_to_light > 0 && distance_to_light < actual_gap) {
        actual_gap = distance_to_light;
        delta_v = this->speed - 0.0; // Treating the red light as a stopped car
        tracking_light = true;
        state_desc = "Braking for Light";
    }

    if (actual_gap < 9999.0) {
        // IDM Gap equation
        s_star = s0 + max(0.0, (this->speed * T) + (this->speed * delta_v) / (2.0 * sqrt(a * b)));
        // IDM Acceleration equation
        acceleration = a * (1.0 - pow(this->speed / v0, 4) - pow(s_star / max(0.1, actual_gap), 2));
    } else {
        // Free flow
        acceleration = a * (1.0 - pow(this->speed / v0, 4));
        state_desc = "Free Flow";
    }

    // Kinematics update
    speed += acceleration * dt;
    if (speed < 0.05) { // Prevent reverse rolling due to math quirks
        speed = 0.0;
        if (tracking_light || actual_gap < 5.0) {
            state_desc = "Queued";
        }
    }
    position += speed * dt;
}

void Edge::addVehicle(shared_ptr<Vehicle> v) {
    v->current_edge = this;
    vehicles.push_back(v);
}

int Edge::getWaitingVehicleCount() {
    int count = 0;
    for (auto& v : vehicles) {
        // If a vehicle is near the end of the edge and moving very slowly, it's queued
        if (v->speed < 1.0 && (length - v->position) < 50.0) {
            count++;
        }
    }
    return count;
}

void AdaptiveAIController::optimizeIntersections(vector<shared_ptr<Node>>& nodes) {
    latest_action = "Monitoring...";
    for (auto& node : nodes) {
        if (node->incoming_edges.size() < 2) continue; // Only manage actual junctions

        Edge* max_queue_edge = nullptr;
        int max_queue = 0;

        // Scan all incoming roads for congestion
        for (Edge* edge : node->incoming_edges) {
            int q = edge->getWaitingVehicleCount();
            if (q > max_queue) {
                max_queue = q;
                max_queue_edge = edge;
            }
        }

        // If there's a significant queue, adapt the lights!
        if (max_queue > 0 && max_queue_edge != nullptr) {
            if (!max_queue_edge->light.is_green) {
                latest_action = "[AI] Switched Node " + to_string(node->id) + " green to Edge " + to_string(max_queue_edge->id) + " (Queue: " + to_string(max_queue) + ")";
                
                // Set all to red
                for (Edge* edge : node->incoming_edges) edge->light.setGreen(false);
                // Set the congested one to green
                max_queue_edge->light.setGreen(true);
            }
        }
    }
}

// ==========================================
// MAIN SIMULATION LOOP
// ==========================================
int main() {
    // 1. Build Network Graph
    auto n0 = make_shared<Node>(0); // Spawn 1
    auto n1 = make_shared<Node>(1); // Spawn 2
    auto n2 = make_shared<Node>(2); // Main Intersection
    auto n3 = make_shared<Node>(3); // Exit Node

    auto edge1 = make_shared<Edge>(1, 200.0, n2.get()); // Road A to Intersection
    auto edge2 = make_shared<Edge>(2, 200.0, n2.get()); // Road B to Intersection
    auto edge3 = make_shared<Edge>(3, 200.0, n3.get()); // Intersection to Exit

    n2->addIncoming(edge1.get());
    n2->addIncoming(edge2.get());
    n2->addOutgoing(edge3.get());

    // Initial state: Edge 1 gets Green, Edge 2 gets Red
    edge1->light.setGreen(true);
    edge2->light.setGreen(false);

    vector<shared_ptr<Edge>> all_edges = {edge1, edge2, edge3};
    vector<shared_ptr<Node>> all_nodes = {n0, n1, n2, n3};
    AdaptiveAIController ai_brain;

    ofstream log_file("simulation_log.txt");
    double dt = 1.0;
    int next_vid = 1;

    for (int tick = 0; tick <= 30; tick++) {
        ostringstream frame_buffer;
        frame_buffer << "\n=== Tick: " << tick << "s ===\n";

        // --- TRAFFIC SPAWNER ---
        // Spawn cars on Edge 2 heavily to trigger an AI response
        if (tick % 3 == 0 && tick < 15) {
            edge2->addVehicle(make_shared<Vehicle>(next_vid++, edge2.get(), 10.0));
        }
        // Spawn cars on Edge 1 lightly
        if (tick % 8 == 0 && tick < 20) {
            edge1->addVehicle(make_shared<Vehicle>(next_vid++, edge1.get(), 10.0));
        }

        // --- AI OPTIMIZATION ---
        ai_brain.optimizeIntersections(all_nodes);
        frame_buffer << "AI Status: " << ai_brain.latest_action << "\n\n";

        frame_buffer << left << setw(8) << "Vehicle" << setw(8) << "Edge" 
                     << setw(10) << "Pos(m)" << setw(10) << "Vel(m/s)" 
                     << setw(10) << "Acc(m/s2)" << "State\n";
        frame_buffer << "------------------------------------------------------------\n";

        // --- PHYSICS UPDATE ---
        for (auto& edge : all_edges) {
            // Sort vehicles by position (furthest ahead first)
            sort(edge->vehicles.begin(), edge->vehicles.end(), [](const shared_ptr<Vehicle>& a, const shared_ptr<Vehicle>& b) {
                return a->position > b->position;
            });

            for (size_t i = 0; i < edge->vehicles.size(); i++) {
                auto v = edge->vehicles[i];
                Vehicle* leader = (i == 0) ? nullptr : edge->vehicles[i - 1].get();
                
                bool facing_red = !edge->light.is_green;
                double dist_to_light = edge->length - v->position;

                v->update(dt, leader, facing_red, dist_to_light);

                frame_buffer << "V" << left << setw(7) << v->id 
                             << setw(8) << v->current_edge->id 
                             << setw(10) << fixed << setprecision(1) << v->position 
                             << setw(10) << v->speed 
                             << setw(10) << v->acceleration 
                             << v->state_desc << "\n";
            }
        }

        // --- QUEUE SENSOR TELEMETRY ---
        frame_buffer << "\n[Sensors] Edge 1 Q: " << edge1->getWaitingVehicleCount() 
                     << " (Light: " << (edge1->light.is_green ? "GREEN" : "RED") << ")\n";
        frame_buffer << "[Sensors] Edge 2 Q: " << edge2->getWaitingVehicleCount() 
                     << " (Light: " << (edge2->light.is_green ? "GREEN" : "RED") << ")\n";

        // Print and Log
        cout << frame_buffer.str();
        log_file << frame_buffer.str();
    }

    log_file.close();
    return 0;
}