Smart Traffic Management Simulation

A microscopic traffic simulation engine built in C++ designed to model realistic vehicle flows, detect congestion, and provide a testing ground for Adaptive AI traffic light control systems.

This project implements a continuous mathematical model for vehicle physics (Intelligent Driver Model) operating over a directed graph network, allowing for emergent traffic behaviors like shockwaves and phantom jams without relying on artificial noise.

🏗 System Architecture

Topology: Directed Graph (Nodes = Intersections, Edges = Road Segments).

Physics Engine: Intelligent Driver Model (IDM) for continuous, realistic acceleration and braking.

Intersection Logic: "Virtual Vehicle" constraints mapping traffic light states (Red/Yellow) directly into the IDM equations.

Simulation Loop: Time-stepped ($\Delta t$) Euler integration.

🚀 Development Roadmap

This project is structured into progressive phases. We begin with a rigid, orderly system and iteratively add realism (physics, congestion, deadlocks) to ensure the core data structures are stable before being stressed by AI and heavy traffic loads.

Phase 1: Foundation (The Graph and Signals)

Goal: Build the static environment, establish the routing network, and implement basic, orderly vehicle movement.

1.1 The Road Network (Directed Graph):

Implement Node (Intersection) and Edge (Road segment) classes.

Define edge properties: length, start_node, end_node, capacity.

Create a simple grid network (e.g., a $3 \times 3$ intersection layout).

1.2 Traffic Light State Machine:

Implement a TrafficController for each Node.

Create a Fixed-Time cycle (e.g., 30s North-South Green, 30s East-West Green).

1.3 Orderly Vehicle Movement (Dummy Physics):

Implement a basic Vehicle class with properties: position, constant_speed, destination.

Implement the routing algorithm (A* or Dijkstra) so vehicles know their path.

Movement Logic: Vehicles travel at an exact, constant speed. If a light is red, they stop instantly. If green, they move instantly. No acceleration math yet.

🎯 Phase 1 Milestone: You can spawn a vehicle, watch it traverse multiple intersections following a calculated path, and obey red lights without crashing.

Phase 2: Microscopic Physics (Intelligent Driver Model)

Goal: Replace the "dummy" movement with realistic, continuous mathematical physics.

2.1 IDM Implementation:

Add IDM properties to vehicles: $a_{max}$ (acceleration), $b$ (braking), $T$ (safe time gap), $s_0$ (minimum gap).

Update the $\Delta t$ simulation loop to calculate acceleration based on the distance to the vehicle directly in front.

2.2 Edge Queue Management:

Modify Edges to maintain a list of active vehicles on that road, strictly sorted by their position. This is required so Vehicle $N$ knows Vehicle $N-1$ is its leader.

2.3 The "Virtual Vehicle" Light Logic:

Remove the "instant stop" logic from Phase 1.

If a light is red, spawn a stationary "Virtual Vehicle" at the stop line. The real vehicles' IDM physics will naturally force them to smoothly brake and line up behind it.

🎯 Phase 2 Milestone: Vehicles exhibit realistic shockwaves, phantom traffic jams, and smooth acceleration/deceleration. They stop seamlessly at red lights using purely mathematical formulas.

Phase 3: Congestion Detection & System Stress

Goal: Scale up the simulation, introduce realistic traffic volumes, and build the sensor network to identify trouble spots.

3.1 Random O-D Spawning:

Implement a TrafficGenerator that continuously spawns vehicles at random network edges with destinations at other random edges.

Implement "Virtual Spawn Queues" so cars don't overlap if they try to spawn on an already full road.

3.2 Sensor Network (Edge Telemetry):

Implement a system to calculate real-time metrics for every edge: average_speed, vehicle_count, and flow_rate.

3.3 Congestion Detection Logic:

Define thresholds (e.g., "If average speed on Edge A is $< 10\%$ of the speed limit for $> 15$ seconds, mark as CONGESTED").

Create a central monitoring module that logs or visually highlights congested edges.

🎯 Phase 3 Milestone: The simulation runs hundreds of vehicles simultaneously. The system successfully detects, flags, and logs roads that have become congested due to volume or poor traffic light timing.

Phase 4: Extreme Scenarios & Deadlock Management

Goal: Protect the simulation from freezing during gridlock and handle unpredictable anomalies.

4.1 Strict Intersection Locking (Preventing "Blocking the Box"):

Update the entry logic: A vehicle cannot cross an intersection on a green light unless the road it wants to enter has enough physical space to hold it.

4.2 Wait-For Graph (Cycle Detection):

Implement the background graph analysis algorithm. Detect if Edge A is waiting for Edge B, which is waiting for Edge C, which is waiting for Edge A.

4.3 Deadlock Resolution (The "Hand of God"):

When a cycle is detected, temporarily override the traffic lights to flush the gridlock.

Implement the "Eviction Failsafe": If a vehicle sits at $0$ speed for 5 simulated minutes, permanently delete it from the simulation and log an error.

4.4 (Optional) Accident Injection:

Trigger random events that drop an edge's capacity to $0$ by placing a permanent "stopped vehicle" on it, forcing the network to dynamically reroute.

🎯 Phase 4 Milestone: The simulation is bulletproof. Even under massive overload or physical blockage, it resolves deadlocks on its own and never completely freezes.

Phase 5 (Future Scope): AI & Optimization

Once Phase 1-4 are stable, you can replace the fixed-time traffic lights with your Adaptive AI Control (e.g., Max-Pressure or Reinforcement Learning) and test if the AI actually reduces the congestion detected in Phase 3!

🛠 Building and Running

This project uses CMake for build configuration.

mkdir build
cd build
cmake ..
make
./traffic_sim
