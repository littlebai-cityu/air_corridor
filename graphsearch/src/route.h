#pragma once
#include <iostream>
#include <vector>
// Defines the Route structure, which encapsulates the properties and metrics of a navigation route.
struct Route {
    std::vector<std::vector<int>> index;
    std::vector<std::vector<double>> position;
    double length, traversal, turning, climbing, space, conflict; // Associated costs

    Route() :
        length(0), traversal(0), turning(0), climbing(0), space(0), conflict(0)
    {}

    // Calculates the total cost
    double total() const {
        return traversal + turning + climbing + space + conflict;
    }
 };

// Represents the acceleration information for a segment of a route, detailing the three stages of movement: accelerate, flight, decelerate.
struct WaypointAccInfo {
    double total_seconds = 0.0;

    double t1 = 0.0;
    double v1 = 0.0;
    double a1 = 0.0;
    double delta_s1 = 0.0;

    double t2 = 0.0;
    double v2 = 0.0;
    double a2 = 0.0;
    double delta_s2 = 0.0;

    double t3 = 0.0;
    double v3 = 0.0;
    double a3 = 0.0;
    double delta_s3 = 0.0;

    WaypointAccInfo() {}

};
// Stores the status of a drone at a specific waypoint or position along the route, capturing its velocity, acceleration, and travel distance.
struct WaypointDistanceInfo {
    double vel = 0.0;
    double acc = 0.0;
    double travel_dist = 0.0;
};