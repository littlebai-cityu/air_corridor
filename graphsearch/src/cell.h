#pragma once
#include <cmath>
#include <iostream>
#include "route.h"
const double PI = 3.141592653589793;
const double INF = std::numeric_limits<double>::infinity();

// Defines a structure to record the usage of a cell, indicating whether it is a protection zone or part of a route.
struct route_occupation{
    bool protected_=false;  // Indicates if the cell is a protected area, not to be used by routes.
    bool reachable=true;    // Indicates if the cell can be passed through by a route.
    int route_id=-1;        // Identifier for the route occupying the cell, if any.
};

// For distributed planning usage,
// Defines a structure for representing an agent (e.g., a route) in the network, 
// including its route, identifier, and a coefficient for conflict cost.
struct Agent{
    Route route;                        //The route agent
    int id;                             //Identifier
    double conflict_coefficient=0.2;    // Coefficient used in calculating the cost of conflicts involving this agent.
};

// Defines a structure to store conflicts occurring in the route network, with their position and time.
struct Conflict{
    int i,j,k,l;                        // Grid coordinates (i, j, k) and time (l)
    std::vector<route_occupation> ids;  // List of route_occupation instances involved in the conflict.
};

// Specific to Conops3, this structure is used to track head-on conflicts between routes.
struct head_conflict{
    int i1,j1,k1;
    int i2,j2,k2;
    int l;                              // time p1,l->p2,l+1 vs p2,l ->p1,l+1
    int route_id1, route_id2;
};

// Used in Conops 2 to record the usage of cells by drones, including the time span of occupation.
struct drone_occupation {
    std::string drone_id;
    double start_time;      //currently only store the start and end time
    double end_time;
    // TODO: consider storing start and end positions along with times.
};

// The primary structure for representing a cell in the grid, containing various attributes for pathfinding and conflict resolution.
struct Cell {
    int i, j, k, l;                                     // Grid coordinates and time index
    double x, y, z, t;                                  // Spatial and temporal coordinates.
    double g, h;                                        // Total cost (g) and heuristic value (h) for pathfinding.
    double cost_coeff, cost_coeff_backup;               // traversal cost, associated with risk, of the cell and its backup for distributed planning.
    bool closed;                                        // Used for path planning
    bool reachable, reserved, protected_, access=true;  // Status flags:
                                                        // reachable: whether can pass (other drones or obstacles); 
                                                        // reserved: near vertiports, not consider conflicts; 
                                                        // protected: whether protected, only for sequential; 
                                                        // access: obstacle or not
    bool reach_subspace;                                // Indicates if the cell belongs to a sub search space in distributed planning
    std::vector<route_occupation> occupied_routes;      // Records routes using the cell.
    Cell *parent;                                       // Pointer to the parent cell in the pathfinding algorithm
    std::vector<drone_occupation> occupied_drones;      // For Conops2, tracks drones' trajectories.
    int operate=-1;                                     // For Conops3, used to check head conflicts
    std::vector<drone_occupation> tem_drone;            // For Conops2, temporarily stores drone trajectories for conflict checking
    std::vector<int> from_cell_indexes;                 // For Conops2, stores inflow cell indexes in route generation.
    std::vector<int> to_cell_indexes;                   // For Conops2, stores outflow cell indexes in route generation.
    // Constructor initializing the cell with default values.
    Cell() :
        i(0), j(0), k(0), l(0),
        x(0), y(0), z(0), t(0),
        g(0), h(0),
        cost_coeff(1.0),
        closed(false),
        reachable(true), reserved(false), protected_(false), parent(nullptr)
    {}
 };
/*
std::clamp is introduced in c++17, but we use no other features in c++17
so we implement it here to lower the standard requirement to c++14
*/
template <typename T>
T clamp(const T& n, const T& lower, const T& upper) {
    return std::max(lower, std::min(n, upper));
}

// Calculates the turning angle in degrees between two consecutive path segments on the XY plane.
// This is used to determine how sharp a turn is between two path segments.
inline double turning_angle(const Cell *prev, const Cell *current, const Cell *next) {
    // TODO: check the angle for current == prev, whether zero
    const double x1 = current->x - prev->x, y1 = current->y - prev->y, x2 = next->x - current->x, y2 = next->y - current->y;
    const double cos_value = (x1 * x2 + y1 * y2) / ( std::sqrt(x1*x1 + y1*y1) * std::sqrt(x2*x2 + y2*y2) );
    return std::acos( clamp(cos_value, -1.0, 1.0) ) * 180.0 / PI; // return the angle in degrees
}

// Calculates the climb (or descent) angle in degrees between two cells in 3D space, relative to the XY plane.
// Optionally calculates the linear distance between the two cells.
inline double climb_angle(const Cell *from, const Cell *to, double *lenl1 = nullptr) {
    // TODO: check the angle for current == prev, whether zero
    const double lx = to->x - from->x;
    const double ly = to->y - from->y;
    const double lz = to->z - from->z;
    const double len_l1 = std::sqrt(lx*lx + ly*ly + lz*lz);
    if (lenl1) { *lenl1 = len_l1; }
    // Return the climb angle in degrees, using the asin function for the vertical component
    return std::asin( clamp(lz / len_l1, -1.0, 1.0) ) * 180.0 / PI;
}

// Calculates the 3D Euclidean distance between two cells, used to determine spatial separation.
inline double distance3d(const Cell *a, const Cell *b) {
    const double dx = a->x - b->x;
    const double dy = a->y - b->y;
    const double dz = a->z - b->z;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

// Calculates the 4D distance (3D space plus time) between two cells, treating time separately.
// This function is useful for applications where spatial and temporal distances contribute independently.
inline double distance4d_sep(const Cell *a, const Cell *b) {
    const double dx = a->x - b->x;
    const double dy = a->y - b->y;
    const double dz = a->z - b->z;
    return std::sqrt(dx * dx + dy * dy + dz * dz) + std::abs(b->t - a->t);
}
// Calculates the 4D distance between two cells, treating time as an equivalent component to spatial dimensions.
// This is another approach to 4D distance calculation where time is directly integrated into the distance metric.
inline double distance4d_direct(const Cell *a, const Cell *b) {
    const double dx = a->x - b->x;
    const double dy = a->y - b->y;
    const double dz = a->z - b->z;
    return 2*std::sqrt(dx * dx + dy * dy + dz * dz);
}
