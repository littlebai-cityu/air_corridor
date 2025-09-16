#pragma once

#include <iostream>
#include <cmath>
#include <unistd.h>
#include <fstream>
#include <sstream>
#include <sys/time.h>

#include <pybind11/pybind11.h>
#include <pybind11/eval.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>

#include "cell.h"

namespace py = pybind11;
using pybind11::array_t;
using pybind11::dict;
using pybind11::list;
using pybind11::make_tuple;
using pybind11::len;

namespace conop3{

/*
//data strcture to store route and protection cells for one route segment
*/
struct route_protection{
    std::vector<std::vector<int32_t>> route_cells;
    std::vector<std::vector<int32_t>> protection_zone_cell;
};

// This function prints the conflict and head conflict sets for each route
// Parameters:
// - conflicts: occupied ids and location
// - inv_conflicts: head conflicts
// - route_number: 
void print_conflicts(const std::vector<Conflict> & conflicts , std::vector<head_conflict> & inv_conflicts, int route_number);

route_protection line_traversal(const std::vector<int32_t>& p0, const std::vector<int32_t>& p1, const int& space_length, 
const int& xlim, const int& ylim, const int& zlim, const double& grid_res);
// This function calculates the accelerate and decelerate stages
// It makes the acceralation and deceleration equal the time window size exactly
// Parameters:
// - p: the route segment
// - max_v/max_a : drone dynamics
WaypointAccInfo generate_traj_1d(double p, double max_v, double max_a, double t);

// This function calculates the distance info along the route segment at the exact time
// Parameters:
// - info: the acc/dec information to calculate the position
// - time: exact time to get position along the route
WaypointDistanceInfo sample_traj(const WaypointAccInfo& info, double time);

// This function calculates the exact position
std::vector<double> get_3d_from_1d(const std::vector<double> & previous_waypoint, const std::vector<double> & current_waypoint,double delta);





}

