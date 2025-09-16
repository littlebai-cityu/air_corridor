#pragma once

#include<thread>      //std::thread
#include<utility>     //std::ref
#include <random>       // std::default_random_engine
#include <chrono>       // std::chrono::system_clock
#include "thetastar1.h"
#include "drone.h"

namespace conop1{

// This function is a sequential version for Conops1
// Parameters:
// - cells_type: indicate obstacles, flyable zones, airports, etc.
// - cells_cost: traversal cost coefficient
// - xs, ys, zs: location info
// - turning_cost_angle,turning_cost_consecutive,climb_cost_ascend,climb_cost_descend,climb_threshold,space_ccoef: operational costs
// - space_length: route seperation size
// - nonuniform, operational: indicates whether use nonuniform costs and operational costs
// - route_number: number of routes need planning
// - dep / dest: location departures and destinations of routes
// - centerd / centera: airport center of departures and destinations 
// - shuffled=1: run 1 time, >1: run shuffled times and return the route with min cost
// Returns:
// - route sets
py::list conops1_route_seq(const array_t<int32_t> & cells_type, const array_t<double> & cells_cost,  const array_t<double> & xs, const array_t<double> & ys, 
 const array_t<double> & zs, double  grid_res, double turning_cost_angle, double turning_cost_consecutive, double climb_cost_ascend, double climb_cost_descend, 
 double climb_threshold, double space_ccoef, int space_length, bool nonuniform, bool operational, int route_number, const py::list & dep, 
 const py::list & dest, const py::list & centerd, const py::list & centera, const int & shuffled, AStar_conops1 & solver);

// This function is a parallel version for Conops1
// Parameters:
// - cells_type: indicate obstacles, flyable zones, airports, etc.
// - cells_cost: traversal cost coefficient
// - xs, ys, zs: location info
// - turning_cost_angle,turning_cost_consecutive,climb_cost_ascend,climb_cost_descend,climb_threshold,space_ccoef: operational costs
// - space_length: route seperation size
// - nonuniform, operational: indicates whether use nonuniform costs and operational costs
// - route_number: number of routes need planning
// - dep / dest: location departures and destinations of routes
// - centerd / centera: airport center of departures and destinations 
// - method=2: introduce conflict cost, select under max gain, =3: introduce conflict cost, randomly select
// Returns:
// - route sets
py::list conops1_route_dis(const array_t<int32_t> & cells_type, const array_t<double> & cells_cost,  const array_t<double> & xs, const array_t<double> & ys, 
 const array_t<double> & zs, double  grid_res, double turning_cost_angle, double turning_cost_consecutive, double climb_cost_ascend, double climb_cost_descend, 
 double climb_threshold, double space_ccoef, int space_length, bool nonuniform, bool operational, int route_number, const py::list & dep, 
 const py::list & dest, const py::list & centerd, const py::list & centera,const int & method, AStar_conops1 & solver);

// This function generates trajectory for the orders for Conops1
// Parameters:
// - route: the route used to generate trajectory
// - start_time: the start_time of the trajectory
std::vector<std::vector<double>> trajectory_generation(const std::vector<std::vector<double>> & route, const uint32_t & start_time);

// This function perform sequential planning algorithm on one route sequence
// Parameters:
// - temp_cost: cost vector for each route
// Return:
py::list one_sequential(AStar_conops1 & solver,int route_number,std::vector<std::vector<int32_t>>departures,std::vector<std::vector<int32_t>> destinations,
std::vector<std::vector<double>> center_d, std::vector<std::vector<double>> center_a,int space_length,int xlim,int ylim,int zlim,double grid_res,std::vector<double> & temp_cost);
// protected:
//     int grid_res;
}