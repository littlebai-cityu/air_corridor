#pragma once

#include<thread>      //std::thread
#include<utility>     //std::ref
#include <random>       // std::default_random_engine
#include <chrono>       // std::chrono::system_clock
#include "drone.h"
#include "astar_conops3.h"
#include <glog/logging.h>

namespace conop3{
struct Cells {
    /*used for cell index*/
    std::vector<Cell> cells;
    double min_x,min_y,min_z,min_t,max_x,max_y,max_z, max_t;
    int stride_all, stride_i, stride_j, stride_k;
    int grid_size[4];
    int grid_steps[4];

    inline int index(const int i, const int j, const int k, const int l) {          // 4d index
        return i * stride_i + j * stride_j + k * stride_k + l;
    }

    inline Cell *get_cell(const int i, const int j, const int k, const int l) {     //return 4d cells 
        return &cells[index(i, j, k, l)];
    }

    inline Cell *get_cell_from_pos(const double & x, const double & y, const double & z, const double & t) {
        int i=static_cast<int>(std::floor((x-min_x)/grid_steps[0]));
        int j=static_cast<int>(std::floor((y-min_y)/grid_steps[1]));
        int k=0.0;
        if(grid_steps[2]!=0.0){
            k=static_cast<int>(std::floor((z-min_z)/grid_steps[2]));
        }
        int l=static_cast<int>(std::floor((t-min_t)/grid_steps[3]));
        // to avoid some numerical issues.
        if (i >= grid_size[0] || j >= grid_size[1] || k >= grid_size[2] || t >= grid_size[3]) {
            LOG(WARNING) << "get_cell_from_pos: out of bound. " << i << " " << j << " " << k;
            i = std::min(i, grid_size[0] - 1);
            j = std::min(j, grid_size[1] - 1);
            k = std::min(k, grid_size[2] - 1);
            l = std::min(l, grid_size[3] - 1);
        }

        int idx = index(i,j,k,l);
        Cell *ptr = &cells.at(idx);
        return ptr;
    }
    inline std::tuple<int, int, int, int> get_index_from_pos(const double & x, const double & y, const double & z){
        int i=static_cast<int>(std::floor((x-min_x)/grid_steps[0]));
        int j=static_cast<int>(std::floor((y-min_y)/grid_steps[1]));
        int k=0.0;
        if(grid_steps[2]!=0.0){
            k=static_cast<int>(std::floor((z-min_z)/grid_steps[2]));
        }
        int t=static_cast<int>(std::floor((t-min_t)/grid_steps[3]));
        return {i,j,k,t};
    }
};

// This function is a sequential version for Conops3, grid graph based, sequential, simutaneously generate routes and trajectories
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
// - window_size: size of temporal window
// Returns:
// - routes/trajectories
py::list conops3_route_seq(const array_t<int32_t> & cells_type, const array_t<double> & cells_cost,  const array_t<double> & xs, const array_t<double> & ys,
 const array_t<double> & zs, double  grid_res, double turning_cost_angle, double turning_cost_consecutive, double climb_cost_ascend, double climb_cost_descend,
 double climb_threshold, double space_ccoef, int space_length, bool nonuniform, bool operational, int route_number, const py::list & dep,
 const py::list & dest, const py::list & centerd, const py::list & centera, const int & shuffled, const int & window_size, bool conops_is_3);

// This function is a distributed version for Conops3, grid graph based, distributed, simutaneously generate routes and trajectories
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
// - method: not used
// - window_size: size of temporal window
// Returns:
// - routes/trajectories
py::list conops3_route_dis(const array_t<int32_t> & cells_type, const array_t<double> & cells_cost,  const array_t<double> & xs, const array_t<double> & ys,
 const array_t<double> & zs,  double  grid_res, double turning_cost_angle, double turning_cost_consecutive, double climb_cost_ascend, double climb_cost_descend,
 double climb_threshold, double space_ccoef, int space_length, bool nonuniform, bool operational, int route_number, const py::list & dep,
 const py::list & dest, const py::list & centerd, const py::list & centera,const int & method, const int & window_size);



//roadmap based, sequential, simutaneously generate routes and trajectories
// py::list conops3_route_dis_r(const array_t<int32_t> & cells_type, const array_t<double> & cells_cost,  const array_t<double> & xs, const array_t<double> & ys,
//  const array_t<double> & zs, double turning_cost_angle, double turning_cost_consecutive, double climb_cost_ascend, double climb_cost_descend,
//  double climb_threshold, double space_ccoef, int space_length, bool nonuniform, bool operational, int route_number, const py::list & dep,
//  const py::list & dest, const py::list & centerd, const py::list & centera, const int & method);

// //roadmap based, sequential, simutaneously generate routes and trajectories
// py::list conops3_route_seq_r(const array_t<int32_t> & cells_type, const array_t<double> & cells_cost,  const array_t<double> & xs, const array_t<double> & ys,
//  const array_t<double> & zs, double turning_cost_angle, double turning_cost_consecutive, double climb_cost_ascend, double climb_cost_descend,
//  double climb_threshold, double space_ccoef, int space_length, bool nonuniform, bool operational, int route_number, const py::list & dep,
//  const py::list & dest, const py::list & centerd, const py::list & centera, const int & shuffled);

// This function perform sequential planning algorithm on one route sequence
// Parameters:
// - temp_cost: cost vector for each route
py::list one_sequential(AStar_conops3 & solver,int route_number,std::vector<std::vector<int32_t>>departures,
std::vector<std::vector<int32_t>> destinations,std::vector<std::vector<double>> center_d,
std::vector<std::vector<double>> center_a,int space_length,int xlim,int ylim,int zlim,double grid_res, std::vector<double> & temp_cost, bool conops_is_3);
}