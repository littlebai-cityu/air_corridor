#pragma once

#include<thread>      //std::thread
#include<utility>     //std::ref
#include <random>       // std::default_random_engine
#include <chrono>       // std::chrono::system_clock
#include "thetastar2.h"
#include "drone.h"
#include <glog/logging.h>

namespace conop2{
struct Cells {
    /*used for cell index*/
    std::vector<Cell> cells;
    double min_x,min_y,min_z,max_x,max_y,max_z;
    int stride_all, stride_i, stride_j;
    int grid_size[3];
    int grid_steps[3];

    inline int index(const int & i, const int & j, const int & k) {
        return i * stride_i + j * stride_j + k;
    }

    inline Cell *get_cell(const int & i, const int & j, const int & k) {
        int idx = index(i,j,k);
        if (idx >= static_cast<int>(cells.size())){
            LOG(ERROR) << "get_cell: out of bound. " << i << " " << j << " " << k;
            return nullptr;
        }
        Cell * ptr = &cells.at(idx);
        return ptr;
    }

    inline Cell *get_cell_from_pos(const double & x, const double & y, const double & z) {
        int i=static_cast<int>(std::floor((x-min_x)/grid_steps[0]));
        int j=static_cast<int>(std::floor((y-min_y)/grid_steps[1]));
        int k=0.0;
        if(grid_steps[2]!=0.0){
            k=static_cast<int>(std::floor((z-min_z)/grid_steps[2]));
        }
        // to avoid some numerical issues.
        if (i >= grid_size[0] || j >= grid_size[1] || k >= grid_size[2]) {
            LOG(WARNING) << "get_cell_from_pos: out of bound. " << i << " " << j << " " << k;
            i = std::min(i, grid_size[0] - 1);
            j = std::min(j, grid_size[1] - 1);
            k = std::min(k, grid_size[2] - 1);
        }

        int idx = index(i,j,k);
        Cell *ptr = &cells.at(idx);
        return ptr;
    }
    inline std::tuple<int, int, int> get_index_from_pos(const double & x, const double & y, const double & z){
        int i=static_cast<int>(std::floor((x-min_x)/grid_steps[0]));
        int j=static_cast<int>(std::floor((y-min_y)/grid_steps[1]));
        int k=0.0;
        if(grid_steps[2]!=0.0){
            k=static_cast<int>(std::floor((z-min_z)/grid_steps[2]));
        }
        return {i,j,k};
    }
};

// This function is a sequential version for Conops2, sequentially generate routes, intersections can exist
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
py::list conops2_route_seq(const array_t<int32_t> & cells_type, const array_t<double> & cells_cost,  const array_t<double> & xs, const array_t<double> & ys, 
 const array_t<double> & zs, double  grid_res,  double turning_cost_angle, double turning_cost_consecutive, double climb_cost_ascend, double climb_cost_descend, 
 double climb_threshold, double space_ccoef, int space_length, bool nonuniform, bool operational, int route_number, const py::list & dep, 
 const py::list & dest, const py::list & centerd, const py::list & centera, const int & shuffled, AStar_conops2 & solver);

// This function is a parallel version for Conops2
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
// Returns:
// - route sets
py::list conops2_route_dis(const array_t<int32_t> & cells_type, const array_t<double> & cells_cost,  const array_t<double> & xs, const array_t<double> & ys, 
 const array_t<double> & zs,double  grid_res, double turning_cost_angle, double turning_cost_consecutive, double climb_cost_ascend, double climb_cost_descend, 
 double climb_threshold, double space_ccoef, int space_length, bool nonuniform, bool operational, int route_number, const py::list & dep, 
 const py::list & dest, const py::list & centerd, const py::list & centera,const int & method, AStar_conops2 & solver);


// This function generates trajectory for the orders for Conops2
// Parameters:
// - cells: the copy of the current environment that contains all ongoing trajectories
// - route: the route used to generate trajectory
// - start_time: the start_time of the trajectory
// - drone1: dynamics used to generate trajectory
std::vector<std::vector<double>> trajectory_generation(Cells & cells, const std::vector<std::vector<double>> & route, const uint32_t & start_time, const DroneLimits &drone1);

// This function generates trajectory for the orders for Conops2
// Parameters:
// - previous_waypoint/current_waypoint/segments: the segment
// - segment_start_time: the start_time of the segment
// - cells: environment cells
// - max_delay: the maximum temporal overlap of conflicts
// - period_a/period_b: total period of the airspace used by others
// Returns:
// - conflict_flag: False: conflict exists, True: no conflict
bool check_conflicts(const std::vector<double>& previous_waypoint, const std::vector<double>& current_waypoint, const double & segment_start_time, 
    Cells & cells, std::vector<std::vector<double>> & segments, double & max_delay, double & period_a, double & period_b);

// This function is called inside check_conflicts() to store the current trajectory into environment temporally for comparison
// Parameters:
// - cells: environment cells
// - segments: current segments to be compared
void waypoints_update(Cells & cells, const std::vector<std::vector<double>> & segments);

// This function updates the environment cells after the trajectory is generated
// Parameters:
// - cells: environment cells
// - trajectories: the generated trajectory
// - drone_id: which drone will use this trajectories
void airspace_update(Cells & cells, const std::vector<std::vector<double>>& trajectories, const std::string & drone_id);

// This function perform sequential planning algorithm on one route sequence
// Parameters:
// - temp_cost: cost vector for each route
py::list one_sequential(AStar_conops2 & solver,int route_number,std::vector<std::vector<int32_t>>departures,std::vector<std::vector<int32_t>> destinations,
std::vector<std::vector<double>> center_d, std::vector<std::vector<double>> center_a,int space_length,int xlim,int ylim,int zlim,double grid_res, std::vector<double> & temp_cost);

}