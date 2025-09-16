#pragma once

#include <queue>
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include<iomanip>
#include<future> //std::future std::promise
#include "cell.h"
#include "utils_conops3.h"
namespace py = pybind11;
using pybind11::array_t;
using pybind11::dict;

namespace conop3{
/*
part of data structure for open list
*/
struct CellEntry {
    double priority;
    Cell *cell;
    CellEntry(double p, Cell *c) : priority(p), cell(c) {}
    bool operator>(const CellEntry &rhs) const { return priority > rhs.priority; }
};
typedef std::priority_queue<CellEntry, std::vector<CellEntry>, std::greater<CellEntry>> pri_queue;

/*
use lazy deletion
data structure for open list
*/
class PriQueue : public pri_queue {
  public:
    PriQueue() : pri_queue() {}
    //operations
    Cell *pop() {
        while (!pri_queue::empty()) {
            Cell* c = pri_queue::top().cell;
            pri_queue::pop();
            if (!c->closed) {
                return c;
            }
        }
        return nullptr;
    }
    void push(Cell *c) {
        pri_queue::emplace(c->g + c->h, c);
    }
    void clear() {
        while (!empty()) {
            pri_queue::pop();
        }
    }
};

class AStar_conops3 {
public:
    // Constructor of the AStar_conops3 class.
    // Parameters:
    // - cells_type: grid cell type
    // - cells_cost: cost of grid cells
    // - xs, ys, zs: data along x, y, and z coordinates
    // - turning_cost_angle: turning angle cost coef
    // - turning_cost_consecutive: penalty coefficient on short turn
    // - climb_cost_ascend: Cost for ascending.
    // - climb_cost_descend: Cost for descending.
    // - climb_threshold: climbing angle threshold.
    // - space_ccoef: Coefficient for space cost.
    // - space_length: Width of route.
    // - nonuniform: Boolean indicating if the grid has nonuniform characteristics.
    // - operational: Boolean indicating if the system is operational or in a testing phase.
    // - window_size: the temporal window size, default 20min, 3s/cell
    AStar_conops3(const array_t<int32_t> & cells_type, const array_t<double> & cells_cost, const array_t<double> & xs, 
    const array_t<double> & ys, const array_t<double> & zs, double  grid_res, double turning_cost_angle, double turning_cost_consecutive,
    double climb_cost_ascend, double climb_cost_descend, double climb_threshold, double space_ccoef, int space_length,
    bool nonuniform, bool operational, const int & window_size);

    // Destructor of the AStar_conops3 class.
    virtual ~AStar_conops3(){};

    // This function is core function of Astar, it finds route between start and end point, it stores the route in reference p
    // Parameters:
    // - i1: start point
    // - i2: end point
    // - p: route to be stored
    void solve(array_t<int64_t> i1, array_t<int64_t> i2, std::promise<Route> & p);

    // This function replans route in subspace for distributed planning,
    // Parameters:
    // - agent: the OD pair to be replanned
    // - p: route to be stored
    // - conflicts_along_route: id of routes that has conflicts with this one
    // - gain: the improvement of the replan
    // - prev_conflict: the number of conflicts in previous route network
    // - neighbor_size: subspace size for re-search
    // - method:greedy one (method == 2) or stochastic one (method ==3)
    void solve_subspace(Agent agent, std::vector<Agent> agents, std::promise<Route> & p, std::promise<std::vector<int>> & conflicts_along_route,
     std::promise<double> & gain, int prev_conflict, int neighbor_size, int method);

    bool is_reserved(int64_t i, int64_t j, int64_t k, int64_t l) {
        return cells[index(i, j, k, l)].reserved;
    }
    bool is_accessed(int64_t i, int64_t j, int64_t k, int64_t l) {
        return cells[index(i, j, k, l)].access;
    }

    bool is_reachable(int64_t i, int64_t j, int64_t k, int64_t l) {
        return cells[index(i, j, k,l)].reachable;
    }
    /*
    should rebuild when switch between parallel and sequential
    */
    bool is_protected(int64_t i, int64_t j, int64_t k) {
        /*parallel version*/
        return false;
        /*sequential*/
        // return cells[index(i, j, k)].protected_;
    }
    void set_reserved(int64_t i, int64_t j, int64_t k, int64_t l, bool reserved) {
        cells[index(i, j, k, l)].reserved = reserved;
    }
    void set_reachable(int64_t i, int64_t j, int64_t k, int64_t l, bool reachable, int operate) {
        cells[index(i, j, k, l)].reachable = reachable;
        if(operate==-1){cells[index(i, j, k, l)].operate = operate;}//used by operate
    }
    /*
    Set occupied cell
    */
    void set_occupied_id(int64_t i, int64_t j, int64_t k, int64_t l, int id, bool route_or_buffer) {
        route_occupation occupied;
        Cell & c=cells[index(i, j, k, l)];
        if(c.access==false)return;/*obstacle*/
        occupied={route_or_buffer?false:true,false,id};//{whether protected, whether reachable, route_id}
        if(c.occupied_routes.empty()) {c.occupied_routes.push_back(occupied);}/*first itself*/
        else if(c.occupied_routes.back().route_id!=id){c.occupied_routes.push_back(occupied);}/*other routes exist*/
    }
    /*
    Cancell the set for cells
    */
    void unset_occupied_id(int64_t i, int64_t j, int64_t k, int64_t l, int id, bool route_or_buffer) {
        route_occupation occupied;
        Cell & c=cells[index(i, j, k, l)];
        if(c.access==false)return;/*obstacle*/
        if(c.occupied_routes.size()==1){
            occupied={route_or_buffer?false:true,false,id};
            if(c.occupied_routes.back().route_id==id){/*only remain itself*/
                if(c.occupied_routes.back().protected_==occupied.protected_ and c.occupied_routes.back().reachable==occupied.reachable){
                    c.occupied_routes.clear();
                }
            }
        }else if(c.occupied_routes.size()>1){
            for(std::vector<route_occupation>::iterator it=c.occupied_routes.begin();it!=c.occupied_routes.end();it++){
                if((*it).route_id==id){/*remove itself if correct*/
                    occupied=*it;
                    if(!occupied.reachable && ((!occupied.protected_ && route_or_buffer) || (occupied.protected_ && !route_or_buffer))){
                        c.occupied_routes.erase(it);
                    }
                    break;
                }
            }
        }
    }
    void set_protected(int64_t i, int64_t j, int64_t k, int64_t l, bool protect) {
        cells[index(i, j, k, l)].protected_ = protect;
    }

    // Calculates the total number of conflicts within all grid cells of the pathfinding grid.
    // A conflict occurs when a cell is used by two or more routes simultaneously.
    int conflict_size();
    
    // Retrieves all cells experiencing conflicts, called after cal_conflicted_routes
    // Conflicts arise when a cell is used by route-route or route-buffer
    std::vector<Conflict> conflict_cell_route_calculation();

    // This function calculates the head conflicts, which are i->i+1 and i+1->i
    // Parameters:
    // - route_parallel: all routes
    std::vector<head_conflict> get_inverse_conflicts(const std::vector<Agent> & route_parallel);

    // Checks the conflicts along the planned route; it also updates the grid cells; 
    // Parameters:
    // - p / id: route and its route id
    // Return:
    // - route ids: routes that has conflict with the route
    std::vector<int> cal_conflicted_routes(Route p,int id);

    // Calculates traversal cost for route r, used for distributed planning, should be careful on operational costs, which is not included here
    // Parameters:
    // - r: route to be calculated
    double calculate_costs(Route r);

    // This function move the time window forward based on new order event
    // It also block cells occupied by ongoing trajectories
    // Parameters:
    // - time_refpoint: start time of current time window
    // - trajectories_for_conflicts: ongoing trajectories that current drone should avoid
    void update_cells(const double & time_refpoint, const std::vector<std::vector<std::vector<double>>> & trajectories_for_conflicts);

protected:
    // Initialize the cells of the grid with their positions, costs, and types.
    void init_cells(const double *xs, const double *ys, const double *zs, const int32_t *cells_type, const double *cells_cost);
    
    // Reset the cells of the grid 
    // Clear heuristics, costs, and pointers to reuse cells
    void reset();

    inline int index(const int i, const int j, const int k, const int l) {//updated: search in 4D 
        return i * stride_i + j * stride_j + k * stride_k + l;
    }

    inline int index3d(const int i, const int j, const int k) {//updated: search in 4D 
        return i * stride_i3 + j * stride_j3 + k;
    }

    inline Cell *get_cell(const int i, const int j, const int k, const int l) {//updated: search in 4D 
        return &cells[index(i, j, k, l)];
    }

    virtual double get_heuristic(const Cell *a) {
        return distance4d_direct(end, a);
    }

    // Calculates the cost (distance and risk) between two cells in the map
    // Parameters
    // - from / to: A pointer to the starting cell and ending cell
    double get_traversal_cost(const Cell *from, const Cell *to);

    // This function works for distributed planning, it calculates congestion cost
    // Parameters:
    // - from / to: pointers to start and end point
    // Return:
    // - conflict cost
    double get_conflict_cost(const Cell *from, const Cell *to);

    // This function calculates adjacent cells to be visited next
    // Compared to 3D version, the 4D one can only move forward in time dimension
    // Parameters
    // - c: current cell 
    // Return
    // - cells to be added to open list
    std::vector<Cell *> get_adjacent_cells(const Cell *c);

    // This function calculates adjacent cells to be visited next, only for distributed planning
    // Parameters:
    // - c: current cell 
    // Return:
    // - cells to be added to open list
    std::vector<Cell *> get_adjacent_cells_subspace(const Cell *c);
    
    // This function returns the route after the search
    // Parameter:
    // - is_conflict_included: whether include congestion cost
    Route get_route(bool is_conflict_included);

    // This function updates current cell for next search
    // Parameters:
    // - c/adj: current cell and adjacent cells in openlists 
    void update_cell(Cell *c, Cell *adj);
    
    // This function calculates cost
    // Parameters:
    // - c/adj: current cell and adjacent cells in openlists
    void compute_cost(Cell *c, Cell *adj);    

protected:
    double turning_cost_angle, turning_cost_consecutive;
    double climb_cost_ascend, climb_cost_descend, climb_threshold;
    double space_cost_coef,conflict_cost_coef=0;
    int space_length;
    double  grid_res;
    bool protect_overlap=false;
    std::vector<Cell> cells;

    PriQueue open;

    Cell *start;
    Cell *end;
    Cell *current;

    int grid_size[4];
    int stride_all, stride_i, stride_j, stride_k, stride_i3, stride_j3;
    bool nonuniform, operational;
    // bool conops_is_3;
};

// This function cancel actions performed on environment
// Parameters:
// - p: route that need cancel
// - id: route id for the route
// - solver: 
void backtrack_cell(const Route& p, const int& id,AStar_conops3 & solver);

// This function writes route occupation infor into the cells 
// Parameters:
// - route_parallel: all routes, includes id, route,conflict cost coefficient
void write_cells(AStar_conops3& solver,const std::vector<Agent>& route_parallel);
}

