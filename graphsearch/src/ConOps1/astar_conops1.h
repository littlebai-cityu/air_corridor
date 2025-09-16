#pragma once
#include <queue>
#include<future> //std::future std::promise
#include <algorithm>
#include <iomanip>
#include "utils_conops1.h"

using pybind11::array_t;

namespace conop1{
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


class AStar_conops1 {
public:
    // Constructor of the AStar_conops1 class.
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
    AStar_conops1(const array_t<int32_t> & cells_type, const array_t<double> & cells_cost, const array_t<double> & xs, const array_t<double> & ys, const array_t<double> & zs, 
    double  grid_res, double turning_cost_angle, double turning_cost_consecutive, double climb_cost_ascend, double climb_cost_descend, double climb_threshold, double space_ccoef,
    int space_length, bool nonuniform, bool operational);

    // Destructor of the AStar_conops1 class.
    virtual ~AStar_conops1(){};

    // This function is core function of Astar, it finds route between start and end point, it stores the route in reference p
    // Parameters:
    // - i1: start point
    // - i2: end point
    // - p: route to be stored
    void solve(array_t<int64_t> i1, array_t<int64_t> i2, std::promise<Route> & p);
    double calculate_cell_distance(int i1, int j1, int k1, int i2, int j2, int k2, double grid_res);

    // This function replans route in subspace for distributed planning,
    // Parameters:
    // - agent: the OD pair to be replanned
    // - p: route to be stored
    // - conflicts_along_route: id of routes that has conflicts with this one
    // - gain: the improvement of the replan
    // - prev_conflict: the number of conflicts in previous route network
    // - neighbor_size: subspace size for re-search
    // - method:greedy one (method == 2) or stochastic one (method ==3)
    void solve_subspace(Agent agent, std::promise<Route> & p, std::promise<std::vector<int>> & conflicts_along_route,
     std::promise<double> & gain, int prev_conflict, int neighbor_size, int method);

    bool is_reserved(int64_t i, int64_t j, int64_t k) {
        return cells[index(i, j, k)].reserved;
    }
    bool is_accessed(int64_t i, int64_t j, int64_t k) {
        return cells[index(i, j, k)].access;
    }

    bool is_reachable(int64_t i, int64_t j, int64_t k) {
        return cells[index(i, j, k)].reachable;
    }
    
    // Should switch between distributed and sequential
    bool is_protected(int64_t i, int64_t j, int64_t k) {
        /*distributed version*/
        // Cell & c=cells[index(i, j, k)];
        // if(c.occupied_routes.size()>0){
        //     for(route_occupation occupied:c.occupied_routes){
        //         if(occupied.protected_==true){
        //             return true;
        //         }
        //     }
        // }
        // return false;
        /*sequential*/
        return cells[index(i, j, k)].protected_;
    }
    void set_reserved(int64_t i, int64_t j, int64_t k, bool reserved) {
        cells[index(i, j, k)].reserved = reserved;
    }
    void set_reachable(int64_t i, int64_t j, int64_t k, bool reachable) {
        cells[index(i, j, k)].reachable = reachable;
    }
    // Set route_occupation for the cell
    // Parameters:
    // -route_or_buffer: whether route id use the cell as path or as buffer, true: as path, false: as buffer
    void set_occupied_id(int64_t i, int64_t j, int64_t k, int id, bool route_or_buffer) {
        route_occupation occupied;
        Cell & c=cells[index(i, j, k)];
        if(c.access==false)return;      //Cell is obstacle, not operate on it
        occupied={route_or_buffer?false:true,false,id};

        // Push back the cell, do not push back repeatedly
        if(c.occupied_routes.empty()) {c.occupied_routes.push_back(occupied);}   
        else if(c.occupied_routes.back().route_id!=id){c.occupied_routes.push_back(occupied);}
    }

    //Unset route_occupation for the cell
    // Parameters:
    // -route_or_buffer: whether route id use the cell as path or as buffer, true: as path, false: as buffer
    void unset_occupied_id(int64_t i, int64_t j, int64_t k, int id, bool route_or_buffer) {
        route_occupation occupied;
        Cell & c=cells[index(i, j, k)];
        if(c.access==false)return;                          //Cell is obstacle, not operate on it
        if(c.occupied_routes.size()==1){
            occupied={route_or_buffer?false:true,false,id};
            if(c.occupied_routes.back().route_id==id){      // only remain itself
                if(c.occupied_routes.back().protected_==occupied.protected_ and c.occupied_routes.back().reachable==occupied.reachable){
                    c.occupied_routes.clear();
                }
            }
        }else if(c.occupied_routes.size()>1){
            for(std::vector<route_occupation>::iterator it=c.occupied_routes.begin();it!=c.occupied_routes.end();it++){
                if((*it).route_id==id){                     // remove itself if correct
                    occupied=*it;
                    if(!occupied.reachable && ((!occupied.protected_ && route_or_buffer) || (occupied.protected_ && !route_or_buffer))){
                        c.occupied_routes.erase(it);
                    }
                    break;
                }
            }
        }
    }
    void set_protected(int64_t i, int64_t j, int64_t k, bool protect) {
        cells[index(i, j, k)].protected_ = protect;
    }

    // Calculates the total number of conflicts within all grid cells of the pathfinding grid.
    // A conflict occurs when a cell is used by two or more routes simultaneously.
    int conflict_size();

    // Retrieves all cells experiencing conflicts, called after cal_conflicted_routes
    // Conflicts arise when a cell is used by route-route or route-buffer
    std::vector<Conflict> conflict_cell_route_calculation();
    
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

protected:
    // Initialize the cells of the grid with their positions, costs, and types.
    void init_cells(const double *xs, const double *ys, const double *zs, const int32_t *cells_type, const double *cells_cost);

    // Reset the cells of the grid 
    // Clear heuristics, costs, and pointers to reuse cells
    void reset();

    // from 3d to 1d index
    inline int index(const int i, const int j, const int k) {
        return i * stride_i + j * stride_j + k;
    }

    // return pointer of the cell with index
    inline Cell *get_cell(const int i, const int j, const int k) {
        return &cells[index(i, j, k)];
    }

    virtual double get_heuristic(const Cell *a) {
        return min_traversal_cost * distance3d(end, a);
    }

    // Calculates the cost (distance and risk) between two cells in the map
    // Parameters
    // - from / to: A pointer to the starting cell and ending cell
    virtual double get_traversal_cost(const Cell *from, const Cell *to);

    // Calculates the cost (turning) between two cells in the map, require position of parent cell
    // Parameters
    // - from / to: A pointer to the starting cell and ending cell
    double get_turning_cost(const Cell *from, const Cell *to);

    // Calculates the cost (climb / descend) between two cells in the map, it limits the angle under a threshold
    // Parameters
    // - from / to: A pointer to the starting cell and ending cell
    double get_climb_cost(const Cell *from, const Cell *to);
    
    // This class function calculates space costs between cells, 
    // only paths and buffers will be added in space cost
    // Parameters:
    // - from / to: pointers to start and end point
    // Return:
    // -Space cost, includes start point if it's departure point
    double get_space_cost(const Cell *from, const Cell *to);
    
    // This function works for distributed planning, it calculates congestion cost
    // Parameters:
    // - from / to: pointers to start and end point
    // Return:
    // - conflict cost
    double get_conflict_cost(const Cell *from, const Cell *to);

    // This function calculates adjacent cells to be visited next
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

    // This function calculates cost, can be risk, distance, turning, climb, space, and conflict costs
    // Parameters:
    // - c/adj: current cell and adjacent cells in openlists
    virtual void compute_cost(Cell *c, Cell *adj);
    

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

    int grid_size[3];
    // int grid_res;
    int stride_all, stride_i, stride_j;
    bool nonuniform, operational;
    double min_traversal_cost = INF;
};

// This function cancel actions performed on environment
// Parameters:
// - p: route that need cancel
// - id: route id for the route
// - solver: 
// - space_length: route seperation size
// - xlim/ylim/zlim: environment size
void backtrack_cell(const Route& p, const int& id,AStar_conops1 & solver, const int& space_length, const int& xlim, const int& ylim,\
 const int& zlim, const double& grid_res);

// This function writes route occupation infor into the cells 
// Parameters:
// - route_parallel: all routes, includes id, route,conflict cost coefficient
void write_cells(AStar_conops1& solver,const std::vector<Agent>& route_parallel,const int& space_length,
const int& xlim, const int& ylim, const int& zlim, const double& grid_res);
}

