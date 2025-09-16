#pragma once

#include "astar_conops2.h"
#include "cell.h"
namespace conop2{

class ThetaStar2 : public AStar_conops2 {
public:
    using AStar_conops2::AStar_conops2;

protected:
    // Check whether two point can see other
    // Parameters:
    // - from/to: 
    // Return:
    // - whether is line of sight
    bool line_of_sight(const Cell *from, const Cell *to, double *cost = nullptr);

    // Get distance and risk cost between from and to
    double get_traversal_cost(const Cell *from, const Cell *to) override;
    
    void compute_cost(Cell *c, Cell *adj) override;

    // This function uses line of sight to calculate cost
    double get_heuristic(const Cell *a) override;
};

}