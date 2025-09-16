#pragma once

#include "astar_conops1.h"
#include "cell.h"
namespace conop1{

class ThetaStar1 : public AStar_conops1 {
public:
    using AStar_conops1::AStar_conops1;

protected:
    // Check whether two point can see other
    // Parameters:
    // - from/to: 
    // Return:
    // - whether is line of sight
    bool line_of_sight(const Cell *from, const Cell *to, double *coeff = nullptr);

    // Get distance and risk cost between from and to
    double get_traversal_cost(const Cell *from, const Cell *to) override;

    double get_heuristic(const Cell *a) override;

    // This function uses line of sight to calculate cost
    void compute_cost(Cell *c, Cell *adj) override;
};

}