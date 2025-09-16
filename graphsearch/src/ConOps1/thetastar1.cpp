#include "thetastar1.h"
namespace conop1{

bool ThetaStar1::line_of_sight(const Cell *from, const Cell *to, double *coeff) {
    const int dx = to->i - from->i;
    const int dy = to->j - from->j;
    const int dz = to->k - from->k;

    const int stepX = dx >= 0 ? 1 : -1;
    const int stepY = dy >= 0 ? 1 : -1;
    const int stepZ = dz >= 0 ? 1 : -1;

    const double tDeltaX = dx == 0 ? INF : stepX / double(dx);
    const double tDeltaY = dy == 0 ? INF : stepY / double(dy);
    const double tDeltaZ = dz == 0 ? INF : stepZ / double(dz);

    double tMaxX = dx == 0 ? INF : 0.5 * stepX / double(dx);
    double tMaxY = dy == 0 ? INF : 0.5 * stepY / double(dy);
    double tMaxZ = dz == 0 ? INF : 0.5 * stepZ / double(dz);

    int c[3] = {from->i, from->j, from->k};

    const Cell *cell = get_cell(c[0], c[1], c[2]);
    if (!cell->reachable) {
        return false;
    }

    double dt = 0, t = 0, prev_cost = 0;
    if (coeff) {
        prev_cost = cell->cost_coeff;
        *coeff = 0;
    }
    
    while (c[0] != to->i || c[1] != to->j || c[2] != to->k) {
        if (tMaxX < tMaxY) {
            if (tMaxX < tMaxZ) {
                dt = tMaxX - t;
                t = tMaxX;
                c[0] += stepX;
                tMaxX += tDeltaX;
            } else if (tMaxX > tMaxZ) {
                dt = tMaxZ - t;
                t = tMaxZ;
                c[2] += stepZ;
                tMaxZ += tDeltaZ;
            } else {
                dt = tMaxX - t;
                t = tMaxX;
                c[0] += stepX;
                tMaxX += tDeltaX;
                c[2] += stepZ;
                tMaxZ += tDeltaZ;
            }
        } else if (tMaxX > tMaxY) {
            if (tMaxY < tMaxZ) {
                dt = tMaxY - t;
                t = tMaxY;
                c[1] += stepY;
                tMaxY += tDeltaY;
            } else if (tMaxY > tMaxZ) {
                dt = tMaxZ - t;
                t = tMaxZ;
                c[2] += stepZ;
                tMaxZ += tDeltaZ;
            } else {
                dt = tMaxY - t;
                t = tMaxY;
                c[1] += stepY;
                tMaxY += tDeltaY;
                c[2] += stepZ;
                tMaxZ += tDeltaZ;
            }
        } else {
            if (tMaxX < tMaxZ) {
                dt = tMaxX - t;
                t = tMaxX;
                c[0] += stepX;
                tMaxX += tDeltaX;
                c[1] += stepY;
                tMaxY += tDeltaY;
            } else if (tMaxX > tMaxZ) {
                dt = tMaxZ - t;
                t = tMaxZ;
                c[2] += stepZ;
                tMaxZ += tDeltaZ;
            } else {
                dt = tMaxX - t;
                t = tMaxX;
                c[0] += stepX;
                tMaxX += tDeltaX;
                c[1] += stepY;
                tMaxY += tDeltaY;
                c[2] += stepZ;
                tMaxZ += tDeltaZ;
            }
        }

        cell = get_cell(c[0], c[1], c[2]);

        if (!cell->reachable) {                 // not line of sight
            return false;
        }

        if (coeff) {        
            *coeff += dt * prev_cost;
            prev_cost = cell->cost_coeff;
        }
    }

    if (coeff && t > 0) {                       // get coefficient to calculate the cost over segment (distance & risk)
        *coeff += (1.0 - t) * get_cell(c[0], c[1], c[2])->cost_coeff;
    }
    return true;
}

double ThetaStar1::get_traversal_cost(const Cell *from, const Cell *to) {
    double coeff = 1.0;
    if (line_of_sight(from, to, &coeff)) {
        return distance3d(from, to) * coeff;
    } else {
        return INF;
    }
}

double ThetaStar1::get_heuristic(const Cell *a) {
    if (nonuniform) {
        return min_traversal_cost * distance3d(end, a);         //use min_traversal_cost to ensure the heuristic not over estimate 
    } else {
        return distance3d(end, a);
    }
}

void ThetaStar1::compute_cost(Cell *c, Cell *adj) {
    if (c->parent) {
        double g_new = c->parent->g;
        if (nonuniform) {
            g_new += get_traversal_cost(c->parent, adj);
        } else {
            if (line_of_sight(c->parent, adj)) {
                g_new += distance3d(c->parent, adj);
            } else {
                g_new = INF;
            }
        }
        if (g_new < INF) {
            if (operational) {
                g_new += get_turning_cost(c->parent, adj) +
                         get_climb_cost(c->parent, adj) +
                         get_space_cost(c->parent, adj) +
                         get_conflict_cost(c->parent,adj);                
            }
            if (adj->g-g_new>1.0e-5) {
                // if(adj->g==INF or protect_overlap==true){
                    adj->parent = c->parent;
                    adj->g = g_new;
                //     protect_overlap=false;
                // }
            }
        }
    }
    AStar_conops1::compute_cost(c, adj);
}


}