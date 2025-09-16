#include "astar_conops2.h"
#include <glog/logging.h>
namespace conop2{

AStar_conops2::AStar_conops2(const array_t<int32_t> & cells_type, const array_t<double> & cells_cost, const array_t<double> & xs, const array_t<double> & ys, 
                const array_t<double> & zs,double  grid_res,  double turning_cost_angle, double turning_cost_consecutive, double climb_cost_ascend, double climb_cost_descend, 
                double climb_threshold, double space_ccoef, int space_length, bool nonuniform, bool operational):
    turning_cost_angle(turning_cost_angle), turning_cost_consecutive(turning_cost_consecutive), climb_cost_ascend(climb_cost_ascend), climb_cost_descend(climb_cost_descend), 
    climb_threshold(climb_threshold), space_cost_coef(space_ccoef), space_length(space_length), grid_res(grid_res), nonuniform(nonuniform), operational(operational)
{
    // Calculate the grid size
    for(int i = 0; i < 3; i++) {
        this->grid_size[i] = cells_type.shape(i);
    }

    // Compute strides for 1D indexing of 3D grid.
    stride_all = grid_size[0] * grid_size[1] * grid_size[2];
    stride_i = grid_size[1] * grid_size[2];
    stride_j = grid_size[2];
    
    //Initialize the grid cells with the provided data
    init_cells(xs.data(), ys.data(), zs.data(), cells_type.data(), cells_cost.data());
}

void AStar_conops2::init_cells(const double *xs, const double *ys, const double *zs, const int32_t *cells_type, const double *cells_cost) {
    // Resize the cell container
    cells.resize(stride_all);
    
    // Iterate to initialize cells.
    for (int i = 0; i < grid_size[0]; i++) {
        for (int j = 0; j < grid_size[1]; j++) {
            for (int k = 0; k < grid_size[2]; k++) {
                // Get current cell
                Cell *c = get_cell(i,j,k);
                
                // Set the grid indices and physical coordinates for the cell
                c->i = i; c->j = j; c->k = k;
                c->x = xs[i]; c->y = ys[j]; c->z = zs[k];
                
                // Assign the cost coefficient
                c->cost_coeff = cells_cost[index(i, j, k)];

                // Update the minimum traversal cost, for thetastar use
                min_traversal_cost = std::min(c->cost_coeff, min_traversal_cost);
                
                // Determine the cell type and set its properties
                int type = cells_type[index(i, j, k)];
                if (type < 0) {
                    c->reachable = false;   // The cell cannot be passed through.
                    c->access = false;      // The cell is an obstacle.
                }else if (type > 0) {
                    c->reserved = true;     // Reserve cells near vertiports
                }
            }
        }
    }
}

void AStar_conops2::reset() {
    
    for (auto & c : cells) {
        c.h = 0;
        c.g = INF;
        c.parent = nullptr;
        c.closed = false;
    }
    open.clear();
    current = nullptr;
}

int AStar_conops2::conflict_size(){
    int conflict_size=0;
    for (auto &c : cells){
        if(c.occupied_routes.size()>1){                         // cell is used by multiple routes
            int times_path=0;
            int times_buffer=0;
            for(route_occupation occupied:c.occupied_routes){
                if(occupied.protected_==false){                 // # of routes use the cell as path instead of buffer
                    times_path++;
                }else{
                    times_buffer++;
                }
            }
            if(times_path>0){
                conflict_size += times_path*3 + times_buffer;   // path-path conflicts * 3 + path-buffer conflicts
            }
        }
    }
    return conflict_size;
}

std::vector<Conflict> AStar_conops2::conflict_cell_route_calculation(){
    std::vector<Conflict> conflicts;
    conflicts.clear();
    for (auto &c : cells){
        //belong to 2 or more routes
        if(c.occupied_routes.size()>1){
            for(route_occupation occupied:c.occupied_routes){
                if(occupied.protected_==false){             //unless all routes use it as buffer, otherwise it has conflicts
                    conflicts.push_back({c.i,c.j,c.k,0,c.occupied_routes});
                    break;
                }
            }
        }
    }
    return conflicts;
}

std::vector<int> AStar_conops2::cal_conflicted_routes(Route p,int id){
    std::vector<int> route_ids;                                                                             // Store routes that have conflicts with route p
    route_ids.clear();
    std::vector<std::vector<int32_t>> protection_zone_cells;
    protection_zone_cells.clear();
    for(size_t j=0;j<p.index.size()-1;j++){
        route_protection results = line_traversal(p.index.at(j), p.index.at(j+1),
                                    space_length, grid_size[0],grid_size[1],grid_size[2], grid_res);                  // Get all grid cells used by the route
        protection_zone_cells.insert(protection_zone_cells.end(), results.protection_zone_cell.begin(), 
                                    results.protection_zone_cell.end());
        for(auto voxel : results.route_cells){                                                      
            bool reserved=cells[index(voxel.at(0), voxel.at(1), voxel.at(2))].reserved;                     // Currently not consider vertiport terminal airspace
            if(!reserved){
                set_occupied_id(voxel.at(0), voxel.at(1), voxel.at(2),id,true);                             // Add current route as using path
                std::vector<route_occupation>& occupied_routes=cells[index(voxel.at(0),
                                    voxel.at(1), voxel.at(2))].occupied_routes;
                for(auto occupied_route:occupied_routes){                                                   // Current route use cell as path,
                                                                                                            // any route occupation will lead to conflicts
                    if(occupied_route.route_id!=id)route_ids.push_back(occupied_route.route_id);
                }
            }
        }
    }
    for(auto voxel : protection_zone_cells){                                                                // For all buffer zones
        std::vector<route_occupation>& occupied_routes=cells[index(voxel.at(0), voxel.at(1),
         voxel.at(2))].occupied_routes;
        if(!occupied_routes.empty()){
            bool reserved=cells[index(voxel.at(0), voxel.at(1), voxel.at(2))].reserved;
            if(!reserved){
                set_occupied_id(voxel.at(0), voxel.at(1), voxel.at(2),id,false);                            // Add current route as using buffer
                for(auto occupied:occupied_routes){                                                         
                    if(occupied.route_id!=id and occupied.protected_==false){                               // If others use cell as path, then conflict exists
                        route_ids.push_back(occupied.route_id);
                    }
                }
            }
        }
    }
    if(route_ids.empty())return route_ids;
    
    // Delete repeated elements in route_ids
    std::vector<int>::iterator vector_iterator;
    std::sort(route_ids.begin(),route_ids.end());
    vector_iterator = std::unique(route_ids.begin(),route_ids.end());
    if(vector_iterator != route_ids.end()){
        route_ids.erase(vector_iterator,route_ids.end());
    }
    return route_ids;
}

double AStar_conops2::get_traversal_cost(const Cell *from, const Cell *to) {
    return distance3d(from, to) * (from->cost_coeff + to->cost_coeff) * 0.5;
}

double AStar_conops2::get_turning_cost(const Cell *from, const Cell *to) {
    if (from->parent && turning_cost_angle > 0.0) {
        return turning_cost_angle * std::abs(turning_angle(from->parent, from, to))
         * (1 + turning_cost_consecutive * std::exp(- distance3d(from->parent, from)));
    }
    return 0.0;
}

double AStar_conops2::get_climb_cost(const Cell *from, const Cell *to) {
    if (climb_cost_ascend > 0.0 || climb_cost_descend > 0.0) {
        double len_l1 = 0.0;
        double ang = climb_angle(from, to, &len_l1);

        if (std::abs(ang) <= climb_threshold) {
            if (ang > 0) {
                return climb_cost_ascend * std::abs(ang) * len_l1;
            } else {
                return climb_cost_descend * std::abs(ang) * len_l1;
            }
        } else {
            return INF;
        }
    }
    return 0.0;
}

double AStar_conops2::get_space_cost(const Cell *from, const Cell *to) {
    if (space_cost_coef <= 0.0) { return 0.0; }

    int count = 0;
    int count_proctected=0;

    int dx = to->i - from->i;
    int dy = to->j - from->j;
    int dz = to->k - from->k;

    int stepX = dx >= 0 ? 1 : -1;
    int stepY = dy >= 0 ? 1 : -1;
    int stepZ = dz >= 0 ? 1 : -1;

    double tDeltaX = dx == 0 ? INF : stepX / double(dx);
    double tDeltaY = dy == 0 ? INF : stepY / double(dy);
    double tDeltaZ = dz == 0 ? INF : stepZ / double(dz);

    double tMaxX = dx == 0 ? INF : 0.5 * stepX / double(dx);
    double tMaxY = dy == 0 ? INF : 0.5 * stepY / double(dy);
    double tMaxZ = dz == 0 ? INF : 0.5 * stepZ / double(dz);

    int c0[3] = {from->i, from->j, from->k};
    
    // For segment from 'from' to 'to',
    // 'from' is included only if it is origin point to avoid repeatation
    if (from->i==start->i and  from->j==start->j and from->k==start->k){
        for (int i = std::max(0, c0[0] - space_length); i < std::min(c0[0] + space_length + 1, grid_size[0]); i++) {
            for (int j = std::max(0, c0[1] - space_length); j < std::min(c0[1] + space_length + 1, grid_size[1]); j++) {
                for (int k = std::max(0, c0[2] - space_length); k < std::min(c0[2] + space_length + 1, grid_size[2]); k++) {
                    if (!is_protected(i, j, k) and !get_cell(i, j, k)->reserved  ) { count += 1;}count_proctected++;
                }
            }
        }
    }

    // Calculate conflicts in line segment
   while (c0[0] != to->i || c0[1] != to->j || c0[2] != to->k) {
        if (tMaxX < tMaxY) {
            if (tMaxX < tMaxZ) {                                                                // move path along x axis
                c0[0] += stepX; tMaxX += tDeltaX;
                int i=-1;
                if(stepX>0 and (c0[0]+space_length)<grid_size[0]) i=c0[0]+space_length;         // along +x axis
                if(stepX<0 and (c0[0]-space_length)>=0) i=c0[0]-space_length;                   // along -x axis
                if(i!=-1){                                                                      // update count
                    for (int j = std::max(0, c0[1] - space_length); j < std::min(c0[1] + space_length + 1, grid_size[1]); j++) {
                        for (int k = std::max(0, c0[2] - space_length); k < std::min(c0[2] + space_length + 1, grid_size[2]); k++) {
                            if (!is_protected(i, j, k) and !get_cell(i, j, k)->reserved  ) { count += 1;}count_proctected++;
                        }
                    }
                }
            }else if (tMaxX > tMaxZ) {                                                          // move path along z axis
                c0[2] += stepZ; tMaxZ += tDeltaZ;
                int k=-1;
                if(stepZ>0 and (c0[2]+space_length)<grid_size[2]) k=c0[2]+space_length;         // along +z axis
                if(stepZ<0 and (c0[2]-space_length)>=0) k=c0[2]-space_length;                   // along -z axis
                if(k!=-1){                                                                      // update count
                    for (int i = std::max(0, c0[0] - space_length); i < std::min(c0[0] + space_length + 1, grid_size[0]); i++) {
                        for (int j = std::max(0, c0[1] - space_length); j < std::min(c0[1] + space_length + 1, grid_size[1]); j++) {
                            if (!is_protected(i, j, k) and !get_cell(i, j, k)->reserved  ) { count += 1;}count_proctected++;
                        }
                    }
                }
            }else {                                                                             // move path along x/z axis
                c0[0] += stepX; tMaxX += tDeltaX;
                c0[2] += stepZ; tMaxZ += tDeltaZ;
                int imin=-1,imax=-1,kmin=-1,kmax=-1;
                // First include all cells, then remove repeated cells, to count correctly
                for (int i = std::max(0, c0[0] - space_length); i < std::min(c0[0] + space_length + 1, grid_size[0]); i++) {
                    for (int j = std::max(0, c0[1] - space_length); j < std::min(c0[1] + space_length + 1, grid_size[1]); j++) {
                        for (int k = std::max(0, c0[2] - space_length); k < std::min(c0[2] + space_length + 1, grid_size[2]); k++) {
                            if (!is_protected(i, j, k) and !get_cell(i, j, k)->reserved  ) { count += 1;}count_proctected++;
                        }
                    }
                }
                if(stepZ>0){kmin=c0[2]-stepZ;kmax=c0[2];}                                       // four directions
                else {kmin=c0[2];kmax=c0[2]-stepZ;}
                if(stepX>0){imin=c0[0]-stepX;imax=c0[0];}
                else {imin=c0[0];imax=c0[0]-stepX;}
                for (int i = imin; i <= imax; i++) {
                    for (int j = std::max(0, c0[1] - space_length); j < std::min(c0[1] + space_length + 1, grid_size[1]); j++) {
                        for (int k = kmin; k <= kmax; k++) {
                            if (!is_protected(i, j, k)  and !get_cell(i, j, k)->reserved  ) { count -= 1;}count_proctected--;
                        }
                    }
                }
            }
        } else if (tMaxX > tMaxY) {
            if (tMaxY < tMaxZ) {                                                                // move path along y axis
                c0[1] += stepY; tMaxY += tDeltaY;
                int j=-1;
                if(stepY>0 and (c0[1]+space_length)<grid_size[1]) j=c0[1]+space_length;        // along +y axis
                if(stepY<0 and (c0[1]-space_length)>=0) j=c0[1]-space_length;                   // along -y axis
                if(j!=-1){                                                                      // update count
                    for (int i = std::max(0, c0[0] - space_length); i < std::min(c0[0] + space_length + 1, grid_size[0]); i++) {
                        for (int k = std::max(0, c0[2] - space_length); k < std::min(c0[2] + space_length + 1, grid_size[2]); k++) {
                            if (!is_protected(i, j, k) and !get_cell(i, j, k)->reserved  ) { count += 1;}count_proctected++;
                        }
                    }
                }
            } else if (tMaxY > tMaxZ) {                                                         // move path along z axis
                c0[2] += stepZ; tMaxZ += tDeltaZ;
                int k=-1;
                if(stepZ>0 and (c0[2]+space_length)<grid_size[2]) k=c0[2]+space_length;         // along +z axis
                if(stepZ<0 and (c0[2]-space_length)>=0) k=c0[2]-space_length;                   // along -z axis
                if(k!=-1){                                                                      // update count
                    for (int i = std::max(0, c0[0] - space_length); i < std::min(c0[0] + space_length + 1, grid_size[0]); i++) {
                        for (int j = std::max(0, c0[1] - space_length); j < std::min(c0[1] + space_length + 1, grid_size[1]); j++) {
                            if (!is_protected(i, j, k) and !get_cell(i, j, k)->reserved  ) { count += 1;}count_proctected++;
                        }
                    }
                }
            } else {                                                                            // move path along y/z axis
                c0[1] += stepY; tMaxY += tDeltaY;
                c0[2] += stepZ; tMaxZ += tDeltaZ;
                int jmin=-1,jmax=-1,kmin=-1,kmax=-1;
                // First include all cells, then remove repeated cells, to count correctly
                for (int i = std::max(0, c0[0] - space_length); i < std::min(c0[0] + space_length + 1, grid_size[0]); i++) {
                    for (int j = std::max(0, c0[1] - space_length); j < std::min(c0[1] + space_length + 1, grid_size[1]); j++) {
                        for (int k = std::max(0, c0[2] - space_length); k < std::min(c0[2] + space_length + 1, grid_size[2]); k++) {
                            if (!is_protected(i, j, k) and !get_cell(i, j, k)->reserved  ) { count += 1;}count_proctected++;
                        }
                    }
                }
                if(stepZ>0){kmin=c0[2]-stepZ;kmax=c0[2];}
                else {kmin=c0[2];kmax=c0[2]-stepZ;}
                if(stepY>0){jmin=c0[1]-stepY;jmax=c0[1];}
                else {jmin=c0[1];jmax=c0[1]-stepY;}
                for (int i = std::max(0, c0[0] - space_length); i < std::min(c0[0] + space_length + 1, grid_size[0]); i++) {
                    for (int j = jmin; j <= jmax; j++) {
                        for (int k = kmin; k <= kmax; k++) {
                            if (!is_protected(i, j, k) and !get_cell(i, j, k)->reserved  ) { count -= 1;}count_proctected--;
                        }
                    }
                }
            }
        } else {
            if (tMaxX < tMaxZ) {                                                                // move path along x/y axis
                c0[0] += stepX; tMaxX += tDeltaX;
                c0[1] += stepY; tMaxY += tDeltaY;
                int jmin=-1,jmax=-1,imin=-1,imax=-1;
                // First include all cells, then remove repeated cells, to count correctly
                for (int i = std::max(0, c0[0] - space_length); i < std::min(c0[0] + space_length + 1, grid_size[0]); i++) {
                    for (int j = std::max(0, c0[1] - space_length); j < std::min(c0[1] + space_length + 1, grid_size[1]); j++) {
                        for (int k = std::max(0, c0[2] - space_length); k < std::min(c0[2] + space_length + 1, grid_size[2]); k++) {
                            if (!is_protected(i, j, k) and !get_cell(i, j, k)->reserved  ) { count += 1;}count_proctected++;
                        }
                    }
                }
                if(stepX>0){imin=c0[0]-stepX;imax=c0[0];}
                else {imin=c0[0];imax=c0[0]-stepX;}
                if(stepY>0){jmin=c0[1]-stepY;jmax=c0[1];}
                else {jmin=c0[1];jmax=c0[1]-stepY;}
                for (int i = imin; i <= imax; i++) {
                    for (int j = jmin; j <=jmax; j++) {
                        for (int k = std::max(0, c0[2] - space_length); k < std::min(c0[2] + space_length + 1, grid_size[2]); k++) {
                            if (!is_protected(i, j, k) and !get_cell(i, j, k)->reserved  ) { count -= 1;}count_proctected--;
                        }
                    }
                }
            } else if (tMaxX > tMaxZ) {                                                         // move path along z axis
                c0[2] += stepZ; tMaxZ += tDeltaZ;
                int k=-1;
                if(stepZ>0 and (c0[2]+space_length)<grid_size[2]) k=c0[2]+space_length;         // along +z axis
                if(stepZ<0 and (c0[2]-space_length)>=0) k=c0[2]-space_length;                   // along -z axis
                if(k!=-1){                                                                      // update count
                    for (int i = std::max(0, c0[0] - space_length); i < std::min(c0[0] + space_length + 1, grid_size[0]); i++) {
                        for (int j = std::max(0, c0[1] - space_length); j < std::min(c0[1] + space_length + 1, grid_size[1]); j++) {
                            if (!is_protected(i, j, k) and !get_cell(i, j, k)->reserved  ) { count += 1;}count_proctected++;
                        }
                    }
                }
            } else {                                                                            // move path along x/y/z axis
                c0[0] += stepX; tMaxX += tDeltaX;
                c0[1] += stepY; tMaxY += tDeltaY;
                c0[2] += stepZ; tMaxZ += tDeltaZ;
                int imin=-1,imax=-1,jmin=-1,jmax=-1,kmin=-1,kmax=-1;
                // First include all cells, then remove repeated cells, to count correctly
                for (int i = std::max(0, c0[0] - space_length); i < std::min(c0[0] + space_length + 1, grid_size[0]); i++) {
                    for (int j = std::max(0, c0[1] - space_length); j < std::min(c0[1] + space_length + 1, grid_size[1]); j++) {
                        for (int k = std::max(0, c0[2] - space_length); k < std::min(c0[2] + space_length + 1, grid_size[2]); k++) {
                            if (!is_protected(i, j, k) and !get_cell(i, j, k)->reserved  ) { count += 1;}count_proctected++;
                        }
                    }
                }
                if(stepZ>0){kmin=c0[2]-stepZ;kmax=c0[2];}
                else {kmin=c0[2];kmax=c0[2]-stepZ;}
                if(stepY>0){jmin=c0[1]-stepY;jmax=c0[1];}
                else {jmin=c0[1];jmax=c0[1]-stepY;}
                if(stepX>0){imin=c0[0]-stepX;imax=c0[0];}
                else {imin=c0[0];imax=c0[0]-stepX;}
                for (int i = imin; i <= imax; i++) {
                    for (int j = jmin; j <= jmax; j++) {
                        for (int k = kmin; k <= kmax; k++) {
                            if (!is_protected(i, j, k) and !get_cell(i, j, k)->reserved  ) { count -= 1;}count_proctected--;
                        }
                    }
                }
            }
        }
    }
    if (count-count_proctected!=0)protect_overlap=true;
    return space_cost_coef * count;
}

double AStar_conops2::get_conflict_cost(const Cell *from, const Cell *to) {
    if (conflict_cost_coef <= 0.0) { return 0.0; }

    double count = 0.0;

    int dx = to->i - from->i;
    int dy = to->j - from->j;
    int dz = to->k - from->k;

    int stepX = dx >= 0 ? 1 : -1;
    int stepY = dy >= 0 ? 1 : -1;
    int stepZ = dz >= 0 ? 1 : -1;

    double tDeltaX = dx == 0 ? INF : stepX / double(dx);
    double tDeltaY = dy == 0 ? INF : stepY / double(dy);
    double tDeltaZ = dz == 0 ? INF : stepZ / double(dz);

    double tMaxX = dx == 0 ? INF : 0.5 * stepX / double(dx);
    double tMaxY = dy == 0 ? INF : 0.5 * stepY / double(dy);
    double tMaxZ = dz == 0 ? INF : 0.5 * stepZ / double(dz);

    int c0[3] = {from->i, from->j, from->k};
    int protection_length = std::ceil(5.0 / grid_res);
    // no buffer zone
    if(space_length==0){
        // similarly, if it is origin point then include it in conflict cost, else omit it
        if (from->i==start->i and  from->j==start->j and from->k==start->k){
            if(!get_cell(c0[0], c0[1], c0[2])->reserved){
                if(get_cell(c0[0], c0[1], c0[2])->occupied_routes.empty()){         // no route uses the cell
                }else{                                                              // multiple routes uses the cell
                    for(auto & occupied : get_cell(c0[0], c0[1], c0[2])->occupied_routes){
                        if(occupied.protected_==false){
                            count+=1;
                        }
                    }
                }
            }
        }
        //walk along the path to calculate the cost
        while (c0[0] != to->i || c0[1] != to->j || c0[2] != to->k) {
            if (tMaxX < tMaxY) {
                if (tMaxX < tMaxZ) {                                                // walk along x axis
                    c0[0] += stepX; tMaxX += tDeltaX;
                }else if (tMaxX > tMaxZ) {                                          // walk along z axis
                c0[2] += stepZ; tMaxZ += tDeltaZ;
                }else {                                                             // walk along x/z axis
                c0[0] += stepX; tMaxX += tDeltaX;
                c0[2] += stepZ; tMaxZ += tDeltaZ;
                }
            } else if (tMaxX > tMaxY) {
                if (tMaxY < tMaxZ) {                                                // walk along y axis
                    c0[1] += stepY; tMaxY += tDeltaY;
                }else if (tMaxY > tMaxZ) {                                          // walk along z axis
                    c0[2] += stepZ; tMaxZ += tDeltaZ;
                }else {                                                             // walk along y/z axis
                    c0[1] += stepY; tMaxY += tDeltaY;
                    c0[2] += stepZ; tMaxZ += tDeltaZ;
                }
            } else {
                if (tMaxX < tMaxZ) {                                                // walk along x/y axis
                    c0[0] += stepX; tMaxX += tDeltaX;
                    c0[1] += stepY; tMaxY += tDeltaY;
                } else if (tMaxX > tMaxZ) {                                         // walk along z axis
                    c0[2] += stepZ; tMaxZ += tDeltaZ;
                }else {                                                             // walk along x/y/z axis
                    c0[0] += stepX; tMaxX += tDeltaX;
                    c0[1] += stepY; tMaxY += tDeltaY;
                    c0[2] += stepZ; tMaxZ += tDeltaZ;
                }
            }
            for(auto & occupied : get_cell(c0[0], c0[1], c0[2])->occupied_routes){
                if(occupied.protected_==false){
                    count+=1;
                }
            }
        }
        return conflict_cost_coef * count;
    }

    // if space length is positive, then we should consider buffer zones
    // if it is origin point then include it in conflict cost, else omit it
    if (from->i==start->i and  from->j==start->j and from->k==start->k){
        for (int i = std::max(0, c0[0] - space_length); i < std::min(c0[0] + space_length + 1, grid_size[0]); i++) {
            for (int j = std::max(0, c0[1] - space_length); j < std::min(c0[1] + space_length + 1, grid_size[1]); j++) {
                for (int k = std::max(0, c0[2] - space_length); k < std::min(c0[2] + space_length + 1, grid_size[2]); k++) {
                    if(!get_cell(i, j, k)->reserved){
                        if(get_cell(i, j, k)->occupied_routes.empty()){                             // exactly no route
                        }else{                                                                      // conflicts appear
                            if(i==c0[0] and j==c0[1] and k==c0[2]){                                 // cell is used by current route as path, both path and protected zone lead to conflicts
                                for(auto & occupied : get_cell(i, j, k)->occupied_routes){
                                    if(occupied.protected_==false){                                 // route
                                        count+=3;
                                    }else count+=1;                                                 // buffer
                                }
                            }else{                                                                  // cell is used by current route as buffer, only path lead to conflicts
                                for(auto & occupied : get_cell(i, j, k)->occupied_routes){
                                    if(occupied.protected_==false){
                                        count+=1;
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
    }
    
    //walk along the path to calculate the cost, include path-path conflicts and path-buffer conflicts
    while (c0[0] != to->i || c0[1] != to->j || c0[2] != to->k) {
        // First traverse all occupied airspace (path and buffers), only calculate it as buffer
        if (tMaxX < tMaxY) {
            if (tMaxX < tMaxZ) {                                                                                //walk along x axis
                c0[0] += stepX; tMaxX += tDeltaX;
                int i=-1;
                if(stepX>0 and (c0[0]+space_length)<grid_size[0]) i=c0[0]+space_length;                         //along +x axis
                if(stepX<0 and (c0[0]-space_length)>=0) i=c0[0]-space_length;                                   //along -x axis
                if(i!=-1){                                                                                     
                    for (int j = std::max(0, c0[1] - space_length); j < std::min(c0[1] + space_length + 1, grid_size[1]); j++) {
                        for (int k = std::max(0, c0[2] - space_length); k < std::min(c0[2] + space_length + 1, grid_size[2]); k++) {
                            if(!get_cell(i, j, k)->reserved){
                                if(get_cell(i, j, k)->occupied_routes.empty()){                                 
                                }else{                                                                          //only path introduces conflicts, path-buffer conflicts
                                    for(auto & occupied : get_cell(i, j, k)->occupied_routes){
                                        if(occupied.protected_==false){                                         
                                            count+=1;
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }else if (tMaxX > tMaxZ) {                                                                          //walk along z axis
                c0[2] += stepZ; tMaxZ += tDeltaZ;
                int k=-1;
                if(stepZ>0 and (c0[2]+space_length)<grid_size[2]) k=c0[2]+space_length;                         //along +z axis
                if(stepZ<0 and (c0[2]-space_length)>=0) k=c0[2]-space_length;                                   //along -z axis
                if(k!=-1){
                    for (int i = std::max(0, c0[0] - space_length); i < std::min(c0[0] + space_length + 1, grid_size[0]); i++) {
                        for (int j = std::max(0, c0[1] - space_length); j < std::min(c0[1] + space_length + 1, grid_size[1]); j++) {
                            if(!get_cell(i, j, k)->reserved){
                                if(get_cell(i, j, k)->occupied_routes.empty()){
                                }else{
                                    for(auto & occupied : get_cell(i, j, k)->occupied_routes){
                                        if(occupied.protected_==false){
                                            count+=1;                                                           
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }else {                                                                                             //walk along x/z axis
                c0[0] += stepX; tMaxX += tDeltaX;
                c0[2] += stepZ; tMaxZ += tDeltaZ;
                int imin=-1,imax=-1,kmin=-1,kmax=-1;

                //first include all cells, then delete repeated cells
                for (int i = std::max(0, c0[0] - space_length); i < std::min(c0[0] + space_length + 1, grid_size[0]); i++) {
                    for (int j = std::max(0, c0[1] - space_length); j < std::min(c0[1] + space_length + 1, grid_size[1]); j++) {
                        for (int k = std::max(0, c0[2] - space_length); k < std::min(c0[2] + space_length + 1, grid_size[2]); k++) {
                            if(!get_cell(i, j, k)->reserved){
                                if(get_cell(i, j, k)->occupied_routes.empty()){/*exactly no route*/
                                }else{/*conflicts appear*/
                                    /*protected zone, only route lead to conflicts*/
                                    for(auto & occupied : get_cell(i, j, k)->occupied_routes){
                                        if(occupied.protected_==false){/*route*/
                                            count+=1;//
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
                if(stepZ>0){kmin=c0[2]-stepZ;kmax=c0[2];}
                else {kmin=c0[2];kmax=c0[2]-stepZ;}
                if(stepX>0){imin=c0[0]-stepX;imax=c0[0];}
                else {imin=c0[0];imax=c0[0]-stepX;}
                for (int i = imin; i <= imax; i++) {
                    for (int j = std::max(0, c0[1] - space_length); j < std::min(c0[1] + space_length + 1, grid_size[1]); j++) {
                        for (int k = kmin; k <= kmax; k++) {
                            if(!get_cell(i, j, k)->reserved){
                                if(get_cell(i, j, k)->occupied_routes.empty()){/*exactly no route*/
                                }else{/*conflicts appear*/
                                    /*protected zone, only route lead to conflicts*/
                                    for(auto & occupied : get_cell(i, j, k)->occupied_routes){
                                        if(occupied.protected_==false){/*route*/
                                            count-=1;//
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }
        } else if (tMaxX > tMaxY) {
            if (tMaxY < tMaxZ) {                                                                                //walk along y axis
                c0[1] += stepY; tMaxY += tDeltaY;
                int j=-1;
                if(stepY>0 and (c0[1]+space_length)<grid_size[1]) j=c0[1]+space_length;                         //along +y axis
                if(stepY<0 and (c0[1]-space_length)>=0) j=c0[1]-space_length;                                   //along -y axis
                if(j!=-1){
                    for (int i = std::max(0, c0[0] - space_length); i < std::min(c0[0] + space_length + 1, grid_size[0]); i++) {
                        for (int k = std::max(0, c0[2] - space_length); k < std::min(c0[2] + space_length + 1, grid_size[2]); k++) {
                            if(!get_cell(i, j, k)->reserved){
                                if(get_cell(i, j, k)->occupied_routes.empty()){
                                }else{
                                    /*protected zone, only route lead to conflicts*/
                                    for(auto & occupied : get_cell(i, j, k)->occupied_routes){
                                        if(occupied.protected_==false){
                                            count+=1;
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            } else if (tMaxY > tMaxZ) {                                                                         //walk along z axis
                c0[2] += stepZ; tMaxZ += tDeltaZ;
                int k=-1;
                if(stepZ>0 and (c0[2]+space_length)<grid_size[2]) k=c0[2]+space_length;
                if(stepZ<0 and (c0[2]-space_length)>=0) k=c0[2]-space_length;
                if(k!=-1){/*need update*/
                    for (int i = std::max(0, c0[0] - space_length); i < std::min(c0[0] + space_length + 1, grid_size[0]); i++) {
                        for (int j = std::max(0, c0[1] - space_length); j < std::min(c0[1] + space_length + 1, grid_size[1]); j++) {
                            if(!get_cell(i, j, k)->reserved){
                                if(get_cell(i, j, k)->occupied_routes.empty()){
                                }else{
                                    for(auto & occupied : get_cell(i, j, k)->occupied_routes){
                                        if(occupied.protected_==false){
                                            count+=1;
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            } else {                                                                                            //*walk along y/z axis
                c0[1] += stepY; tMaxY += tDeltaY;
                c0[2] += stepZ; tMaxZ += tDeltaZ;
                int jmin=-1,jmax=-1,kmin=-1,kmax=-1;
                /*first include all cells*/
                for (int i = std::max(0, c0[0] - space_length); i < std::min(c0[0] + space_length + 1, grid_size[0]); i++) {
                    for (int j = std::max(0, c0[1] - space_length); j < std::min(c0[1] + space_length + 1, grid_size[1]); j++) {
                        for (int k = std::max(0, c0[2] - space_length); k < std::min(c0[2] + space_length + 1, grid_size[2]); k++) {
                            if(!get_cell(i, j, k)->reserved){
                                if(get_cell(i, j, k)->occupied_routes.empty()){/*exactly no route*/
                                }else{/*conflicts appear*/
                                    /*protected zone, only route lead to conflicts*/
                                    for(auto & occupied : get_cell(i, j, k)->occupied_routes){
                                        if(occupied.protected_==false){/*route*/
                                            count+=1;//
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
                /*then get rid of repeated cells*/
                if(stepZ>0){kmin=c0[2]-stepZ;kmax=c0[2];}
                else {kmin=c0[2];kmax=c0[2]-stepZ;}
                if(stepY>0){jmin=c0[1]-stepY;jmax=c0[1];}
                else {jmin=c0[1];jmax=c0[1]-stepY;}
                for (int i = std::max(0, c0[0] - space_length); i < std::min(c0[0] + space_length + 1, grid_size[0]); i++) {
                    for (int j = jmin; j <= jmax; j++) {
                        for (int k = kmin; k <= kmax; k++) {
                            if(!get_cell(i, j, k)->reserved){
                                if(get_cell(i, j, k)->occupied_routes.empty()){/*exactly no route*/
                                }else{/*conflicts appear*/
                                    /*protected zone, only route lead to conflicts*/
                                    for(auto & occupied : get_cell(i, j, k)->occupied_routes){
                                        if(occupied.protected_==false){/*route*/
                                            count-=1;//
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }
        } else {
        if (tMaxX < tMaxZ) {                                                                        //walk along x/y axis
                c0[0] += stepX; tMaxX += tDeltaX;
                c0[1] += stepY; tMaxY += tDeltaY;
                int jmin=-1,jmax=-1,imin=-1,imax=-1;

                //first include all cells, then delete repeated cells
                for (int i = std::max(0, c0[0] - space_length); i < std::min(c0[0] + space_length + 1, grid_size[0]); i++) {
                    for (int j = std::max(0, c0[1] - space_length); j < std::min(c0[1] + space_length + 1, grid_size[1]); j++) {
                        for (int k = std::max(0, c0[2] - space_length); k < std::min(c0[2] + space_length + 1, grid_size[2]); k++) {
                            if(!get_cell(i, j, k)->reserved){
                                if(get_cell(i, j, k)->occupied_routes.empty()){
                                }else{
                                    for(auto & occupied : get_cell(i, j, k)->occupied_routes){
                                        if(occupied.protected_==false){
                                            count+=1;
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
                if(stepX>0){imin=c0[0]-stepX;imax=c0[0];}
                else {imin=c0[0];imax=c0[0]-stepX;}
                if(stepY>0){jmin=c0[1]-stepY;jmax=c0[1];}
                else {jmin=c0[1];jmax=c0[1]-stepY;}
                for (int i = imin; i <= imax; i++) {
                    for (int j = jmin; j <=jmax; j++) {
                        for (int k = std::max(0, c0[2] - space_length); k < std::min(c0[2] + space_length + 1, grid_size[2]); k++) {
                            if(!get_cell(i, j, k)->reserved){
                                if(get_cell(i, j, k)->occupied_routes.empty()){
                                }else{
                                    for(auto & occupied : get_cell(i, j, k)->occupied_routes){
                                        if(occupied.protected_==false){
                                            count-=1;
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            } else if (tMaxX > tMaxZ) {                                                             //walk along z axis
                c0[2] += stepZ; tMaxZ += tDeltaZ;
                int k=-1;
                if(stepZ>0 and (c0[2]+space_length)<grid_size[2]) k=c0[2]+space_length;          
                if(stepZ<0 and (c0[2]-space_length)>=0) k=c0[2]-space_length;
                if(k!=-1){
                    for (int i = std::max(0, c0[0] - space_length); i < std::min(c0[0] + space_length + 1, grid_size[0]); i++) {
                        for (int j = std::max(0, c0[1] - space_length); j < std::min(c0[1] + space_length + 1, grid_size[1]); j++) {
                            if(!get_cell(i, j, k)->reserved){
                                if(get_cell(i, j, k)->occupied_routes.empty()){
                                }else{
                                    for(auto & occupied : get_cell(i, j, k)->occupied_routes){
                                        if(occupied.protected_==false){
                                            count+=1;
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            } else {                                                                                //walk along x/y/z axis
                c0[0] += stepX; tMaxX += tDeltaX;
                c0[1] += stepY; tMaxY += tDeltaY;
                c0[2] += stepZ; tMaxZ += tDeltaZ;
                int imin=-1,imax=-1,jmin=-1,jmax=-1,kmin=-1,kmax=-1;
                
                //first include all cells, then delete repeated cells
                for (int i = std::max(0, c0[0] - space_length); i < std::min(c0[0] + space_length + 1, grid_size[0]); i++) {
                    for (int j = std::max(0, c0[1] - space_length); j < std::min(c0[1] + space_length + 1, grid_size[1]); j++) {
                        for (int k = std::max(0, c0[2] - space_length); k < std::min(c0[2] + space_length + 1, grid_size[2]); k++) {
                            if(!get_cell(i, j, k)->reserved){
                                if(get_cell(i, j, k)->occupied_routes.empty()){/*exactly no route*/
                                }else{/*conflicts appear*/
                                    /*protected zone, only route lead to conflicts*/
                                    for(auto & occupied : get_cell(i, j, k)->occupied_routes){
                                        if(occupied.protected_==false){/*route*/
                                            count+=1;//
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
                if(stepZ>0){kmin=c0[2]-stepZ;kmax=c0[2];}
                else {kmin=c0[2];kmax=c0[2]-stepZ;}
                if(stepY>0){jmin=c0[1]-stepY;jmax=c0[1];}
                else {jmin=c0[1];jmax=c0[1]-stepY;}
                if(stepX>0){imin=c0[0]-stepX;imax=c0[0];}
                else {imin=c0[0];imax=c0[0]-stepX;}
                for (int i = imin; i <= imax; i++) {
                    for (int j = jmin; j <= jmax; j++) {
                        for (int k = kmin; k <= kmax; k++) {
                            if(!get_cell(i, j, k)->reserved){
                                if(get_cell(i, j, k)->occupied_routes.empty()){
                                }else{
                                    for(auto & occupied : get_cell(i, j, k)->occupied_routes){
                                        if(occupied.protected_==false){
                                            count-=1;
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
        
        // Then traverse path cells, only calculate it as path
        for(auto & occupied : get_cell(c0[0], c0[1], c0[2])->occupied_routes){
            if(occupied.protected_==false){                                                         //path-path
                count+=3;
            }else count+=1;                                                                         //path-buffer
        }
    }
    return conflict_cost_coef * count;
}

std::vector<Cell *> AStar_conops2::get_adjacent_cells(const Cell *c) {
    std::vector<Cell *> r;
    // 1: For concept comparasion, only consider x/y/z neighbors
    for (int i = std::max(0, c->i - 1); i < std::min(grid_size[0], c->i + 2); i++) {
        Cell* adj = get_cell(i, c->j, c->k);
        if (adj != c && adj->reachable) {
            int ind_to = index(i, c->j, c->k);                          // check direction
            bool head_conflict = false;
            for(auto& ind_from:c->from_cell_indexes){
                if(ind_from == ind_to){head_conflict=true; break;}
            }
            if(head_conflict==false)r.push_back(adj);
        }
    }
    for (int j = std::max(0, c->j - 1); j < std::min(grid_size[1], c->j + 2); j++) {
        Cell* adj = get_cell(c->i, j, c->k);
        if (adj != c && adj->reachable) {
            int ind_to = index(c->i, j, c->k);
            bool head_conflict = false;
            for(auto& ind_from:c->from_cell_indexes){
                if(ind_from == ind_to){head_conflict=true; break;}
            }
            if(head_conflict==false)r.push_back(adj);
        }
    }
    for (int k = std::max(0, c->k - 1); k < std::min(grid_size[2], c->k + 2); k++) {
        Cell* adj = get_cell(c->i, c->j, k);
        if (adj != c && adj->reachable) {
            int ind_to = index(c->i, c->j, k);
            bool head_conflict = false;
            for(auto& ind_from:c->from_cell_indexes){
                if(ind_from == ind_to){head_conflict=true;break;}
            }
            if(head_conflict==false)r.push_back(adj);
        }
    }
    // 2: For real applications, consider cube surrounding
    // for (int i = std::max(0, c->i - 1); i < std::min(grid_size[0], c->i + 2); i++) {
    //     for (int j = std::max(0, c->j - 1); j < std::min(grid_size[1], c->j + 2); j++) {
    //         for (int k = std::max(0, c->k - 1); k < std::min(grid_size[2], c->k + 2); k++) {
    //             Cell* adj = get_cell(i, j, k);
    //             if (adj != c && adj->reachable) {
    //                 int ind_to = index(i, j, k);
    //                 bool head_conflict = false;
    //                 for(auto& ind_from:c->from_cell_indexes){
    //                     if(ind_from == ind_to){head_conflict=true;break;}
    //                 }
    //                 if(head_conflict==false)r.push_back(adj);
    //             }
    //         }
    //     }
    // }
    return r;
}

std::vector<Cell *> AStar_conops2::get_adjacent_cells_subspace(const Cell *c) {
    std::vector<Cell *> r;

    // 1: For concept comparasion, only consider x/y/z neighbors
    for (int i = std::max(0, c->i - 1); i < std::min(grid_size[0], c->i + 2); i++) {
        Cell* adj = get_cell(i, c->j, c->k);
        if (adj != c && adj->reachable && adj->reach_subspace) {
            int ind_to = index(i, c->j, c->k);                          // check direction
            bool head_conflict = false;
            for(auto& ind_from:c->from_cell_indexes){
                if(ind_from == ind_to){head_conflict=true; break;}
            }
            if(head_conflict==false)r.push_back(adj);
        }
    }
    for (int j = std::max(0, c->j - 1); j < std::min(grid_size[1], c->j + 2); j++) {
        Cell* adj = get_cell(c->i, j, c->k);
        if (adj != c && adj->reachable && adj->reach_subspace) {
            int ind_to = index(c->i, j, c->k);
            bool head_conflict = false;
            for(auto& ind_from:c->from_cell_indexes){
                if(ind_from == ind_to){head_conflict=true; break;}
            }
            if(head_conflict==false)r.push_back(adj);
        }
    }
    for (int k = std::max(0, c->k - 1); k < std::min(grid_size[2], c->k + 2); k++) {
        Cell* adj = get_cell(c->i, c->j, k);
        if (adj != c && adj->reachable && adj->reach_subspace) {
            int ind_to = index(c->i, c->j, k);
            bool head_conflict = false;
            for(auto& ind_from:c->from_cell_indexes){
                if(ind_from == ind_to){head_conflict=true;break;}
            }
            if(head_conflict==false)r.push_back(adj);
        }
    }
    // 2: For real applications, consider cube surrounding
    // for (int i = std::max(0, c->i - 1); i < std::min(grid_size[0], c->i + 2); i++) {
    //     for (int j = std::max(0, c->j - 1); j < std::min(grid_size[1], c->j + 2); j++) {
    //         for (int k = std::max(0, c->k - 1); k < std::min(grid_size[2], c->k + 2); k++) {
    //             Cell* adj = get_cell(i, j, k);
    //             if (adj != c && adj->reachable && adj->reach_subspace) {
    //                 int ind_to = index(i, j, k);
    //                 bool head_conflict = false;
    //                 for(auto& ind_from:c->from_cell_indexes){
    //                     if(ind_from == ind_to){head_conflict=true;break;}
    //                 }
    //                 if(head_conflict==false)r.push_back(adj);
    //             }
    //         }
    //     }
    // }
    return r;
}

void AStar_conops2::update_cell(Cell *c, Cell *adj) {
    double g_old = adj->g;
    compute_cost(c, adj);
    if (adj->g < g_old) {
        adj->h = get_heuristic(adj);
        open.push(adj);
    }
}

void AStar_conops2::compute_cost(Cell *c, Cell *adj) {
    double g_new = c->g;
    if (nonuniform) {                                   // Consider distance and risk
        g_new += get_traversal_cost(c, adj);
    } else {                                            // Consider distance only
        g_new += distance3d(c, adj);
    }
    if (operational) {                                  // Consider turning, climbing, space, and conflict costs
        g_new += get_turning_cost(c, adj) + get_climb_cost(c, adj) +
        get_space_cost(c, adj)+get_conflict_cost(c,adj);
    }
    if (adj->g-g_new>1.0e-5) {
        // if(adj->g==INF or protect_overlap==true){
            adj->parent = c;
            adj->g = g_new;
        //     protect_overlap=false;
        // }
    }
}

void AStar_conops2::solve(array_t<int64_t> i1, array_t<int64_t> i2, std::promise<Route> & p) {
    reset();                                                                // init cells for new search
    start = get_cell(i1.at(0), i1.at(1), i1.at(2));
    end = get_cell(i2.at(0), i2.at(1), i2.at(2));
    start->g = 0;
    open.push(start);
    uint64_t expanded = 0;
    while (true) {
        Cell *c = open.pop();
        if (c == nullptr) {                                                 // algorithm can not find route
            break;
        }
        current = c;
        if (c == end) {                                                     // successfully find the route
            LOG(INFO)<<"succeeded, expanded:" << expanded;
            p.set_value(get_route(false));
            return;
        }
        c->closed = true;
        
        // Iterate over cells
        auto cells = get_adjacent_cells(c);
        for (auto adj : cells) {
            if (!adj->closed) {
                update_cell(c, adj);
            }
        }
        expanded++;
    }
    LOG(INFO)<<"failed, expanded: " <<expanded;
    Route route=get_route(false);
    route.traversal=INF;
    p.set_value(route);
}

void AStar_conops2::solve_subspace(Agent agent, std::promise<Route> & p, std::promise<std::vector<int>> & 
 conflicts_along_route, std::promise<double> & gain, int prev_conflict, int neighbor_size, int method){
    conflict_cost_coef=agent.conflict_coefficient;                                                                      // get conflict cost coefficient
    backtrack_cell(agent.route,agent.id,*this,space_length,this->grid_size[0],this->grid_size[1], this->grid_size[2], grid_res);  // release the airspace occupied by the previous path of the route (agent)
    for (std::vector<std::vector<int>>::iterator it = agent.route.index.begin(); it != agent.route.index.end()-1; ++it){// flag the subspace as true to get search subspace
        int i1 = (*it).at(0);
        int j1 = (*it).at(1);
        int k1 = (*it).at(2);
        int i2 = (*(it+1)).at(0);
        int j2 = (*(it+1)).at(1);
        int k2 = (*(it+1)).at(2);
        int dx = i2 - i1;
        int dy = j2 - j1;
        int dz = k2 - k1;

        int stepX = dx >= 0 ? 1 : -1;
        int stepY = dy >= 0 ? 1 : -1;
        int stepZ = dz >= 0 ? 1 : -1;

        double tDeltaX = dx == 0 ? INF : stepX / double(dx);
        double tDeltaY = dy == 0 ? INF : stepY / double(dy);
        double tDeltaZ = dz == 0 ? INF : stepZ / double(dz);

        double tMaxX = dx == 0 ? INF : 0.5 * stepX / double(dx);
        double tMaxY = dy == 0 ? INF : 0.5 * stepY / double(dy);
        double tMaxZ = dz == 0 ? INF : 0.5 * stepZ / double(dz);
        int c0[3] = {i1, j1, k1};
        while (c0[0] != i2 || c0[1] != j2 || c0[2] != k2) {
            if (tMaxX < tMaxY) {
                if (tMaxX < tMaxZ) {
                    c0[0] += stepX; tMaxX += tDeltaX;
                } else if (tMaxX > tMaxZ) {
                    c0[2] += stepZ; tMaxZ += tDeltaZ;
                }
                else {
                    c0[0] += stepX; tMaxX += tDeltaX;
                    c0[2] += stepZ; tMaxZ += tDeltaZ;
                }
            } else if (tMaxX > tMaxY) {
                if (tMaxY < tMaxZ) {
                    c0[1] += stepY; tMaxY += tDeltaY;
                } else if (tMaxY > tMaxZ) {
                    c0[2] += stepZ; tMaxZ += tDeltaZ;
                }
                else {
                    c0[1] += stepY; tMaxY += tDeltaY;
                    c0[2] += stepZ; tMaxZ += tDeltaZ;
                }
            } else {
                if (tMaxX < tMaxZ) {
                    c0[0] += stepX; tMaxX += tDeltaX;
                    c0[1] += stepY; tMaxY += tDeltaY;
                } else if (tMaxX > tMaxZ) {
                    c0[2] += stepZ; tMaxZ += tDeltaZ;
                } else {
                    c0[0] += stepX; tMaxX += tDeltaX;
                    c0[1] += stepY; tMaxY += tDeltaY;
                    c0[2] += stepZ; tMaxZ += tDeltaZ;
                }
            }
            for (int i = std::max(0, c0[0] - neighbor_size); i < std::min(c0[0] + neighbor_size + 1, grid_size[0]); i++) {
                for (int j = std::max(0, c0[1] - neighbor_size); j < std::min(c0[1] + neighbor_size + 1, grid_size[1]); j++) {
                    for (int k = std::max(0, c0[2] - neighbor_size); k < std::min(c0[2] + neighbor_size + 1, grid_size[2]); k++) {
                        cells[index(i, j, k)].reach_subspace=true;
                    }
                }
            }
        }
    }

    //similar to solve() function
    reset();
    std::vector<int> index;
    index=agent.route.index.front();
    start=get_cell(index.at(0),index.at(1),index.at(2));
    index=agent.route.index.back();
    end=get_cell(index.at(0),index.at(1),index.at(2));
    start->g = 0;
    open.push(start);
    uint64_t expanded = 0;

    double original_cost_current_env=calculate_costs(agent.route);                              // original route, include conflict cost
    double original_cost_original_env=agent.route.total();                                      // original route, not include conflict cost
    while (true) {
        Cell *c = open.pop();
        if (c == nullptr) {
            break;
        }
        current = c;
        if (c == end) {
           Route route = get_route(false);                                                     // current route, not include congestion cost
            Route route_cur=get_route(true);                                                    // current route, include congestion cost
            std::vector<int> conflicts_route=cal_conflicted_routes(route,agent.id);             // id of routes conflicted with the route, this will update the grid cells
            int conflict_temp=conflict_size();
            int conflict_gain=prev_conflict-conflict_temp;
            double route_gain=original_cost_original_env-route.total();
            double gain_temp=route_gain*2+double(conflict_gain);                                // gain function
            // double gain_temp=route_gain*2/(agent.id+1)+
            // double(conflict_gain)*(agent.id+1)*(agent.id+1)*(agent.id+1);                    // another gain function, consider route priority, routes sorted by their priority
            
            // cur_ori==cur_cur -> no conflict now          cur_ori<cur_cur -> code runs well
            // ori_cur>cur_cur -> code has bugs             ori_cur==cur_cur -> route not change
            std::ostringstream oss;
            oss << agent.id << " expanded: " << std::setw(6) << std::right << expanded;
            oss<<" ori_ori: "<<std::setprecision(2)<<std::fixed<<std::setw(7) << std::right<<original_cost_original_env;    // ori_ori: previous route, not include congestion cost
            oss<<" ori_cur: "<<std::setprecision(2)<<std::fixed<<std::setw(7) << std::right<<original_cost_current_env;     // ori_cur: previous route, include congestion cost
            oss<<" cur_ori: "<<std::setprecision(2)<<std::fixed<<std::setw(7) << std::right<<route.total();                 // cur_ori: current route, not include congestion cost
            oss<<" cur_cur: "<<std::setprecision(2)<<std::fixed<<std::setw(7) << std::right<<route_cur.total();             // cur_cur: current route, include congestion cost
            oss<<" route_gain: "<<std::setprecision(2)<<std::fixed<<std::setw(7) << std::right<<route_gain;
            oss<<" conflict: "<<std::setw(4) << std::right<<conflict_gain;
            oss<<" = "<<std::setw(4) << std::right<<prev_conflict;
            oss<<"-"<<std::setw(4) << std::right<<conflict_temp;
            oss<<" gain:"<<std::setprecision(2)<<std::fixed<<std::setw(6)<< std::right<<gain_temp;
            if(std::fabs(gain_temp)<1e-5 and conflicts_route.empty())oss<<"  not conflict";
            else if(std::fabs(gain_temp)<1e-5 and !conflicts_route.empty())oss<<"     local opt";
            else oss<<"              ";
            oss<<"  conflicts: ";
            for(auto i:conflicts_route)oss<<i<<' ';
            LOG(INFO)<<oss.str();
            // if(route_cur.total()-original_cost_current_env>1e-1) LOG(WARNING)<<"wrong ";                                 // ori_cur<cur_cur -> bugs exist, Theta* not guarantee the optimal route so this may happen
            // if(route_cur.total()-route.total()<1e-5 and !conflicts_route.empty()){}                                      // congestion cost zero but conflict exist in routes
            
            conflicts_along_route.set_value(conflicts_route);
            if((route_gain<0.0 and conflict_gain==0) or conflict_gain<0) gain_temp=-INF;                                    // Just for faster run because Theta* not guarantee totally optimal
            gain.set_value(gain_temp);
            p.set_value(route);
            return;
        }
        c->closed = true;
        auto cells = get_adjacent_cells_subspace(c);
        for (auto adj : cells) {
            if (!adj->closed) update_cell(c, adj);
        }
        expanded++;
    }
    LOG(INFO)<<"failed, expanded: " << expanded;
}

double AStar_conops2::calculate_costs(Route r){
    // TODO: correctly calculate turning and climbing cost, as the parent node has been cleared
    double cost=0;
    for(long unsigned int l=0;l<r.index.size()-1;l++){
        int i=r.index[l][0],j=r.index[l][1],k=r.index[l][2];
        int i1=r.index[l+1][0],j1=r.index[l+1][1],k1=r.index[l+1][2];
        cost+=get_traversal_cost(&cells[index(i, j, k)],&cells[index(i1, j1, k1)]);
        cost+=get_conflict_cost(&cells[index(i, j, k)],&cells[index(i1, j1, k1)]);
    }
    return cost;
}
/*
Return the route
parameter is_conflict_included decides whether include congestion cost
*/
Route AStar_conops2::get_route(bool is_conflict_included=false) {
    Route r;
    const Cell *c = this->current;
    while (c) {
        r.index.push_back({c->i, c->j, c->k});
        r.position.push_back({c->x, c->y, c->z});
        if (!c->parent) {
            break;
        }
        r.length += distance3d(c->parent, c);
        r.traversal += get_traversal_cost(c->parent, c);
        r.turning += get_turning_cost(c->parent, c);
        r.climbing += get_climb_cost(c->parent, c);
        r.space += get_space_cost(c->parent, c);
        if(is_conflict_included) r.conflict += get_conflict_cost(c->parent,c);
        c = c->parent;
    }
    if (c != this->start) {
        LOG(INFO)<<"Failed to find route between cell ("<<start->i<<", "<<start->j<<", "<<start->k<<") and ("<<end->i<<", "<<end->j<<", "<<end->k<<")";
        r.position.clear();
        r.index.clear();
    }
    else {
        std::reverse(r.index.begin(), r.index.end());
        std::reverse(r.position.begin(), r.position.end());
    }
    return r;
}

void backtrack_cell(const Route& p, const int& id,AStar_conops2 & solver, const int& space_length, const int& xlim, const int& ylim, const int& zlim, const double& grid_res)
{
    std::vector<std::vector<int32_t>> protection_zone_cells;
    protection_zone_cells.clear();
    for(size_t j=0;j<p.index.size()-1;j++){
        route_protection results = line_traversal(p.index.at(j), p.index.at(j+1), space_length, xlim,ylim,zlim, grid_res);
        protection_zone_cells.insert(protection_zone_cells.end(), results.protection_zone_cell.begin(), 
        results.protection_zone_cell.end());
        for(auto voxel : results.route_cells){
            if(solver.is_reserved(voxel.at(0),voxel.at(1),voxel.at(2))==false){
                solver.unset_occupied_id(voxel.at(0), voxel.at(1), voxel.at(2),id,true);
            }
        }
    }
    for(auto voxel : protection_zone_cells){
        if(solver.is_reserved(voxel.at(0),voxel.at(1),voxel.at(2))==false){
            solver.unset_occupied_id(voxel.at(0), voxel.at(1), voxel.at(2),id,false);
        }
    }
}

void write_cells(AStar_conops2& solver,const std::vector<Agent>& route_parallel,const int& space_length, const int& xlim, const int& ylim, const int& zlim, const double& grid_res)//, const int& ki)
{
    for (auto agent : route_parallel){
        Route p=agent.route;
        int id=agent.id;
        std::vector<std::vector<int32_t>> protection_zone_cells;
        protection_zone_cells.clear();
        LOG(INFO)<<"route waypoint size:"<<p.index.size()<<", traversal cost "<<p.traversal<<", turning cost "<<p.turning
        <<", climbing cost "<<p.climbing <<", space cost "<<p.space<<", total cost "<<p.total();
        for(size_t j=0;j<p.index.size()-1;j++){
            route_protection results = line_traversal(p.index.at(j), p.index.at(j+1), space_length, xlim,ylim,zlim, grid_res);
            protection_zone_cells.insert(protection_zone_cells.end(), results.protection_zone_cell.begin(), results.protection_zone_cell.end());
            for(auto voxel : results.route_cells){
                if(solver.is_reserved(voxel.at(0),voxel.at(1),voxel.at(2))==false){
                    solver.set_occupied_id(voxel.at(0), voxel.at(1), voxel.at(2),id,true);
                }
            }
        }
        for(auto voxel : protection_zone_cells){
            if(solver.is_reserved(voxel.at(0),voxel.at(1),voxel.at(2))==false){
                solver.set_occupied_id(voxel.at(0), voxel.at(1), voxel.at(2),id,false);
            }
        }
    }
}

}