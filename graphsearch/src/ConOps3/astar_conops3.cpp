#include "astar_conops3.h"
#include <glog/logging.h>
namespace conop3{

AStar_conops3::AStar_conops3(const array_t<int32_t> & cells_type, const array_t<double> & cells_cost, 
                const array_t<double> & xs, const array_t<double> & ys, const array_t<double> & zs, 
                double  grid_res,
                double turning_cost_angle, double turning_cost_consecutive, double climb_cost_ascend, 
                double climb_cost_descend, double climb_threshold, double space_ccoef, int space_length,
                bool nonuniform, bool operational, const int & window_size):
    turning_cost_angle(turning_cost_angle), turning_cost_consecutive(turning_cost_consecutive), climb_cost_ascend(climb_cost_ascend), climb_cost_descend(climb_cost_descend), 
    climb_threshold(climb_threshold), space_cost_coef(space_ccoef), space_length(space_length),grid_res(grid_res), nonuniform(nonuniform), operational(operational)
{   
    // Calculate the grid size
    for(int i = 0; i < 3; i++) {                                            // The space dimension
        this->grid_size[i] = cells_type.shape(i);
    }
    this->grid_size[3] = window_size;                                       // The temporal dimension
    // this->grid_size[3] = int(400 * grid_res / 5);
    // index for locate cell
    stride_all = grid_size[0] * grid_size[1] * grid_size[2]*grid_size[3];   // 4d index
    stride_i = grid_size[1] * grid_size[2]*grid_size[3];
    stride_j = grid_size[2] * grid_size[3];
    stride_k = grid_size[3];                                                // time dimension
    stride_i3 = grid_size[1] * grid_size[2];                                // 3d index
    stride_j3 = grid_size[2];
    LOG(INFO)<<stride_all<<','<<stride_i<<','<<stride_j<<','<<stride_k<<','<<grid_size[0]<<','<<grid_size[1]<<','<<grid_size[2]<<','<<grid_size[3];

    //Initialize the grid cells with the provided data
    init_cells(xs.data(), ys.data(), zs.data(), cells_type.data(), cells_cost.data());
}

void AStar_conops3::init_cells(const double *xs, const double *ys, const double *zs, const int32_t *cells_type, const double *cells_cost) {
    // Resize the cell container
    cells.resize(stride_all);

    // Iterate to initialize cells.
    for (int i = 0; i < grid_size[0]; i++) {
        for (int j = 0; j < grid_size[1]; j++) {
            for (int k = 0; k < grid_size[2]; k++) {
                for (int l = 0; l < grid_size[3]; l++) {
                    // Get current cell
                    Cell *c = get_cell(i,j,k,l);

                    // Set the grid indices and physical coordinates for the cell
                    c->i = i; c->j = j; c->k = k; c->l = l;                                     // 4D
                    c->x = xs[i]; c->y = ys[j]; c->z = zs[k]; c->t = static_cast<double>(l);    // 4D
                    
                    // Assign the cost coefficient
                    c->cost_coeff = cells_cost[index3d(i, j, k)];                               // Find using 3d cells
            
                    // Determine the cell type and set its properties
                    int type = cells_type[index3d(i, j, k)];                                    // Find using 3d cells
                    if (type < 0) {
                        c->reachable = false;                                                   // The cell cannot be passed through.
                        c->access = false;                                                      // The cell is an obstacle.
                    }else if (type > 0) {
                        c->reserved = true;                                                     // Reserve cells near vertiports
                    }
                }
            }
        }
    }
    LOG(INFO)<<"Finish init cells";
}

void AStar_conops3::update_cells(const double & time_refpoint, const std::vector<std::vector<std::vector<double>>> & trajectories_for_conflicts){
    
    // Move time forward, clear existing trajectories
    for (int i = 0; i < grid_size[0]; i++) {
        for (int j = 0; j < grid_size[1]; j++) {
            for (int k = 0; k < grid_size[2]; k++) {
                for (int l = 0; l < grid_size[3]; l++) {
                    Cell *c = get_cell(i,j,k,l);
                    c->t = time_refpoint+static_cast<double>(l);                                // Move time forward
                    c->reachable = c->access;
                    c->operate = -1;                                                            // Clear existing trajectories
                }
            }
        }
    }

    // Add existing trajectories in current time window
    for(int i = 0; i < static_cast<int>(trajectories_for_conflicts.size()); i++){
        auto & trajectory = trajectories_for_conflicts[i];
        for(auto & waypoint : trajectory){
            int i = static_cast<int>(waypoint[0]);
            int j = static_cast<int>(waypoint[1]);
            int k = static_cast<int>(waypoint[2]);
            int l = static_cast<int>(waypoint[3]-time_refpoint);                                // Relative time in current time window
            
            if(l>=0){                                                                           // Only add trajectories that still valid in current time window
                Cell *c = get_cell(i,j,k,l);
                if(c->reserved!=true){                                                          // Only for not reserved cells
                    c->reachable = false;                                                       // Occupied by others
                    c->operate = i;                                                             // Occupied by which drone
                }
                
            }
            if(l>=grid_size[3])LOG(ERROR)<<"time window is not large enough";                   // TODO: consider condition l>gridsize[3]
        }
    }
}

void AStar_conops3::reset() {
    
    for (auto & c : cells) {
        c.h = 0;
        c.g = INF;
        c.parent = nullptr;
        c.closed = false;
    }
    open.clear();
    current = nullptr;
}

int AStar_conops3::conflict_size(){
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
                conflict_size += times_path;                    // path-path conflicts * 3 + path-buffer conflicts, here only consider path-path conflicts
            }
        }
    }
    return conflict_size;
}

std::vector<Conflict> AStar_conops3::conflict_cell_route_calculation(){
    std::vector<Conflict> conflicts;
    conflicts.clear();
    for (auto &c : cells){
        //belong to 2 or more routes
        if(c.occupied_routes.size()>1){
            for(route_occupation occupied:c.occupied_routes){
                if(occupied.protected_==false){                 // unless all routes use it as buffer, otherwise it has conflicts
                    conflicts.push_back({c.i,c.j,c.k,c.l,c.occupied_routes});
                    break;
                }
            }
        }
    }
    return conflicts;
}

std::vector<head_conflict> AStar_conops3::get_inverse_conflicts(const std::vector<Agent> & route_parallel){
    // TODO: speed up the comparison in the future
    std::vector<head_conflict> head_conflicts; 
    for(auto &agent:route_parallel){
        for(int i=0;i<static_cast<int>(agent.route.index.size()-1);i++){
            std::vector<int> from=agent.route.index[i];                     // This agent: (p_i, l)   -> (p_i+1, l+1)
            std::vector<int> to=agent.route.index[i+1];                     // Inverse:    (p_i+1, l) -> (p_i, l+1)
            if(from[3]+1 != to[3]) LOG(ERROR)<<"route is not consecutive";  // 'to' should be the next point of 'from'
            Cell* inv_from=get_cell(from[0],from[1],from[2],to[3]);     
            Cell* inv_to=get_cell(to[0],to[1],to[2],from[3]);
            std::vector<route_occupation> occupies1=inv_from->occupied_routes;
            std::vector<route_occupation> occupies2=inv_to->occupied_routes;
            for(auto &occupy1:occupies1){
                for(auto &occupy2:occupies2){
                    if ((agent.id!=occupy1.route_id) & (occupy1.route_id == occupy2.route_id)){
                        head_conflicts.push_back({from[0],from[1],from[2],to[0],to[1],to[2],from[3],agent.id,occupy1.route_id});
                    }
                }
            }
        }
    }
    return head_conflicts;
}

std::vector<int> AStar_conops3::cal_conflicted_routes(Route p,int id){
    std::vector<int> route_ids;                                     // Store routes that have conflicts with route p
    route_ids.clear();
    
    for(auto voxel:p.index){                                        // Check conflicts
        bool reserved=cells[index(voxel.at(0), voxel.at(1), voxel.at(2), voxel.at(3))].reserved;
        if(!reserved){
            set_occupied_id(voxel.at(0), voxel.at(1), voxel.at(2),voxel.at(3),id,true); // Add current route to the cell
            std::vector<route_occupation>& occupies=cells[index(voxel.at(0), voxel.at(1),
                                            voxel.at(2),voxel.at(3))].occupied_routes;
            for(auto occupied:occupies){                                                // Any route occupation will lead to conflicts
                if(occupied.route_id!=id)route_ids.push_back(occupied.route_id);
            }
        }
    }
    for(int i=0;i<static_cast<int>(p.index.size()-1);i++){              // Check head conflicts
        std::vector<int> from=p.index[i];                               // This agent: (p_i, l)   -> (p_i+1, l+1)
        std::vector<int> to=p.index[i+1];                               // Inverse:    (p_i+1, l) -> (p_i, l+1)
        if(from[3]+1 != to[3]) LOG(ERROR)<<"route is not consecutive";  // 'to' should be the next point of 'from'
        Cell* inv_from=get_cell(from[0],from[1],from[2],to[3]);
        Cell* inv_to=get_cell(to[0],to[1],to[2],from[3]);
        std::vector<route_occupation> occupies1=inv_from->occupied_routes;
        std::vector<route_occupation> occupies2=inv_to->occupied_routes;
        for(auto &occupy1:occupies1){
            for(auto &occupy2:occupies2){
                if ((id!=occupy1.route_id) & (occupy1.route_id == occupy2.route_id)){
                    route_ids.push_back(occupy1.route_id);
                }
            }
        }
    }
    if(route_ids.empty())return route_ids;                              // Not conflict with others
    
    // Delete repeated elements in route_ids
    std::vector<int>::iterator vector_iterator;
    std::sort(route_ids.begin(),route_ids.end());
    vector_iterator = std::unique(route_ids.begin(),route_ids.end());
    if(vector_iterator != route_ids.end()){
        route_ids.erase(vector_iterator,route_ids.end());
    }
    return route_ids;
}

double AStar_conops3::get_traversal_cost(const Cell *from, const Cell *to) {
    return distance4d_sep(from, to) * (from->cost_coeff + to->cost_coeff) * 0.5;
}

double AStar_conops3::get_conflict_cost(const Cell *from, const Cell *to) {
    double count = 0.0;
    
    if (from->i==start->i and  from->j==start->j and from->k==start->k){    // If it starts from starting node, add into conflict cost
        if(!from->reserved){
            if(from->occupied_routes.empty()){                              // exactly no route
            }else{                                                          // conflicts appear
                for(auto & occupied : from->occupied_routes){
                    if(occupied.protected_==false) count+=1;
                }
            }
        }
    }
    
    for(auto & occupied : to->occupied_routes){                             // Check the to node for conflict cost
        if(occupied.protected_==false) count+=1;
    }
    
    //TODO: currently not apply for any-angle
    Cell* from_inverse=get_cell(from->i,from->j,from->k,to->l);             // Check whether inverse conflict exists
    Cell* to_inverse=get_cell(to->i,to->j,to->k,from->l);
    if(from->l+1 != to->l) LOG(ERROR)<<"route is not consecutive";          // 'to' should be the next point of 'from'
    if(!from_inverse->occupied_routes.empty() & !to_inverse->occupied_routes.empty()){
        for(auto & occupied_from : from_inverse->occupied_routes){
            for(auto & occupied_to : to_inverse->occupied_routes){
                if(occupied_from.route_id==occupied_to.route_id){
                    count+=1;
                }
            }
        }
    }
    return conflict_cost_coef * count;
}

std::vector<Cell *> AStar_conops3::get_adjacent_cells(const Cell *c) {
    std::vector<Cell *> r;
    if(c->l<=grid_size[3]-2){
        Cell* adj = get_cell(c->i, c->j, c->k, c->l+1);
        if(adj->reachable){r.push_back(adj);}                                                   // Hover
        for (int i = std::max(0, c->i - 1); i < std::min(grid_size[0], c->i + 2); i++) {        // Move forward along time
            Cell* adj = get_cell(i, c->j, c->k, c->l+1);
            Cell* c1 = get_cell(i, c->j, c->k, c->l);
            Cell* c2 = get_cell(c->i, c->j, c->k, c->l+1);
            if (((adj->i!=c->i) | (adj->j!=c->j) | (adj->k!=c->k)) & adj->reachable) {
                if(c1->reachable==false && c2->reachable==false && c1->operate==c2->operate);   // Avoid head conflicts
                else r.push_back(adj);
            }
        }
        for (int j = std::max(0, c->j - 1); j < std::min(grid_size[1], c->j + 2); j++) {
            Cell* adj = get_cell(c->i, j, c->k, c->l+1);
            Cell* c1=get_cell(c->i, j, c->k, c->l);
            Cell* c2=get_cell(c->i, c->j, c->k, c->l+1);
            if (((adj->i!=c->i) | (adj->j!=c->j) | (adj->k!=c->k)) & adj->reachable) {
                if(c1->reachable==false && c2->reachable==false && c1->operate==c2->operate);   // Avoid head conflicts
                else r.push_back(adj);
            }
        }
        for (int k = std::max(0, c->k - 1); k < std::min(grid_size[2], c->k + 2); k++) {
            Cell* adj = get_cell(c->i, c->j, k, c->l+1);
            Cell* c1=get_cell(c->i, c->j, k, c->l);
            Cell* c2=get_cell(c->i, c->j, c->k, c->l+1);
            if (((adj->i!=c->i) | (adj->j!=c->j) | (adj->k!=c->k)) & adj->reachable) {
                if(c1->reachable==false && c2->reachable==false && c1->operate==c2->operate);   // Avoid head conflicts
                else r.push_back(adj);
            }
        }
    }
    return r;
}

std::vector<Cell *> AStar_conops3::get_adjacent_cells_subspace(const Cell *c) {
    std::vector<Cell *> r;

    if(c->l<=grid_size[3]-2){
        Cell* adj = get_cell(c->i, c->j, c->k, c->l+1);
        if(adj->reachable){r.push_back(adj);}                                                   // Hover
        for (int i = std::max(0, c->i - 1); i < std::min(grid_size[0], c->i + 2); i++) {        // Move forward along time
            Cell* adj = get_cell(i, c->j, c->k, c->l+1);
            if (((adj->i!=c->i) | (adj->j!=c->j) | (adj->k!=c->k)) & adj->reach_subspace & adj->reachable) {
                r.push_back(adj);
            }
        }
        for (int j = std::max(0, c->j - 1); j < std::min(grid_size[1], c->j + 2); j++) {        // For adjacent cells in subspace, no need to check cell's reachable status, 
            Cell* adj = get_cell(c->i, j, c->k, c->l+1);                                        // since the distributed planning does not operate on cell's researchable directly
            if (((adj->i!=c->i) | (adj->j!=c->j) | (adj->k!=c->k)) & adj->reach_subspace & adj->reachable) {
                r.push_back(adj);
            }
        }
        for (int k = std::max(0, c->k - 1); k < std::min(grid_size[2], c->k + 2); k++) {
            Cell* adj = get_cell(c->i, c->j, k, c->l+1);
            if (((adj->i!=c->i) | (adj->j!=c->j) | (adj->k!=c->k)) & adj->reach_subspace & adj->reachable) {
                r.push_back(adj);
            }
        }
    }

    return r;
}

void AStar_conops3::update_cell(Cell *c, Cell *adj) {
    double g_old = adj->g;
    compute_cost(c, adj);
    if (adj->g < g_old) {
        adj->h = get_heuristic(adj);
        open.push(adj);
    }
}

void AStar_conops3::compute_cost(Cell *c, Cell *adj) {
    double g_new = c->g;
    if (nonuniform) {
        g_new += get_traversal_cost(c, adj);
    } else {
        g_new += distance4d_sep(c, adj);
    }
    g_new += +get_conflict_cost(c,adj);
    if (adj->g-g_new>1.0e-5) {
        // if(adj->g==INF or protect_overlap==true){
            adj->parent = c;
            adj->g = g_new;
        //     protect_overlap=false;
        // }
            
    }
}

void AStar_conops3::solve(array_t<int64_t> i1, array_t<int64_t> i2, std::promise<Route> & p) {
    uint64_t expanded = 0;
    reset();                                                                // init cells for new search
    start = get_cell(i1.at(0), i1.at(1), i1.at(2), i1.at(3));
    end = get_cell(i2.at(0), i2.at(1), i2.at(2), this->grid_size[3]-1);     // Arrive at the end of time window

    for(int l=0;l<this->grid_size[3];l++){                                  // ensure the origin is available all the time
        set_reachable(start->i,start->j,start->k,l,true,-1);
    }

    for(int l=0;l<this->grid_size[3];l++){                                  // ensure the destination is available all the time
        set_reachable(end->i,end->j,end->k,l,true,-1);
    }
    start->g = 0;
    open.push(start);
    while (true) {
        Cell *c = open.pop();
        if (c == nullptr) break;
        current = c;
        
        if (c->i == end->i and c->j == end->j and c->k == end->k) {         // successfully find the route
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
    LOG(INFO)<<"failed, expanded: " << expanded;
    Route route=get_route(false);
    route.traversal=INF;
    p.set_value(route);
}

void AStar_conops3::solve_subspace(Agent agent, std::vector<Agent> agents, std::promise<Route> & p, std::promise<std::vector<int>> & 
 conflicts_along_route, std::promise<double> & gain, int prev_conflict, int neighbor_size, int method){
    conflict_cost_coef=agent.conflict_coefficient;                                                                      // get conflict cost coefficient
    backtrack_cell(agent.route,agent.id,*this);                                                                         // release the airspace occupied by the previous path of the route (agent)
    for (std::vector<std::vector<int>>::iterator it = agent.route.index.begin(); it != agent.route.index.end()-1; ++it){// flag the subspace as true to get search subspace
        int i1 = (*it).at(0);
        int j1 = (*it).at(1);
        int k1 = (*it).at(2);
        int c0[3] = {i1, j1, k1};
        for (int i = std::max(0, c0[0] - neighbor_size); i < std::min(c0[0] + neighbor_size + 1, grid_size[0]); i++) {
            for (int j = std::max(0, c0[1] - neighbor_size); j < std::min(c0[1] + neighbor_size + 1, grid_size[1]); j++) {
                for (int k = std::max(0, c0[2] - neighbor_size); k < std::min(c0[2] + neighbor_size + 1, grid_size[2]); k++) {
                    for(int l = 0; l<grid_size[3] ; l++) cells[index(i, j, k, l)].reach_subspace=true;
                }
            }
        }
    }

    //similar to solve() function
    reset();
    std::vector<int> index_;
    index_=agent.route.index.front();
    start=get_cell(index_.at(0),index_.at(1),index_.at(2),index_.at(3));
    index_=agent.route.index.back();
    end=get_cell(index_.at(0),index_.at(1),index_.at(2),index_.at(3));

    for(int l=0;l<this->grid_size[3];l++){                                                      // Ensure the destination is available
        set_reachable(end->i,end->j,end->k,l,true,-1);
    }

    //TODO: to avoid other ODs pass vertiport, but maybe vertiport is also allowed to passby?
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
        if (c->i == end->i and c->j == end->j and c->k == end->k) {                             // successfully find the route
            Route route = get_route(false);                                                     // current route, not include congestion cost
            Route route_cur=get_route(true);                                                    // current route, include congestion cost
            std::vector<int> conflicts_route=cal_conflicted_routes(route,agent.id);             // id of routes conflicted with the route, this will update the grid cells
            for(auto &a:agents){
                if(a.id==agent.id)a.route=agent.route;                                          // to calculate inv_conflicts
            }
            int conflict_temp=conflict_size()+int(get_inverse_conflicts(agents).size()/2);      // conflicts
            int conflict_gain=prev_conflict-conflict_temp;
            double route_gain=original_cost_original_env-route.total();
            double gain_temp=route_gain*2+double(conflict_gain);                                // gain function
            
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
            if (!adj->closed) {
                update_cell(c, adj);
            }
        }
        expanded++;
    }
    LOG(INFO)<<"failed, expanded: " << expanded;
}

double AStar_conops3::calculate_costs(Route r){
    double cost=0;
    for(long unsigned int l=0;l<r.index.size()-1;l++){
        int i=r.index[l][0],j=r.index[l][1],k=r.index[l][2], m=r.index[l][3];
        int i1=r.index[l+1][0],j1=r.index[l+1][1],k1=r.index[l+1][2], m1=r.index[l+1][3];
        cost+=get_traversal_cost(&cells[index(i, j, k, m)],&cells[index(i1, j1, k1, m1)]);
        cost+=get_conflict_cost(&cells[index(i, j, k, m)],&cells[index(i1, j1, k1, m1)]);
    }
    return cost;
}

Route AStar_conops3::get_route(bool is_conflict_included=false) {
    Route r;
    const Cell *c = this->current;
    while (c) {
        r.index.push_back({c->i, c->j, c->k, c->l});
        r.position.push_back({c->x, c->y, c->z, c->t});
        if (!c->parent) {
            break;
        }
        r.length += distance4d_sep(c->parent, c);
        r.traversal += get_traversal_cost(c->parent, c);
        if(is_conflict_included) r.conflict += get_conflict_cost(c->parent,c);
        c = c->parent;
    }
    if (c != this->start) {
        LOG(INFO)<<"Failed to find route between cell ("<<start->i<<", "<<start->j<<", "<<start->k<<start->l<<") and ("<<end->i<<", "<<end->j<<", "<<end->k<<end->l<<")";
        r.position.clear();
        r.index.clear();
    }
    else {
        std::reverse(r.index.begin(), r.index.end());
        std::reverse(r.position.begin(), r.position.end());
    }
    return r;
}

void backtrack_cell(const Route& p, const int& id,AStar_conops3 & solver)
{
    for(auto voxel : p.index){
        if(solver.is_reserved(voxel.at(0),voxel.at(1),voxel.at(2),voxel.at(3))==false){
            solver.unset_occupied_id(voxel.at(0), voxel.at(1), voxel.at(2),voxel.at(3),id,true);
        }
    }
}

void write_cells(AStar_conops3& solver,const std::vector<Agent>& route_parallel)
{
    for (auto agent : route_parallel){
        Route p=agent.route;
        int id=agent.id;
        LOG(INFO)<<"route waypoint size:"<<p.index.size()<<", traversal cost "<<p.traversal<<", turning cost "<<p.turning
        <<", climbing cost "<<p.climbing <<", space cost "<<p.space<<", total cost "<<p.total();
        for(auto voxel : p.index){
            if(solver.is_reserved(voxel.at(0),voxel.at(1),voxel.at(2),voxel.at(3))==false){
                solver.set_occupied_id(voxel.at(0), voxel.at(1), voxel.at(2),voxel.at(3),id,true);
            }
        }
    }
}

}