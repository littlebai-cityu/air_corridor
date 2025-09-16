#include "conops2.h"
#include <glog/logging.h>
namespace conop2{

py::list conops2_route_seq(const array_t<int32_t> & cells_type, const array_t<double> & cells_cost,  const array_t<double> & xs, const array_t<double> & ys, 
 const array_t<double> & zs, double  grid_res, double turning_cost_angle, double turning_cost_consecutive, double climb_cost_ascend, double climb_cost_descend, 
 double climb_threshold, double space_ccoef, int space_length, bool nonuniform, bool operational, int route_number, const py::list & dep, 
 const py::list & dest, const py::list & centerd, const py::list & centera, const int & shuffled, AStar_conops2 & solver) {
    std::vector<std::vector<int32_t>> departures = py::cast<std::vector<std::vector<int32_t>>>(dep);
    std::vector<std::vector<int32_t>> destinations = py::cast<std::vector<std::vector<int32_t>>>(dest);
    std::vector<std::vector<double>> center_d = py::cast<std::vector<std::vector<double>>>(centerd);
    std::vector<std::vector<double>> center_a = py::cast<std::vector<std::vector<double>>>(centera);
    int xlim=cells_type.shape(0);
    int ylim=cells_type.shape(1);
    int zlim=cells_type.shape(2);
    std::cout << "x,y,z"<< xlim << ylim << zlim << std::endl;
    struct timeval t1,t2;
    double timeuse;
    gettimeofday(&t1,NULL);
    std::vector<double> min_cost;                                                          // Store total cost for each route in every one_sequential()
    min_cost.clear();
    py::list norm_route;
    
    int protection_length = std::ceil(10/grid_res);
    std::cout << "vertiports_protection_length: " << protection_length << std::endl;
    for(auto dep : departures){                                                             // To avoid conflicts near vertiports, which belongs to terminal airspace
        for(int l=std::max(0,dep.at(0)-protection_length);l<std::min(dep.at(0)+protection_length+1,xlim);l++)
                for(int m=std::max(0,dep.at(1)-protection_length);m<std::min(dep.at(1)+protection_length+1,ylim);m++)
                    for(int n=std::max(0,dep.at(2)-1);n<std::min(dep.at(2)+2,zlim);n++)
                        solver.set_reserved(l,m,n,true);
    }
    norm_route=one_sequential(solver,route_number,departures,destinations,center_d,center_a,space_length,xlim,ylim,zlim, grid_res, min_cost);
    float cost=0;
    for(int i=0;i<route_number;i++)cost+=min_cost.at(i);
    /*output the costs*/
    std::ostringstream oss;
    oss<<"total cost for the sequence is "<<cost<<"; each are ";
    for(int i=0;i<route_number;i++) oss<<min_cost.at(i)<<' ';
    LOG(INFO) << oss.str();
    gettimeofday(&t2,NULL);
    timeuse = (t2.tv_sec - t1.tv_sec) + (double)(t2.tv_usec - t1.tv_usec)/1000000.0;
    LOG(INFO)<<"route generation time = "<<timeuse;
    return norm_route;
}

py::list conops2_route_dis(const array_t<int32_t> & cells_type, const array_t<double> & cells_cost,  const array_t<double> & xs, const array_t<double> & ys, 
 const array_t<double> & zs,double  grid_res, double turning_cost_angle, double turning_cost_consecutive, double climb_cost_ascend, double climb_cost_descend, 
 double climb_threshold, double space_ccoef, int space_length, bool nonuniform, bool operational, int route_number, const py::list & dep, 
 const py::list & dest, const py::list & centerd, const py::list & centera, const int & method, AStar_conops2 & solver) {
    struct timeval t1,t2;
    double timeuse;
    std::vector<std::vector<double>> center_d = py::cast<std::vector<std::vector<double>>>(centerd);            // Type transfer via pybind11
    std::vector<std::vector<double>> center_a = py::cast<std::vector<std::vector<double>>>(centera);
    std::vector<std::vector<int32_t>> departures = py::cast<std::vector<std::vector<int32_t>>>(dep);            // Type transfer via pybind11
    std::vector<std::vector<int32_t>> destinations = py::cast<std::vector<std::vector<int32_t>>>(dest);
    std::vector<py::list> p0,p1;
    int xlim=cells_type.shape(0),ylim=cells_type.shape(1),zlim=cells_type.shape(2);
    std::vector<Conflict> conflicts;
    conflicts.clear();
    
    for (int i=0; i < route_number;i++){
        p0.push_back(dep[i]);                                                                                   // OD pairs
        p1.push_back(dest[i]);
    }
    int protection_length = std::ceil(12.5/grid_res);
    for(auto dep : departures){                                                             // To avoid conflicts near vertiports, which belongs to terminal airspace
        for(int l=std::max(0,dep.at(0)-protection_length);l<std::min(dep.at(0)+protection_length+1,xlim);l++)
                for(int m=std::max(0,dep.at(1)-protection_length);m<std::min(dep.at(1)+protection_length+1,ylim);m++)
                    for(int n=std::max(0,dep.at(2)-1);n<std::min(dep.at(2)+2,zlim);n++)
                        solver.set_reserved(l,m,n,true);
    }

    // Plan
    gettimeofday(&t1,NULL);
    std::vector<std::promise<Route>> promiseObj;
    std::vector<std::future<Route>> futureObj;
    for (int i=0; i < route_number;i++){
        std::promise<Route> promiseOb;
        std::future<Route> futureOb = promiseOb.get_future();
        promiseObj.push_back(std::move(promiseOb));
        futureObj.push_back(std::move(futureOb));
    }
    std::thread threads[route_number];
    LOG(INFO) << "All threads starts!";
    for (int i=0; i < route_number;i++) threads[i] = std::thread(&AStar_conops2::solve, solver, p0.at(i), p1.at(i),std::ref(promiseObj[i]));
    for (auto &thread : threads)thread.join();
    LOG(INFO) << "All threads succeed!";
    py::list norm_route;
    std::vector<Agent> route_parallel;
    for (int i=0; i < route_number;i++){
        Route route=futureObj.at(i).get();
        route_parallel.push_back({route,i,0.2});
        LOG(INFO)<<"route waypoint size:"<<route.index.size()<<", traversal cost "<<route.traversal<<", turning cost "<<route.turning
        <<", climbing cost "<<route.climbing <<", space cost "<<route.space<<", total cost "<<route.total();
    }
    write_cells(solver,route_parallel,space_length,xlim,ylim,zlim,grid_res);                                             // Update environment
    conflicts=solver.conflict_cell_route_calculation();                                                         // Get all conflicts, how many routes in each conflicts
    int conflicts_size=solver.conflict_size();                                                                  // Get number of conflicts
    LOG(INFO)<<"conflict size: "<<conflicts_size;
    print_conflicts(conflicts,route_number);                                                                    // Print conflict sets for each route
    gettimeofday(&t2,NULL);
    timeuse = (t2.tv_sec - t1.tv_sec) + (double)(t2.tv_usec - t1.tv_usec)/1000000.0;
    LOG(INFO)<<"time = "<<timeuse<<'s';
    
    // Replan to avoid conflicts
    int neighbor_size=5;                                                                                       // Decide neighbor size for local search in subspace
    int times=0;
    int not_conflict_count=0,local_opt_count=0;
    std::vector<std::promise<double>> promiseGain;
    std::vector<std::future<double>> futureGain;
    std::vector<std::promise<std::vector<int>>> promiseconflicts_along_route;
    std::vector<std::future<std::vector<int>>> futureconflicts_along_route;
    int max_iteration=10;
    int Num_feasible_routes=1;
    while((Num_feasible_routes<route_number or conflicts.size()>0) and times<=max_iteration)                    // Breaks if gains are too small
    {
        // plan in subspace
        promiseObj.clear();
        futureObj.clear();
        for (int i=0; i < route_number;i++){
            std::promise<Route> promiseOb;
            std::future<Route> futureOb = promiseOb.get_future();
            promiseObj.push_back(std::move(promiseOb));
            futureObj.push_back(std::move(futureOb));
        }
        promiseGain.clear();
        futureGain.clear();
        for (int i=0; i < route_number;i++){
            std::promise<double> promiseOb;
            std::future<double> futureOb = promiseOb.get_future();
            promiseGain.push_back(std::move(promiseOb));
            futureGain.push_back(std::move(futureOb));
        }
        promiseconflicts_along_route.clear();
        futureconflicts_along_route.clear();
        for (int i=0; i < route_number;i++){
            std::promise<std::vector<int>> promiseOb;
            std::future<std::vector<int>> futureOb = promiseOb.get_future();
            promiseconflicts_along_route.push_back(std::move(promiseOb));
            futureconflicts_along_route.push_back(std::move(futureOb));
        }
        LOG(INFO)<< "All threads starts!";
        for (int i=0; i < route_number;i++){
            threads[i] = std::thread(&AStar_conops2::solve_subspace, solver, route_parallel[i], std::ref(promiseObj[i]), std::ref(promiseconflicts_along_route[i]),
            std::ref(promiseGain[i]), conflicts_size, neighbor_size, method);
        }
        for (auto &thread : threads)thread.join();
        LOG(INFO) << "All threads joined!";
        for(auto agent : route_parallel)backtrack_cell(agent.route,agent.id,solver,space_length,xlim,ylim,zlim, grid_res);// Cancel actions performed on environment cells

        // Select the best route to be updated
        double gain=-INF;                                                                                       // Gain for selected route
        int gain_index;                                                                                         // Index of selected route
        Route gain_route;                                                                                       // Gain for selected route
        std::vector<std::vector<int>> conflict_routes_local_opts;                                               // Conflicts along routes where routes stuck in local optimals
        conflict_routes_local_opts.clear();
        not_conflict_count=0;
        local_opt_count=0;
        std::vector<Agent> gain_indexes;                                                                        // for Stochastic one, method 3
        gain_indexes.clear();                                                                                   
        std::vector<int> valid_routes;                                                                          // Valid routes that will be outputed
        valid_routes.clear();
        Num_feasible_routes=0;
        
        // Collect route results from threads
        for (int i=0; i < route_number;i++)
        {
            Route p=futureObj.at(i).get();                                                                      // get route
            double gain_temp=futureGain.at(i).get();                                                            // calculate gain value of conflicts and path
            std::vector<int> conflicts_along_route = futureconflicts_along_route.at(i).get();
            
            if(conflicts_along_route.empty())valid_routes.push_back(i);

            bool no_conflict_and_change=true;

            if(gain_temp<=0.0 and conflicts_along_route.empty())Num_feasible_routes++;
            if(std::fabs(gain_temp)<1e-5 and gain_temp!=-INF){                                                  // Route not change in this iteration, maybe stuck in local optimal or no conflict
                if(conflicts_along_route.empty())no_conflict_and_change=true;
                else no_conflict_and_change=false;
                if(no_conflict_and_change==true){                                                               // No conflict
                    not_conflict_count++;
                }else if(no_conflict_and_change==false){                                                        // Local optimal, may need to update conflict cost coefficient
                    conflict_routes_local_opts.push_back(conflicts_along_route);
                    local_opt_count++;
                }
            }else if (std::fabs(gain_temp)>=1e-5 and gain_temp!=-INF){                                          // Route change in this iteration, put it into candidate route for update
                if(gain_temp>gain){ gain=gain_temp; gain_index=i; gain_route=p; }
            }else if (gain_temp==-INF){                                                                         // Route stuck in local optimal, may need to update conflict cost coefficient
                conflict_routes_local_opts.push_back(conflicts_along_route);
                local_opt_count++;
            }
        }
        // Update either route, local search space, coefficient
        std::ostringstream oss;
        oss<<"gain:"<<std::setprecision(2)<<std::fixed<<std::setw(7) << std::left<<gain<<"  ";
        if(not_conflict_count+local_opt_count<route_number){                                                    // Some route can be selected, includes those have negative conlict gain
            route_parallel.at(gain_index)={gain_route,gain_index,route_parallel.at(gain_index).conflict_coefficient};
            neighbor_size=5;
            times=0;
            oss<<"route "<<gain_index<<" selected";
        }else if(not_conflict_count+local_opt_count==route_number and not_conflict_count<route_number){         // Only local opt exists, first time only update the local opt ones' coefficient, later update all conflict_coefficient*/
            times++;
            oss<<"conflict_coefficient:";
            if(times<8){                                                                                        // Update local opt one
                for(auto conflict_routes_local_opt:conflict_routes_local_opts){
                    for(auto i : conflict_routes_local_opt){
                        route_parallel[i].conflict_coefficient+=(0.2);oss<<route_parallel[i].conflict_coefficient<<' ';
                    }
                }
            }else{                                                                                               // Update all routes
                for(int i=0;i<route_number;i++){
                    route_parallel[i].conflict_coefficient+=(0.2+times*0.1);oss<<route_parallel[i].conflict_coefficient<<' '; //for faster run
                }
            }
            oss<<"   "<<times<<" time";
        }
        LOG(INFO)<<oss.str();
        if(times>2){                                                                                            // enlarge search space gradually
            neighbor_size+=5;
            LOG(INFO)<<"neighbor size "<<neighbor_size;
        }
        if(times>8){                                                                                            // try global search
            neighbor_size=1000;
            LOG(INFO)<<"global search";
        }
        conflicts.clear();
        write_cells(solver,route_parallel,space_length,xlim,ylim,zlim, grid_res);                                         // Update environment
        conflicts=solver.conflict_cell_route_calculation();                                                     // Get all conflicts, how many routes in each conflicts
        conflicts_size=solver.conflict_size();                                                                  // Get number of conflicts
        LOG(INFO)<<"conflicts size: "<<conflicts_size;
        print_conflicts(conflicts,route_number);
        if(gain<-2 or gain != -INF)break;                                                                       //spatial separation threshold
    }
    double length=0;
    for (int i=0; i < route_number;i++){
        length+=route_parallel[i].route.length;
        norm_route.append(route_parallel[i].route.position);
    }
    LOG(INFO)<<"total route length: "<<length;
    std::ostringstream oss;
    oss<<"weights:";
    for (auto & route: route_parallel)oss<<' '<<route.conflict_coefficient;
    LOG(INFO)<<oss.str();

    gettimeofday(&t2,NULL);
    timeuse = (t2.tv_sec - t1.tv_sec) + (double)(t2.tv_usec - t1.tv_usec)/1000000.0;
    LOG(INFO)<<"route generation time = "<<timeuse<<'s';
    return norm_route;
}

std::vector<std::vector<double>> trajectory_generation(Cells & cells, const std::vector<std::vector<double>> & route, const uint32_t & start_time, const DroneLimits &drone){
    //TODO: check whether need to update/remove elements in environment cells if the stored time has passed
    std::vector<std::vector<double>> trajectory;
    trajectory.clear();
    trajectory.push_back({route[0][0],route[0][1],route[0][2],static_cast<double>(start_time)});//the first 4d point
    double cumulative_time=static_cast<double>(start_time), _sample_step=50;
    std::vector<double> previous_waypoint, current_waypoint;
    for (size_t i = 1; i < route.size(); ++i) {
        std::vector<std::vector<double>> segments;
        segments.clear();
        previous_waypoint = route[i - 1];
        current_waypoint = route[i];
        double delta_p_x = current_waypoint[0] - previous_waypoint[0];
        double delta_p_y = current_waypoint[1] - previous_waypoint[1];
        double p_horizontal = std::sqrt(delta_p_x * delta_p_x + delta_p_y * delta_p_y);
        double delta_p_z = current_waypoint[2] - previous_waypoint[2];
        double p = std::sqrt(delta_p_z * delta_p_z + delta_p_x * delta_p_x + delta_p_y * delta_p_y);    // Delta distance
        if (std::fabs(delta_p_x) < 1e-6 && std::fabs(delta_p_y) < 1e-6 && std::fabs(delta_p_z) < 1e-6) continue;
        double ratio_z = std::fabs(delta_p_z) / p;
        double ratio_hor = p_horizontal / p;
        double max_v, max_a;
        std::string se;
        double minus_tol = 1e-3;
        if (std::fabs(delta_p_z) < 1e-6 || ratio_z < 1e-6) {
            max_v = drone.max_fly_speed_h- minus_tol;
            max_a = drone.max_fly_acc_h- minus_tol;
            se="horizontal fly";
        } else if (std::fabs(p_horizontal) < 1e-6 || ratio_hor < 1e-6) {
            max_v = drone.max_fly_speed_v- minus_tol;
            max_a = drone.max_fly_acc_v- minus_tol;
            se="vertical fly";
        } else {
            max_v = std::min(drone.max_fly_speed_h / ratio_hor, drone.max_fly_speed_v / ratio_z)- minus_tol;
            max_a = std::min(drone.max_fly_acc_h / ratio_hor, drone.max_fly_acc_v / ratio_z)- minus_tol;
            se="climb/descend fly";
        }

        WaypointAccInfo hor_info = generate_traj_1d(p, max_v, max_a);
        double segment_start_time=trajectory.back()[3];                                                 // The start time of using the route segment
        for (double t=segment_start_time+_sample_step; t < cumulative_time+hor_info.total_seconds * 1e3; t += _sample_step) {
            WaypointDistanceInfo dis_info = sample_traj(hor_info, (t-cumulative_time) * 1e-3);          // Get the position at each timestep
            std::vector<double> delta_p = get_3d_from_1d(previous_waypoint, current_waypoint, dis_info.travel_dist);
            std::vector<double> point4d;
            point4d.push_back(delta_p[0] + previous_waypoint[0]);
            point4d.push_back(delta_p[1] + previous_waypoint[1]);
            point4d.push_back(delta_p[2] + previous_waypoint[2]);
            point4d.push_back(t);
            segments.push_back(point4d);                                                                // Each 4d point
        }
        cumulative_time+=hor_info.total_seconds * 1e3;
        if(segments.back()[3]!=cumulative_time)segments.push_back({current_waypoint[0],current_waypoint[1],current_waypoint[2],cumulative_time});

        // Interact with environment cells to check conflicts 
        double delay=0;
        bool conflict_flag=false;
        double period_a = 10000000000,period_b = 0, max_delay=0;                                        // Time period [a,b] of traversed airspace used by other drones after current time
        double period_a_tmp= 10000000000, period_b_tmp = 0, max_delay_tmp = 0;                          // Not used here, only store temporary values
        
        conflict_flag=check_conflicts(previous_waypoint,current_waypoint, segment_start_time, cells, segments, max_delay, period_a, period_b);
        if(!conflict_flag){                                                                             // Conflict exists, temporally resolve conflict
            
            // The first attempt, try to use minimum delay, which is the maximum temporal overlap of all conflicts
            std::vector<std::vector<double>> segments_backup=segments;
            LOG(INFO)<<"first attempt temporal resolution,max delay "<<max_delay<<"ms, start time: "<<segment_start_time<<", "<< max_delay+1 <<" ms delay required";
            delay=max_delay+1;

            for(auto&seg:segments)seg[3]+=delay;                                                        // Add delay to the existing segment
            segments.insert(segments.begin(), trajectory.back());                                       // Add the delay parts the current segment

            if(!check_conflicts(previous_waypoint, current_waypoint, segment_start_time, cells, segments, max_delay_tmp, period_a_tmp, period_b_tmp)){
                LOG(WARNING)<<"first attempt failed, this segment is between waypoint"<<previous_waypoint[0]<<','<<previous_waypoint[1]<<','
                    <<previous_waypoint[2]<<", and "<<current_waypoint[0]<<','<<current_waypoint[1]<<','<<current_waypoint[2];
            }else {
                LOG(INFO)<<"first attempt succeed";
                cumulative_time+=delay;
                trajectory.insert(trajectory.end(), segments.begin()+1, segments.end());
                continue;
            }

            // The second attempt, try to delay till all routes finished use the airspace
            segments.clear();
            segments.insert(segments.end(), segments_backup.begin(), segments_backup.end());            // Restore the segments
            delay=0;
            LOG(INFO)<<"second attempt temporal resolution," <<" period_a: "<< period_a<<", period_b: "<<period_b<<", start time: "<<segment_start_time<<", "<< period_b + 1 - segment_start_time<<" ms delay required";
            if(period_a != 10000000000 and period_b != 0){
                if(segment_start_time+std::floor(hor_info.total_seconds * 1e3)>period_a){               // Time has overlap
                    delay=period_b + 1 - segment_start_time;
                }
            }
            if(delay!=0){
                for(auto&seg:segments)seg[3]+=delay;                                                    // Add delay to the existing segment
                segments.insert(segments.begin(), trajectory.back());                                   // Add the delay parts the current segment
            }
            if(!check_conflicts(previous_waypoint, current_waypoint, segment_start_time, cells, segments, max_delay_tmp, period_a_tmp, period_b_tmp)){
                LOG(WARNING)<<"second attempt failed, this segment is between waypoint"<<previous_waypoint[0]<<','<<previous_waypoint[1]<<','
                    <<previous_waypoint[2]<<", and "<<current_waypoint[0]<<','<<current_waypoint[1]<<','<<current_waypoint[2];
                trajectory.clear();                                                                     // Return empty trajectory
                return trajectory;
            }else {
                LOG(INFO)<<"second attempt succeed";
                cumulative_time+=delay;
                trajectory.insert(trajectory.end(), segments.begin()+1, segments.end());
                continue;
            }
        }
        trajectory.insert(trajectory.end(), segments.begin(), segments.end());                          // No conflict, directly add the trajectory
    }                                                                                                   // Each route segment

    if(trajectory.back()[3]!=cumulative_time)trajectory.push_back({route.back()[0],route.back()[1],route.back()[2],cumulative_time});   // The last waypoint
    airspace_update(cells,trajectory,"drone");                                                          // Update environment cell
    return trajectory;
}

bool check_conflicts(const std::vector<double>& previous_waypoint, const std::vector<double>& current_waypoint, const double & segment_start_time, 
    Cells & cells, std::vector<std::vector<double>> & segments, double & max_delay, double & period_a, double & period_b){
    Cell* p0 = cells.get_cell_from_pos(previous_waypoint[0], previous_waypoint[1], previous_waypoint[2]);
    Cell* p1 = cells.get_cell_from_pos(current_waypoint[0], current_waypoint[1], current_waypoint[2]);
    bool no_conflict=true;
    std::vector<Cell* > neighbors;
    neighbors.clear();

    // For astar generated routes, do not need to traverse the cells 
    /*int dx = p1->i - p0->i;
    int dy = p1->j - p0->j;
    int dz = p1->k - p0->k;

    int stepX = dx > 0 ? 1 : -1;
    int stepY = dy > 0 ? 1 : -1;
    int stepZ = dz > 0 ? 1 : -1;

    double tDeltaX = dx == 0 ? INF : stepX / double(dx);
    double tDeltaY = dy == 0 ? INF : stepY / double(dy);
    double tDeltaZ = dz == 0 ? INF : stepZ / double(dz);

    double tMaxX = dx == 0 ? INF : 0;
    double tMaxY = dy == 0 ? INF : 0;
    double tMaxZ = dz == 0 ? INF : 0;

    int c[3] = {p0->i, p0->j, p0->k};
    std::array<int, 3>  c0 = {p0->i, p0->j, p0->k};*/

    std::vector<std::array<int, 3>> paths;
    paths.push_back({p0->i, p0->j, p0->k});

    // Try to only push newly added neighbors to save memory
    // For astar generated routes, do not need to traverse the cells 
    /*while (c[0] != p1->i || c[1] != p1->j || c[2] != p1->k) {
        if (tMaxX < tMaxY) {
            if (tMaxX < tMaxZ) {//X min
                c0[0]=c[0]+int((stepX-1)/2);
                c0[1]=c[1]+int((-stepY-1)/2);
                c0[2]=c[2]+int((-stepZ-1)/2);
                tMaxX += tDeltaX; c[0] += stepX;
            } else if (tMaxX > tMaxZ) {//Z min
                c0[0]=c[0]+int((-stepX-1)/2);
                c0[1]=c[1]+int((-stepY-1)/2);
                c0[2]=c[2]+int((stepZ-1)/2);
                c[2] += stepZ; tMaxZ += tDeltaZ;
            } else {//X & Z min
                c0[0]=c[0]+int((stepX-1)/2);
                c0[1]=c[1]+int((-stepY-1)/2);
                c0[2]=c[2]+int((stepZ-1)/2);
                if(tDeltaX < tDeltaZ){//dX min
                    c[0] += stepX; tMaxX += tDeltaX;
                }else if(tDeltaX > tDeltaZ){//dZ min
                    c[2] += stepZ;tMaxZ += tDeltaZ;
                }else{//dX=dZ
                    c[0] += stepX; tMaxX += tDeltaX;
                    c[2] += stepZ;tMaxZ += tDeltaZ;
                }
            }
        } else if (tMaxX > tMaxY) {
            if (tMaxY < tMaxZ) {//Y min
                c0[0]=c[0]+int((-stepX-1)/2);
                c0[1]=c[1]+int((stepY-1)/2);
                c0[2]=c[2]+int((-stepZ-1)/2);
                c[1] += stepY; tMaxY += tDeltaY;
            } else if (tMaxY > tMaxZ) {//Z min
                c0[0]=c[0]+int((-stepX-1)/2);
                c0[1]=c[1]+int((-stepY-1)/2);
                c0[2]=c[2]+int((stepZ-1)/2);
                c[2] += stepZ; tMaxZ += tDeltaZ;
            } else {//Y&Z min
                c0[0]=c[0]+int((-stepX-1)/2);
                c0[1]=c[1]+int((stepY-1)/2);
                c0[2]=c[2]+int((stepZ-1)/2);
                if(tDeltaY < tDeltaZ){//dY min
                    c[1] += stepY; tMaxY += tDeltaY;
                }else if(tDeltaY > tDeltaZ){//dZ min
                    c[2] += stepZ;tMaxZ += tDeltaZ;
                }else{//dY=dZ
                    c[1] += stepY; tMaxY += tDeltaY;
                    c[2] += stepZ;tMaxZ += tDeltaZ;
                }
            }
        } else {
            if (tMaxX < tMaxZ) {//X&Y min
                c0[0]=c[0]+int((stepX-1)/2);
                c0[1]=c[1]+int((stepY-1)/2);
                c0[2]=c[2]+int((-stepZ-1)/2);
                if(tDeltaX < tDeltaY){//dX min
                    c[0] += stepX; tMaxX += tDeltaX;
                }else if(tDeltaX > tDeltaY){//dY min
                    c[1] += stepY; tMaxY += tDeltaY;
                }else{//dx=dy
                    c[0] += stepX; tMaxX += tDeltaX;
                    c[1] += stepY; tMaxY += tDeltaY;
                }
            } else if (tMaxX > tMaxZ) {//Z min
                c0[0]=c[0]+int((-stepX-1)/2);
                c0[1]=c[1]+int((-stepY-1)/2);
                c0[2]=c[2]+int((stepZ-1)/2);
                c[2] += stepZ; tMaxZ += tDeltaZ;
            } else {//X&Y&Z min
                c0[0]=c[0]+int((stepX-1)/2);
                c0[1]=c[1]+int((stepY-1)/2);
                c0[2]=c[2]+int((stepZ-1)/2);
                if (tDeltaX < tDeltaY) {
                    if (tDeltaX < tDeltaZ) {//dx min
                        c[0] += stepX;
                        tMaxX += tDeltaX;
                    } else if (tDeltaX > tDeltaZ) {//dZ min
                        c[2] += stepZ;
                        tMaxZ += tDeltaZ;
                    } else {//dx=dz min
                        c[0] += stepX;tMaxX += tDeltaX;
                        c[2] += stepZ;tMaxZ += tDeltaZ;
                    }
                } else if (tDeltaX > tDeltaY) {
                    if (tDeltaY < tDeltaZ) {//dy min
                        c[1] += stepY;tMaxY += tDeltaY;
                    } else if (tDeltaY > tDeltaZ) {//dz min
                        c[2] += stepZ; tMaxZ += tDeltaZ;
                    } else {//dy=dz min
                        c[1] += stepY; tMaxY += tDeltaY;
                        c[2] += stepZ; tMaxZ += tDeltaZ;
                    }
                } else {
                    if (tDeltaX < tDeltaZ) {//dx=dy min
                        c[0] += stepX; tMaxX += tDeltaX;
                        c[1] += stepY; tMaxY += tDeltaY;
                    } else if (tDeltaX > tDeltaZ) {//dz min
                        c[2] += stepZ; tMaxZ += tDeltaZ;
                    } else {//dx = dy = dz
                        c[0] += stepX; tMaxX += tDeltaX;
                        c[1] += stepY; tMaxY += tDeltaY;
                        c[2] += stepZ; tMaxZ += tDeltaZ;
                    }
                }
            }
        }

        paths.push_back(c0);
    }*/
    
    // Simplified for astar generated routes, can be removed if not use astar
    paths.push_back({p1->i, p1->j, p1->k});

    // Clear the temporal drones for new storage
    for(auto & c:paths){
        for (int i = std::max(0, c[0] - 1); i < std::min(cells.grid_size[0], c[0] + 2); i++) {
            for (int j = std::max(0, c[1] - 1); j < std::min(cells.grid_size[1], c[1] + 2); j++) {
                for (int k = std::max(0, c[2] - 1); k < std::min(cells.grid_size[2], c[2] + 2); k++) {
                    cells.get_cell(i,j,k)->tem_drone.clear();
                }
            }
        }
    }
    waypoints_update(cells,segments);                                   // Update temporal storage
    
    // Compare the temporal drone and the environment cells to check conflicts
    for(auto & c:paths){
        Cell * center=cells.get_cell(c[0],c[1],c[2]);
        double center_a, center_b, neighbor_a, neighbor_b;
        if(center->tem_drone.empty()){                                  // Though the line between waypoints use the cell, 
            continue;                                                   // the segment points may not use it because of large sampling time
        }
        center_a=center->tem_drone.front().start_time;
        center_b=center->tem_drone.front().end_time;
        // LOG(INFO)<<"read drones in"<<i<<','<<j<<','<<k;
        if(!center->occupied_drones.empty()){               // Drone_occupation
            for(auto & drone : center->occupied_drones){
                //determine the value of a and b
                if(drone.end_time>segment_start_time){//if end_time is after the start time, overlap may happens, record the period
                    neighbor_a=drone.start_time;
                    neighbor_b=drone.end_time;
                    if(neighbor_b+1<center_a or center_b+1<neighbor_a){
                        //no need to delay
                    }else{
                        no_conflict=false;
                        double temp_delay=neighbor_b-center_a;
                        max_delay=max_delay>temp_delay?max_delay:temp_delay;
                        LOG(WARNING)<<"From segment ("<< previous_waypoint[0]<<','<< previous_waypoint[1]<<','<< previous_waypoint[2] << ") to ("
                            << current_waypoint[0]<<','<< current_waypoint[1]<<','<< current_waypoint[2] << ")"
                            <<", conflict at airspace "<< center->x<<','<< center->y <<','<<center->z<<')'<<"at time ["<<center_a<<','<<center_b<<']';
                    }
                    period_a=period_a<neighbor_a?period_a:neighbor_a;   // Get earliest use time for the whole segment
                    period_b=period_b>neighbor_b?period_b:neighbor_b;   // Get last use time for the whole segment
                }
            }
        }
    }
    // LOG(INFO)<<"check conflict finished";
    return no_conflict;
}

void waypoints_update(Cells & cells, const std::vector<std::vector<double>> & segments){
    // TODO: the same with airspace_update(), but for this one and for astar generated routes, we can directly store the segments to each related cells 
    int i,j,k;
    int i1, j1, k1;
    drone_occupation drone;
    drone.drone_id="drone_-1";
    std::tie(i, j, k)=cells.get_index_from_pos(segments.front()[0], segments.front()[1], segments.front()[2]);
    Cell* cell = cells.get_cell(i,j,k);
    drone.start_time = segments[0][3];                                                                  // Global time
    drone.end_time = segments[0][3];
    for(auto & segment:segments){
        std::tie(i1, j1, k1)=cells.get_index_from_pos(segment[0], segment[1], segment[2]);
        if(i!=i1 or j!=j1 or k!=k1){
            i=i1;j=j1;k=k1;
            drone.end_time = segment[3];                                                                // Update time range and then push
            cell->tem_drone.emplace_back(drone);                                                        // Push to current cell
            cell = cells.get_cell(i1,j1,k1);                                                            // Point to next cell
            drone.start_time=segment[3];
            drone.end_time=segment[3];                                                                  // This is the global time, instead of relative flight time
        }
    }
    drone.end_time = segments.back()[3];                                                                // Update time range and then push
    cell->tem_drone.emplace_back(drone);                                                                // Push to current cell
}

void airspace_update(Cells & cells, const std::vector<std::vector<double>>& trajectories, const std::string & drone_id){
    // TODO: In this version, trajectories release the airspace cell after the 'drone point' leaves the airspace cell, but there 
    // can still be conflicts for + intersections, we need to release the airspace cell after the 'drone space' leaves the 
    // airspace cell. 
    // For the astar generated routes, each segment occupies two cells, we can first calculate the time of the segment, then 
    // we can assign the time to both cells. Step 1: split the trajectories based on whether the xyz is integer, then the fragment
    // can be stored to both cell
    int i,j,k;
    int i1, j1, k1;
    drone_occupation drone;
    drone.drone_id=drone_id;
    drone.start_time = trajectories[0][3];
    drone.end_time = trajectories[0][3];

    std::vector<double> last_point = trajectories.front();
    i = -1; j = -1; k =-1;
    uint64_t last_occupy_time = last_point[3];

    for(int iseg = 1; iseg < static_cast<int>(trajectories.size()); iseg++){
        std::vector<double> point = trajectories.at(iseg);
        drone.start_time = last_occupy_time;
        
        for (double a = 0.0; a <= 1; a += 0.001){
            std::vector<double> last_position, cur_pos;
            last_position.push_back(last_point[0]);last_position.push_back(last_point[1]);last_position.push_back(last_point[2]);
            cur_pos.push_back(last_position[0]+ (point[0] - last_position[0]) * a);
            cur_pos.push_back(last_position[1]+ (point[1] - last_position[1]) * a);
            cur_pos.push_back(last_position[2]+ (point[2] - last_position[2]) * a);

            std::tie(i1, j1, k1)=cells.get_index_from_pos(cur_pos[0], cur_pos[1], cur_pos[2]);

            // Note: the two point might be inside one cell, so we need to check whether the cell is changed
            if( (i!=i1) or (j!=j1) or (k!=k1) ){
                int delta_idx = abs(i-i1)+abs(j-j1)+abs(k-k1);
                if ( (i != -1) && delta_idx > 1) {
                }
                i=i1; j=j1; k=k1;
                drone.end_time = point[3];                                                  // Update time range and then push
                Cell* cell = cells.get_cell(i1,j1,k1);                                      // Point to next cell
                if (drone.start_time != drone.end_time){
                    // LOG(INFO)<<"write conflicts in"<<cell->i<<','<<cell->j<<','<<cell->k<<i1<<','<<j1<<','<<k1;
                    cell->occupied_drones.push_back(drone);                                 // Push to current cell
                }
            }
        } 
        last_occupy_time = drone.end_time;
        last_point = point;
    }
}

py::list one_sequential(AStar_conops2 & solver,int route_number,std::vector<std::vector<int32_t>>departures,
std::vector<std::vector<int32_t>> destinations,std::vector<std::vector<double>> center_d,
std::vector<std::vector<double>> center_a,int space_length,int xlim,int ylim,int zlim,double grid_res, std::vector<double> & temp_cost)
{
    py::list norm_route;
    Route p;
    std::vector<std::promise<Route>> promiseObj;
    std::vector<std::future<Route>> futureObj;
    for (size_t i=0; i < departures.size();i++)
    {
        std::promise<Route> promiseOb;
        std::future<Route> futureOb = promiseOb.get_future();
        promiseObj.push_back(std::move(promiseOb));
        futureObj.push_back(std::move(futureOb));
    }
    temp_cost.clear();
    for (size_t i=0; i < departures.size();i++)
    {
        py::list p0 = py::cast(departures.at(i));
        py::list p1 = py::cast(destinations.at(i));
        std::vector<std::vector<int32_t>> protection_zone_cells;
        protection_zone_cells.clear();
        solver.solve(p0,p1,std::ref(promiseObj[i]));
        p = futureObj.at(i).get();
        LOG(INFO)<<"route waypoint size:"<<p.index.size()<<", traversal cost "<<p.traversal<<", turning cost "<<p.turning
        <<", climbing cost "<<p.climbing <<", space cost "<<p.space<<", total cost "<<p.total();
        if(p.total()<1e-5)temp_cost.push_back(INF);
        else temp_cost.push_back(p.total());
        if (p.position.size() > 1 and p.total()!=INF) {                         //add the route information to associated nodes, each node records the nodes out and nodes in
            bool conflict = false;
            for(size_t j=0;j<p.index.size()-1;j++)
            {
                route_protection results = line_traversal(p.index.at(j), p.index.at(j+1), space_length, xlim,ylim,zlim, grid_res);
                protection_zone_cells.insert(protection_zone_cells.end(), results.protection_zone_cell.begin(), results.protection_zone_cell.end());
                for (auto voxel : results.protection_zone_cell) {
                    // 允许路径交叉，但不允许路径旁10米范围内有其他路径
                    if (!solver.get_cell(voxel[0], voxel[1], voxel[2])->occupied_routes.empty() &&
                        !(voxel[0] == p.index[j][0] && voxel[1] == p.index[j][1] && voxel[2] == p.index[j][2]) &&
                        !(voxel[0] == p.index[j+1][0] && voxel[1] == p.index[j+1][1] && voxel[2] == p.index[j+1][2])) {
                        conflict = true;
                        break;
                    }
                }

                if (conflict) {
                    LOG(WARNING) << "Path segment conflict detected, skipping this route.";
                    break;
                }
            }

            if (!conflict) {
                for (size_t j = 0; j < p.index.size() - 1; j++) {
                    solver.add_route_seg(p.index[j][0], p.index[j][1], p.index[j][2], p.index[j + 1][0], p.index[j + 1][1], p.index[j + 1][2]);
                }
                norm_route.append(p.position);
            }
        }
    }
    return norm_route;
}

}