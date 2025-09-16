#include "conops1.h"
#include <glog/logging.h>
namespace conop1{

py::list conops1_route_seq(const array_t<int32_t> & cells_type, const array_t<double> & cells_cost,  const array_t<double> & xs, const array_t<double> & ys,
 const array_t<double> & zs, double  grid_res, double turning_cost_angle, double turning_cost_consecutive, double climb_cost_ascend, double climb_cost_descend,
 double climb_threshold, double space_ccoef, int space_length, bool nonuniform, bool operational, int route_number, const py::list & dep,
 const py::list & dest, const py::list & centerd, const py::list & centera, const int & shuffled, AStar_conops1 & solver) {
    std::vector<std::vector<int32_t>> departures = py::cast<std::vector<std::vector<int32_t>>>(dep);
    std::vector<std::vector<int32_t>> destinations = py::cast<std::vector<std::vector<int32_t>>>(dest);
    std::vector<std::vector<double>> center_d = py::cast<std::vector<std::vector<double>>>(centerd);
    std::vector<std::vector<double>> center_a = py::cast<std::vector<std::vector<double>>>(centera);
    int xlim=cells_type.shape(0);
    int ylim=cells_type.shape(1);
    int zlim=cells_type.shape(2);
    struct timeval t1,t2;
    double timeuse;
    gettimeofday(&t1,NULL);
    std::vector<double> temp_cost;                                                          // Store total cost for each route in every one_sequential()
    temp_cost.clear();
    double min_cost[route_number];
    py::list norm_route,temp_route;
    std::ostringstream oss;
    std::vector<int> sequence;                                                              // To get shuffle indexes
    int protection_length = std::ceil(10/grid_res);
    std::cout << "vertiports_protection_length: " << protection_length << std::endl;
    for(auto dep : departures){                                                             // To avoid conflicts near vertiports, which belongs to terminal airspace
        for(int l=std::max(0,dep.at(0)-protection_length);l<std::min(dep.at(0)+protection_length+1,xlim);l++)
                for(int m=std::max(0,dep.at(1)-protection_length);m<std::min(dep.at(1)+protection_length+1,ylim);m++)
                    for(int n=std::max(0,dep.at(2)-1);n<std::min(dep.at(2)+2,zlim);n++)
                        solver.set_reserved(l,m,n,true);
    }

    if(shuffled==1){                                                                        // not shuffle
        norm_route=one_sequential(solver,route_number,departures,destinations,center_d,center_a,space_length,xlim,ylim,zlim, grid_res, temp_cost);
        float cost=0;
        for(int i=0;i<route_number;i++)cost+=temp_cost.at(i);
        oss.str(""); oss.clear();                                                           // output the costs
        oss<<"total cost for the sequence is "<<cost<<"; each are ";
        for(int i=0;i<route_number;i++) oss<<temp_cost.at(i)<<' ';
        LOG(INFO) << oss.str();
    }else if(shuffled>1){                                                                   // shuffle
        double cost=INF;                                                                    // sum of total cost for all routes
        LOG(INFO)<<"shuffled times:"<<shuffled;
        unsigned seed_min_cost;
        unsigned seed = 1;// std::chrono::system_clock::now().time_since_epoch().count();
        std::default_random_engine engine(seed);

        for(int i=0;i<shuffled;i++){
            LOG(INFO)<<"shuffling..., "<<i<<" times";
            sequence.clear();
            for(int i=0;i<route_number;i++)sequence.push_back(i);
            std::shuffle (sequence.begin(), sequence.end(), engine);//get shuffled route sequences with time-based seed

            std::vector<std::vector<int32_t>> dep_,dest_;           
            std::vector<std::vector<double>> center_d_,center_a_;
            dep_.clear();
            dest_.clear();
            center_d_.clear();
            center_a_.clear();
            for(int i=0;i<route_number;i++){
                dep_.push_back(departures.at(sequence[i]));
                dest_.push_back(destinations.at(sequence[i]));
                center_d_.push_back(center_d.at(sequence[i]));
                center_a_.push_back(center_a.at(sequence[i]));
            }

            //run each one sequential()
            AStar_conops1 copied_solver = solver;                                           // not change the solver itself
            temp_route=one_sequential(copied_solver,route_number,dep_,dest_,center_d_,center_a_,space_length,xlim,ylim,zlim,grid_res, temp_cost);
            double sum_temp_cost=0;                                                         // used to select the minimum one
            for(int i=0;i<route_number;i++){
                if(temp_cost.at(i)==INF)sum_temp_cost+=10000;
                else{sum_temp_cost+=temp_cost.at(i);}
                
            }
            if(cost>sum_temp_cost){
                cost=sum_temp_cost;
                for(int i=0;i<route_number;i++)min_cost[sequence[i]]=temp_cost.at(i);
                norm_route=temp_route;
                seed_min_cost = seed;
            }
        }
        // output the minimum one
        oss.str(""); oss.clear();
        oss<<"minimum total cost for the shuffled sequences: "<<cost<<"; each are ";
        for(int i=0;i<route_number;i++) oss<<min_cost[i]<<' ';
        LOG(INFO) << oss.str();

        // Here we need to update the environment in solver using the minimum cost one, 
        // because we need to update environment if we do not find route during schedulling
        sequence.clear();
        for(int i=0;i<route_number;i++)sequence.push_back(i);
        std::shuffle (sequence.begin(), sequence.end(), std::default_random_engine(seed_min_cost)); //shuffle with minimum cost seed
        std::vector<std::vector<int32_t>> dep_,dest_;
        std::vector<std::vector<double>> center_d_,center_a_;
        dep_.clear();
        dest_.clear();
        center_d_.clear();
        center_a_.clear();
        for(int i=0;i<route_number;i++){
            dep_.push_back(departures.at(sequence[i]));
            dest_.push_back(destinations.at(sequence[i]));
            center_d_.push_back(center_d.at(sequence[i]));
            center_a_.push_back(center_a.at(sequence[i]));
        }
        temp_route=one_sequential(solver,route_number,dep_,dest_,center_d_,center_a_,space_length,xlim,ylim,zlim,grid_res, temp_cost);
    }
    gettimeofday(&t2,NULL);
    timeuse = (t2.tv_sec - t1.tv_sec) + (double)(t2.tv_usec - t1.tv_usec)/1000000.0;
    LOG(INFO)<<"route generation time = "<<timeuse;
    return norm_route;
}


py::list conops1_route_dis(const array_t<int32_t> & cells_type, const array_t<double> & cells_cost,  const array_t<double> & xs, const array_t<double> & ys,
 const array_t<double> & zs, double  grid_res, double turning_cost_angle, double turning_cost_consecutive, double climb_cost_ascend, double climb_cost_descend,
 double climb_threshold, double space_ccoef, int space_length, bool nonuniform, bool operational, int route_number, const py::list & dep,
 const py::list & dest, const py::list & centerd, const py::list & centera, const int & method, AStar_conops1 & solver) {
    struct timeval t1,t2;
    double timeuse;
    std::vector<std::vector<int32_t>> departures = py::cast<std::vector<std::vector<int32_t>>>(dep);            // Type transfer via pybind11
    std::vector<std::vector<int32_t>> destinations = py::cast<std::vector<std::vector<int32_t>>>(dest);
    std::vector<std::vector<double>> center_d = py::cast<std::vector<std::vector<double>>>(centerd);
    std::vector<std::vector<double>> center_a = py::cast<std::vector<std::vector<double>>>(centera);
    std::vector<py::list> p0,p1;
    int xlim=cells_type.shape(0),ylim=cells_type.shape(1),zlim=cells_type.shape(2);
    std::vector<Conflict> conflicts;
    conflicts.clear();
    
    for (int i=0; i < route_number;i++){
        p0.push_back(dep[i]);                                                                                   // OD pairs
        p1.push_back(dest[i]);
    }
    int protection_length = std::ceil(12.5/grid_res);
    for(auto dep : departures){                                                                                 // To avoid conflicts near vertiports, which belongs to terminal airspace
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
    for (int i=0; i < route_number;i++) threads[i] = std::thread(&AStar_conops1::solve, solver, p0.at(i), p1.at(i),std::ref(promiseObj[i]));
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
    write_cells(solver,route_parallel,space_length,xlim,ylim,zlim, grid_res);                                             // Update environment
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
    int max_iteration=30;                                                                                       // If too much iterations, directly fail
    int Num_feasible_routes=1;
    while((Num_feasible_routes<route_number or conflicts.size()>0) and times<=max_iteration)
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
        for (int i=0; i < route_number;i++){                                                                    // Here replans the routes
            threads[i] = std::thread(&ThetaStar1::solve_subspace, solver, route_parallel[i], std::ref(promiseObj[i]), std::ref(promiseconflicts_along_route[i]),
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
                if(method==2){
                    if(gain_temp>gain){ gain=gain_temp; gain_index=i; gain_route=p; }
                }else if(method==3) { gain_indexes.push_back({p,i,gain_temp}); }
            }else if (gain_temp==-INF){
                conflict_routes_local_opts.push_back(conflicts_along_route);
                local_opt_count++;
            }
        }
        if(method==3){                                                                                          // select route stochastically
            if(!gain_indexes.empty()){
                unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
                double t1=0.0,t2=0.0;
                for(auto &agent:gain_indexes)t1+=std::abs(agent.conflict_coefficient);
                std::uniform_real_distribution<double> distribution(0.0,t1);
                std::default_random_engine generator(seed);
                double number = distribution(generator);
                for(auto &agent:gain_indexes){
                    if(t2<=number and t2+std::abs(agent.conflict_coefficient)>number){
                        gain_index=agent.id;
                        gain_route=agent.route;
                        gain=agent.conflict_coefficient;
                    }else t2+=std::abs(agent.conflict_coefficient);
                }
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
        
        if(times>=max_iteration){for (auto &i: valid_routes)norm_route.append(route_parallel[i].route.position);}   // If too many iterations, output valid routes only
    }
    if(conflicts.size()==0){
        double length=0;
        for (int i=0; i < route_number;i++){
            length+=route_parallel[i].route.length;
            norm_route.append(route_parallel[i].route.position);
        }
        LOG(INFO)<<"length: "<<length;
        std::ostringstream oss;
        oss<<"weights:";
        for (auto & route: route_parallel) oss<<' '<<route.conflict_coefficient;
        LOG(INFO)<<oss.str();
    }
    gettimeofday(&t2,NULL);
    timeuse = (t2.tv_sec - t1.tv_sec) + (double)(t2.tv_usec - t1.tv_usec)/1000000.0;
    LOG(INFO)<<"route generation time = "<<timeuse<<'s';
    return norm_route;
}

std::vector<std::vector<double>> trajectory_generation(const std::vector<std::vector<double>> & route, const uint32_t & start_time){
    std::vector<std::vector<double>> trajectory;                                                                            // 1~3: pos; 4: time;
    for (size_t i=0;i<route.size();i++){
        trajectory.push_back({route[i][0],route[i][1],route[i][2],static_cast<double>(start_time+3*i)});
    }
    return trajectory;
}

py::list one_sequential(AStar_conops1 & solver,int route_number,std::vector<std::vector<int32_t>>departures,std::vector<std::vector<int32_t>> destinations,std::vector<std::vector<double>> center_d,
std::vector<std::vector<double>> center_a,int space_length,int xlim,int ylim,int zlim,double grid_res, std::vector<double> & temp_cost)
{
    py::list norm_route;
    Route p;
    std::vector<std::promise<Route>> promiseObj;
    std::vector<std::future<Route>> futureObj;
    std::cout << "one_sequential_grid_res: " << grid_res << std::endl;

    for (size_t i=0; i < departures.size();i++)
    {
        std::promise<Route> promiseOb;
        std::future<Route> futureOb = promiseOb.get_future();
        promiseObj.push_back(std::move(promiseOb));
        futureObj.push_back(std::move(futureOb));
    }
    temp_cost.clear();
    int a=0;
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
        if (p.position.size() > 1 and p.total()!=INF)
        {
            a++;
            for(size_t j=0;j<p.index.size()-1;j++)
            {
                route_protection results = line_traversal(p.index.at(j), p.index.at(j+1), space_length, xlim,ylim,zlim, grid_res);
                protection_zone_cells.insert(protection_zone_cells.end(), results.protection_zone_cell.begin(), results.protection_zone_cell.end());
                for(auto voxel : results.route_cells)
                {
                    if(solver.is_reserved(voxel.at(0),voxel.at(1),voxel.at(2))==false)              // Set route cells as unreachable
                    {solver.set_reachable(voxel.at(0), voxel.at(1), voxel.at(2), false);}
                }
            }
            for(auto voxel : protection_zone_cells)                                                 // Set buffer cells an buffer and unreachable
            {
                if(solver.is_reserved(voxel.at(0),voxel.at(1),voxel.at(2))==false){
                    solver.set_protected(voxel.at(0), voxel.at(1), voxel.at(2), true);
                    solver.set_reachable(voxel.at(0), voxel.at(1), voxel.at(2), false);
                }
            }
            norm_route.append(p.position);
            if(a>=route_number)break;
        }
    }
    return norm_route;
}

}