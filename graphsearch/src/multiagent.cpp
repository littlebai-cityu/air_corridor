#include "multiagent.h"

#include <glog/logging.h>
//first plan route network, then plan trajectories; spatially separated routes
py::list conops1(const array_t<int32_t> & cells_type, const array_t<double> & cells_cost,  const array_t<double> & xs, const array_t<double> & ys,
 const array_t<double> & zs, double  grid_res, double turning_cost_angle, double turning_cost_consecutive, double climb_cost_ascend, double climb_cost_descend,
 double climb_threshold, double space_ccoef, int space_length, bool nonuniform, bool operational, int route_number, const py::list & dep, 
 const py::list & dest, const py::list & centerd, const py::list & centera, const int & shuffled_method, const bool & seq_dis,
  const std::vector<std::tuple<int, std::vector<int32_t>, std::vector<int32_t>>> & orders, const bool & sparse, const bool & hub_to_hub, const int & order,
  const int & aggregated_demands_percent){
    /******************                  init log                           ******************/
    FLAGS_alsologtostderr = true;
    FLAGS_colorlogtostderr = true;
    FLAGS_logbufsecs = 0;
    FLAGS_max_log_size = 100;
    FLAGS_stop_logging_if_full_disk = true;
    FLAGS_v = 1;
    std::string type="conops1";
    if(sparse)type+="_sparse_";else type+="_dense_";
    if(hub_to_hub)type+="hub_to_hub";else type+="hub_to_destination";
    type+="order";type+=std::to_string(order); type+="_percent"; type+=std::to_string(aggregated_demands_percent);
    google::InitGoogleLogging(type.c_str());
    google::SetLogDestination(google::GLOG_INFO, "/home/shuangxia/conops/graphsearch/src/log/conops1_4d/info");
    google::SetLogDestination(google::GLOG_WARNING, "/home/shuangxia/conops/graphsearch/src/log/conops1_4d/warning");
    struct timeval t1,t2,t3;
    double timeuse;
    gettimeofday(&t1,NULL);
    LOG(INFO) << "Start ConOps1";
    LOG(INFO) << "Start route generation";
    conop1::AStar_conops1 solver(cells_type, cells_cost, xs, ys, zs, grid_res, turning_cost_angle, turning_cost_consecutive, 
    climb_cost_ascend, climb_cost_descend, climb_threshold, space_ccoef, space_length, nonuniform, operational);

    /******************     first plan route network, spatially separated   ******************/
    //TODO: multiple routes for selection for each vertiport OD pair
    //TODO: for hub to destinations, 
    std::cout << "seq_dis: " << seq_dis << std::endl;
    py::list conops1_routes, conops1_trajectories, conops1_results;
    if(seq_dis==0){//seq
        conops1_routes = conop1::conops1_route_seq(cells_type,cells_cost,xs,ys,zs, grid_res, turning_cost_angle,turning_cost_consecutive,climb_cost_ascend,climb_cost_descend,
        climb_threshold, space_ccoef, space_length, nonuniform, operational, route_number, dep, dest, centerd, centera, shuffled_method, solver);
    }else{
        conops1_routes = conop1::conops1_route_dis(cells_type,cells_cost,xs,ys,zs, grid_res, turning_cost_angle,turning_cost_consecutive,climb_cost_ascend,climb_cost_descend,
        climb_threshold, space_ccoef, space_length, nonuniform, operational, route_number, dep, dest, centerd, centera, shuffled_method, solver);
    }
    LOG(INFO) << "route generation finished";
    gettimeofday(&t2,NULL);
    timeuse = (t2.tv_sec - t1.tv_sec) + (double)(t2.tv_usec - t1.tv_usec)/1000000.0;
    LOG(INFO)<<"conops 1 route generation use time = "<<timeuse;

    /******************          generate route map for mapping             ******************/
    LOG(INFO)<<"generating route map";
    std::vector<std::vector<std::vector<double>>> routes= py::cast<std::vector<std::vector<std::vector<double>>>>(conops1_routes);
    if(static_cast<int>(routes.size())<route_number){
        LOG(WARNING)<<"fail to generate all routes, only generate "<<routes.size() << " routes";
    }
    std::unordered_map<std::string, std::vector<std::vector<double>>> route_map;
    std::unordered_map<std::string, int> route_current_time;
    for(auto & route:routes){
        std::vector<double> origin=route.front();
        std::vector<double> destination=route.back();
        std::string key = "";
        for (double & o : origin) {key+=std::to_string(static_cast<int>(o));key+=",";}
        for (double & d : destination) {key+=std::to_string(static_cast<int>(d));key+=",";}
        route_map[key]=route;
        route_current_time[key]=0;
    }
    LOG(INFO)<<"route map generation finished.";
    /******************                 generate trajectories               ******************/
    LOG(INFO) << "Start trajectory generation";
    for(auto & order : orders){                                             // In the format (time, ori, dest)
        int start_time = ceil(std::get<0>(order)/3)*3;                      // To be times of 3 second
        LOG(INFO)<<"start_time"<<start_time;
        std::vector<int32_t> origin = std::get<1>(order);
        std::vector<int32_t> destination = std::get<2>(order);
        int protection_length = std::ceil(12.5/grid_res);
        std::cout << "vertiports_protection_length: " << protection_length << std::endl;                                                          // To avoid conflicts near vertiports, which belongs to terminal airspace
        int xlim=cells_type.shape(0);
        int ylim=cells_type.shape(1);
        int zlim=cells_type.shape(2);
        for(int l=std::max(0,origin.at(0)-protection_length);l<std::min(origin.at(0)+protection_length+1,xlim);l++)
                for(int m=std::max(0,origin.at(1)-protection_length);m<std::min(origin.at(1)+protection_length+1,ylim);m++)
                    for(int n=std::max(0,origin.at(2)-1);n<std::min(origin.at(2)+2,zlim);n++)
                        solver.set_reserved(l,m,n,true);

        std::string key = "";
        for (int32_t & o : origin) {key+=std::to_string(o);key+=",";}
        for (int32_t & d : destination) {key+=std::to_string(d);key+=",";}
        if (route_map.find(key) != route_map.end()){                        // map the route for sorting and searching
            std::vector<std::vector<double>> & route = route_map[key];
            if(route_current_time[key] >= start_time) {
                start_time = route_current_time[key]+ 3;  // check whether tube is crowded, if crowded, then delay 3 sec
                // LOG()
            }
            route_current_time[key] = start_time;
            conops1_trajectories.append(conop1::trajectory_generation(route,start_time));
        }else{                                                              // route is not pre-generated, need to plan a new one
            LOG(WARNING)<<"cannot find route from the routes,generate the new route now";
            std::promise<Route> promiseOb;
            std::future<Route> futureOb = promiseOb.get_future();
            std::vector<std::vector<int32_t>> protection_zone_cells;
            protection_zone_cells.clear();
            py::list p0 = py::cast(origin);
            py::list p1 = py::cast(destination);
            std::cout << p0 <<","<<p1 << std::endl;
            solver.solve(p0,p1,std::ref(promiseOb));
            Route p = futureOb.get();
            if (p.position.size() > 1 and p.total()!=INF and p.total()>=1e-5){
                for(size_t j=0;j<p.index.size()-1;j++)
                {
                    conop1::route_protection results = conop1::line_traversal(p.index.at(j), p.index.at(j+1), space_length, cells_type.shape(0),cells_type.shape(1),cells_type.shape(2), grid_res);
                    protection_zone_cells.insert(protection_zone_cells.end(), results.protection_zone_cell.begin(), results.protection_zone_cell.end());
                    for(auto voxel : results.route_cells)
                    {
                        if(solver.is_reserved(voxel.at(0),voxel.at(1),voxel.at(2))==false)  // set route cells as unreachable
                        {solver.set_reachable(voxel.at(0), voxel.at(1), voxel.at(2), false);}
                    }
                }
                for(auto voxel : protection_zone_cells)
                {
                    if(solver.is_reserved(voxel.at(0),voxel.at(1),voxel.at(2))==false){
                        solver.set_protected(voxel.at(0), voxel.at(1), voxel.at(2), true);
                        solver.set_reachable(voxel.at(0), voxel.at(1), voxel.at(2), false);
                    }
                }
                conops1_routes.append(p.position);
                LOG(INFO)<<"new route generated";
                route_map[key]=p.position;                                                  // Map the route for sorting and searching
                route_current_time[key] = 0;
                if(route_current_time[key] >= start_time) start_time = route_current_time[key]+ 3;  // check whether tube is crowded, if crowded, then delay 3 sec
                route_current_time[key] = start_time;
                conops1_trajectories.append(conop1::trajectory_generation(p.position,start_time));
            }else{
                LOG(WARNING)<<"failed to generate new route";
            }
        }
    }
    conops1_results.append(conops1_routes);
    conops1_results.append(conops1_trajectories);
    LOG(INFO) << "trajectory generation finished";
    gettimeofday(&t3,NULL);
    timeuse = (t3.tv_sec - t2.tv_sec) + (double)(t3.tv_usec - t2.tv_usec)/1000000.0;
    LOG(INFO)<<"conops 1 trajectory generation use time = "<<timeuse;
    timeuse = (t3.tv_sec - t1.tv_sec) + (double)(t3.tv_usec - t1.tv_usec)/1000000.0;
    LOG(INFO)<<"conops 1 use time = "<<timeuse;
    google::ShutdownGoogleLogging();
    return conops1_results;
}

//first plan route network, then plan trajectories; intersected routes
py::list conops2(const array_t<int32_t> & cells_type, const array_t<double> & cells_cost,  const array_t<double> & xs, const array_t<double> & ys, 
 const array_t<double> & zs, double  grid_res, double turning_cost_angle, double turning_cost_consecutive, double climb_cost_ascend, double climb_cost_descend, 
 double climb_threshold, double space_ccoef, int space_length, bool nonuniform, bool operational, int route_number, const py::list & dep, 
 const py::list & dest, const py::list & centerd, const py::list & centera, const int & shuffled_method, const bool & seq_dis, 
 const std::vector<std::tuple<int, std::vector<int32_t>, std::vector<int32_t>>> & orders, const bool & sparse, const bool & hub_to_hub, const int & order,
 const int & aggregated_demands_percent,
 int order_route_number, const py::list & order_dep,  const py::list & order_dest, const py::list & order_centerd, 
 const py::list & order_centera, const int & time_step, const int & window_size, array_t<int32_t> & updated_cells_type){
    
    /******************init log******************/
    FLAGS_alsologtostderr = true;
    FLAGS_colorlogtostderr = true;
    FLAGS_logbufsecs = 0;
    FLAGS_max_log_size = 100;
    FLAGS_stop_logging_if_full_disk = true;
    FLAGS_v = 1;
    std::string type="conops2";
    if(sparse)type+="_sparse_";else type+="_dense_";
    if(hub_to_hub)type+="hub_to_hub";else type+="hub_to_destination";
    type+="order";type+=std::to_string(order);type+="_percent"; type+=std::to_string(aggregated_demands_percent);
    google::InitGoogleLogging(type.c_str());
    google::SetLogDestination(google::GLOG_INFO, "/home/shuangxia/conops/graphsearch/src/log/conops2_4d/info");
    google::SetLogDestination(google::GLOG_WARNING, "/home/shuangxia/conops/graphsearch/src/log/conops2_4d/warning");
    struct timeval t1,t2,t3;
    double timeuse;
    gettimeofday(&t1,NULL);
    LOG(INFO) << "Start ConOps2";
    LOG(INFO) << "Start route generation";
    int xlim=cells_type.shape(0);
    int ylim=cells_type.shape(1);
    int zlim=cells_type.shape(2);
    conop2::AStar_conops2 solver(cells_type, cells_cost, xs, ys, zs, grid_res, turning_cost_angle, turning_cost_consecutive, 
    climb_cost_ascend, climb_cost_descend, climb_threshold, space_ccoef, space_length, nonuniform, operational);
    //first plan route network, intersected
    py::list conops2_routes, conops2_trajectories, conops2_results;

    if(seq_dis==0){                                                                                     //seq, generate routes sequentially with spatial conflicts exists
        conops2_routes = conop2::conops2_route_seq(cells_type,cells_cost,xs,ys,zs, grid_res, turning_cost_angle,turning_cost_consecutive,climb_cost_ascend,climb_cost_descend,
        climb_threshold, space_ccoef, space_length, nonuniform, operational, route_number, dep, dest, centerd, centera, shuffled_method, solver);
    }else{
        conops2_routes = conop2::conops2_route_dis(cells_type,cells_cost,xs,ys,zs, grid_res, turning_cost_angle,turning_cost_consecutive,climb_cost_ascend,climb_cost_descend,
        climb_threshold, space_ccoef, space_length, nonuniform, operational, route_number, dep, dest, centerd, centera, shuffled_method, solver);
    }
    LOG(INFO) << "route generation finished";
    gettimeofday(&t2,NULL);
    timeuse = (t2.tv_sec - t1.tv_sec) + (double)(t2.tv_usec - t1.tv_usec)/1000000.0;
    LOG(INFO)<<"conops 2 route generation use time = "<<timeuse;
    
    std::vector<std::vector<std::vector<double>>> routes= py::cast<std::vector<std::vector<std::vector<double>>>>(conops2_routes);
    if(static_cast<int>(routes.size())<route_number){
        LOG(WARNING)<<"fail to generate all routes, only generate "<<routes.size() << " routes";
    }
    for(auto& route:routes){
        std::ostringstream oss;
        for(auto waypoint: route)oss<<"["<<waypoint[0]<<','<<waypoint[1]<<','<<waypoint[2]<<"] ";
        LOG(INFO)<<oss.str();
    }
    /******************generate trajectories******************/
    LOG(INFO)<<"generating route map";
    // std::vector<std::vector<std::vector<double>>> routes= py::cast<std::vector<std::vector<std::vector<double>>>>(routes);
    if(static_cast<int>(routes.size())<route_number){
        LOG(WARNING)<<"fail to generate all routes, only generate "<<routes.size() << " routes";
    }
    std::unordered_map<std::string, std::vector<std::vector<double>>> route_map;
    std::unordered_map<std::string, int> route_current_time;
    for(auto & route:routes){
        std::vector<double> origin=route.front();
        std::vector<double> destination=route.back();
        std::string key = "";
        for (double & o : origin) {key+=std::to_string(static_cast<int>(o));key+=",";}
        for (double & d : destination) {key+=std::to_string(static_cast<int>(d));key+=",";}
        route_map[key]=route;
        route_current_time[key]=0;
    }
    LOG(INFO)<<"route map generation finished.";
    LOG(INFO) << "Start trajectory generation";
    for(auto & order : orders){                                             // In the format (time, ori, dest)
        int start_time = ceil(std::get<0>(order)/3)*3;                      // To be times of 3 second
        LOG(INFO)<<"start_time"<<start_time;
        std::vector<int32_t> origin = std::get<1>(order);
        std::vector<int32_t> destination = std::get<2>(order);
        std::string key = "";
        for (int32_t & o : origin) {key+=std::to_string(o);key+=",";}
        for (int32_t & d : destination) {key+=std::to_string(d);key+=",";}
        if (route_map.find(key) == route_map.end()){                        // map the route for sorting and searching
            // std::vector<std::vector<double>> & route = route_map[key];                                                     // route is not pre-generated, need to plan a new one
            LOG(WARNING)<<"cannot find route from the routes,generate the new route now";
            std::promise<Route> promiseOb;
            std::future<Route> futureOb = promiseOb.get_future();
            py::list p0 = py::cast(origin);
            py::list p1 = py::cast(destination);
            solver.solve(p0,p1,std::ref(promiseOb));
            Route p = futureOb.get();
            if (p.position.size() > 1 and p.total()!=INF and p.total()>=1e-5){
                for(size_t j=0;j<p.index.size()-1;j++)
                {
                    solver.add_route_seg(p.index[j][0],p.index[j][1],p.index[j][2],p.index[j+1][0],p.index[j+1][1],p.index[j+1][2]);
                }
                routes.push_back(p.position);
                LOG(INFO)<<"new route generated";
                route_map[key]=p.position;                                                  // Map the route for sorting and searching
            }else{
                LOG(WARNING)<<"failed to generate new route";
            }
        }
    }
    // Step 1: adjust the cell_type info, set routes to be 0 and other areas to be -1
    size_t size1 = updated_cells_type.shape(0);
    size_t size2 = updated_cells_type.shape(1);
    size_t size3 = updated_cells_type.shape(2);
    size_t stride_i = size2 * size3;
    size_t stride_j = size3;

    auto buf = updated_cells_type.request();                                    // get pointer to adjust value
    int* ptr = static_cast<int*>(buf.ptr);

    for (size_t i = 0; i < size1; i++) {                                        // change the whole areas to be -1        
        for (size_t j = 0; j < size2; j++) {
            for (size_t k = 0; k < size3; k++) {
                size_t ind =i * stride_i + j * stride_j + k;
                ptr[ind] = -1;
            }
        }
    }
    // int protection_length = std::ceil(5/grid_res);
    // for (auto& route : routes) { // change routes to be 0, so traj can only follow routes
    //     for (auto waypoint : route) {
    //         for (int l = std::max(0, static_cast<int>(waypoint[0])) - protection_length; l < std::min(static_cast<int>(waypoint[0]) + protection_length + 1, xlim); l++) {
    //             for (int m = std::max(0, static_cast<int>(waypoint[1])) - protection_length; m < std::min(static_cast<int>(waypoint[1]) + protection_length + 1, ylim); m++) {
    //                 for (int n = std::max(0, static_cast<int>(waypoint[2])) - protection_length; n < std::min(static_cast<int>(waypoint[2]) + protection_length + 1, zlim); n++) {
    //                     int ind = l * stride_i + m * stride_j + n;
    //                     ptr[ind] = 0;
    //                     std::cout << l <<m << n <<std::endl;
    //                 }
    //             }
    //         }
    //     }
    // }
    for(auto& route:routes){                                                    // change routes to be 0, so traj can only follow routes
        for(auto waypoint: route){
            int i = static_cast<int>(waypoint[0]);
            int j = static_cast<int>(waypoint[1]);
            int k = static_cast<int>(waypoint[2]);
            int ind =i * stride_i + j * stride_j + k;
            ptr[ind] = 0;
        }
    }
    LOG(INFO)<<"finished update the cells using planned routes";

    // Step 2: use concop 3 to plan trajectories
    conops2_routes = conop3::conops3_route_seq(updated_cells_type,cells_cost,xs,ys,zs, grid_res, turning_cost_angle,turning_cost_consecutive,climb_cost_ascend,climb_cost_descend,
        climb_threshold, space_ccoef, space_length, nonuniform, operational, order_route_number, order_dep, order_dest, order_centerd, order_centera, 0, window_size, false);

    routes= py::cast<std::vector<std::vector<std::vector<double>>>>(conops2_routes);
    std::vector<std::vector<std::vector<double>>> clean_routes;                         // get routes without any repeatation
    for(size_t j=0;j<routes.size();j++){
        std::vector<std::vector<double>> route = routes[j];
        clean_routes.push_back(route);
    }

    for(auto & route : clean_routes){
        for(auto & waypoint : route)waypoint[3]=time_step * waypoint[3];            // relative time, from [timestep s] to [s] unit
    }

    for(size_t i=0;i<clean_routes.size();i++){
        auto & route = clean_routes[i];
        conops2_trajectories.append(route);
    }
    
    conops2_results.append(conops2_routes);
    conops2_results.append(conops2_trajectories);
    LOG(INFO) << "trajectory generation finished";
    
    gettimeofday(&t3,NULL);
    timeuse = (t3.tv_sec - t2.tv_sec) + (double)(t3.tv_usec - t2.tv_usec)/1000000.0;
    LOG(INFO)<<"conops 2 trajectory generation use time = "<<timeuse;
    timeuse = (t3.tv_sec - t1.tv_sec) + (double)(t3.tv_usec - t1.tv_usec)/1000000.0;
    LOG(INFO)<<"conops 2 use time = "<<timeuse;
    google::ShutdownGoogleLogging();
    return conops2_results;
}

//plan route network and trajectory simutaneously
py::list conops3(const array_t<int32_t> & cells_type, const array_t<double> & cells_cost,  const array_t<double> & xs, const array_t<double> & ys, 
 const array_t<double> & zs, double  grid_res, double turning_cost_angle, double turning_cost_consecutive, double climb_cost_ascend, double climb_cost_descend, 
 double climb_threshold, double space_ccoef, int space_length, bool nonuniform, bool operational, int route_number, const py::list & dep, 
 const py::list & dest, const py::list & centerd, const py::list & centera, const int & shuffled_method, const bool & seq_dis,
 const int & time_step, const int & window_size, const bool & sparse, const bool & hub_to_hub, const int & order,
 const int & aggregated_demands_percent){
    /******************init log******************/
    FLAGS_alsologtostderr = true;
    FLAGS_colorlogtostderr = true;
    FLAGS_logbufsecs = 0;
    FLAGS_max_log_size = 100;
    FLAGS_stop_logging_if_full_disk = true;
    FLAGS_v = 1;
    std::string type="conops3";
    if(sparse)type+="_sparse_";else type+="_dense_";
    if(hub_to_hub)type+="hub_to_hub";else type+="hub_to_destination";
    type+="order";type+=std::to_string(order);type+="_percent"; type+=std::to_string(aggregated_demands_percent);
    google::InitGoogleLogging(type.c_str());
    google::SetLogDestination(google::GLOG_INFO, "/home/shuangxia/conops/graphsearch/src/log/conops3_4d/info");
    google::SetLogDestination(google::GLOG_WARNING, "/home/shuangxia/conops/graphsearch/src/log/conops3_4d/warning");
    struct timeval t1,t2;
    double timeuse;
    gettimeofday(&t1,NULL);
    LOG(INFO) << "Start ConOps3";
    LOG(INFO) << "Start route generation";
    py::list conops3_routes, conops3_trajectories, conops3_results;
    if(seq_dis==0){//seq, generate routes sequentially with spatial conflicts exists
        conops3_routes = conop3::conops3_route_seq(cells_type,cells_cost,xs,ys,zs,grid_res, turning_cost_angle,turning_cost_consecutive,climb_cost_ascend,climb_cost_descend,
        climb_threshold, space_ccoef, space_length, nonuniform, operational, route_number, dep, dest, centerd, centera, shuffled_method, window_size, true);
    }else{
        conops3_routes = conop3::conops3_route_dis(cells_type,cells_cost,xs,ys,zs,grid_res, turning_cost_angle,turning_cost_consecutive,climb_cost_ascend,climb_cost_descend,
        climb_threshold, space_ccoef, space_length, nonuniform, operational, route_number, dep, dest, centerd, centera, shuffled_method, window_size);
    }
    std::vector<std::vector<std::vector<double>>> routes= py::cast<std::vector<std::vector<std::vector<double>>>>(conops3_routes);
    LOG(INFO) << "route generation finished";
    
    if(static_cast<int>(routes.size())<route_number){
        LOG(ERROR)<<"fail to generate all routes, only generate "<<routes.size() << " routes";
    }
    std::vector<std::vector<std::vector<double>>> clean_routes;                         // get routes without any repeatation
    for(size_t j=0;j<routes.size();j++){
        std::vector<std::vector<double>> route = routes[j];
        clean_routes.push_back(route);
    }

    for(auto & route : clean_routes){
        for(auto & waypoint : route)waypoint[3]=time_step * waypoint[3];            // relative time, from [timestep s] to [s] unit
    }

    for(auto& route:clean_routes){
        std::ostringstream oss;
        for(auto waypoint: route)oss<<"["<<waypoint[0]<<','<<waypoint[1]<<','<<waypoint[2]<<','<<waypoint[3]<<"] ";
        LOG(INFO)<<oss.str();
    }
    conops3_results.append(clean_routes);
    
    //then plan trajectories for each demand
    LOG(INFO) << "Start trajectory generation";
    for(size_t i=0;i<clean_routes.size();i++){
        auto & route = clean_routes[i];
        conops3_trajectories.append(route);
    }
    conops3_results.append(conops3_trajectories);
    LOG(INFO) << "trajectory generation finished";
    gettimeofday(&t2,NULL);
    timeuse = (t2.tv_sec - t1.tv_sec) + (double)(t2.tv_usec - t1.tv_usec)/1000000.0;
    LOG(INFO)<<"conops 3 use time = "<<timeuse;
    google::ShutdownGoogleLogging();
    return conops3_results;
}

py::list conops4(const array_t<int32_t> & cells_type, const array_t<double> & cells_cost,  const array_t<double> & xs, const array_t<double> & ys, 
 const array_t<double> & zs, double  grid_res, double turning_cost_angle, double turning_cost_consecutive, double climb_cost_ascend, double climb_cost_descend, 
 double climb_threshold, double space_ccoef, int space_length, bool nonuniform, bool operational, int route_number, const py::list & dep, 
 const py::list & dest, const py::list & centerd, const py::list & centera, const int & shuffled_method, const bool & seq_dis, 
 const std::vector<std::tuple<int, std::vector<int32_t>, std::vector<int32_t>>> & orders, const bool & sparse, const bool & hub_to_hub, const int & order,
 const int & aggregated_demands_percent,
 int order_route_number, const py::list & order_dep,  const py::list & order_dest, const py::list & order_centerd, 
 const py::list & order_centera, const int & time_step, const int & window_size, array_t<int32_t> & updated_cells_type){
    
    /******************init log******************/
    FLAGS_alsologtostderr = true;
    FLAGS_colorlogtostderr = true;
    FLAGS_logbufsecs = 0;
    FLAGS_max_log_size = 100;
    FLAGS_stop_logging_if_full_disk = true;
    FLAGS_v = 1;
    google::InitGoogleLogging("20v");
    google::SetLogDestination(google::GLOG_INFO, "/home/shuangxia/conops/graphsearch/src/log/info");
    google::SetLogDestination(google::GLOG_WARNING, "/home/shuangxia/conops/graphsearch/src/log/warning");
    LOG(INFO) << "Start ConOps4";
    LOG(INFO) << "Start route generation";
    conop1::AStar_conops1 solver(cells_type, cells_cost, xs, ys, zs, grid_res, turning_cost_angle, turning_cost_consecutive, 
    climb_cost_ascend, climb_cost_descend, climb_threshold, space_ccoef, space_length, nonuniform, operational);
    //first plan route network, intersected
    py::list conops4_routes, conops4_results;                                                                                 //seq, generate routes sequentially with spatial conflicts exists
    py::list norm_route;
    Route p;
    std::vector<std::promise<Route>> promiseObj;
    std::vector<std::future<Route>> futureObj;
    std::vector<std::vector<int32_t>> departures = py::cast<std::vector<std::vector<int32_t>>>(dep);
    std::vector<std::vector<int32_t>> destinations = py::cast<std::vector<std::vector<int32_t>>>(dest);
    for (size_t i=0; i < departures.size();i++)
    {
        std::promise<Route> promiseOb;
        std::future<Route> futureOb = promiseOb.get_future();
        promiseObj.push_back(std::move(promiseOb));
        futureObj.push_back(std::move(futureOb));
    }
    int a=0;
    for (size_t i=0; i < departures.size();i++)
    {   
        py::list p0 = py::cast(departures.at(i));
        py::list p1 = py::cast(destinations.at(i));
        std::vector<std::vector<int32_t>> protection_zone_cells;
        protection_zone_cells.clear();
        solver.solve(p0,p1,std::ref(promiseObj[i]));
        p = futureObj.at(i).get();
        if (p.position.size() > 1 and p.total()!=INF)
        {
            norm_route.append(p.position);
            if(a>=route_number)break;
        }
    }
    conops4_results.append(norm_route);
    conops4_results.append(norm_route);
    return conops4_results;
 }

PYBIND11_MODULE(Graphsearch, m) {
m.def("conops1", &conops1);
m.def("conops2", &conops2);
m.def("conops3", &conops3);
m.def("conops4", &conops4);
}
