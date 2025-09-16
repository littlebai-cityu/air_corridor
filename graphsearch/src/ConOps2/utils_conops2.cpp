#include "utils_conops2.h"
#include <random>
#include <glog/logging.h>
namespace conop2{

route_protection line_traversal(const std::vector<int32_t>& p0, const std::vector<int32_t>& p1, const int& space_length, 
const int& xlim, const int& ylim, const int& zlim, const double& grid_res)
{
    route_protection results;
    results.route_cells.clear();
    results.protection_zone_cell.clear();
    int dx = p1[0] - p0[0];
    int dy = p1[1] - p0[1];
    int dz = p1[2] - p0[2];

    int stepX = dx >= 0 ? 1 : -1;
    int stepY = dy >= 0 ? 1 : -1;
    int stepZ = dz >= 0 ? 1 : -1;

    double tDeltaX = dx == 0 ? INF : stepX / double(dx);
    double tDeltaY = dy == 0 ? INF : stepY / double(dy);
    double tDeltaZ = dz == 0 ? INF : stepZ / double(dz);

    double tMaxX = dx == 0 ? INF : 0.5 * stepX / double(dx);
    double tMaxY = dy == 0 ? INF : 0.5 * stepY / double(dy);
    double tMaxZ = dz == 0 ? INF : 0.5 * stepZ / double(dz);

    int c0[3] = {p0[0], p0[1], p0[2]};
    results.route_cells.push_back({p0[0], p0[1], p0[2]});
    
    // 计算保护区的长度
    int protection_length = std::ceil(5.0 / grid_res);
    // std::cout << "protection_length: " << protection_length << std::endl;
    // std::cout << "space_length: " << space_length << std::endl;

    // 初始点的保护区
    for (int i = std::max(0, c0[0] - protection_length); i < std::min(c0[0] + protection_length + 1, xlim); i++) {
        for (int j = std::max(0, c0[1] - protection_length); j < std::min(c0[1] + protection_length + 1, ylim); j++) {
            for (int k = std::max(0, c0[2] - protection_length); k < std::min(c0[2] + protection_length + 1, zlim); k++) {
                results.protection_zone_cell.push_back({i, j, k});
            }
        }
    }

    // 遍历路径
    while (c0[0] != p1[0] || c0[1] != p1[1] || c0[2] != p1[2]) {
        if (tMaxX < tMaxY) {
            if (tMaxX < tMaxZ) {
                c0[0] += stepX; tMaxX += tDeltaX;
                int i = -1;
                if (stepX > 0 && (c0[0] + protection_length) < xlim) i = c0[0] + protection_length;
                if (stepX < 0 && (c0[0] - protection_length) >= 0) i = c0[0] - protection_length;
                if (i != -1) {
                    for (int j = std::max(0, c0[1] - protection_length); j < std::min(c0[1] + protection_length + 1, ylim); j++) {
                        for (int k = std::max(0, c0[2] - protection_length); k < std::min(c0[2] + protection_length + 1, zlim); k++) {
                            results.protection_zone_cell.push_back({i, j, k});
                        }
                    }
                }
            } else if (tMaxX > tMaxZ) {
                c0[2] += stepZ; tMaxZ += tDeltaZ;
                int k = -1;
                if (stepZ > 0 && (c0[2] + protection_length) < zlim) k = c0[2] + protection_length;
                if (stepZ < 0 && (c0[2] - protection_length) >= 0) k = c0[2] - protection_length;
                if (k != -1) {
                    for (int i = std::max(0, c0[0] - protection_length); i < std::min(c0[0] + protection_length + 1, xlim); i++) {
                        for (int j = std::max(0, c0[1] - protection_length); j < std::min(c0[1] + protection_length + 1, ylim); j++) {
                            results.protection_zone_cell.push_back({i, j, k});
                        }
                    }
                }
            } else {
                c0[0] += stepX; tMaxX += tDeltaX;
                c0[2] += stepZ; tMaxZ += tDeltaZ;
                for (int i = std::max(0, c0[0] - protection_length); i < std::min(c0[0] + protection_length + 1, xlim); i++) {
                    for (int j = std::max(0, c0[1] - protection_length); j < std::min(c0[1] + protection_length + 1, ylim); j++) {
                        for (int k = std::max(0, c0[2] - protection_length); k < std::min(c0[2] + protection_length + 1, zlim); k++) {
                            results.protection_zone_cell.push_back({i, j, k});
                        }
                    }
                }
            }
        } else if (tMaxX > tMaxY) {
            if (tMaxY < tMaxZ) {
                c0[1] += stepY; tMaxY += tDeltaY;
                int j = -1;
                if (stepY > 0 && (c0[1] + protection_length) < ylim) j = c0[1] + protection_length;
                if (stepY < 0 && (c0[1] - protection_length) >= 0) j = c0[1] - protection_length;
                if (j != -1) {
                    for (int i = std::max(0, c0[0] - protection_length); i < std::min(c0[0] + protection_length + 1, xlim); i++) {
                        for (int k = std::max(0, c0[2] - protection_length); k < std::min(c0[2] + protection_length + 1, zlim); k++) {
                            results.protection_zone_cell.push_back({i, j, k});
                        }
                    }
                }
            } else if (tMaxY > tMaxZ) {
                c0[2] += stepZ; tMaxZ += tDeltaZ;
                int k = -1;
                if (stepZ > 0 && (c0[2] + protection_length) < zlim) k = c0[2] + protection_length;
                if (stepZ < 0 && (c0[2] - protection_length) >= 0) k = c0[2] - protection_length;
                if (k != -1) {
                    for (int i = std::max(0, c0[0] - protection_length); i < std::min(c0[0] + protection_length + 1, xlim); i++) {
                        for (int j = std::max(0, c0[1] - protection_length); j < std::min(c0[1] + protection_length + 1, ylim); j++) {
                            results.protection_zone_cell.push_back({i, j, k});
                        }
                    }
                }
            } else {
                c0[1] += stepY; tMaxY += tDeltaY;
                c0[2] += stepZ; tMaxZ += tDeltaZ;
                for (int i = std::max(0, c0[0] - protection_length); i < std::min(c0[0] + protection_length + 1, xlim); i++) {
                    for (int j = std::max(0, c0[1] - protection_length); j < std::min(c0[1] + protection_length + 1, ylim); j++) {
                        for (int k = std::max(0, c0[2] - protection_length); k < std::min(c0[2] + protection_length + 1, zlim); k++) {
                            results.protection_zone_cell.push_back({i, j, k});
                        }
                    }
                }
            }
        } else {
            if (tMaxX < tMaxZ) {
                c0[0] += stepX; tMaxX += tDeltaX;
                c0[1] += stepY; tMaxY += tDeltaY;
                for (int i = std::max(0, c0[0] - protection_length); i < std::min(c0[0] + protection_length + 1, xlim); i++) {
                    for (int j = std::max(0, c0[1] - protection_length); j < std::min(c0[1] + protection_length + 1, ylim); j++) {
                        for (int k = std::max(0, c0[2] - protection_length); k < std::min(c0[2] + protection_length + 1, zlim); k++) {
                            results.protection_zone_cell.push_back({i, j, k});
                        }
                    }
                }
            } else if (tMaxX > tMaxZ) {
                c0[2] += stepZ; tMaxZ += tDeltaZ;
                int k = -1;
                if (stepZ > 0 && (c0[2] + protection_length) < zlim) k = c0[2] + protection_length;
                if (stepZ < 0 && (c0[2] - protection_length) >= 0) k = c0[2] - protection_length;
                if (k != -1) {
                    for (int i = std::max(0, c0[0] - protection_length); i < std::min(c0[0] + protection_length + 1, xlim); i++) {
                        for (int j = std::max(0, c0[1] - protection_length); j < std::min(c0[1] + protection_length + 1, ylim); j++) {
                            results.protection_zone_cell.push_back({i, j, k});
                        }
                    }
                }
            } else {
                c0[0] += stepX; tMaxX += tDeltaX;
                c0[1] += stepY; tMaxY += tDeltaY;
                c0[2] += stepZ; tMaxZ += tDeltaZ;
                for (int i = std::max(0, c0[0] - protection_length); i < std::min(c0[0] + protection_length + 1, xlim); i++) {
                    for (int j = std::max(0, c0[1] - protection_length); j < std::min(c0[1] + protection_length + 1, ylim); j++) {
                        for (int k = std::max(0, c0[2] - protection_length); k < std::min(c0[2] + protection_length + 1, zlim); k++) {
                            results.protection_zone_cell.push_back({i, j, k});
                        }
                    }
                }
            }
        }
        results.route_cells.push_back({c0[0], c0[1], c0[2]});
    }
    return results;
}

void print_conflicts(const std::vector<Conflict> & conflicts , int route_number)
{
    std::ostringstream oss;
    oss<<"conflicts: ";
    for(int i=0;i<route_number;i++)
    {
        oss<<i<<" {";
        std::vector<int> route_ids;
        for(auto& conflict:conflicts){
            for(auto& occupies : conflict.ids){
                if(occupies.route_id==i){
                    for(auto& occupies : conflict.ids)route_ids.push_back(occupies.route_id);
                    break;
                }
            }
        }
        std::vector<int>::iterator vector_iterator;
        std::sort(route_ids.begin(),route_ids.end());
        vector_iterator = std::unique(route_ids.begin(),route_ids.end());
        if(vector_iterator != route_ids.end()){
            route_ids.erase(vector_iterator,route_ids.end());
        }
        for(auto j:route_ids) if(i!=j)oss<<j<<' ';
        oss<<" } ";
    }
    LOG(INFO)<<oss.str();
}

WaypointAccInfo generate_traj_1d(double p, double max_v, double max_a) {
    WaypointAccInfo hor_info;

    // 三段式, 加速匀速减速
    if (max_v * max_v / max_a < p) {
        hor_info.t1 = max_v / max_a;
        hor_info.a1 = max_a;
        hor_info.v1 = 0;
        hor_info.delta_s1 = 0.5 * max_v * max_v / max_a;

        hor_info.t2 = (p - max_v * max_v / max_a) / max_v;
        hor_info.a2 = 0;
        hor_info.v2 = max_v;
        hor_info.delta_s2 = p - max_v * max_v / max_a;

        hor_info.t3 = hor_info.t1;
        hor_info.a3 = -max_a;
        hor_info.v3 = max_v;
        hor_info.delta_s3 = hor_info.delta_s1;
    }

    // 两段式
    else {
        hor_info.t1 = std::sqrt(p / max_a);
        hor_info.a1 = max_a;
        hor_info.v1 = 0;
        hor_info.delta_s1 = p * 0.5;

        hor_info.t2 = 0;
        hor_info.a2 = 0;
        hor_info.v2 = hor_info.v1 + hor_info.a1 * hor_info.t1;
        hor_info.delta_s2 = 0;

        hor_info.t3 = hor_info.t1;
        hor_info.a3 = -max_a;
        hor_info.v3 = hor_info.v2;
        hor_info.delta_s3 = hor_info.delta_s1;
    }

    hor_info.total_seconds = hor_info.t1 + hor_info.t2 + hor_info.t3;
    return hor_info;
}

WaypointDistanceInfo sample_traj(const WaypointAccInfo& info, double time) {
    WaypointDistanceInfo dis_info;
    if (time <= info.t1) {
        dis_info.acc = info.a1;
        dis_info.vel = info.a1 * time;
        dis_info.travel_dist = 0.5 * info.a1 * time * time;
    } else if (time <= info.t1 + info.t2) {
        dis_info.acc = 0;
        dis_info.vel = info.v2;
        dis_info.travel_dist = info.delta_s1 + info.v2 * (time - info.t1);
    } else if (time <= info.total_seconds) {
        dis_info.acc = info.a3;
        dis_info.vel = info.v2 + info.a3 * (time - info.t1 - info.t2);
        dis_info.travel_dist =
            info.delta_s1 + info.delta_s2 + info.v3 * (time - info.t1 - info.t2) +
            0.5 * info.a3 * (time - info.t1 - info.t2) * (time - info.t1 - info.t2);
    } else {
        dis_info.acc = 0;
        dis_info.vel = 0;
        dis_info.travel_dist = info.delta_s1 + info.delta_s2 + info.delta_s3;
    }
    return dis_info;
}

std::vector<double> get_3d_from_1d(const std::vector<double> & previous_waypoint, const std::vector<double> & current_waypoint,double delta) {
    double delta_p_x = current_waypoint[0] - previous_waypoint[0];
    double delta_p_y = current_waypoint[1] - previous_waypoint[1];
    double delta_p_z = current_waypoint[2] - previous_waypoint[2];
    double p = std::sqrt(delta_p_z * delta_p_z + delta_p_x * delta_p_x + delta_p_y * delta_p_y);

    std::vector<double> point;
    point.push_back(delta_p_x / p * delta);
    point.push_back(delta_p_y / p * delta);
    point.push_back(delta_p_z / p * delta);
    return point;
}

}
