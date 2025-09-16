from pathlib import Path
from numpy.linalg.linalg import norm
from graphsearch.src.build.Graphsearch import conops1, conops2, conops3, conops4
# from environment import Environment
from utils import parse_args
# from visualization import VispyCanvas
RESULT_DIR = "results_new_1"
import os
from scenarios.test_scene_4d import demand_generation
from scenarios.custom_env import custom_env_builder

from graphsearch.algorithms import line_traversal
import numpy as np
# import matplotlib.pyplot as plt
import pickle
from random import sample, seed
import random
from copy import copy, deepcopy
from math import ceil
import sys
import matplotlib.pyplot as plt
import pandas as pd
from fastkml import kml
import shapely.geometry
# 在graph_search.py中添加Vertiport处理代码

from pathlib import Path
import json
from coordinate_transformer import CoordinateTransformer
from kml_parser import KMLParser

def extract_and_process_vertiports(env):
    """从KML文件中提取vertiport并生成路由对，去除无效的vertiport"""
    # 定义KML文件路径
    kml_file_path = "data/Mk1.kml"
    
    # 定义ENU转换的参考点 (与环境相同的参考点)
    ref_lon = 114.162559945072
    ref_lat = 22.3158202017388
    ref_alt = 0.0
    
    # 初始化KML解析器
    parser = KMLParser(kml_file_path)
    parser.parse_kml()
    
    # 获取提取的数据
    vertiports = parser.get_vertiports().to_dict(orient="records")
    
    # 初始化坐标转换器
    transformer = CoordinateTransformer(ref_lon, ref_lat, ref_alt)
    
    # 处理Vertiport坐标转换
    vertiport_locations_raw = []
    vertiport_names_raw = []
    
    for i, vertiport in enumerate(vertiports):
        # 只取第一个坐标点作为vertiport位置
        if vertiport["coordinates"]:
            lon, lat, alt = vertiport["coordinates"][0]
            # 转换为ENU坐标
            enu_coords = transformer.geo_to_enu(lat, lon, alt)
            # 转换为网格坐标 (基于参考分辨率)
            grid_x = int(enu_coords[0] / env.grid_res)
            grid_y = int(enu_coords[1] / env.grid_res)
            grid_z = 0  # 默认为地面高度
            
            # 确保坐标在网格范围内
            if (0 <= grid_x < env.cells_type.shape[0] and 
                0 <= grid_y < env.cells_type.shape[1]):
                vertiport_locations_raw.append([grid_x, grid_y, grid_z])
                vertiport_names_raw.append(vertiport["name"])
    
    # 合并相同名称的近距离vertiport点
    vertiport_locations = []
    vertiport_names = []
    
    # 首先按名称分组
    name_to_locations = {}
    for i, name in enumerate(vertiport_names_raw):
        if name not in name_to_locations:
            name_to_locations[name] = []
        name_to_locations[name].append(vertiport_locations_raw[i])
    
    # 处理每个名称的位置
    for name, locations in name_to_locations.items():
        if len(locations) == 1:
            # 如果只有一个位置，直接使用
            vertiport_locations.append(locations[0])
            vertiport_names.append(name)
        else:
            # 如果有多个位置，使用平均坐标
            avg_x = sum(loc[0] for loc in locations) // len(locations)
            avg_y = sum(loc[1] for loc in locations) // len(locations)
            avg_z = sum(loc[2] for loc in locations) // len(locations)
            vertiport_locations.append([avg_x, avg_y, avg_z])
            vertiport_names.append(name)
            print(f"注意: 合并了 {len(locations)} 个同名的 '{name}' 位置")
    
    # 检查vertiport是否位于有效位置（不在障碍物中且在网格范围内）
    valid_indices = []
    valid_vertiport_locations = []
    valid_vertiport_names = []
    
    for i, loc in enumerate(vertiport_locations):
        x, y, z = loc
        is_valid = True
        
        # 检查是否在网格范围内
        if not (0 <= x < env.cells_type.shape[0] and 0 <= y < env.cells_type.shape[1]):
            print(f"移除: Vertiport {i+1} ({vertiport_names[i]}) 位置超出网格范围")
            is_valid = False
        # 检查是否位于障碍物中
        elif env.cells_type[x, y, z] != 0:
            print(f"移除: Vertiport {i+1} ({vertiport_names[i]}) 位于障碍物中")
            is_valid = False
            
        if is_valid:
            valid_indices.append(i)
            valid_vertiport_locations.append(loc)
            valid_vertiport_names.append(vertiport_names[i])
    
    # 更新vertiport信息
    vertiport_locations = valid_vertiport_locations
    vertiport_names = valid_vertiport_names
    
    print(f"找到 {len(vertiport_locations)} 个有效vertiport位置:")
    for i, (loc, name) in enumerate(zip(vertiport_locations, vertiport_names)):
        print(f"Vertiport {i+1}: {name} - 坐标 {loc}")
    
    # 生成不重复的vertiport对
    vertiport_pairs = []
    origin_indices = []
    destination_indices = []
    
    # 用于检查已存在的路由对，避免重复
    existing_routes = set()
    
    for i in range(len(vertiport_locations)):
        for j in range(len(vertiport_locations)):
            if i != j:  # 避免同一个vertiport到自身的路线
                # 创建一个唯一键来检查是否已存在此路由
                route_key = (i, j)
                reverse_key = (j, i)
                
                # 检查这个路由或其反向是否已经存在
                if route_key not in existing_routes and reverse_key not in existing_routes:
                    vertiport_pairs.append((vertiport_locations[i], vertiport_locations[j]))
                    origin_indices.append(i)
                    destination_indices.append(j)
                    # 记录这个路由已经添加
                    existing_routes.add(route_key)
    
    print(f"生成了 {len(vertiport_pairs)} 个不重复的vertiport对")
    
    # 设置hubs1和hubs2
    hubs1 = [pair[0] for pair in vertiport_pairs]
    hubs2 = [pair[1] for pair in vertiport_pairs]
    
    # 验证并打印路由信息
    print("\n----- 验证路由信息 -----")
    for i in range(min(10, len(vertiport_pairs))):
        origin_name = vertiport_names[origin_indices[i]]
        dest_name = vertiport_names[destination_indices[i]]
        print(f"路由 {i+1}: 从 {origin_name} 到 {dest_name}")
    
    if len(vertiport_pairs) > 10:
        print(f"... 及其他 {len(vertiport_pairs) - 10} 条路由")
    
    # 如果没有有效的vertiport，创建一些默认位置
    if not vertiport_locations:
        print("警告: 没有有效的vertiport！创建默认位置...")
        max_x, max_y = env.cells_type.shape[0], env.cells_type.shape[1]
        
        # 在网格中寻找无障碍的位置
        default_locations = []
        default_names = []
        
        for i in range(1, 4):
            for j in range(1, 4):
                x = i * (max_x // 4)
                y = j * (max_y // 4)
                z = 0
                
                # 确保位置无障碍
                if env.cells_type[x, y, z] == 0:
                    default_locations.append([x, y, z])
                    default_names.append(f"默认Vertiport {len(default_locations)}")
                    
                    if len(default_locations) >= 3:  # 至少生成3个默认vertiport
                        break
            
            if len(default_locations) >= 3:
                break
        
        # 生成默认路由对
        default_pairs = []
        default_origins = []
        default_destinations = []
        
        for i in range(len(default_locations)):
            for j in range(len(default_locations)):
                if i != j:
                    default_pairs.append((default_locations[i], default_locations[j]))
                    default_origins.append(i)
                    default_destinations.append(j)
        
        # 使用默认值
        vertiport_locations = default_locations
        vertiport_names = default_names
        vertiport_pairs = default_pairs
        hubs1 = [pair[0] for pair in default_pairs]
        hubs2 = [pair[1] for pair in default_pairs]
        
        print(f"创建了 {len(default_locations)} 个默认vertiport和 {len(default_pairs)} 个路由对")
    
    return hubs1, hubs2, vertiport_locations, vertiport_names


def main():
    args = parse_args()
    dis=False                                       #whether distributed or sequential, default: False
  
    shuffle_times = 0            #for sequential, whether to shuffle the sequence to get better performance, 1: not shuffle, n: shuffle n times
    conops = 2                       #1: conops1; 2: conops2; 3: conops3
    sparse = False                               #True: sparse building, False: dense building
    hub_to_hub = True                      #True: hub to hub, False: hub to destinations
    total_orders = 15                  #n: n orders, default: 1000
    aggregated_demands_percent = 1 #percent in (0,1], how many routes will be pre-generated, default:0.9
    
    start_time_h2d = 9      #start time of order for hub2destination orders, default: 8h
    end_time_h2d = 21       #end time of order for hub2destination orders, default: 22h
    peak_times = [12, 18]   #time peaks for hub2destination orders, default: 12h, 18h
    peak_width = 1          #Width of each peak, default: 1h
    peak1_percent = 0.5     #percent of orders for the first peak
    start_time_h2h = 8      #start time of order for hub2hub orders, default: 9h
    end_time_h2h = 22       #end time of order for hub2hub orders, default: 21h
    time_step = 3      #temporal resolution, only apply for conops3, default: 2sqrt2 s
    window_size = 400       #temporal grid size for conops3, default: 400 (18.9min)
    grid_res = 10
    num_vertiports = 0
    # random_order_ratio = 1 - num_vertiports/num_locations
    print("args:", args)
    np.random.seed(42)
    random.seed(42)
    random_order_ratio = 1  # 不使用随机订单，全部使用vertiport对
    num_locations = 10  # 每个vertiport对生成一个订单
    ########################    scenario 1: hub to destination, dense   ########################
        # Initialize the environment for a dense scenario.
    env = custom_env_builder(
        kml_path='data/Mk1.kml',
        ref_lon=114.162559945072,
        ref_lat=22.3158202017388,
        ref_alt=0.0,
        resolution=grid_res,
        buffer_cells=0.5
    )
    # Add this code after extracting vertiports

    # 随机初始化环境
    hubs1 = []
    hubs2 = []
    
    # Generate demand based on the dense environment settings.
    demand = demand_generation(env, num_random_destinations= num_locations, random_order_ratio= random_order_ratio)
    # Generates temporal demand between hubs
    _, _ = demand.hub_hub_demand_temporal_generation(total_orders, start_time_h2h, end_time_h2h)
    # Generates spatial demand
    order_milliseconds = demand.hub_hub_demand_spatial_generation(hubs1, hubs2)
        
    # 不再依赖 hubs1 和 hubs2，而是直接从 order_milliseconds 中获取起点和终点
    if order_milliseconds:  # 检查是否有订单
        # 从 order_milliseconds 中获取唯一的起点和终点对
        # 每个 order_millisecond 是 (timestamp, origin, destination)
        unique_pairs = set()
        for order in order_milliseconds:
            # 将坐标元组转换为可哈希类型
            orig = tuple(order[1])
            dest = tuple(order[2])
            unique_pairs.add((orig, dest))
        
        # 转换回列表以便采样
        unique_pairs_list = list(unique_pairs)
        
        # 根据 aggregated_demands_percent 参数采样唯一的路径对
        seed(1)  # 保持相同的随机种子以保证可重复性
        samples_count = int(len(unique_pairs_list) * aggregated_demands_percent)
        samples_count = max(1, samples_count)  # 确保至少采样一条路径
        
        # 如果需要的样本数量超过实际路径对数量，就全部使用
        if samples_count >= len(unique_pairs_list):
            sampled_pairs = unique_pairs_list
        else:
            sampled_pairs = sample(unique_pairs_list, samples_count)
        
        # 提取起点和终点
        vertiports_o = [list(pair[0]) for pair in sampled_pairs]
        vertiports_d = [list(pair[1]) for pair in sampled_pairs]
        
        route_number = len(vertiports_d)
        ori_i = vertiports_o
        des_i = vertiports_d
        
        print(f"已采样 {route_number} 条独立路径，占总订单数的 {aggregated_demands_percent*100:.1f}%")
    else:
        # 如果没有订单，创建一条虚拟路径作为后备
        print("警告：找不到订单！创建虚拟路径。")
        # 假设 env 有有效的单元格坐标
        vertiports_o = [[0, 0, 0]]
        vertiports_d = [[env.cells_type.shape[0]//2, env.cells_type.shape[1]//2, 0]]
        route_number = 1
        ori_i = vertiports_o
        des_i = vertiports_d
    
    ##########################################conops 1##########################################
    # Conops 1 focuses on first generating 3D spatially separated routes and then sending 4D trajectory signals to manage traffic.
    if conops == 1:
        # Executes Conops 1
        if dis == False:
            results = conops1(env.cells_type, env.cells_cost, env.xs, env.ys, env.zs, env.grid_res,
                            *args.turning_parameters, *args.climb_parameters, *args.space_parameters,
                            args.nonuniform, args.operational, route_number, ori_i, des_i, vertiports_o, vertiports_d,
                            shuffle_times, False, order_milliseconds, sparse, hub_to_hub, total_orders,
                            int(10 * aggregated_demands_percent))
        else:
            results = conops1(env.cells_type, env.cells_cost, env.xs, env.ys, env.zs, env.grid_res,
                            *args.turning_parameters, *args.climb_parameters, *args.space_parameters,
                            args.nonuniform, args.operational, route_number, ori_i, des_i, vertiports_o, vertiports_d,
                            3, True, order_milliseconds, sparse, hub_to_hub, total_orders,
                            int(10 * aggregated_demands_percent))
    ##########################################conops 2##########################################
    # Conops 2 takes a slightly different approach by first generating 3D routes and then solving temporal conflicts
    if conops==2:
        time_step = max(1, int(3* env.grid_res / 5))  # 确保至少为1
        # Executes Conops 2
        # Prepares 4D vertiports
        order_vertiports_o = []
        order_vertiports_d = []
        for order in order_milliseconds:
            o=list(copy(order[1]))  # Convert tuple to list
            d=list(copy(order[2]))  # Convert tuple to list
            t=copy(order[0])
            o.append(ceil(t/time_step))  # start time [ms], to be transformed to [timestep s]
            d.append(ceil(t/time_step))  # not use here, can be arrival time in the future
            order_vertiports_o.append(o)
            order_vertiports_d.append(d)
        print("\n----- 坐标维度检查 -----")
        # 打印部分坐标值用于调试
        if ori_i and des_i:
            print(f"空间规划3D坐标示例:")
            print(f"ori_i[0] = {ori_i[0]}, 维度: {len(ori_i[0])}")
            print(f"des_i[0] = {des_i[0]}, 维度: {len(des_i[0])}")

        if order_vertiports_o and order_vertiports_d:
            print(f"时间调度4D坐标示例:")
            print(f"order_vertiports_o[0] = {order_vertiports_o[0]}, 维度: {len(order_vertiports_o[0])}")
            print(f"order_vertiports_d[0] = {order_vertiports_d[0]}, 维度: {len(order_vertiports_d[0])}")

        order_ori_i = order_vertiports_o
        order_des_i = order_vertiports_d
        order_route_number = len(order_vertiports_o)
        cells_type = np.zeros((env.cells_type.shape[0], env.cells_type.shape[1], env.cells_type.shape[2]),dtype=np.int32)
        # Executes Conops 2
        if dis == False:
            results = conops2(env.cells_type, env.cells_cost, env.xs, env.ys, env.zs, env.grid_res,
                            *args.turning_parameters, *args.climb_parameters, *args.space_parameters,
                            args.nonuniform, args.operational, route_number, order_ori_i, order_des_i, vertiports_o, vertiports_d,
                            shuffle_times, False, order_milliseconds, sparse, hub_to_hub, total_orders,
                            int(10 * aggregated_demands_percent), 
                            order_route_number, order_ori_i, order_des_i, order_vertiports_o, order_vertiports_d, time_step, window_size, cells_type)
        else:
            results = conops2(env.cells_type, env.cells_cost, env.xs, env.ys, env.zs, env.grid_res,
                            *args.turning_parameters, *args.climb_parameters, *args.space_parameters,
                            args.nonuniform, args.operational, route_number, ori_i, des_i, vertiports_o, vertiports_d,
                            2, True, order_milliseconds, sparse, hub_to_hub, total_orders,
                            int(10 * aggregated_demands_percent),
                            order_route_number, order_ori_i, order_des_i, order_vertiports_o, order_vertiports_d, time_step, window_size, cells_type)
    ##########################################conops 3##########################################
    # Conops 3 directly generates 3D routes along with 4D trajectory signals, aiming for an integrated spatial-temporal planning from the start.
    if conops==3:
        time_step = int(3* env.grid_res / 5)
        # Prepares 4D vertiports
        vertiports_o = []
        vertiports_d = []
        for order in order_milliseconds:
            o=list(copy(order[1]))  # Convert tuple to list
            d=list(copy(order[2]))  # Convert tuple to list
            t=copy(order[0])
            o.append(ceil(t/time_step))  # start time [ms], to be transformed to [timestep s]
            d.append(ceil(t/time_step))  # not use here, can be arrival time in the future
            vertiports_o.append(o)
            vertiports_d.append(d)

        ori_i = vertiports_o
        des_i = vertiports_d
        route_number = len(vertiports_o)
        # Executes Conops 3
        if dis==False:
            results = conops3(env.cells_type, env.cells_cost, env.xs, env.ys, env.zs, env.grid_res,
                          *args.turning_parameters, *args.climb_parameters, *args.space_parameters,
                          args.nonuniform, args.operational, route_number, ori_i, des_i, vertiports_o, vertiports_d,
                          shuffle_times, False, time_step, window_size, sparse, hub_to_hub, total_orders,
                          int(10 * aggregated_demands_percent))
        else:
            results = conops3(env.cells_type, env.cells_cost, env.xs, env.ys, env.zs, env.grid_res,
                            *args.turning_parameters, *args.climb_parameters, *args.space_parameters,
                            args.nonuniform, args.operational, route_number, ori_i, des_i, vertiports_o, vertiports_d,
                            2, True, time_step, window_size, sparse, hub_to_hub, total_orders,
                            int(10 * aggregated_demands_percent))

    if conops==4:
        # Executes Conops 4
        # Prepares 4D vertiports
        order_vertiports_o = []
        order_vertiports_d = []
        for order in order_milliseconds:
            o=copy(order[1])
            d=copy(order[2])
            t=copy(order[0])
            o.append(ceil(t/time_step))#start time [ms], to be transformed to [timestep s]
            d.append(ceil(t/time_step))#not use here, can be arrival time in the future
            order_vertiports_o.append(o)
            order_vertiports_d.append(d)

        order_ori_i = order_vertiports_o
        order_des_i = order_vertiports_d
        order_route_number = len(order_vertiports_o)
        cells_type = np.zeros((env.cells_type.shape[0], env.cells_type.shape[1], env.cells_type.shape[2]),dtype=np.int32)

        results = conops4(env.cells_type, env.cells_cost, env.xs, env.ys, env.zs, env.grid_res,
                    *args.turning_parameters, *args.climb_parameters, *args.space_parameters,
                    args.nonuniform, args.operational, route_number, ori_i, des_i, vertiports_o, vertiports_d,
                    shuffle_times, False, order_milliseconds, sparse, hub_to_hub, total_orders,
                    int(10 * aggregated_demands_percent), 
                    order_route_number, order_ori_i, order_des_i, order_vertiports_o, order_vertiports_d, time_step, window_size, cells_type)
        with open('./20v.pkl', 'wb') as f:
            pickle.dump(results[0], f)
    # ################################       get and save results  ###############################
    routes=results[0] #relative time, s unit
    trajectories=results[1] #relative time, ms unit
    print(len(routes),len(trajectories))
    if len(routes)<route_number:
        print("not all routes are generated")
    print(len(routes), route_number)
    if sparse==False and hub_to_hub==False:
        name = "dense_hub_to_destination"
    elif sparse==True and hub_to_hub==False:
        name = "sparse_hub_to_destination"
    elif sparse==False and hub_to_hub==True:
        name = "dense_hub_to_hub"
    elif sparse==True and hub_to_hub==True:
        name = "sparse_hub_to_hub"
    name = name +'_'+ str(total_orders)+"orders"+"_"+str(int(10*aggregated_demands_percent))+"aggregated"+"_"+str(dis)+"dis"+str(num_locations)+"destinations"
    with open('./results/conops'+str(conops)+'/trajectories' + name +'.pkl', 'wb') as f:
        pickle.dump(trajectories, f)
    with open('./results/conops'+str(conops)+ name + '.pkl', 'wb') as f:
        pickle.dump(routes, f)


    import os

    # Before saving files, create directories if they don't exist
    save_dir = f'./results/conops{str(conops)}'
    if not os.path.exists(save_dir):
        os.makedirs(save_dir)

    # Add debug logging to check values before saving
    print(f"Saving {len(trajectories)} trajectories and {len(routes)} routes")
    print(f"Save directory: {save_dir}")
    print(f"File name: {name}")

    # Save files with error handling
    try:
        trajectory_path = os.path.join(save_dir, f'trajectories{name}.pkl')
        route_path = os.path.join(save_dir, f'{name}.pkl')
        top_path = os.path.join(save_dir, f'Top{name}conops{str(conops)}.png')
        side_path = os.path.join(save_dir, f'Side{name}conops{str(conops)}.png')
        env_path = os.path.join(save_dir, f'Env{name}conops{str(conops)}.png')
        with open(trajectory_path, 'wb') as f:
            pickle.dump(trajectories, f)
        print(f"Saved trajectories to {trajectory_path}")
        
        with open(route_path, 'wb') as f:
            pickle.dump(routes, f)
        print(f"Saved routes to {route_path}")
        env.plot_env(hubs1, hubs2, env_path)
        print(f"Saved environment to {env_path}")
        env.save(trajectories, top_path)
        print(f"Saved top view to {top_path}")
        
        env.save_path(trajectories, side_path)
        print(f"Saved side view to {side_path}")

    except Exception as e:
        print(f"Error saving files: {str(e)}")
    # env.save(trajectories,'./results_new_1/conops'+str(conops)+'/Top' + name + 'conops.png')
    # env.save_path(trajectories,'./results_new_1/conops'+str(conops)+'/Side' + name + 'conops1.png')

if __name__ == "__main__":
    main()
