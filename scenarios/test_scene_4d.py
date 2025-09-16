from pathlib import Path
from matplotlib.colors import ListedColormap
from time import time
import numpy as np
import matplotlib.pyplot as plt
import sys
import os
sys.path.append(os.path.abspath("./"))
sys.path.append(os.path.abspath("../"))
import copy
from scipy.io import loadmat, savemat
import tables
from matplotlib.colors import LinearSegmentedColormap
from matplotlib.collections import LineCollection
from matplotlib.colors import Normalize
height_max = 120
height_min = 60
import matplotlib.patches as patches
from matplotlib.path import Path
from scipy.stats import gaussian_kde
import pickle
import random
from scipy.ndimage import gaussian_filter1d
from matplotlib.cm import ScalarMappable
class demand_generation:
    def __init__(self, env, num_random_destinations, location_deviation=0.01, random_order_ratio=0.3):
        """
        env: 环境变量（保留以支持扩展）
        num_random_destinations: 订单分布的最终位置数量（包括原始 destinations 和新增的随机位置）
        location_deviation: 订单随机生成时的偏移范围（仅 x, y 方向，单位：经纬度或米）
        random_order_ratio: 多少比例的订单在随机生成的目的地上（其余在 `destinations`）
        """
        self.env = env
        self.num_random_destinations = num_random_destinations  # 订单最终分布的总位置数
        self.location_deviation = location_deviation  # 订单位置偏移范围
        self.random_order_ratio = random_order_ratio  # 随机订单比例
        np.random.seed(42)
        random.seed(42)
    def hub_destination_demand_temporal_generation(self,total_orders,start_time,end_time,peak_times,peak_width, peak1_percent, min_interval=60, max_interval=120, duration = 4):
        """
        Generate order times following a bimodal distribution.

        Parameters:
        total_orders (int): Total number of orders.
        start_time (int): Start time of the orders (in hours).
        end_time (int): End time of the orders (in hours).
        peak_times (list of int): Times of the two peaks (in hours).
        peak_width (float): Width of each peak (standard deviation in hours).
        peak1_percent (float between 0 and 1): percent of orders for the first peak

        Returns:
        np.array: Array of order times.
        """
        """shuangxia version: 不考虑时间分布，仅考虑间隔生成订单"""
        start_time_sec = start_time * 3600  
        end_time_sec = start_time_sec + duration * 3600
        order_times_seconds = np.random.uniform(low=start_time_sec, high=end_time_sec, size=total_orders)
        order_times_milliseconds = np.round(order_times_seconds).astype(int)
        order_times_hours = np.round(order_times_seconds / 3600).astype(int)
        self.order_times_hours = order_times_hours
        self.order_times_milliseconds = order_times_milliseconds
        self.total_orders = total_orders
        self.start_time = start_time_sec
        self.end_time = end_time_sec
        # 按时间间隔生成订单
        # order_times_seconds = []
        # current_time = start_time  # 以小时为单位
        # while len(order_times_seconds) < total_orders:
        #     order_times_seconds.append(current_time)
        #     # 生成 30-60 秒的随机间隔
        #     interval = random.uniform(min_interval, max_interval)
        #     current_time += interval  # 增加时间
        # #  total_orders
        # order_times_seconds = np.array(order_times_seconds[:total_orders])

        # # 转换为毫秒（1 秒 = 1000 毫秒）
        # order_times_milliseconds = np.round(order_times_seconds).astype(int)
        # self.total_orders = total_orders
        # self.start_time = start_time
        # self.end_time = end_time
        # self.order_times_seconds = order_times_seconds
        # order_times_hours = np.round(order_times_seconds / 3600).astype(int)
        # self.order_times_hours = order_times_hours
        # self.order_times_milliseconds = order_times_milliseconds

        return order_times_seconds, order_times_milliseconds
    
        # Divide orders between the two peaks
        peak1_orders = int(total_orders * peak1_percent)  # 60% of orders for the first peak
        peak2_orders = total_orders - peak1_orders  # Remaining 40% for the second peak

        # Generate times for each peak
        peak1_times = np.random.normal(peak_times[0], peak_width, peak1_orders)
        peak2_times = np.random.normal(peak_times[1], peak_width, peak2_orders)
        order_times_hours = np.concatenate((peak1_times, peak2_times))

        # Clip times to be within the start and end times
        order_times_hours = np.clip(order_times_hours, start_time, end_time)

        order_times_milliseconds = (order_times_hours - start_time) * 3600  # 1 hour = 3600000 milliseconds, changed to seconds now, 1h=3600s
        order_times_milliseconds = np.round(order_times_milliseconds).astype(int)

        self.total_orders=total_orders
        self.start_time=start_time
        self.end_time=end_time
        self.order_times_hours=order_times_hours
        self.order_times_milliseconds=order_times_milliseconds

        return order_times_hours, order_times_milliseconds

    def hub_hub_demand_temporal_generation(self,total_orders,start_time,end_time, min_interval=60, max_interval=120, duration = 4):
        start_time_sec = start_time * 3600  
        end_time_sec = start_time_sec + duration * 3600
        order_times_seconds = np.random.uniform(low=start_time_sec, high=end_time_sec, size=total_orders)
        order_times_milliseconds = np.round(order_times_seconds).astype(int)
        order_times_hours = np.round(order_times_seconds / 3600).astype(int)
        self.order_times_hours = order_times_hours
        self.order_times_milliseconds = order_times_milliseconds
        self.total_orders = total_orders
        self.start_time = start_time_sec
        self.end_time = end_time_sec
        return order_times_seconds, order_times_milliseconds
    
        order_times_hours = np.random.uniform(start_time, end_time, total_orders)
        order_times_milliseconds = (order_times_hours - start_time) * 3600  # 1 hour = 3600000 milliseconds, changed to seconds now, 1h=3600s
        order_times_milliseconds = np.round(order_times_milliseconds).astype(int)

        self.total_orders=total_orders
        self.start_time=start_time
        self.end_time=end_time
        self.order_times_hours=order_times_hours
        self.order_times_milliseconds=order_times_milliseconds

        return order_times_hours, order_times_milliseconds

    def hub_destination_demand_spatial_generation(self, hub, destinations):
        """
        生成枢纽到目的地的订单，使得订单仅随机分布到 `num_random_destinations` 个位置
        """
        self.hub = hub
        self.destinations = destinations[:]
        num_fixed_orders = int(self.total_orders * (1 - self.random_order_ratio))
        num_random_orders = self.total_orders - num_fixed_orders
        num_fixed_destinations = len(destinations)
        num_random_destinations = self.num_random_destinations - num_fixed_destinations
        fixed_destinations = destinations
        additional_destinations = []
        num_existing_dest = len(destinations)
        # 首先获取所有空白单元格的坐标（行、列）
        free_cells = np.argwhere(self.env.cells_type == 0)  # 每行为 [x, y]

        # 筛选出周围四个邻居（前后左右）均为空白的单元格
        filtered_free_cells = []
        rows, cols = self.env.cells_type.shape[:2]
        for cell in free_cells:
            x, y = cell[:2]
            valid = True
            
            # 检查 3x3 网格中的所有单元格是否为空白
            for dx in [-1, 0, 1]:
                for dy in [-1, 0, 1                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                              ]:
                    nx, ny = x + dx, y + dy
                    
                    # 判断是否在边界内
                    if nx < 0 or ny < 0 or nx >= rows or ny >= cols:
                        valid = False
                        break
                    
                    # 检查邻居是否为空白（假设 z 维度为 0，或者你可以修改为特定的 z 维度）
                    if self.env.cells_type[nx, ny, 0] != 0:
                        valid = False
                        break
                if not valid:
                    break
            
            # 如果所有周围 3x3 网格中的单元格都是空白的，则添加到结果列表中
            if valid:
                filtered_free_cells.append(cell)

        filtered_free_cells = np.array(filtered_free_cells)

        # 若需要选取目的地，确保过滤后的单元格数量足够
        if num_random_destinations > 0 and len(filtered_free_cells) > 0:
            # 从过滤后的空白单元格中随机选择目标单元格
            selected_free_cells = filtered_free_cells[np.random.choice(len(filtered_free_cells),
                                                                    num_random_destinations,
                                                                    replace=False)]
            for cell in selected_free_cells:
                x, y = cell[:2]  # 只取 x, y
                z = 0  # 这里可以根据需要计算或设定 z 值
                additional_destinations.append((x, y, z))
        print(additional_destinations)
        # **确保每个目的地至少有 1 个订单**
        assigned_fixed_destinations = list(fixed_destinations)  # 先给每个目的地 1 个订单
        assigned_random_destinations = list(additional_destinations)  # 先给每个随机目的地 1 个订单

        remaining_fixed_orders = num_fixed_orders - len(fixed_destinations)  # 计算剩余订单
        remaining_random_orders = num_random_orders - len(additional_destinations)

        # **剩余订单随机分配**
        if remaining_fixed_orders > 0:
            assigned_fixed_destinations += random.choices(fixed_destinations, k=remaining_fixed_orders)

        if remaining_random_orders > 0:
            assigned_random_destinations += random.choices(additional_destinations, k=remaining_random_orders)

        # 统计实际分配的唯一订单位置
        # 先把列表转换成元组，确保可以放入 `set()`
        unique_fixed_destinations = set(tuple(dest) for dest in assigned_fixed_destinations)
        unique_random_destinations = set(tuple(dest) for dest in assigned_random_destinations)

        # 计算总共有多少个不同的目的地
        total_unique_destinations = len(unique_fixed_destinations | unique_random_destinations)  # 计算并集
        # 打印统计结果
        print(f"✅ 分配的固定目的地数量: {len(unique_fixed_destinations)}")
        print(f"✅ 分配的随机目的地数量: {len(unique_random_destinations)}")
        print(f"✅ 实际分配后总共不同的目的地数: {total_unique_destinations}")
        self.order_milliseconds = []
        self.order_hours = []

        # **将订单按 80% 固定 + 20% 随机的方式存储**
        for i in range(num_fixed_orders):
            self.order_milliseconds.append((self.order_times_milliseconds[i], hub, assigned_fixed_destinations[i]))
            self.order_hours.append((self.order_times_hours[i], hub, assigned_fixed_destinations[i]))

        for i in range(num_random_orders):
            index = num_fixed_orders + i
            self.order_milliseconds.append((self.order_times_milliseconds[index], hub, assigned_random_destinations[i]))
            self.order_hours.append((self.order_times_hours[index], hub, assigned_random_destinations[i]))

        self.order_milliseconds.sort(key=lambda x: x[0])
        return self.order_milliseconds


    def _generate_nearby_location(self, location):
        """
        在给定位置的附近随机生成位置，但仅在 x, y 方向偏移，z 方向不变
        location: 传入的原始位置 (x, y, z)
        """
        x_offset = np.random.uniform(-self.location_deviation, self.location_deviation)
        y_offset = np.random.uniform(-self.location_deviation, self.location_deviation)
        z_value = location[2]  # 保持 z 方向高度不变
        return location[0] + x_offset, location[1] + y_offset, z_value

    def hub_hub_demand_spatial_generation(self, hubs_o, hubs_d):
        """
        生成 Hub 到 Hub 的订单，使用有限数量的hub（比如10个），这些hub既可作为起点也可作为终点
        """
        # 计算固定和随机订单的数量
        num_fixed_orders = int(self.total_orders * (1 - self.random_order_ratio))
        num_random_orders = self.total_orders - num_fixed_orders
        
        # 将现有hub合并并转换为列表以便随机选择
        existing_hubs = []
        for hub in hubs_o:
            if tuple(hub) not in [tuple(h) for h in existing_hubs]:
                existing_hubs.append(hub)
        for hub in hubs_d:
            if tuple(hub) not in [tuple(h) for h in existing_hubs]:
                existing_hubs.append(hub)
        
        # 总共需要的hub数量
        total_desired_hubs = 10  # 可以根据需要调整
        num_existing_hubs = len(existing_hubs)
        num_additional_hubs = max(0, total_desired_hubs - num_existing_hubs)
        
        print(f"已有枢纽数量: {num_existing_hubs}，需要生成 {num_additional_hubs} 个新枢纽")
        
        # 生成新的hub位置
        additional_hubs = []
        if num_additional_hubs > 0:
            # 获取所有空白单元格
            free_cells = np.argwhere(self.env.cells_type == 0)
            
            # 过滤周围3x3区域均为空白的单元格
            filtered_free_cells = []
            rows, cols = self.env.cells_type.shape[:2]
            for cell in free_cells:
                x, y = cell[:2]
                valid = True
                
                # 检查3x3网格中的所有单元格是否为空白
                for dx in [-2, -1, 0, 1, 2]:
                    for dy in [-2, -1, 0, 1, 2]:
                        nx, ny = x + dx, y + dy
                        if nx < 0 or ny < 0 or nx >= rows or ny >= cols:
                            valid = False
                            break
                        if self.env.cells_type[nx, ny, 0] != 0:
                            valid = False
                            break
                    if not valid:
                        break
                
                if valid:
                    filtered_free_cells.append(cell)

            filtered_free_cells = np.array(filtered_free_cells)

            # 生成新的hub位置
            if len(filtered_free_cells) > 0:
                selected_indices = np.random.choice(
                    len(filtered_free_cells), min(num_additional_hubs, len(filtered_free_cells)), replace=False)
                selected_free_cells = filtered_free_cells[selected_indices]
                
                for cell in selected_free_cells:
                    x, y = cell[:2]
                    z = 0  # 可以根据需要设定z值
                    additional_hubs.append((x, y, z))
        
        # 合并所有hub
        all_hubs = existing_hubs + additional_hubs
        
        # 确保不超过指定数量的hub
        if len(all_hubs) > total_desired_hubs:
            all_hubs = all_hubs[:total_desired_hubs]
        
        # 创建随机的起点-终点对，确保起点和终点不是同一个hub
        order_pairs = []
        for _ in range(self.total_orders):
            # 随机选择两个不同的hub作为起点和终点
            origin = destination = None
            while origin == destination or origin is None:
                origin_idx = np.random.randint(0, len(all_hubs))
                dest_idx = np.random.randint(0, len(all_hubs))
                if origin_idx != dest_idx:
                    origin = all_hubs[origin_idx]
                    destination = all_hubs[dest_idx]
            
            order_pairs.append((origin, destination))
        
        # 分为固定订单和随机订单
        fixed_pairs = order_pairs[:num_fixed_orders]
        random_pairs = order_pairs[num_fixed_orders:]
        
        # 使用可哈希的元组形式进行统计
        def make_hashable(pair):
            return (tuple(pair[0]), tuple(pair[1]))
        
        unique_fixed_pairs = set(make_hashable(pair) for pair in fixed_pairs)
        unique_random_pairs = set(make_hashable(pair) for pair in random_pairs)
        
        # 统计用作起点和终点的hub数量
        used_as_origin = set(tuple(pair[0]) for pair in order_pairs)
        used_as_destination = set(tuple(pair[1]) for pair in order_pairs)
        
        # 打印统计结果
        print(f"✅ 总hub数量: {len(all_hubs)}")
        print(f"✅ 作为起点的hub数量: {len(used_as_origin)}")
        print(f"✅ 作为终点的hub数量: {len(used_as_destination)}")
        print(f"✅ 分配的固定 Hub-Hub 对数量: {len(unique_fixed_pairs)}")
        print(f"✅ 分配的随机 Hub-Hub 对数量: {len(unique_random_pairs)}")
        print(f"✅ 实际分配后总共不同的 Hub-Hub 对数: {len(unique_fixed_pairs | unique_random_pairs)}")
        
        # 存储订单
        self.order_milliseconds = []
        self.order_hours = []
        
        # 分配固定订单
        for i in range(num_fixed_orders):
            origin, destination = fixed_pairs[i]
            self.order_milliseconds.append((self.order_times_milliseconds[i], origin, destination))
            self.order_hours.append((self.order_times_hours[i], origin, destination))
        
        # 分配随机订单
        for i in range(num_random_orders):
            index = num_fixed_orders + i
            if i < len(random_pairs):
                origin, destination = random_pairs[i]
                self.order_milliseconds.append((self.order_times_milliseconds[index], origin, destination))
                self.order_hours.append((self.order_times_hours[index], origin, destination))
        
        # 订单按时间排序
        self.order_milliseconds.sort(key=lambda x: x[0])
        
        return self.order_milliseconds
    
    def _generate_nearby_location(self, location):
        """
        在给定位置的附近随机生成位置，但仅在 x, y 方向偏移, z 方向不变
        location: 传入的原始位置 (x, y, z)
        """
        x_offset = np.random.uniform(-self.location_deviation, self.location_deviation)
        y_offset = np.random.uniform(-self.location_deviation, self.location_deviation)
        z_value = location[2]  # 保持 z 方向高度不变
        return location[0] + x_offset, location[1] + y_offset, z_value
    def plot_h2d_temp_dist(self,filename):
        plt.cla()
        # Plotting the distribution of the example order times
        plt.hist(self.order_times_hours, bins=60, range=(self.start_time, self.end_time))
        # plt.title('Order Times Distribution')
        plt.xlabel('Time (hour)', fontsize=20)
        plt.ylabel('Number of Orders', fontsize=20)
        plt.savefig(filename)

    def plot_h2d_temp_dist_per_h2d(self,filename):
        plt.cla()
        fig, axs = plt.subplots(len(self.destinations), figsize=(10, 6), sharex=True)
        # fig.suptitle('Order Time Distribution by Destination')
        for i, destination in enumerate(self.destinations):
            # Extract order times for this destination
            times = [order[0] for order in self.order_hours if order[2] == destination]

            # Plot on the respective subplot
            axs[i].hist(times, bins=60, alpha=0.7)
            axs[i].set_title(destination)
            axs[i].set_ylabel('Number of Orders')

        # Set common labels
        plt.xlabel('Order Time (hours)')
        plt.tight_layout(rect=[0, 0.03, 1, 0.95])  # Adjust the layout to make room for the suptitle
        plt.savefig(filename)

    def plot_h2h_temp_dist(self,filename):
        plt.clf()
        # Plotting the distribution of the example order times
        plt.hist(self.order_times_hours, bins=60, range=(self.start_time, self.end_time))
        # plt.title('Uniform Distribution of Order Times')
        plt.xlabel('Time (hour)', fontsize=20)
        plt.ylabel('Number of Orders', fontsize=20)
        plt.savefig(filename)
        
    def plot_h2h_temp_dist_per_h2h(self, filename):
        plt.clf()
        hub_destination_pairs = list(zip(self.hubs_o, self.hubs_d))
        num_plots = len(hub_destination_pairs)

        fig, axs = plt.subplots(num_plots, figsize=(10, num_plots * 3), sharex=True)
        # fig.suptitle('Order Time Distribution by Hub to Destination')
        for i, pair in enumerate(hub_destination_pairs):
            # Extract order times for this hub-destination pair
            times = [order[0] for order in self.order_hours if order[1] == pair[0] and order[2] == pair[1]]
            # Plot on the respective subplot
            if num_plots > 1:
                ax = axs[i]
            else:
                ax = axs
            ax.hist(times, bins=60, alpha=0.7)
            ax.set_title(f'{pair[0]} to {pair[1]}')
            ax.set_ylabel('Number of Orders')

        # Set common labels
        plt.xlabel('Order Time (hours)')
        plt.tight_layout(rect=[0, 0.03, 1, 0.95])  # Adjust the layout to make room for the suptitle
        plt.savefig(filename)
def make_hashable(obj):
    """
    递归地将所有嵌套的列表转换为元组，使对象变为可哈希类型。
    处理所有可能的嵌套结构，确保数据可以安全地放入 set 中。
    """
    if isinstance(obj, list):
        return tuple(make_hashable(e) for e in obj)
    elif isinstance(obj, dict):
        return tuple((k, make_hashable(v)) for k, v in obj.items())
    elif isinstance(obj, (set, tuple)):
        return tuple(make_hashable(e) for e in obj)
    return obj


def pad_array_to_multiple(arr, multiple):
    new_shape = [int(np.ceil(s / multiple) * multiple) for s in arr.shape]
    new_arr = np.zeros(new_shape, dtype=arr.dtype)
    new_arr[:arr.shape[0], :arr.shape[1]] = arr
    return new_arr
class dense_env:
    def __init__(self, grid_res=5):
        hdf5_path = "./data/mong kok/cells_height.hdf5"
        self.grid_res = grid_res
        # grid_res = 4 * grid_res
        hdf5_file = tables.open_file(hdf5_path, mode='r')
        self.cells_height = np.ceil(hdf5_file.root.data[:,:])
        self.cells_height = np.ceil(hdf5_file.root.data[:,:]/self.grid_res)
        hdf5_file.close()
        I,J = self.cells_height.shape
        cells_height=np.zeros((int(I/self.grid_res), int(J/self.grid_res)))
        for i in range(int(I/self.grid_res)):
            for j in range(int(J/self.grid_res)):
                cells_height[i,j]=np.min(self.cells_height[int(np.floor(i * self. grid_res)):int(np.ceil((i + 1) * self. grid_res)),int(np.floor(j * self. grid_res)):int(np.ceil((j + 1) * self. grid_res))])
        self.cells_height=cells_height # use 10m resolution
        z = int(np.ceil((height_max - height_min) / (2*self.grid_res)))
        cells_type = np.zeros((self.cells_height.shape[0], self.cells_height.shape[1], z),dtype=np.int32)
        self.base_map = np.zeros((self.cells_height.shape[0],self.cells_height.shape[1], 12))
        I,J,K = cells_type.shape
        for i in range(I):
            for j in range(J):
                for k in range(K):
                    if k == 0:
                        if self.cells_height[i,j] > 3:
                            self.base_map[i,j,k]=-1
                    # else:
                    
                    
                    if self.cells_height[i,j] > k + height_min / self.grid_res:
                        for l in range(k + 1):  # Ensure all previous layers are set to -1
                            cells_type[i, j, l] = -1
        self.bounds=np.array([[0,0,0],[cells_type.shape[0], cells_type.shape[1], cells_type.shape[2]]])
        self.xs = np.arange(0,230)
        self.ys = np.arange(0,200)
        self.zs = np.arange(0,200)
        self.size = [self.xs.shape[0], self.ys.shape[0], self.zs.shape[0]]
        self.cells_type=cells_type[:,:,self.bounds[0][2]:self.bounds[1][2]].astype(np.int32)
        self.cells_cost=np.ones(self.size)
        with open('././results/conops3_4d_20ver_10m/trajectoriesdense_hub_to_destination_2000orders_10aggregated_Falsedis.pkl', 'rb') as f:
            routes=pickle.load(f)
        airspace = np.zeros((184, 189, 6))        
        for trajectory in routes:
            for waypoint in trajectory:
                airspace[int(waypoint[0]), int(waypoint[1]), int(waypoint[2])] += 1
        airspace = np.array(airspace)
        airspace_2d = np.max(airspace, axis=2)
        # 找出大于 1 的格子的位置和对应的值
        # 筛选大于 1 的格子的值
        greater_than_one = airspace_2d[airspace_2d > 1]

        # 使用 np.unique 统计每个值出现的次数
        unique_values, counts = np.unique(greater_than_one, return_counts=True)
        from matplotlib.colors import BoundaryNorm
        # 打印统计结果
        # print("大于 1 的格子值及其数量：")
        # for value, count in zip(unique_values, counts):
        #     print(f"值: {value}，数量: {count} 个格子")
        # 1. 根据频率调整颜色分布：数量多的值分配更多颜色
        cumulative_steps = np.cumsum(counts) / counts.sum()  # 计算累计频率步长
        boundaries = np.concatenate(([1], unique_values))  # 边界：包括1

        # 生成 Blues colormap，并根据累积分布进行采样
        blues = plt.get_cmap('viridis')
        self.colors = blues(np.concatenate(([0.3], 0.3 + 0.7 * cumulative_steps)))  # 非均匀采样
                # 创建不均匀的 ListedColormap 和 BoundaryNorm
        self.cmap = ListedColormap(self.colors)
        self.norm = BoundaryNorm(boundaries, self.cmap.N)
    def plot_env(self, hubs1, hubs2, file_name):
        fig, ax = plt.subplots()
        colors = ["gray", "white"]  # 定义颜色从灰色到白色
        cmap = LinearSegmentedColormap.from_list("gray_white", colors)
        img=ax.imshow(np.transpose(self.cells_type[:,:,0]), cmap=cmap, interpolation='nearest',origin='lower')
        ax.set_xticks([])
        ax.set_yticks([])
        # fig.colorbar(img)
        if isinstance(hubs1[0], list):
            for hub in hubs1:
                ax.plot(hub[0], hub[1], marker='^',color='blue')  
        else:
            ax.plot(hubs1[0], hubs1[1], marker='^',color='blue')  
        for hub in hubs2:
            ax.plot(hub[0], hub[1], marker='v',color='red')  
        # ax.set_title('Mong Kok',fontsize=20)
        ax.set_xlim([0, self.cells_type.shape[0]])
        ax.set_ylim([0, self.cells_type.shape[1]])
        plt.savefig(file_name)
    def save(self,norm_path, file_name):
        # === 1. 统计路径密度 ===
        airspace = np.zeros((
            int(184 * 5 / self.grid_res),
            int(189 * 5 / self.grid_res),
            int(6 * 5 / self.grid_res)
        ))
        for traj in norm_path:
            for wp in traj:
                x, y, z = map(int, wp[:3])
                airspace[x, y, z] += 1

        airspace_2d = np.max(airspace, axis=2)

        # === 2. clip 并 log 平滑密度，用于更均衡的可视化 ===
        max_clip = 200  # 可调节
        airspace_2d_clipped = np.clip(airspace_2d, 0, max_clip)
        airspace_2d_log = np.log1p(airspace_2d_clipped)  # log(1 + x)

        vmin = np.min(airspace_2d_log)
        vmax = np.max(airspace_2d_log)
        ticks = np.linspace(0, 5, 6, dtype=int) 

        norm = Normalize(vmin=vmin, vmax=vmax)
        custom_rdgb = LinearSegmentedColormap.from_list(
            'custom_rdgb',
            ["#d73027", "#444444", "#313695"],  # 红 → 深灰 → 蓝
            N=256
        )
        colors = ['#2166ac', '#d9d9d9', '#b2182b']  # 红 - 深灰 - 深蓝
        custom_cmap = LinearSegmentedColormap.from_list('BlueGrayRed', colors)
        cmap = custom_cmap
        # cmap = plt.cm.RdBu

        # === 3. 初始化图像 ===
        fig, ax = plt.subplots(figsize=(12, 8))
        ax.set_xticks([])
        ax.set_yticks([])
        # ax.set_position([0.1, 0.1, 0.6, 0.6])
        # === 4. 背景图层（建筑） ===
        cmap_map = LinearSegmentedColormap.from_list("gray_white", ["gray", "white"])
        ax.imshow(np.transpose(self.cells_type[:, :, -1]), cmap=cmap_map,
                interpolation='nearest', origin='lower', zorder=1)

        # === 5. 绘制路径 ===
        print("Rendering trajectories with density-based colors...")
        for trajectory in norm_path:
            x = [p[0] for p in trajectory]
            y = [p[1] for p in trajectory]

            # Get log-smoothed density values for coloring
            alpha = [airspace_2d_log[int(p[0]), int(p[1])] for p in trajectory]
            alpha = gaussian_filter1d(alpha, sigma=3)
            
            # Set line width range (not too dramatic)
            min_width = 3  # Min line width
            max_width =  4 # Max line width
            
            alpha_min = np.min(airspace_2d_log)
            alpha_max = np.max(airspace_2d_log)
            
            # Inverted formula: higher density = thinner lines
            # Using a smaller range (1.5 to 3.0) to make differences subtle
            linewidths = max_width - (max_width - min_width) * (alpha - alpha_min) / (alpha_max - alpha_min)
            
            # Create line segments
            points = np.array([x, y]).T.reshape(-1, 1, 2)
            segments = np.concatenate([points[:-1], points[1:]], axis=1)

            # Create line collection with variable widths
            lc = LineCollection(segments, cmap=cmap, array=alpha, norm=norm,
                                linewidths=linewidths, alpha=1.0, zorder=3)
            ax.add_collection(lc)

        # === 6. 起点终点 ===
        for traj in norm_path:
            ax.plot(traj[0][0], traj[0][1], marker='o', color='forestgreen', markersize=4)
            ax.plot(traj[-1][0], traj[-1][1], marker='^', color='firebrick', markersize=4)
        # === NEW: 添加 padding 给边界 ===
        all_x = [p[0] for traj in norm_path for p in traj]
        all_y = [p[1] for traj in norm_path for p in traj]
        margin = 10  # 可根据图像大小微调
        ax.set_xlim(-30, 225)
        ax.set_ylim(-10,200)

        # === 7. 图例 + 颜色条 ===
        handles = [
            plt.Line2D([0], [0], color='gray', lw=3, label='Building'),
            plt.Line2D([0], [0], color='#0055aa', lw=2, label='Trajectory'),
            plt.Line2D([0], [0], marker='o', color='forestgreen', markersize=6, label='Departure'),
            plt.Line2D([0], [0], marker='^', color='firebrick', markersize=6, label='Destination')
        ]
        # ax.legend(handles=handles, loc='upper right', fontsize=8)

        sm = ScalarMappable(cmap=cmap, norm=norm)
        sm.set_array([])
        # cbar = plt.colorbar(sm, ax=ax, orientation='vertical', pad=0.01)
        # Make the colorbar narrower (controls width)
        cbar = plt.colorbar(sm, ax=ax, orientation='vertical', pad=0.01, fraction=0.02)  # Default is usually 0.15
        cbar.ax.tick_params(labelsize=10) 
        cbar.set_label('Log-scaled Trajectory Density (log(x))', fontsize=20)
        cbar.set_ticks(ticks)
        cbar.set_ticklabels([f'{tick}' for tick in ticks])
        plt.tight_layout()
        plt.savefig(file_name, dpi=300)
        plt.close(fig)
        print(f"Saved figure to {file_name}")
    def plot(self, norm_path, file_name):
        # === 1. 统计路径密度 ===
        airspace = np.zeros((
            int(184 * 5 / self.grid_res),
            int(189 * 5 / self.grid_res),
            int(6 * 5 / self.grid_res)
        ))
        for traj in norm_path:
            for wp in traj:
                x, y, z = map(int, wp[:3])
                airspace[x, y, z] += 1

        airspace_2d = np.max(airspace, axis=2)

        # === 2. log 平滑密度值用于可视化 ===
        max_clip = 200
        airspace_2d_clipped = np.clip(airspace_2d, 0, max_clip)
        airspace_2d_log = np.log1p(airspace_2d_clipped)  # log(1 + x)

        # colormap 设置
        norm = Normalize(vmin=np.min(airspace_2d_log), vmax=np.max(airspace_2d_log))
        custom_cmap = LinearSegmentedColormap.from_list(
            'RedGrayBlue', ["#b2182b", "#444444", "#2166ac"]  # 红 - 深灰 - 深蓝
        )
        cmap = custom_cmap

        # === 3. 初始化图像 ===
        fig = plt.figure(figsize=(12, 8))
        ax = fig.add_axes([0.1, 0.1, 0.75, 0.8])  # 手动设置黑框位置大小
        ax.set_xticks([])
        ax.set_yticks([])

        # === 4. 背景图层（建筑） ===
        cmap_map = LinearSegmentedColormap.from_list("gray_white", ["gray", "white"])
        ax.imshow(np.transpose(self.cells_type[:, :, -1]), cmap=cmap_map,
                interpolation='nearest', origin='lower', zorder=1)

        # === 5. 绘制路径 ===
        print("Rendering trajectories with density-based colors...")
        for trajectory in norm_path:
            x = [p[0] for p in trajectory]
            y = [p[1] for p in trajectory]
            alpha = [airspace_2d_log[int(p[0]), int(p[1])] for p in trajectory]
            alpha = gaussian_filter1d(alpha, sigma=3)

            points = np.array([x, y]).T.reshape(-1, 1, 2)
            segments = np.concatenate([points[:-1], points[1:]], axis=1)

            lc = LineCollection(segments, cmap=cmap, array=alpha, norm=norm,
                                linewidth=1.2, alpha=1.0, zorder=3)
            ax.add_collection(lc)

        # === 6. 起点和终点 ===
        for traj in norm_path:
            ax.plot(traj[0][0], traj[0][1], marker='o', color='forestgreen', markersize=4)
            ax.plot(traj[-1][0], traj[-1][1], marker='^', color='firebrick', markersize=4)

        # === NEW: 添加 padding 给边界 ===
        all_x = [p[0] for traj in norm_path for p in traj]
        all_y = [p[1] for traj in norm_path for p in traj]
        margin = 10  # 可根据图像大小微调
        ax.set_xlim(min(all_x) - margin, max(all_x) + margin)
        ax.set_ylim(min(all_y) - margin, max(all_y) + margin)

        # === 7. 保存图像 ===
        plt.savefig(file_name, dpi=300, bbox_inches='tight')
        plt.close(fig)
        print(f"Saved figure to {file_name}")
    def save_path(self, norm_path, file_name):
        """绘制简化的侧视图(x-z平面)，显示路径和高度"""
        
        # 创建图形
        fig, ax = plt.subplots(figsize=(12, 6))
        
        # 绘制每条路径
        for traj in norm_path:
            if len(traj) < 2:  # 跳过太短的轨迹
                continue
                
            # 提取坐标
            x_coords = [point[0] for point in traj]
            z_coords = [point[2] for point in traj]
            
            # 将z坐标转换为实际高度(60-120米)
            actual_heights = [z * 10 + 60 for z in z_coords]
            
            # 绘制路径
            ax.plot(x_coords, actual_heights, '-', linewidth=1.0, alpha=0.7)
            
            # 标记起点和终点
            ax.plot(x_coords[0], actual_heights[0], 'go', markersize=5)  # 绿色起点
            ax.plot(x_coords[-1], actual_heights[-1], 'r^', markersize=5)  # 红色终点

        # 添加图例
        ax.plot([], [], 'go', markersize=5, label='出发点')
        ax.plot([], [], 'r^', markersize=5, label='目的地')
        ax.plot([], [], '-', color='blue', linewidth=1.0, label='飞行路径')
        
        # 设置标题和坐标轴标签
        ax.set_title('空中走廊侧视图 (X-Z平面)')
        ax.set_xlabel('X轴距离 (东)')
        ax.set_ylabel('高度 (米)')
        
        # 设置高度范围
        ax.set_ylim(60, 120)
        
        # 添加网格
        ax.grid(True, alpha=0.3)
        
        # 添加图例
        ax.legend(loc='upper right')
        
        # 保存图像
        plt.savefig(file_name, dpi=300, bbox_inches='tight')
        plt.close(fig)  # 关闭图形以避免显示
        
        print(f"已保存侧视图到 {file_name}")
    def save_3dpath(self, norm_path, file_name): 
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        colors_map = ["gray", "white"]  # Define colors from gray to white
        cmap_map = LinearSegmentedColormap.from_list("gray_white", colors_map)
        height_data = self.cells_height * 10
        x, y = np.meshgrid(np.arange(self.cells_height.shape[0]), np.arange(self.cells_height.shape[1]), indexing="ij")
        x = x.ravel()
        y = y.ravel()
        z = np.full_like(x, 60)
        dz = np.clip(height_data.ravel() - 60, 0, 120 - 60)

        mask = (height_data.ravel() > 60) & (height_data.ravel() < 120)
        x = x[mask]
        y = y[mask]
        z = z[mask]
        dz = dz[mask]
        ax.bar3d(x, y, z, dx=0.8, dy=0.8, dz=dz, color='lightgray', alpha=0.7,zorder = 0)

        ax.tick_params(axis='both', which='major', labelsize=6)
        def draw_line_fast_3d(ax, x, y, z, alpha, norm=None, cmap='Blues', nonlinear_func=None):
            colors = self.cmap(self.norm(alpha))
            # 绘制 3D 线段（分段处理）
            for i in range(len(x) - 1):
                ax.plot(
                    [x[i], x[i + 1]], 
                    [y[i], y[i + 1]], 
                    [z[i], z[i + 1]], 
                    color=colors[i], 
                    linewidth=0.8, zorder=3
                )
        airspace = np.zeros((184,189, 6))
        for trajectory in norm_path:
            for waypoint in trajectory:
                airspace[int(waypoint[0]), int(waypoint[1]),int(waypoint[2])] += 1
                
        for trajectory in norm_path:
            x = [point[0] for point in trajectory]
            z_coords = [point[2] for point in trajectory]
            y = [point[1] for point in trajectory]  # Y 轴
            actual_heights_path = [z * 10 + 60 for z in z_coords]  # 高度调整

            # 获取路径对应的 alpha 值
            alpha = np.array([airspace[int(point[0]), int(point[1]), int(point[2])] for point in trajectory])

            # 绘制路径
            draw_line_fast_3d(ax, x, y, actual_heights_path, alpha, self.norm, cmap=self.colors)
        ax.view_init(elev=30, azim=120)  # Adjust as needed
        for index, path in enumerate(norm_path):
            x_coords = [point[0] for point in path]
            y_coords = [point[1] for point in path]
            z_coords = [point[2] for point in path]
            actual_heights_path = [z * 10 + 60 for z in z_coords]
            # Mark the starting point with a circle
            ax.plot(x_coords[0], y_coords[0], actual_heights_path[0],marker='o', color='forestgreen', markersize=2, label='Start Point', zorder=4)

            # Mark the ending point with a triangle
            ax.plot(x_coords[-1], y_coords[-1],  actual_heights_path[-1],marker='^', color='firebrick', markersize=2, label='End Point', zorder=4)

        # 修改 X 轴刻度，确保每隔100个单位标记
        ax.set_zlim([60, 120])

        ax.set_xticks(np.arange(0, 201,50))  # Ticks every 100 units
        ax.set_xticklabels([f"{tick * 10} m" for tick in np.arange(0, 201, 50)])  # X axis labels scaled by 10
        # 设置Z轴刻度在图的左边
        # ax.zaxis.set_ticks_position('left')
        
        # 修改 Y 轴刻度，确保每隔100个单位标记
        ax.set_yticks(np.arange(0, 201,50))  # Ticks every 100 units
        ax.set_yticklabels([f"{tick * 10} m" for tick in np.arange(0, 201, 50)])  # Reverse Y axis labels scaled by 10
        
        # 修改 Z 轴刻度，确保每隔100个单位标记
        ax.set_zticks(np.arange(60, 120,10))  # Ticks every 100 units
        ax.set_zticklabels([f"{tick} m" for tick in np.arange(60, 120,10)])# 不显示Y轴刻度
        # ax.set_yticks([])
        # 不显示网格
        ax.grid(False)
        # ax.set_box_aspect([2, 1, 1]) 
        # 设置视角
        ax.grid(True, which='both', linestyle='--', linewidth=0.5)

        # 保存和显示图像
        # 减小图像边缘空白

        plt.subplots_adjust(left=0, right=1, top=1, bottom=0)
        # plt.tight_layout()
        plt.savefig(file_name, dpi=300)
        plt.show()

    def method_2(self,norm_path,file_name):
        from matplotlib import cm
        from matplotlib.colors import Normalize
        # Load the routes data
        routes = norm_path
        # Set parameters for noise
        sigma_x = sigma_y = sigma_z = 0.0
        sigma_z = 0.0

        # Collect all positions for density calculation
        all_positions = []

        # Process trajectories and add noise
        for trajectory in routes:
            trajectory_with_noise = []
            for i in range(len(trajectory)):
                x = np.random.normal(0, sigma_x, 1)[0] if i > 0 and i < len(trajectory)-1 else 0
                y = np.random.normal(0, sigma_y, 1)[0] if i > 0 and i < len(trajectory)-1 else 0
                z = np.random.normal(0, sigma_z, 1)[0] if i > 0 and i < len(trajectory)-1 else 0
                noisy_point = [trajectory[i][0]+x, trajectory[i][1]+y, trajectory[i][2]+z]
                all_positions.append(noisy_point)
                trajectory_with_noise.append(noisy_point)

        # Discretize the 3D space
        resolution = 0.5  # Grid resolution in spatial coordinates
        discretized_positions = np.round(np.array(all_positions) / resolution) * resolution

        # Count the number of drones passing through each grid point
        unique_positions, counts = np.unique(discretized_positions, axis=0, return_counts=True)

        # Create the plot
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')

        # Normalize the density counts for color mapping
        norm = Normalize(vmin=0, vmax=np.max(counts))

        # Plot each unique point with its density-based color and increased size for better visibility
        for pos, count in zip(unique_positions, counts):
            ax.scatter(pos[0], pos[1], pos[2], color=cm.Blues(norm(count)), s=30, alpha=0.9)  # Increased size and opacity

        # Add color bar for density
        mappable = cm.ScalarMappable(norm=norm, cmap='Blues')
        mappable.set_array(counts)
        cbar = plt.colorbar(mappable)
        cbar.set_label('Drone Density')

        # Set labels and improve visibility
        ax.set_xlabel('X axis', fontsize=10)
        ax.set_ylabel('Y axis', fontsize=10)
        ax.set_zlabel('Z axis', fontsize=10)

        # Set gridlines for better visibility
        ax.grid(True, linestyle='--', linewidth=0.5, alpha=0.8)

        # Adjust the viewing angle for better understanding
        ax.view_init(elev=25, azim=135)  # Adjust as needed

        # Show the plot
        plt.show()
        plt.savefig(file_name, dpi=300)

def check_path(self, norm_path):
        K = self.cells_type.shape[2]
        for path in norm_path:
            for point in path:
                x, y, z = map(int, point)
                if self.cells_type[x, y, z] == -1:
                    print(f'Path point {point} is in a building area!')

        

class sparse_env:
    def __init__(self, grid_res=5):
        self.grid_res = grid_res
        population = loadmat('./data/pop_packages_D3100R5P200.mat')
        destinations = loadmat('./data/delivery_loc_res500_thres500.mat')
        grid_pop = population['grid_pop']  # 1321*1321
        study_area = population['study_area']
        
        # Calculate scaling factor relative to standard grid_res of 5
        scale_factor = 5 / self.grid_res
        
        # Adjust grid size based on grid_res with proper bounds checking
        max_size = min(grid_pop.shape[0], grid_pop.shape[1])
        
        # Ensure we don't try to access beyond array bounds
        reshape_size = int(330 * scale_factor)
        chunk_size = max(1, int(4 / scale_factor))
        
        # Make sure reshape_size * chunk_size doesn't exceed grid dimensions
        max_grid_dim = min(grid_pop.shape[0], grid_pop.shape[1])
        if reshape_size * chunk_size > max_grid_dim:
            # Scale down reshape_size to fit within bounds
            reshape_size = max_grid_dim // chunk_size
            
        # Reshape with appropriate dimensions
        new_grid_pop = grid_pop[:reshape_size*chunk_size, :reshape_size*chunk_size].reshape(
            reshape_size, chunk_size, reshape_size, chunk_size).sum(axis=(1, 3))
        
        # Rest of the function remains the same...
        # Adjust the final dimensions proportionally
        x_dim = int(184 * scale_factor)
        y_dim = int(189 * scale_factor)
        new_grid_pop = new_grid_pop[:x_dim, :y_dim]
        
        # Recalculate z dimension based on grid_res
        z = int(np.ceil((height_max - height_min) / (2 * self.grid_res)))
        
        # Scale hub and destination coordinates
        hub = np.array([population['hub_x'][0][0], population['hub_y'][0][0]])
        hub[0] = (hub[0] - study_area[0][0]) / self.grid_res
        hub[1] = (hub[1] - study_area[0][1]) / self.grid_res
        hub = np.concatenate((hub, np.array([0])))
        
        destinations = np.array(destinations['delivery_coordinate'])
        destinations[:, 0] = (destinations[:, 0] - study_area[0][0]) / self.grid_res
        destinations[:, 1] = (destinations[:, 1] - study_area[0][1]) / self.grid_res
        
        # Create cost cells with appropriate dimensions
        cells_cost = new_grid_pop
        cells_cost = cells_cost * cells_cost.shape[0] * cells_cost.shape[1] / np.sum(cells_cost)  # for normalization
        cells_cost = 0 * cells_cost + 1 * np.ones(cells_cost.shape)  # for risk consideration
        
        self.cells_cost = cells_cost[:, :, np.newaxis]
        self.cells_cost = np.repeat(self.cells_cost, z, axis=2)
        
        cells_type = np.zeros(self.cells_cost.shape)
        
        # Set bounds based on the new dimensions
        self.bounds = np.array([[0, 0, 0], [x_dim, y_dim, z]])
        
        self.xs = np.arange(self.bounds[0, 0], self.bounds[1, 0])
        self.ys = np.arange(self.bounds[0, 1], self.bounds[1, 1])
        self.zs = np.arange(self.bounds[0, 2], self.bounds[1, 2])
        
        self.cells_type = cells_type[:, :, self.bounds[0][2]:self.bounds[1][2]].astype(np.int32)
        self.hub = hub
        self.destinations = destinations
        self.pop = new_grid_pop

    def plot_env(self, hubs1, hubs2, file_name):
        fig, ax = plt.subplots()
        colors = ["white", "gray"]  # 定义颜色从灰色到白色
        cmap = LinearSegmentedColormap.from_list("gray_white", colors)
        img=ax.imshow(np.transpose(self.pop), cmap=cmap, interpolation='nearest',origin='lower')
        ax.set_xticks([])
        ax.set_yticks([])
        # fig.colorbar(img)
        if isinstance(hubs1[0], list):
            for hub in hubs1:
                ax.plot(hub[0], hub[1], marker='^',color='blue')  
        else:
            ax.plot(hubs1[0], hubs1[1], marker='^',color='blue')  
        for hub in hubs2:
            ax.plot(hub[0], hub[1], marker='v',color='red')  
        ax.set_xlim([0, self.cells_type.shape[0]])
        ax.set_ylim([0, self.cells_type.shape[1]])
        plt.savefig(file_name)

    def save(self,norm_path,file_name):
        fig, ax = plt.subplots()
        colors = ["white", "gray"]  # 定义颜色从灰色到白色
        cmap = LinearSegmentedColormap.from_list("gray_white", colors)
        img=ax.imshow(np.transpose(self.pop), cmap=cmap, interpolation='nearest',origin='lower')
        ax.set_xticks([])
        ax.set_yticks([])
        # colorbar=fig.colorbar(img)
        # colorbar.set_label('Thousand people per km^2', fontsize=16)

        for path in norm_path:
            x_coords = [point[0] for point in path]
            y_coords = [point[1] for point in path]
            ax.plot(x_coords, y_coords, linewidth=0.6, color='red')  # 使用标记以更清楚地显示路径

        ax.set_title('Delft (population map)', fontsize=20)
        ax.set_xlim([0, self.cells_type.shape[0]])
        ax.set_ylim([0, self.cells_type.shape[1]])
        plt.savefig(file_name)

    def save_path(self,norm_path,file_name):
        fig, ax = plt.subplots(figsize=(10, 6))  # Adjust figsize for better aspect ratio
        # 定义颜色映射
        colors = ["white", "gray"]
        cmap = ListedColormap(colors)

        # # 创建颜色数组
        # facecolors = np.empty(self.cells_type.shape, dtype=object)
        # facecolors[self.cells_type == -1] = "gray"
        # facecolors[self.cells_type == 0] = "white"

        # # 将三维数据投影到 x-z 平面，使用最小值进行投影
        # projection = np.min(self.cells_type, axis=1)

        # # 绘制投影体素
        # ax.imshow(projection.T != 0, cmap=cmap, alpha=0.5, origin='lower')

        # 绘制路径
        for path in norm_path:
            x_coords = [point[0] for point in path]
            z_coords = [point[2] for point in path]
            ax.plot(x_coords, z_coords, linewidth=0.5, color='red')

        # Set title and axis labels
        ax.set_title('Delft', fontsize=20)
        ax.set_xlabel('X / 10 m', fontsize=14)
        ax.set_ylabel('Z / 10 m', fontsize=14)

        # Set x and z axis limits
        # x_limit = self.cells_type.shape[0] * self.grid_res
        # z_limit = self.cells_type.shape[2] * self.grid_res
        # # ax.set_xlim([0, x_limit])
        # # ax.set_ylim([60, 120])
        # # Set x and z axis ticks
        # x_ticks = np.arange(0, x_limit + 1, 25)  # Adjust interval as necessary
        # z_ticks = np.arange(60, 121, 5)  # Adjust interval as necessary
        # ax.set_xticks(x_ticks)
        # ax.set_yticks(z_ticks)

        # Adjust aspect ratio
        aspect_ratio = (self.cells_type.shape[0] / self.cells_type.shape[2])  # Adjusting to make the plot square
        ax.set_aspect(aspect_ratio)


        # Add grid for better readability
        ax.grid(True, which='both', linestyle='--', linewidth=0.5)
        # Show legend
        gray_patch = plt.Line2D([0], [0], color="gray", lw=4, label='Building')
        red_patch = plt.Line2D([0], [0], color="red", lw=2, label='Path')
        # grid_patch = plt.Line2D([0], [0], color="none", label='One grid = 5m')  # Add grid size information as a legend item

        plt.legend(handles=[gray_patch, red_patch], loc='upper left', fontsize=12)
        plt.legend(handles=[red_patch], loc='upper left', fontsize=12)
        # Save and show the plot
        plt.savefig(file_name)
        plt.show()
