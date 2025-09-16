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
height_max = 120
height_min = 60

class demand_generation:
    def __init__(self, env):
        self.env=env
        np.random.seed(1)

    def hub_destination_demand_temporal_generation(self,total_orders,start_time,end_time,peak_times,peak_width, peak1_percent):
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

    def hub_hub_demand_temporal_generation(self,total_orders,start_time,end_time):
        order_times_hours = np.random.uniform(start_time, end_time, total_orders)
        order_times_milliseconds = (order_times_hours - start_time) * 3600  # 1 hour = 3600000 milliseconds, changed to seconds now, 1h=3600s
        order_times_milliseconds = np.round(order_times_milliseconds).astype(int)

        self.total_orders=total_orders
        self.start_time=start_time
        self.end_time=end_time
        self.order_times_hours=order_times_hours
        self.order_times_milliseconds=order_times_milliseconds

        return order_times_hours, order_times_milliseconds

    def hub_destination_demand_spatial_generation(self,hub,destinations):
        # Randomly assign each order a destination from the list of destinations
        assigned_pairs = np.random.choice(range(len(destinations)), size=self.total_orders)

        self.hub = hub
        self.destinations = destinations
        # Combine order time, hub, and destination into tuples
        self.order_milliseconds = [(self.order_times_milliseconds[i], hub, destinations[pair]) for i, pair in enumerate(assigned_pairs)]
        self.order_hours = [(self.order_times_hours[i], hub, destinations[pair]) for i, pair in enumerate(assigned_pairs)]
        self.order_milliseconds.sort(key=lambda x: x[0])
        return self.order_milliseconds

    def hub_hub_demand_spatial_generation(self,hubs_o,hubs_d):
        if len(hubs_o) != len(hubs_d):
            raise ValueError("The length of hubs_o and hubs_d must be equal.")

        hub_destination_pairs = list(zip(hubs_o, hubs_d))
        # Randomly assign each order a hub-destination pair
        assigned_pairs = np.random.choice(range(len(hub_destination_pairs)), size=self.total_orders)

        self.hubs_o = hubs_o
        self.hubs_d = hubs_d
        # Combine order time, hub, and destination into tuples
        self.order_milliseconds = [(self.order_times_milliseconds[i], hub_destination_pairs[pair][0], hub_destination_pairs[pair][1]) 
                for i, pair in enumerate(assigned_pairs)]
        self.order_hours = [(self.order_times_hours[i], hub_destination_pairs[pair][0], hub_destination_pairs[pair][1]) 
                for i, pair in enumerate(assigned_pairs)]
        self.order_milliseconds.sort(key=lambda x: x[0])
        return self.order_milliseconds

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

class dense_env:
    def __init__(self):
        hdf5_path = "./data/mong kok/cells_height.hdf5"
        self.grid_res = 2.5
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
        
        I,J,K = cells_type.shape
        for i in range(I):
            for j in range(J):
                for k in range(K):
                    # if k == 0:
                    #     if self.cells_height[i,j] > 3:
                    #         cells_type[i,j,k]=-1
                    # else:
                    if self.cells_height[i,j] > k + height_min / self.grid_res:
                        for l in range(k + 1):  # Ensure all previous layers are set to -1
                            cells_type[i, j, l] = -1
        self.bounds=np.array([[0,0,0],[cells_type.shape[0], cells_type.shape[1], cells_type.shape[2]]])
        self.xs = np.arange(self.bounds[0, 0], self.bounds[1, 0])
        self.ys = np.arange(self.bounds[0, 1], self.bounds[1, 1])
        self.zs = np.arange(self.bounds[0, 2], self.bounds[1, 2])
        self.size = [self.xs.shape[0], self.ys.shape[0], self.zs.shape[0]]
        self.cells_type=cells_type[:,:,self.bounds[0][2]:self.bounds[1][2]].astype(np.int32)
        self.cells_cost=np.ones(self.size)

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

    def save(self, norm_path, file_name):
        all_z_coords = []
        fig, ax = plt.subplots()
        colors = ["gray", "white"]  # 定义颜色从灰色到白色
        cmap = LinearSegmentedColormap.from_list("gray_white", colors)
        ax.set_xticks([])
        ax.set_yticks([])
        # fig.colorbar(img)

        for path in norm_path:
            x_coords = [point[0] for point in path]
            y_coords = [point[1] for point in path]
            z_coords = [point[2] for point in path]
            all_z_coords.extend(z_coords)
            ax.plot(x_coords, y_coords, linewidth=0.6, color='red')  # 使用标记以更清楚地显示路径
        # ax.scatter(128,135, color='green') 
        # min_z = min(all_z_coords) - 1
        max_z = max(all_z_coords)
        print(max_z)
        img=ax.imshow(np.transpose(self.cells_type[:,:,int(max_z)]), cmap=cmap, interpolation='nearest',origin='lower')

        # ax.set_zlim(min_z, max_z)
        ax.set_title('Top view',fontsize=20)
        ax.set_xlim([0, self.cells_type.shape[0]])
        ax.set_ylim([0, self.cells_type.shape[1]])
        plt.savefig(file_name)
        # self.check_path(norm_path)

    def save_path(self,norm_path,file_name):
        fig, ax = plt.subplots(figsize=(10, 6))  # Adjust figsize for better aspect ratio
        # 定义颜色映射
        colors = ["white", "gray"]
        cmap = ListedColormap(colors)
        full_grid_size = int(60/self.grid_res)
        # 创建颜色数组
        facecolors = np.empty(self.cells_type.shape, dtype=object)
        new_matrix = np.zeros((self.cells_type.shape[0], self.cells_type.shape[1], full_grid_size))
        # 填充新矩阵
        for x in range(self.cells_type.shape[0]):
            for y in range(self.cells_type.shape[1]):
                if self.cells_type[x, y, 0] == -1:
                    new_matrix[x, y, :] = -1
        facecolors[self.cells_type == -1] = "gray"
        facecolors[self.cells_type == 0] = "white"

        # 将三维数据投影到 x-z 平面，使用最小值进行投影
        projection = np.min(self.cells_type, axis=1)

        # 绘制投影体素
        ax.imshow(projection.T != 0, cmap=cmap, alpha=0.5, origin='lower')

        # 绘制路径
        for path in norm_path:
            x_coords = [point[0] for point in path]
            y_coords = [point[1] for point in path]
            z_coords = [point[2] for point in path]
            ax.plot(x_coords, z_coords, linewidth=0.5, color='red')

        # Set title and axis labels
        ax.set_title('Side view', fontsize=20)
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

        # Save and show the plot
        plt.savefig(file_name)
        plt.show()
    def save_3dpath(self,norm_path,file_name):
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')

        # 定义颜色映射
        colors = ["white", "gray"]
        cmap = ListedColormap(colors)

        # 创建颜色数组
        facecolors = np.empty(self.cells_type.shape, dtype=object)
        facecolors[self.cells_type == -1] = "gray"
        facecolors[self.cells_type == 0] = "white"


        # 绘制路径
        for path in norm_path:
            x_coords = [point[0] for point in path]
            y_coords = [point[1] for point in path]
            z_coords = [point[2] for point in path]
            # all_z_coords.extend(z_coords)
            ax.plot(x_coords, y_coords, z_coords, linewidth=0.5, color='red')

        # 设置视角
        # 绘制体素
        ax.voxels(self.cells_type != 0, facecolors=facecolors, alpha=0.5)
        ax.view_init(elev=30, azim=120)
        # 设置标题和坐标轴标签
        ax.set_title('Mong Kok', fontsize=20)
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')

        # 显示图例
        gray_patch = plt.Line2D([0], [0], color="gray", lw=4, label='Building')
        red_patch = plt.Line2D([0], [0], color="red", lw=2, label='Path')
        plt.legend(handles=[gray_patch, red_patch])

        # 保存和显示图像
        plt.savefig(file_name)
        plt.show()
def check_path(self, norm_path):
        K = self.cells_type.shape[2]
        for path in norm_path:
            for point in path:
                x, y, z = map(int, point)
                if self.cells_type[x, y, z] == -1:
                    print(f'Path point {point} is in a building area!')

        

class sparse_env:
    def __init__(self):
        self.grid_res = 2.5
        population=loadmat('./data/pop_packages_D3100R5P200.mat')
        destinations=loadmat('./data/delivery_loc_res500_thres500.mat')
        grid_pop=population['grid_pop'] #1321*1321, 

        # grid_res=population['grid_res'][0][0] #5, 6.6km*6.6km

        # new_grid_pop = grid_pop[:1320, :1320].reshape(660, 2, 660, 2).sum(axis=(1, 3))
        # new_grid_pop = grid_pop[:1320, :1320].reshape(330, 4, 330, 4).sum(axis=(1, 3))
        new_grid_pop = grid_pop[:1320, :1320].reshape(660, 2, 660, 2).sum(axis=(1, 3))
        new_grid_pop = new_grid_pop[:368, :378]
        # grid_res = 2 * grid_res
        # grid_res = 4 * grid_res
        study_area=population['study_area']
        hub=np.array([population['hub_x'][0][0],population['hub_y'][0][0]])
        hub[0]=(hub[0]-study_area[0][0])/self.grid_res
        hub[1]=(hub[1]-study_area[0][1])/self.grid_res
        hub=np.concatenate((hub,np.array([0])))
        destinations=np.array(destinations['delivery_coordinate'])
        destinations[0:,0]=(destinations[0:,0]-study_area[0][0])/self.grid_res
        destinations[0:,1]=(destinations[0:,1]-study_area[0][1])/self.grid_res
        z = int(np.ceil((height_max - height_min) / (2 * self.grid_res)))
        cells_cost=new_grid_pop
        cells_cost=cells_cost*cells_cost.shape[0]*cells_cost.shape[1]/np.sum(cells_cost)#for normalization
        cells_cost=0*cells_cost+1*np.ones(cells_cost.shape) #for risk consideration
        self.cells_cost=cells_cost[:,:,np.newaxis]
        self.cells_cost = np.repeat(self.cells_cost, z, axis=2)
        cells_type=np.zeros(self.cells_cost.shape)
        self.bounds=np.array([[0,0,0],[cells_type.shape[0],cells_type.shape[1],cells_type.shape[2]]])
        self.bounds=np.array([[0,0,0],[368,378,12]])
        self.xs = np.arange(self.bounds[0, 0], self.bounds[1, 0])
        self.ys = np.arange(self.bounds[0, 1], self.bounds[1, 1])
        self.zs = np.arange(self.bounds[0, 2], self.bounds[1, 2])
        self.cells_type=cells_type[:,:,self.bounds[0][2]:self.bounds[1][2]].astype(np.int32)
        self.hub=hub
        self.destinations=destinations
        self.pop=new_grid_pop

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
