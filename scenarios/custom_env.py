import numpy as np
import matplotlib.pyplot as plt
from ObstacleGrid import ObstacleGridBuilder
import os
from shapely.geometry import Polygon, Point, LineString
import numpy as np
from collections import defaultdict
from coordinate_transformer import CoordinateTransformer

class CustomEnvironment:
    def __init__(self, obstacle_grid_path=None, grid_builder=None, ref_lon=114.162559945072, 
                 ref_lat=22.3158202017388, ref_alt=0.0, resolution=10.0):
        """
        Initialize environment with either a pre-built grid or by creating one
        
        Args:
            obstacle_grid_path: Path to a numpy file with pre-calculated grid
            grid_builder: An instance of ObstacleGridBuilder
            ref_lon/lat/alt: Reference coordinates for the environment
            resolution: Grid resolution in meters
        """
        self.grid_res = resolution
        
        # Load from file or builder
        if obstacle_grid_path and os.path.exists(obstacle_grid_path):
            self.load_from_file(obstacle_grid_path)
        elif grid_builder is not None:
            self.load_from_builder(grid_builder)
        else:
            # Create a default grid builder
            builder = ObstacleGridBuilder(
                kml_path='data/Mk1.kml', 
                ref_lon=ref_lon,
                ref_lat=ref_lat,
                ref_alt=ref_alt,
                resolution=resolution,
                buffer_cells=0.5
            )
            builder.build_grid(min_height=60, max_height=120)
            # 保存网格
            builder.save_to_npz('/home/shuangxia/air_corridor_transportation/data_processing/output/building_grid.npz')

            # 可视化
            builder.visualize_topdown('building_topview.png')
            builder.visualize_grid(limit=50000)  # 限制点数以提高性能
            self.load_from_builder(builder)
            
        # Generate coordinate arrays after grid is loaded
        self.generate_coordinates()
    
    def load_from_file(self, filepath):
        """Load grid data from numpy file"""
        data = np.load(filepath, allow_pickle=True).item()
        self.grid = data['grid']
        self.grid_metadata = data['metadata']
        
        # Create the cell types (0=free, 1=obstacle) and costs
        self.cells_type = self.grid.astype(np.int32)
        # Default cost map: 1.0 for free, infinity for obstacles
        self.cells_cost = np.where(self.cells_type == 0, 1.0, np.inf)
    
    def load_from_builder(self, builder):
        """Load grid directly from an ObstacleGridBuilder instance"""
        self.grid = builder.get_grid()
        if hasattr(builder, 'grid_metadata'):
            self.grid_metadata = builder.grid_metadata
        else:
            # Default metadata if not available
            self.grid_metadata = {
                'min_e': 0,
                'min_n': 0,
                'min_u': 0,
                'resolution': self.grid_res,
                'shape': self.grid.shape
            }
        
        # 修改这里：将所有非零值（包括负值）转换为1表示障碍物
        self.cells_type = np.where(self.grid != 0, -1, 0).astype(np.int32)
        
        # 同样修改代价矩阵
        self.cells_cost = np.where(self.cells_type == 0, 1.0, np.inf)
    
    def generate_coordinates(self):
        """Generate coordinate arrays for xs, ys, zs"""
        min_e = self.grid_metadata.get('min_e', 0)
        min_n = self.grid_metadata.get('min_n', 0)
        min_u = self.grid_metadata.get('min_u', 0)
        res = self.grid_metadata.get('resolution', self.grid_res)
        
        nx, ny, nz = self.grid.shape
        self.xs = np.arange(nx) + min_e
        self.ys = np.arange(ny) + min_n
        self.zs = np.arange(nz) + min_u
    
    def plot_env(self, hubs1, hubs2, filename=None):
        """
        Visualize environment with vertiports (top view)
        
        Args:
            hubs1: List of source vertiport coordinates
            hubs2: List of destination vertiport coordinates
            filename: Optional filename to save the plot
        """
        plt.figure(figsize=(12, 10))
        
        # Create a 2D top-down view of obstacles (max height projection)
        obstacle_map = np.any(self.cells_type > 0, axis=2)
        
        # Plot obstacles
        plt.imshow(obstacle_map.T, origin='lower', cmap='binary', 
                   extent=[0, obstacle_map.shape[0], 0, obstacle_map.shape[1]])
        
        # Plot vertiports
        if isinstance(hubs1[0], list):  # Multiple hub1 locations
            for hub in hubs1:
                plt.scatter(hub[0], hub[1], color='green', s=50, marker='o', label='Source')
        else:  # Single hub1 location
            plt.scatter(hubs1[0], hubs1[1], color='green', s=50, marker='o', label='Source')
            
        for hub in hubs2:
            plt.scatter(hub[0], hub[1], color='red', s=50, marker='x', label='Destination')
        
        # Remove duplicate labels
        handles, labels = plt.gca().get_legend_handles_labels()
        by_label = dict(zip(labels, handles))
        plt.legend(by_label.values(), by_label.keys())
        
        plt.title('Environment with Vertiports')
        plt.xlabel('X (East)')
        plt.ylabel('Y (North)')
        
        if filename:
            plt.savefig(filename, dpi=300, bbox_inches='tight')
            print(f"Saved environment visualization to {filename}")
        
        plt.show()
    
    def save(self, trajectories, filename):
        """
        Save top view of trajectories
        
        Args:
            trajectories: List of trajectory coordinates
            filename: Filename to save the visualization
        """
        plt.figure(figsize=(12, 10))
        
        # Create a 2D top-down view of obstacles
        obstacle_map = np.any(self.cells_type > 0, axis=2)
        plt.imshow(obstacle_map.T, origin='lower', cmap='binary', 
                  extent=[0, obstacle_map.shape[0], 0, obstacle_map.shape[1]], alpha=0.5)
        
        # Plot each trajectory
        for i, traj in enumerate(trajectories):
            xs = [point[0] for point in traj]
            ys = [point[1] for point in traj]
            plt.plot(xs, ys, '-', linewidth=1, alpha=0.7)
            
            # Mark start and end points
            plt.scatter(xs[0], ys[0], color='green', s=30, marker='o')
            plt.scatter(xs[-1], ys[-1], color='red', s=30, marker='x')
        
        plt.title(f'Aerial Corridors - Top View ({len(trajectories)} paths)')
        plt.xlabel('X (East)')
        plt.ylabel('Y (North)')
        plt.grid(True, alpha=0.3)
        
        plt.savefig(filename, dpi=300, bbox_inches='tight')
        print(f"Saved trajectory visualization to {filename}")
    
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
            actual_heights = [z for z in z_coords]
            
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
        # 添加网格
        ax.grid(True, alpha=0.3)
        
        # 添加图例
        ax.legend(loc='upper right')
        
        # 保存图像
        plt.savefig(file_name, dpi=300, bbox_inches='tight')
        plt.close(fig)  # 关闭图形以避免显示
        
        print(f"已保存侧视图到 {file_name}")
        # 在CustomEnvironment类中添加此方法
    def visualize_slice(self, z_level=0, save_path=None, vertiport_locations=None, vertiport_names=None):
        """可视化环境的特定高度切片，验证填充效果"""
        plt.figure(figsize=(12, 10))
        plt.imshow(self.cells_type[:, :, z_level].T, origin='lower', cmap='viridis')
        plt.colorbar(label='单元格类型')
        plt.title(f'环境切片 z={z_level} (高度≈{z_level*self.grid_res}米)')
        
        # 如果提供了vertiport位置，标记它们
        if vertiport_locations:
            for i, loc in enumerate(vertiport_locations):
                if loc[2] == z_level:  # 只标记当前切片中的vertiport
                    plt.plot(loc[0], loc[1], 'ro', markersize=8)
                    name = vertiport_names[i] if vertiport_names else f"Vertiport {i+1}"
                    plt.text(loc[0], loc[1], name, color='white', 
                            fontsize=8, ha='right', va='bottom')
        
        if save_path:
            plt.savefig(save_path, dpi=300, bbox_inches='tight')
            print(f"环境切片已保存到 {save_path}")
        else:
            plt.show()
# Helper functions to create environments - similar to test_scene_4d.py
def custom_env(grid_path=None):
    """Create a custom environment from grid file"""
    return CustomEnvironment(obstacle_grid_path=grid_path)
def custom_env_builder(kml_path='data/Mk1.kml', ref_lon=114.162559945072, 
                      ref_lat=22.3158202017388, ref_alt=0.0, resolution=10.0,
                      buffer_cells=0, fill_buildings=True):
    """Create a custom environment from KML file using ObstacleGridBuilder"""
    builder = ObstacleGridBuilder(
        kml_path=kml_path,
        ref_lon=ref_lon,
        ref_lat=ref_lat,
        ref_alt=ref_alt,
        resolution=resolution,
        buffer_cells=buffer_cells
    )
    builder.build_grid(min_height=60, max_height=120)
    builder.visualize_topdown()
    builder.visualize_grid()
    env = CustomEnvironment(grid_builder=builder, 
                           ref_lon=ref_lon, ref_lat=ref_lat, ref_alt=ref_alt, 
                           resolution=resolution)
    return env