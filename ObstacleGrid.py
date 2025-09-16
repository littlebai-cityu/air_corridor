import numpy as np
from kml_parser import KMLParser
from coordinate_transformer import CoordinateTransformer
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from shapely.geometry import Polygon, Point
import numpy as np
from shapely.geometry import Polygon, Point, MultiPolygon
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from tqdm import tqdm
import xml.etree.ElementTree as ET
import os

class ObstacleGridBuilder:
    def __init__(self, kml_path, ref_lat, ref_lon, ref_alt=0.0, resolution=1.0, buffer_cells=0.5):
        self.kml_path = kml_path
        self.transformer = CoordinateTransformer(ref_lon, ref_lat, ref_alt)
        self.resolution = resolution
        self.buffer_cells = buffer_cells
        self.grid = None
        self.obstacle_voxels = []
        self.offset = None
        self.buildings = []  # 存储建筑物数据

    def extract_building_info(self):
        """从KML文件提取建筑物信息，包括轮廓和高度"""
        print(f"从 {self.kml_path} 提取建筑物信息...")
        
        try:
            # 解析KML文件
            ns = {'kml': 'http://www.opengis.net/kml/2.2'}
            tree = ET.parse(self.kml_path)
            root = tree.getroot()
            
            # 查找所有Placemark元素
            all_placemarks = root.findall('.//kml:Placemark', ns)
            print(f"找到 {len(all_placemarks)} 个Placemark元素")
            # 筛选只包含obstacle的元素
            placemarks = []
            for placemark in all_placemarks:
                name_elem = placemark.find('./kml:name', ns)
                if name_elem is not None and name_elem.text and "obstacle" in name_elem.text.lower():
                    placemarks.append(placemark)
            buildings = []
            
            for placemark in placemarks:
                name_elem = placemark.find('./kml:name', ns)
                name = name_elem.text if name_elem is not None else "未命名建筑"
                
                # 尝试提取建筑物高度
                height = 20.0  # 默认高度20米
                coords_elem = placemark.find('.//kml:coordinates', ns)
                if coords_elem is not None and coords_elem.text:
                    coords_text = coords_elem.text.strip()
                    coords_parts = coords_text.split()
                    if coords_parts and ',' in coords_parts[0]:
                        # 尝试获取第一个坐标的z值
                        coord_parts = coords_parts[0].split(',')
                        if len(coord_parts) >= 3 and coord_parts[2]:
                            try:
                                z_value = float(coord_parts[2])
                                # 检查是否有extrude标签指示应该拉伸
                                extrude_elem = placemark.find('.//kml:extrude', ns)
                                if extrude_elem is not None and extrude_elem.text == '1':
                                    # 获取altitude模式
                                    altitude_mode = placemark.find('.//kml:altitudeMode', ns)
                                    altitude_mode_text = altitude_mode.text if altitude_mode is not None else 'clampToGround'
                                    
                                    # 根据不同的高度模式处理
                                    if altitude_mode_text in ('relativeToGround', 'absolute'):
                                        # 如果z值太小，可能需要放大它
                                        if 0 < z_value < 5.0:  # 如果高度小于5米
                                            # 针对小值的障碍物，乘以一个因子使其更明显
                                            height = z_value * 10.0  # 放大10倍，使3.2439变为32.439米
                                            print(f"从坐标z值提取并放大高度: {z_value}米 → {height}米")
                                        else:
                                            height = max(z_value, 5.0)  # 确保至少有5米高
                                            print(f"从坐标z值提取高度: {height}米")
                            except ValueError:
                                pass
                
                # 查找坐标数据
                coords_elem = placemark.find('.//kml:coordinates', ns)
                if coords_elem is None or not coords_elem.text:
                    continue
                
                # 解析坐标
                coords_text = coords_elem.text.strip()
                coords_list = []
                
                for coord_str in coords_text.split():
                    if not coord_str.strip():
                        continue
                    parts = coord_str.split(',')
                    if len(parts) >= 2:
                        lon = float(parts[0])
                        lat = float(parts[1])
                        alt = float(parts[2]) if len(parts) > 2 and parts[2] else 0.0
                        coords_list.append((lon, lat, alt))
                
                if len(coords_list) < 3:
                    continue  # 跳过点数不足的轮廓
                
                # 将建筑物信息添加到列表中
                buildings.append({
                    'name': name,
                    'height': height,
                    'coordinates': coords_list
                })
            
            self.buildings = buildings
            print(f"成功提取 {len(buildings)} 个建筑物信息")
            return buildings
            
        except Exception as e:
            print(f"提取建筑物信息时出错: {e}")
            return []

    def build_grid(self, min_height=0, max_height=120):
        """构建包含建筑物的3D网格"""
        # 提取建筑物信息
        buildings = self.extract_building_info()
        if not buildings:
            raise ValueError("未检测到有效的建筑物数据")
        
        polygons = []
        all_points = []
        building_heights = []
        
        # 处理每个建筑物为Polygon
        for building in buildings:
            coords = building['coordinates']
            height = building['height']
            en_points = [self.transformer.geo_to_enu(lat, lon, alt)[:2] for lon, lat, alt in coords]
            
            if len(en_points) < 3:
                continue  # 跳过点数不足的
                
            poly = Polygon(en_points)
            if not poly.is_valid:
                poly = poly.buffer(0)
                
            polygons.append(poly)
            all_points.extend(en_points)
            
            # 将高度转换为网格单元数
            height_cells = max(1, int(np.ceil(height / self.resolution)))
            building_heights.append(height_cells)
        
        if not all_points:
            raise ValueError("未检测到有效的建筑物轮廓")
        
        # 计算边界
        all_points = np.array(all_points)
        min_e, min_n = np.min(all_points, axis=0)
        max_e, max_n = np.max(all_points, axis=0)
        
        # 添加边距
        min_e -= self.buffer_cells * self.resolution
        min_n -= self.buffer_cells * self.resolution
        max_e += self.buffer_cells * self.resolution
        max_n += self.buffer_cells * self.resolution
        
        # 确保min_height不高于0以包含地面
        # min_height = min(0, min_height)
        
        size_x = int(np.ceil((max_e - min_e) / self.resolution))
        size_y = int(np.ceil((max_n - min_n) / self.resolution))
        size_z = int(np.ceil((max_height - min_height) / self.resolution))
        
        print(f"网格尺寸: {size_x} x {size_y} x {size_z}")
        print(f"高度范围: {min_height}米 - {max_height}米")
        print(f"分辨率: {self.resolution}米")
        
        self.grid = np.zeros((size_x, size_y, size_z), dtype=np.int8)
        self.offset = (min_e, min_n, min_height)
        
        # 网格化填充
        print("填充建筑物...")
        for i, (poly, height_cells) in enumerate(zip(polygons, building_heights)):
            # print(f"处理建筑物 {i+1}/{len(polygons)} (高度: {height_cells}层)")
            self._fill_polygon_to_height(poly, height_cells)
        
        # 统计占用率
        occupied = np.sum(self.grid == -1)
        total = self.grid.size
        print(f"占用率: {occupied/total*100:.2f}% ({occupied}/{total})")
        
    def _fill_polygon_to_height(self, polygon, height_cells):
        """填充多边形区域到指定高度"""
        # 获取多边形的边界框
        minx, miny, maxx, maxy = polygon.bounds
        
        # 转换为网格索引范围
        min_ix = max(0, int((minx - self.offset[0]) / self.resolution))
        min_iy = max(0, int((miny - self.offset[1]) / self.resolution))
        max_ix = min(self.grid.shape[0]-1, int((maxx - self.offset[0]) / self.resolution) + 1)
        max_iy = min(self.grid.shape[1]-1, int((maxy - self.offset[1]) / self.resolution) + 1)
        
        # 遍历边界框内的每个点
        for ix in range(min_ix, max_ix + 1):
            for iy in range(min_iy, max_iy + 1):
                # 转换回实际坐标
                x = ix * self.resolution + self.offset[0]
                y = iy * self.resolution + self.offset[1]
                
                # 检查点是否在多边形内
                if polygon.contains(Point(x, y)):
                    # 填充从0到建筑高度的所有单元格
                    for iz in range(min(height_cells, self.grid.shape[2])):
                        self.grid[ix, iy, iz] = -1
                        self.obstacle_voxels.append((ix, iy, iz))
    
    def flood_fill_polygon(self, polygon):
        """使用洪水填充算法填充多边形内部"""
        minx, miny, maxx, maxy = polygon.bounds
        
        # 创建一个与轮廓边界框大小相同的2D网格
        width = int((maxx - minx) / self.resolution) + 1
        height = int((maxy - miny) / self.resolution) + 1
        
        # 初始化mask
        mask = np.zeros((width, height), dtype=bool)
        
        # 标记边界上的点
        for x in np.linspace(minx, maxx, width):
            for y in np.linspace(miny, maxy, height):
                ix = int((x - minx) / self.resolution)
                iy = int((y - miny) / self.resolution)
                if 0 <= ix < width and 0 <= iy < height:
                    if polygon.contains(Point(x, y)):
                        mask[ix, iy] = True
        
        return mask

    def get_grid(self):
        if self.grid is None:
            raise RuntimeError("请先调用 build_grid()")
        return self.grid

    def save_to_npz(self, filename='building_grid.npz'):
        """保存网格和元数据"""
        if self.grid is not None:
            np.savez_compressed(
                filename,
                grid=self.grid,
                offset=self.offset,
                resolution=self.resolution,
                shape=self.grid.shape
            )
            print(f"网格已保存为 {filename}")
            
            # 同时保存人类可读的元数据
            base_name = os.path.splitext(filename)[0]
            with open(f"{base_name}_metadata.txt", 'w') as f:
                f.write(f"网格形状: {self.grid.shape}\n")
                f.write(f"分辨率: {self.resolution}米\n")
                f.write(f"原点坐标: {self.offset}\n")
                f.write(f"建筑物数量: {len(self.buildings)}\n")
                occupied = np.sum(self.grid == -1)
                total = self.grid.size
                f.write(f"占用率: {occupied/total*100:.2f}% ({occupied}/{total})\n")

    def visualize_grid(self, limit=10000):
        """3D可视化网格"""
        if not self.obstacle_voxels:
            print("请先运行 build_grid()")
            return
            
        # 采样以避免点太多
        sampled_voxels = self.obstacle_voxels[:limit]
        
        xs = [ix * self.resolution + self.offset[0] for ix, _, _ in sampled_voxels]
        ys = [iy * self.resolution + self.offset[1] for _, iy, _ in sampled_voxels]
        zs = [iz * self.resolution + self.offset[2] for _, _, iz in sampled_voxels]

        fig = plt.figure(figsize=(10, 8))
        ax = fig.add_subplot(111, projection='3d')
        ax.set_title("3D 建筑物网格")
        ax.scatter(xs, ys, zs, c='red', marker='s', alpha=0.3, s=5)
        
        # 设置轴标签
        ax.set_xlabel('东向 (m)')
        ax.set_ylabel('北向 (m)')
        ax.set_zlabel('高度 (m)')
        
        plt.tight_layout()
        plt.savefig('building_grid_3d.png', dpi=300)
        plt.show()

    def visualize_topdown(self, output_file='building_grid_topdown.png'):
        """俯视图可视化"""
        if self.grid is None:
            print("请先运行 build_grid()")
            return
            
        # 创建2D俯视图 - 最大高度投影
        height_map = np.zeros((self.grid.shape[0], self.grid.shape[1]))
        for ix in range(self.grid.shape[0]):
            for iy in range(self.grid.shape[1]):
                # 查找每个(x,y)位置的最大高度
                for iz in range(self.grid.shape[2]-1, -1, -1):
                    if self.grid[ix, iy, iz] == -1:
                        height_map[ix, iy] = iz
                        break
        
        plt.figure(figsize=(12, 10))
        plt.title("建筑物高度图 (俯视图)")
        plt.imshow(height_map.T, origin='lower', cmap='viridis', 
                  extent=[self.offset[0], 
                          self.offset[0] + self.grid.shape[0] * self.resolution,
                          self.offset[1], 
                          self.offset[1] + self.grid.shape[1] * self.resolution])
        plt.colorbar(label='高度 (网格单元)')
        plt.xlabel('东向 (m)')
        plt.ylabel('北向 (m)')
        plt.grid(True, alpha=0.3)
        plt.tight_layout()
        plt.savefig(output_file, dpi=300)
        plt.show()
        print(f"俯视图已保存为 {output_file}")