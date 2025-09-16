import json
import pickle
import numpy as np
import os
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from coordinate_transformer import CoordinateTransformer

import json
import pickle
import numpy as np
import os
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from coordinate_transformer import CoordinateTransformer

def add_vertical_waypoints(routes, transformer=None):
    """
    为每条航线添加垂直起降航点，并将原有航点高度提升60米
    
    Args:
        routes: 现有航线列表
        transformer: 坐标转换器，如果有的话
    
    Returns:
        添加了垂直航点和高度提升后的航线列表
    """
    if transformer is None:
        # 如果没有提供transformer，创建一个
        transformer = CoordinateTransformer(
            ref_lon=114.162559945072,
            ref_lat=22.3158202017388,
            ref_alt=0.0
        )
    
    augmented_routes = []
    for route in routes:
        for waypoint in route:
            # 转换航点坐标
            waypoint[2] = waypoint[2] +6  # 提升60米
    for route in routes:
        if len(route) < 1:
            augmented_routes.append(route)  # 跳过空路径
            continue
        
        # 获取起点和终点
        start_point = route[0]
        end_point = route[-1]
        
        # 提取起点和终点坐标
        start_x, start_y = start_point[0], start_point[1]
        end_x, end_y = end_point[0], end_point[1]
        
        # 创建垂直起飞航点 (0-60米，每10米一个)
        takeoff_waypoints = []
        for alt in range(0, 7, 1):
            takeoff_waypoints.append([start_x, start_y, alt])  # 转换为网格高度单位
        
        # 将原有航点高度全部提升60米
        elevated_route = []
        for point in route:
            # 复制原始点的x和y坐标，z坐标加上6个单位(相当于60米)
            elevated_route.append([point[0], point[1], point[2]])
        
        # 创建垂直降落航点 (120-0米，每10米一个)
        landing_waypoints = []
        for alt in range(6, -1, -1):  # 注意从120米开始降落(60米原高度 + 60米提升)
            landing_waypoints.append([end_x, end_y, alt])
        
        # 将垂直起飞航点添加到航线前面，将垂直降落航点添加到航线后面
        # 注意：去掉起飞航点的最后一点和降落航点的第一点，避免重复
        augmented_route = takeoff_waypoints[:-1] + elevated_route + landing_waypoints[1:]
        augmented_routes.append(augmented_route)
    
    return augmented_routes

def visualize_3d_routes(routes, title="航线三维可视化", save_path=None):
    """
    3D可视化航线，包括垂直起降段
    """
    fig = plt.figure(figsize=(12, 10))
    ax = fig.add_subplot(111, projection='3d')
    
    # 颜色映射
    colors = plt.cm.jet(np.linspace(0, 1, len(routes)))
    
    for i, route in enumerate(routes):
        if len(route) < 2:
            continue
            
        # 提取坐标
        xs = [point[0] for point in route]
        ys = [point[1] for point in route]
        zs = [point[2] for point in route]  # 转换高度到实际米数
        
        # 绘制航线
        ax.plot(xs, ys, zs, '-', color=colors[i], alpha=0.7, linewidth=1.5)
        
        # 标记起点和终点
        ax.scatter(xs[0], ys[0], zs[0], color='green', s=30, marker='o')
        ax.scatter(xs[-1], ys[-1], zs[-1], color='red', s=30, marker='^')
    
    # 设置标题和轴标签
    ax.set_title(title)
    ax.set_xlabel('X (东)')
    ax.set_ylabel('Y (北)')
    ax.set_zlabel('高度 (米)')
    
    # 添加图例
    ax.scatter([], [], [], color='green', s=30, marker='o', label='起飞点')
    ax.scatter([], [], [], color='red', s=30, marker='^', label='降落点')
    ax.legend()
    
    # 保存图像
    if save_path:
        plt.savefig(save_path, dpi=300, bbox_inches='tight')
        print(f"已保存3D航线可视化图像到 {save_path}")
    
    plt.tight_layout()
    plt.show()



# 示例用法
if __name__ == "__main__":
    # 加载现有航线数据
    try:
        with open('./results/conops2/dense_hub_to_hub_15orders_10aggregated_Falsedis10destinations.pkl', 'rb') as f:
            routes = pickle.load(f)
        
        # 添加垂直起降航点
        augmented_routes = add_vertical_waypoints(routes)
        
        # 保存增强后的航线
        with open('./results_new/conops2/trajectories_with_vertical_segments.pkl', 'wb') as f:
            pickle.dump(augmented_routes, f)
        
        # 可视化
        visualize_3d_routes(augmented_routes, "包含垂直起降段的航线", "vertical_segments_visualization.png")
        
        print(f"处理完成! 共处理了 {len(routes)} 条航线.")
    except Exception as e:
        print(f"处理航线时出错: {str(e)}")