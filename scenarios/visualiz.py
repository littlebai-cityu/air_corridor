import numpy as np
import matplotlib.pyplot as plt
from matplotlib.colors import Normalize, LinearSegmentedColormap
from matplotlib.collections import LineCollection
from scipy.stats import gaussian_kde
import pickle

class PathVisualizer:
    def __init__(self, norm_path):
        self.norm_path = norm_path

    def save_path(self, file_name):
        fig, (ax1, ax2, ax3) = plt.subplots(1, 3, figsize=(18, 6))  # 创建三个子图
        ax1.tick_params(axis='both', which='major', labelsize=10)
        ax2.tick_params(axis='both', which='major', labelsize=10)
        ax3.tick_params(axis='both', which='major', labelsize=10)

        # 收集所有点的坐标
        all_x = []
        all_y = []
        all_z = []
        for trajectory in self.norm_path:
            for waypoint in trajectory:
                all_x.append(waypoint[0])
                all_y.append(waypoint[1])
                all_z.append(waypoint[2])

        # 计算点的密度
        xy = np.vstack([all_x, all_y])
        kde = gaussian_kde(xy)
        density = kde(xy)
        log_density = np.log1p(density)

        # 调整颜色映射范围
        norm = Normalize(vmin=np.min(log_density), vmax=np.max(log_density) * 0.8)
        cmap = plt.get_cmap('Blues')

        # 准备批量绘制的数据
        lines_top = []
        colors_top = []
        lines_side = []
        colors_side = []
        lines_perspective = []
        colors_perspective = []

        for trajectory in self.norm_path:
            x = [point[0] for point in trajectory]
            y = [point[1] for point in trajectory]
            z = [point[2] for point in trajectory]
            points = np.vstack([x, y])
            point_density = kde(points)
            log_point_density = np.log1p(point_density)
            color = cmap(norm(log_point_density))

            for i in range(len(x) - 1):
                # Top view
                lines_top.append([(x[i], y[i]), (x[i+1], y[i+1])])
                colors_top.append(color[i])

                # Side view
                lines_side.append([(x[i], z[i]), (x[i+1], z[i+1])])
                colors_side.append(color[i])

                # Perspective view
                lines_perspective.append([(x[i], y[i], z[i]), (x[i+1], y[i+1], z[i+1])])
                colors_perspective.append(color[i])

        # 批量绘制俯视图
        lc_top = LineCollection(lines_top, colors=colors_top, linewidths=0.5)
        ax1.add_collection(lc_top)
        ax1.set_xlabel('X')
        ax1.set_ylabel('Y')
        ax1.set_title('Top-Down View')

        # 批量绘制侧视图
        lc_side = LineCollection(lines_side, colors=colors_side, linewidths=0.5)
        ax2.add_collection(lc_side)
        ax2.set_xlabel('X')
        ax2.set_ylabel('Z')
        ax2.set_title('Side View')

        # 批量绘制透视图
        for line, color in zip(lines_perspective, colors_perspective):
            ax3.plot([line[0][0], line[1][0]], [line[0][1], line[1][1]], zs=[line[0][2], line[1][2]], color=color, linewidth=0.5)
        ax3.set_xlabel('X')
        ax3.set_ylabel('Y')
        ax3.set_zlabel('Z')
        ax3.set_title('Perspective View')

        # 设置所有子图的坐标轴范围和比例一致
        all_axes = [ax1, ax2, ax3]
        for ax in all_axes:
            ax.set_xlim(min(all_x), max(all_x))
            ax.set_ylim(min(all_y), max(all_y))
            if ax == ax3:
                ax.set_zlim(min(all_z), max(all_z))

        plt.savefig(file_name, dpi=300)
        plt.close(fig)  # 保存并关闭图像

# 示例数据加载
# with open('./results/conops1_4d_20ver_10m/trajectoriesdense_hub_to_destination_100orders_10aggregated_Falsedis.pkl', 'rb') as f:
#     routes = pickle.load(f)

# 假设 norm_path 是已经加载的轨迹数据
norm_path = [...]  # 替换为实际数据

visualizer = PathVisualizer(norm_path)
visualizer.save_path('./results/conops1_4d_20ver_10m/side_and_perspective_views.png')