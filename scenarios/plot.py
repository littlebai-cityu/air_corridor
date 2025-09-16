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
# from mayavi import mlab

# # 加载OBJ文件
# mesh = trimesh.load('./data/mong kok/area_overbuilding.obj')

# # 加载高度信息文件
# with h5py.File('./data/mong kok/cells_height.hdf5', 'r') as f:
#     height_data = f['/dataset_name'][...]

# # 假设高度数据是二维数组，创建网格坐标
# x = np.arange(height_data.shape[0])
# y = np.arange(height_data.shape[1])
# x, y = np.meshgrid(x, y)

# # 绘制OBJ文件
# mesh_points = mesh.vertices
# mesh_faces = mesh.faces
# mlab.triangular_mesh([p[0] for p in mesh_points], 
#                      [p[1] for p in mesh_points], 
#                      [p[2] for p in mesh_points], 
#                      mesh_faces)

# # 绘制高度数据
# mlab.surf(x, y, height_data, colormap='terrain')

# # 添加一些装饰
# mlab.title('Simulation Environment')
# mlab.colorbar(title='Height', orientation='vertical')
# mlab.axes(xlabel='X', ylabel='Y', zlabel='Z')

# # 显示图像
# mlab.show()


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