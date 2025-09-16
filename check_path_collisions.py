#!/usr/bin/env python3
# filepath: /home/shuangxia/air_corridor_transportation/check_path_collisions.py
import numpy as np
import pickle
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import os
from pathlib import Path
from scenarios.custom_env import custom_env_builder
import argparse

def check_path_collisions(trajectory_file, env, output_dir=None, visualize=True):
    """检查轨迹是否与建筑物碰撞
    
    参数:
        trajectory_file: 轨迹文件路径（.pkl格式）
        env: 环境对象
        output_dir: 输出目录，如果为None则使用轨迹文件所在目录
        visualize: 是否生成可视化
    
    返回:
        collision_summary: 碰撞统计信息的字典
    """
    print(f"正在检查轨迹文件: {trajectory_file}")
    
    # 创建输出目录
    if output_dir is None:
        output_dir = Path(trajectory_file).parent
    else:
        output_dir = Path(output_dir)
    
    output_dir.mkdir(exist_ok=True, parents=True)
    
    # 加载轨迹
    try:
        with open(trajectory_file, 'rb') as f:
            trajectories = pickle.load(f)
        print(f"已加载 {len(trajectories)} 条轨迹")
    except Exception as e:
        print(f"无法加载轨迹文件: {e}")
        return None
    
    # 初始化结果
    collision_summary = {
        'total_paths': len(trajectories),
        'collision_paths': 0,
        'total_points': 0,
        'collision_points': 0,
        'collision_details': []
    }
    
    # 检查每条路径
    for path_idx, traj in enumerate(trajectories):
        path_has_collision = False
        collision_points = []
        path_points = []
        
        # 对于每个坐标点
        for i in range(len(traj) - 1):  # 每个轨迹至少有起点和终点
            point = traj[i]
            
            # 提取xyz位置（忽略时间戳）
            if len(point) >= 4:  # [x, y, z, t]
                x, y, z = int(point[0]), int(point[1]), int(point[2])
            else:  # [x, y, z]
                x, y, z = int(point[0]), int(point[1]), int(point[2])
            
            # 检查点是否在网格范围内
            if 0 <= x < env.cells_type.shape[0] and 0 <= y < env.cells_type.shape[1] and 0 <= z < env.cells_type.shape[2]:
                # 检查该位置是否是障碍物
                if env.cells_type[x, y, z] == -1:  # -1表示障碍物
                    path_has_collision = True
                    collision_points.append((x, y, z))
                
                path_points.append((x, y, z))
                collision_summary['total_points'] += 1
            else:
                print(f"警告: 路径 {path_idx+1} 点 {i+1} 坐标超出范围: [{x},{y},{z}]")
        
        # 更新统计信息
        if path_has_collision:
            collision_summary['collision_paths'] += 1
            collision_summary['collision_points'] += len(collision_points)
            collision_summary['collision_details'].append({
                'path_index': path_idx,
                'collision_count': len(collision_points),
                'collision_points': collision_points,
                'path_points': path_points
            })
    
    # 生成报告
    report_file = output_dir / f"collision_report_{Path(trajectory_file).stem}.txt"
    with open(report_file, 'w') as f:
        f.write(f"轨迹碰撞检查报告\n")
        f.write(f"====================\n\n")
        f.write(f"检查文件: {trajectory_file}\n")
        f.write(f"总路径数: {collision_summary['total_paths']}\n")
        f.write(f"碰撞路径数: {collision_summary['collision_paths']}\n")
        f.write(f"碰撞率: {collision_summary['collision_paths']/max(1,collision_summary['total_paths'])*100:.2f}%\n")
        f.write(f"总点数: {collision_summary['total_points']}\n")
        f.write(f"碰撞点数: {collision_summary['collision_points']}\n")
        f.write(f"点级碰撞率: {collision_summary['collision_points']/max(1,collision_summary['total_points'])*100:.2f}%\n\n")
        
        if collision_summary['collision_paths'] > 0:
            f.write(f"碰撞路径详情:\n")
            for detail in collision_summary['collision_details']:
                f.write(f"  路径 #{detail['path_index']+1}: {detail['collision_count']} 个碰撞点\n")
    
    print(f"碰撞检查报告已保存到: {report_file}")
    
    # 可视化碰撞路径
    if visualize and collision_summary['collision_paths'] > 0:
        print("生成碰撞路径可视化...")
        visualize_collisions(env, collision_summary, output_dir, trajectory_file)
    
    # 打印摘要
    print(f"\n碰撞检查摘要:")
    print(f"总路径数: {collision_summary['total_paths']}")
    print(f"碰撞路径数: {collision_summary['collision_paths']} ({collision_summary['collision_paths']/max(1,collision_summary['total_paths'])*100:.2f}%)")
    print(f"总点数: {collision_summary['total_points']}")
    print(f"碰撞点数: {collision_summary['collision_points']} ({collision_summary['collision_points']/max(1,collision_summary['total_points'])*100:.2f}%)")
    
    return collision_summary

def visualize_collisions(env, collision_summary, output_dir, trajectory_file):
    """可视化碰撞路径
    
    参数:
        env: 环境对象
        collision_summary: 碰撞统计信息的字典
        output_dir: 输出目录
        trajectory_file: 轨迹文件路径，用于生成输出文件名
    """
    # 限制可视化的路径数量，避免过多图形
    max_vis_paths = min(10, collision_summary['collision_paths'])
    
    # 创建一个新的目录用于存放碰撞路径可视化
    vis_dir = output_dir / f"collision_vis_{Path(trajectory_file).stem}"
    vis_dir.mkdir(exist_ok=True)
    
    # 为总览图做准备
    fig_overview = plt.figure(figsize=(15, 12))
    ax_overview = fig_overview.add_subplot(111, projection='3d')
    ax_overview.set_title(f"所有碰撞路径概览 (显示前{max_vis_paths}条)")
    
    # 为每条碰撞路径生成单独的可视化
    for i, detail in enumerate(collision_summary['collision_details'][:max_vis_paths]):
        path_idx = detail['path_index']
        path_points = np.array(detail['path_points'])
        collision_points = np.array(detail['collision_points'])
        
        # 单独的路径图
        fig = plt.figure(figsize=(10, 8))
        ax = fig.add_subplot(111, projection='3d')
        
        # 绘制路径
        ax.plot(path_points[:, 0], path_points[:, 1], path_points[:, 2], 
                'b-', linewidth=2, alpha=0.7, label='路径')
        
        # 绘制碰撞点
        if len(collision_points) > 0:
            ax.scatter(collision_points[:, 0], collision_points[:, 1], collision_points[:, 2], 
                      c='red', s=50, marker='x', label='碰撞点')
        
        # 设置图形属性
        ax.set_title(f"路径 #{path_idx+1} 碰撞可视化 ({len(collision_points)}个碰撞点)")
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        ax.legend()
        
        # 保存图形
        fig.savefig(vis_dir / f"collision_path_{path_idx+1}.png", dpi=200, bbox_inches='tight')
        plt.close(fig)
        
        # 添加到总览图
        color = plt.cm.jet(i / max_vis_paths)
        ax_overview.plot(path_points[:, 0], path_points[:, 1], path_points[:, 2], 
                         '-', linewidth=2, alpha=0.7, color=color, label=f'路径 #{path_idx+1}')
        if len(collision_points) > 0:
            ax_overview.scatter(collision_points[:, 0], collision_points[:, 1], collision_points[:, 2], 
                               color='red', s=30, marker='x')
    
    # 设置总览图属性
    ax_overview.set_xlabel('X')
    ax_overview.set_ylabel('Y')
    ax_overview.set_zlabel('Z')
    ax_overview.legend()
    
    # 保存总览图
    fig_overview.savefig(vis_dir / "all_collision_paths.png", dpi=300, bbox_inches='tight')
    plt.close(fig_overview)
    
    print(f"碰撞路径可视化已保存到: {vis_dir}")

def create_obstacle_heatmap(env, collision_summary, output_dir, trajectory_file):
    """创建障碍物热图，显示碰撞频率最高的区域
    
    参数:
        env: 环境对象
        collision_summary: 碰撞统计信息
        output_dir: 输出目录
        trajectory_file: 轨迹文件路径
    """
    # 创建热图数据结构
    heatmap = np.zeros(env.cells_type.shape)
    
    # 统计每个位置的碰撞次数
    for detail in collision_summary['collision_details']:
        for x, y, z in detail['collision_points']:
            heatmap[x, y, z] += 1
    
    # 找出碰撞最频繁的三个平面
    collision_counts_xy = np.sum(heatmap, axis=2)
    collision_counts_xz = np.sum(heatmap, axis=1)
    collision_counts_yz = np.sum(heatmap, axis=0)
    
    most_collided_z = np.argmax(np.sum(collision_counts_xy, axis=(0, 1)))
    most_collided_y = np.argmax(np.sum(collision_counts_xz, axis=(0, 1)))
    most_collided_x = np.argmax(np.sum(collision_counts_yz, axis=(0, 1)))
    
    # 创建热图可视化
    fig, axs = plt.subplots(1, 3, figsize=(18, 6))
    
    # XY平面 (最高碰撞的Z值)
    im0 = axs[0].imshow(collision_counts_xy.T, cmap='hot', interpolation='nearest')
    axs[0].set_title(f'XY平面碰撞热图\n(总碰撞: {np.sum(collision_counts_xy)})')
    axs[0].set_xlabel('X')
    axs[0].set_ylabel('Y')
    plt.colorbar(im0, ax=axs[0])
    
    # XZ平面 (最高碰撞的Y值)
    im1 = axs[1].imshow(collision_counts_xz, cmap='hot', interpolation='nearest')
    axs[1].set_title(f'XZ平面碰撞热图\n(总碰撞: {np.sum(collision_counts_xz)})')
    axs[1].set_xlabel('X')
    axs[1].set_ylabel('Z')
    plt.colorbar(im1, ax=axs[1])
    
    # YZ平面 (最高碰撞的X值)
    im2 = axs[2].imshow(collision_counts_yz, cmap='hot', interpolation='nearest')
    axs[2].set_title(f'YZ平面碰撞热图\n(总碰撞: {np.sum(collision_counts_yz)})')
    axs[2].set_xlabel('Y')
    axs[2].set_ylabel('Z')
    plt.colorbar(im2, ax=axs[2])
    
    plt.tight_layout()
    
    # 保存热图
    heatmap_file = output_dir / f"collision_heatmap_{Path(trajectory_file).stem}.png"
    plt.savefig(heatmap_file, dpi=300, bbox_inches='tight')
    plt.close()
    
    print(f"碰撞热图已保存到: {heatmap_file}")

def main():
    parser = argparse.ArgumentParser(description="检查生成的路径是否穿过建筑物内部")
    parser.add_argument("--trajectory", required=True, 
                        help="轨迹文件路径（.pkl格式）")
    parser.add_argument("--kml", default="data/Mk1.kml",
                        help="KML文件路径")
    parser.add_argument("--resolution", type=float, default=5.0, 
                        help="网格分辨率（米）")
    parser.add_argument("--output", default=None, 
                        help="输出目录")
    parser.add_argument("--no-vis", action="store_true", 
                        help="禁用可视化")
    
    args = parser.parse_args()
    
    # 加载环境
    print(f"加载环境...")
    env = custom_env_builder(
        kml_path=args.kml,
        ref_lon=114.162559945072,
        ref_lat=22.3158202017388,
        ref_alt=0.0,
        resolution=args.resolution,
        buffer_cells=0.5
    )
    
    # 检查碰撞
    collision_summary = check_path_collisions(
        args.trajectory, 
        env, 
        output_dir=args.output,
        visualize=not args.no_vis
    )
    
    # 如果有碰撞，生成热图
    if collision_summary and collision_summary['collision_paths'] > 0:
        output_dir = Path(args.output) if args.output else Path(args.trajectory).parent
        create_obstacle_heatmap(env, collision_summary, output_dir, args.trajectory)

if __name__ == "__main__":
    main()