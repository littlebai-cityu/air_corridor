import json
import pickle
import numpy as np
from coordinate_transformer import CoordinateTransformer

def convert_routes_to_geo_json(routes, grid_res=10.0, transformer=None, output_file="routes_geo.json"):
    """
    将网格坐标的航线转换为地理坐标（经纬高）并保存为JSON文件
    
    Args:
        routes: 航线列表（网格坐标）
        grid_res: 网格分辨率，用于将网格坐标转换为实际距离
        transformer: 坐标转换器对象
        output_file: 输出的JSON文件名
    """
    if transformer is None:
        # 如果没有提供transformer，创建一个
        transformer = CoordinateTransformer(
            ref_lon=114.162559945072,
            ref_lat=22.3158202017388,
            ref_alt=0.0
        )
    
    geo_routes = []
    
    for i, route in enumerate(routes):
        geo_waypoints = []
        
        for point in route:
            # 从网格坐标转换为ENU坐标（实际距离，单位：米）
            east = point[0] * grid_res  # 东向坐标
            north = point[1] * grid_res  # 北向坐标
            up = point[2] * grid_res  # 高度坐标
            
            # 从ENU转换为地理坐标（经纬高）
            lat, lon, alt = transformer.enu_to_geo(east, north, up)
            
            geo_waypoints.append({
                "longitude": lon,
                "latitude": lat,
                "height": alt,
                "grid": {"x": float(point[0]), "y": float(point[1]), "z": float(point[2])},
                "enu": {"east": float(east), "north": float(north), "up": float(up)}
            })
        
        geo_routes.append({
            "route_id": i,
            "waypoints": geo_waypoints,
            "num_waypoints": len(geo_waypoints),
            "start_point": {
                "longitude": geo_waypoints[0]["longitude"],
                "latitude": geo_waypoints[0]["latitude"],
                "height": geo_waypoints[0]["height"]
            },
            "end_point": {
                "longitude": geo_waypoints[-1]["longitude"],
                "latitude": geo_waypoints[-1]["latitude"], 
                "height": geo_waypoints[-1]["height"]
            }
        })
    
    # 保存为JSON文件
    with open(output_file, 'w', encoding='utf-8') as f:
        json.dump(geo_routes, f, ensure_ascii=False, indent=4)
    
    print(f"转换完成。已将{len(routes)}条航线保存到JSON文件: {output_file}")
    
    return geo_routes

# 主函数：处理路径并转换
def main():
    try:
        # 加载增加了垂直起降段的航线
        input_file = './results_new/conops2/trajectories_with_vertical_segments.pkl'
        output_file = 'routes_with_intersection.json'
        
        print(f"正在加载航线数据: {input_file}")
        with open(input_file, 'rb') as f:
            routes = pickle.load(f)
        
        print(f"已加载 {len(routes)} 条航线，准备转换为地理坐标...")
        
        # 转换为地理坐标并保存
        geo_routes = convert_routes_to_geo_json(
            routes, 
            grid_res=10.0,  # 根据您的网格分辨率调整
            output_file=output_file
        )
        
        print(f"处理完成! {len(routes)} 条航线已转换为地理坐标并保存")
        
    except Exception as e:
        print(f"处理航线时出错: {str(e)}")

if __name__ == "__main__":
    main()