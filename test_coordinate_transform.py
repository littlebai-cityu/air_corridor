import numpy as np
from coordinate_transformer import CoordinateTransformer

def test_coordinate_round_trip():
    """测试地理坐标→ENU→地理坐标的来回转换精度"""
    
    # 初始化转换器，使用参考点
    transformer = CoordinateTransformer(
        ref_lon=114.162559945072,
        ref_lat=22.3158202017388,
        ref_alt=0.0
    )
    
    # 测试点列表，包括参考点附近的多个点
    test_points = [
        (22.3158202017388, 114.162559945072, 0.0),    # 参考点
        (22.3258202017388, 114.172559945072, 100.0),  # 北偏+东偏+高
        (22.3058202017388, 114.152559945072, 50.0),   # 南偏+西偏+高
        (22.3158202017388, 114.172559945072, 25.0),   # 东偏+高
        (22.3258202017388, 114.162559945072, 75.0),   # 北偏+高
    ]
    
    print("| 原始坐标 (lat,lon,alt) | ENU坐标 (e,n,u) | 转换回的坐标 (lat,lon,alt) | 误差 (米) |")
    print("|------------------------|----------------|--------------------------|-----------|")
    
    for lat, lon, alt in test_points:
        # 地理坐标 → ENU
        east, north, up = transformer.geo_to_enu(lat, lon, alt)
        
        # ENU → 地理坐标
        converted_lat, converted_lon, converted_alt = transformer.enu_to_geo(east, north, up)
        
        # 计算误差 (使用大圆距离近似)
        lat_error = abs(lat - converted_lat) * 111320  # 1度纬度约111.32km
        lon_error = abs(lon - converted_lon) * 111320 * np.cos(np.radians(lat))  # 经度距离随纬度变化
        alt_error = abs(alt - converted_alt)
        
        # 总误差 (欧几里得距离)
        total_error = np.sqrt(lat_error**2 + lon_error**2 + alt_error**2)
        
        print(f"| ({lat:.8f}, {lon:.8f}, {alt:.1f}) | ({east:.2f}, {north:.2f}, {up:.2f}) | ({converted_lat:.8f}, {converted_lon:.8f}, {converted_alt:.1f}) | {total_error:.6f} |")

if __name__ == "__main__":
    test_coordinate_round_trip()