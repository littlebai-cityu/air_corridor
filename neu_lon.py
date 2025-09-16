import numpy as np
from pyproj import Proj, CRS, Transformer
from geopy.distance import geodesic
import math
import json
import os

# WGS84 椭球参数
a = 6378137.0
f = 1 / 298.257223563
e2 = 2 * f - f ** 2

def validate_transformations(geo_points, enu_points, output_file='transformation_results.json', lat0=None, lon0=None, h0=0.0):
    if lat0 is None or lon0 is None:
        # 计算重心作为投影原点
        lat0 = np.mean([pt[0] for pt in geo_points])
        lon0 = np.mean([pt[1] for pt in geo_points])
    print(f"\n投影原点: lat = {lat0}, lon = {lon0}, h = {h0}")

    # 坐标转换器
    crs_geodetic = CRS.from_epsg(4326)
    crs_ecef = CRS.from_proj4("+proj=geocent +datum=WGS84 +units=m +no_defs")
    transformer_to_ecef = Transformer.from_crs(crs_geodetic, crs_ecef)

    transformer_to_enu = Transformer.from_pipeline(f"""
        +proj=pipeline
        +step +proj=cart +ellps=WGS84
        +step +proj=topocentric +lat_0={lat0} +lon_0={lon0} +h_0={h0} +datum=WGS84
    """)

    transformer_to_geo = Transformer.from_crs(
        f"+proj=tmerc +lat_0={lat0} +lon_0={lon0} +datum=WGS84 +units=m +vunits=m +no_defs",
        "epsg:4326",
        always_xy=True
    )

    results = []

    print("\n【ENU -> Geo 转换验证】")
    for i, (geo, enu) in enumerate(zip(geo_points, enu_points)):
        lon_trans, lat_trans = transformer_to_geo.transform(enu[0], enu[1])
        error = geodesic((geo[0], geo[1]), (lat_trans, lon_trans)).meters
        print(f"点{i+1}: 原始: ({geo[0]:.8f}, {geo[1]:.8f}) -> 转换: ({lat_trans:.8f}, {lon_trans:.8f}) 误差: {error:.2f} 米")

    print("\n【Geo -> ENU 转换验证】")
    for i, (geo, enu) in enumerate(zip(geo_points, enu_points)):
        e_trans, n_trans, u_trans = transformer_to_enu.transform(geo[1], geo[0], 0)
        error = np.linalg.norm([e_trans - enu[0] + 50, n_trans - enu[1] + 270, 0])
        x, y, z = transformer_to_ecef.transform(geo[0], geo[1], 0)
        result = {
            "index": i,
            "longitude": geo[1],
            "latitude": geo[0],
            "height": 0.0,
            "cartesian": {"x": x, "y": y, "z": z},
            "neuDistances": {"north": n_trans, "east": e_trans, "up": u_trans}
        }
        results.append(result)
        print(f"点{i+1}: 原始 ENU: ({enu[0]:.2f}, {enu[1]:.2f}, {enu[2]:.2f}) -> 转换 ENU: ({e_trans:.2f}, {n_trans:.2f}, 0.00) 误差: {error:.2f} 米")

    with open(output_file, 'w', encoding='utf-8') as f:
        json.dump(results, f, ensure_ascii=False, indent=4)
    print(f"\n✅ 转换结果已保存为 {output_file}")

# ==================== 示例调用 ====================
if __name__ == "__main__":
    geo_points = [
        (22.3158202017388, 114.162559945072),
        (22.3183900686961, 114.170399552792),
        (22.3180280834786, 114.165985613573),
        (22.3195000046385, 114.164782000316),
        (22.3210639208709, 114.167466983736),
        (22.3169, 114.163750000147),
        (22.3214778986971, 114.163856143964)
    ]

    enu_points = np.array([
        [50.0, 270.0, 0.0],
        [880.0, 540.0, 0.0],
        [420.0, 510.0, 0.0],
        [290.0, 670.0, 0.0],
        [580.0, 840.0, 0.0],
        [185.0, 375.0, 0.0],
        [190.0, 885.0, 0.0]
    ])

    validate_transformations(geo_points, enu_points)
