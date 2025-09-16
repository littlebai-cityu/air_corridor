import numpy as np
from pyproj import CRS, Transformer


class CoordinateTransformer:
    def __init__(self, ref_lon, ref_lat, ref_alt=0.0):
        """
        Initialize the coordinate transformer with a reference point for ENU conversion.

        :param ref_lon: Reference longitude for ENU conversion.
        :param ref_lat: Reference latitude for ENU conversion.
        :param ref_alt: Reference altitude for ENU conversion (default is 0).
        """
        self.ref_lon = ref_lon
        self.ref_lat = ref_lat
        self.ref_alt = ref_alt

        # Define CRS and transformers
        self.crs_geodetic = CRS.from_epsg(4326)  # WGS84 Geodetic coordinates
        self.crs_ecef = CRS.from_proj4("+proj=geocent +datum=WGS84 +units=m +no_defs")

        # Define transformation functions
        self.transformer_to_ecef = Transformer.from_crs(self.crs_geodetic, self.crs_ecef)
        self.transformer_to_enu = Transformer.from_pipeline(f"""
            +proj=pipeline
            +step +proj=cart +ellps=WGS84
            +step +proj=topocentric +lat_0={self.ref_lat} +lon_0={self.ref_lon} +h_0={self.ref_alt} +datum=WGS84
        """)

    def geo_to_ecef(self, lat, lon, alt=0.0):
        """
        Convert WGS84 geodetic coordinates (lat, lon, alt) to ECEF (Earth-Centered, Earth-Fixed) coordinates.
        :param lat: Latitude in degrees.
        :param lon: Longitude in degrees.
        :param alt: Altitude in meters (default is 0).
        :return: (x, y, z) tuple representing ECEF coordinates.
        """
        x, y, z = self.transformer_to_ecef.transform(lat, lon, alt)
        return x, y, z

    def geo_to_enu(self, lat, lon, alt=0.0):
        """
        Convert WGS84 geodetic coordinates (lat, lon, alt) to ENU (East-North-Up) coordinates.
        :param lat: Latitude in degrees.
        :param lon: Longitude in degrees.
        :param alt: Altitude in meters (default is 0).
        :return: (east, north, up) tuple representing ENU coordinates.
        """
        e, n, u = self.transformer_to_enu.transform(lon, lat, alt)
        return e, n, u

    def ecef_to_geo(self, x, y, z):
        """
        Convert ECEF coordinates (x, y, z) back to WGS84 geodetic coordinates (latitude, longitude, altitude).
        :param x: ECEF X coordinate.
        :param y: ECEF Y coordinate.
        :param z: ECEF Z coordinate.
        :return: (latitude, longitude, altitude) tuple in degrees and meters.
        """
        transformer_to_geo = Transformer.from_crs(self.crs_ecef, self.crs_geodetic)
        lat, lon, alt = transformer_to_geo.transform(x, y, z)
        return lat, lon, alt
    def enu_to_geo(self, east, north, up):
        """
        将ENU坐标转换回地理坐标（纬度、经度、高度）
        
        Args:
            east: 东向坐标（米）
            north: 北向坐标（米）
            up: 上向坐标（米）
            
        Returns:
            tuple: (纬度, 经度, 高度)
        """
        # 1. 计算参考点的ECEF坐标
        ref_ecef = self.geo_to_ecef(self.ref_lat, self.ref_lon, self.ref_alt)
        
        # 2. 创建从ENU到ECEF的变换矩阵
        sin_lat = np.sin(np.radians(self.ref_lat))
        cos_lat = np.cos(np.radians(self.ref_lat))
        sin_lon = np.sin(np.radians(self.ref_lon))
        cos_lon = np.cos(np.radians(self.ref_lon))
        
        # 矩阵的列分别对应East, North, Up在ECEF中的方向
        enu_to_ecef = np.array([
            [-sin_lon, -sin_lat*cos_lon, cos_lat*cos_lon],
            [cos_lon, -sin_lat*sin_lon, cos_lat*sin_lon],
            [0, cos_lat, sin_lat]
        ])
        
        # 3. 将ENU坐标转换为相对ECEF的位移
        enu = np.array([east, north, up])
        delta_ecef = enu_to_ecef @ enu  # 使用矩阵乘法
        
        # 4. 计算绝对ECEF坐标
        ecef_x = ref_ecef[0] + delta_ecef[0]
        ecef_y = ref_ecef[1] + delta_ecef[1]
        ecef_z = ref_ecef[2] + delta_ecef[2]
        
        # 5. 将ECEF转换为地理坐标
        # WGS-84参数
        a = 6378137.0  # 赤道半径（米）
        f = 1/298.257223563  # 扁率
        b = a * (1 - f)  # 极半径
        e2 = 1 - (b/a)**2  # 第一偏心率平方
        
        # 计算经度（这是直接的）
        lon = np.degrees(np.arctan2(ecef_y, ecef_x))
        
        # 计算纬度和高度（需要迭代）
        p = np.sqrt(ecef_x**2 + ecef_y**2)
        
        # Ferrari的解法
        lat = np.arctan2(ecef_z, p * (1 - e2))
        
        # 迭代改进纬度
        for _ in range(10):  # 增加迭代次数以提高精度
            N = a / np.sqrt(1 - e2 * np.sin(lat)**2)
            h = p / np.cos(lat) - N
            lat_new = np.arctan2(ecef_z, p * (1 - e2 * N / (N + h)))
            if abs(lat - lat_new) < 1e-10:  # 收敛检查
                break
            lat = lat_new
        
        # 最终高度计算
        N = a / np.sqrt(1 - e2 * np.sin(lat)**2)
        h = p / np.cos(lat) - N
        
        return np.degrees(lat), lon, h