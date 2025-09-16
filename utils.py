import argparse
from distutils.util import strtobool

import numpy as np


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "-a",
        "--sd",
        type=lambda x: bool(strtobool(x)),
        nargs="?",
        const=True,
        default=False,
    )
    parser.add_argument(
        "-b",
        "--h2h",
        type=lambda x: bool(strtobool(x)),
        nargs="?",
        const=True,
        default=False,
    )
    parser.add_argument(
        "-e",
        "--conops",
        type=int,
        nargs=1,
        default=1,
    )
    parser.add_argument(
        "-f",
        "--orders",
        type=int,
        nargs=1,
        default=1000,
    )
    parser.add_argument(
        "-x",
        "--shuffletimes",
        type=int,
        nargs=1,
        default=1,
    )
    parser.add_argument(
        "-y",
        "--aggregated",
        type=float,
        nargs=1,
        default=[0.9],
    )
    parser.add_argument(
        "-dest",
        "--destination",
        type=int,
        nargs=1,
        default=[20],
    )
    parser.add_argument(
        "-t",
        "--turning_parameters",
        nargs=2,
        type=float,
        help="[cost_per_degree, normalizer_for_short_turn], default %(default)s",
        default=[0.0, 1.0],
    )
    parser.add_argument(
        "-c",
        "--climb_parameters",
        nargs=3,
        type=float,
        default=[0.0, 0.0, 90.0],
        help="[cost_per_degree_climb, cost_per_degree_descend, climb_threshold], default %(default)s",
    )
    parser.add_argument(
        "-s",
        "--space_parameters",
        nargs=2,
        type=float,
        default=[0.0, 0],
        help="[cost_per_cell, protected_zone_size], default %(default)s",
    )
    parser.add_argument(
        "-g",
        "--grid_steps",
        nargs=3,
        type=float,
        help="grid steps in xyz direction, default %(default)s",
        default=[10.0, 10.0, 10.0],
    )
    parser.add_argument(
        "-z",
        "--zlimits",
        nargs=2,
        type=float,
        help="normal layers height limits, default %(default)s meters",
        default=[90, 100]
    )
    parser.add_argument(
        "-v",
        "--epsilon_v",
        type=float,
        help="value threshold for route prioritizing, default %(default)s",
        default=1000,
    )
    parser.add_argument(
        "-d",
        "--epsilon_d",
        type=float,
        help="distance threshold for route prioritizing,  default %(default)s meters",
        default=300,
    )
    parser.add_argument(
        "-p", "--prioritization", type=lambda x: bool(strtobool(x)),
        nargs="?",
        const=True,
        default=True,
        help="enable route prioritization, default %(default)s"
    )
    parser.add_argument(
        "--nonuniform",
        type=lambda x: bool(strtobool(x)),
        nargs="?",
        const=True,
        default=True,
        help="extend algorithms to use non-uniform traversal cost, default %(default)s",
    )
    parser.add_argument(
        "--operational",
        type=lambda x: bool(strtobool(x)),
        nargs="?",
        const=True,
        default=True,
        help="extend algorithms to use operational cost, default %(default)s",
    )
    args = parser.parse_args()
    print(args)

    assert np.all(np.array(args.grid_steps) > 0)
    assert args.zlimits[0] < args.zlimits[1]

    return args

import pyproj
def geo_to_ecef(lon_rad, lat_rad, alt):
    '''
    
    Coordinate transform

    '''
    a = 6378137.0
    finv = 298.257223563
    f = 1 / finv
    e2 = 1 - (1 - f) ** 2
    v = a / np.sqrt(1 - e2 * np.sin(lat_rad) * np.sin(lat_rad))

    x = (v + alt) * np.cos(lat_rad) * np.cos(lon_rad)
    y = (v + alt) * np.cos(lat_rad) * np.sin(lon_rad)
    z = (v * (1 - e2) + alt) * np.sin(lat_rad)

    return np.array([x, y, z])

def geo_to_enu(lon, lat, alt, x0, lon0, lat0):
    '''
    
    Coordinate transform
    
    '''
    lon_rad = lon * (np.pi / 180.0)
    lat_rad = lat * (np.pi / 180.0)

    x = geo_to_ecef(lon_rad, lat_rad, alt)

    return np.array([
        [-np.sin(lon0), np.cos(lon0), 0],
        [-np.cos(lon0)*np.sin(lat0), -np.sin(lon0)*np.sin(lat0), np.cos(lat0)],
        [np.cos(lon0)*np.cos(lat0), np.sin(lon0)*np.cos(lat0), np.sin(lat0)]
    ]) @ ( np.array(x) - np.array(x0) )
def ecef_to_geo(x,y,z):
    '''
    
    Coordinate transform
    
    '''
    ecef = pyproj.Proj(proj='geocent', ellps='WGS84', datum='WGS84')
    lla = pyproj.Proj(proj='latlong', ellps='WGS84', datum='WGS84')
    lon, lat, alt = pyproj.transform(ecef, lla, x, y, z, radians=False)
    return lon, lat, alt

def enu_to_geo(x, y, z, x0, lon0, lat0):
    '''
    
    Coordinate transform
    
    '''
    (X,Y,Z)=np.array([
        [-np.sin(lon0), -np.sin(lat0)*np.cos(lon0), np.cos(lat0)*np.cos(lon0)],
        [np.cos(lon0), -np.sin(lat0) * np.sin(lon0), np.cos(lat0)*np.sin(lon0)],
        [0, np.cos(lat0), np.sin(lat0)]
    ]) @ np.array([x,y,z]) + np.array(x0)
    lon, lat, alt = ecef_to_geo(X, Y, Z)
    return lon, lat, alt

def enu_to_geo_adjust(x, y, z, x0, lon0, lat0):
    '''
    
    Coordinate transform
    
    '''
    lon,lat,alt=enu_to_geo(x, y, 0, x0, lon0, lat0)
    alt=z
    return np.round(lon,8),np.round(lat,8),alt



