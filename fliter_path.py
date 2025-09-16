import xml.etree.ElementTree as ET
import json
import numpy as np
from pyproj import CRS, Transformer
from geopy.distance import geodesic

# === Configuration ===
KML_FILE_PATH = "data/Mk1.kml"
REF_LON = 114.162559945072
REF_LAT = 22.3158202017388
REF_ALT = 0.0
VERTIPORT_TO_REMOVE = "Vertiport 2"  # Remove this vertiport & related paths


# === Coordinate Transformation Class ===
class CoordinateTransformer:
    def __init__(self, ref_lon, ref_lat, ref_alt=0.0):
        self.ref_lon = ref_lon
        self.ref_lat = ref_lat
        self.ref_alt = ref_alt

        # Define CRS and transformers
        self.crs_geodetic = CRS.from_epsg(4326)
        self.crs_ecef = CRS.from_proj4("+proj=geocent +datum=WGS84 +units=m +no_defs")

        self.transformer_to_ecef = Transformer.from_crs(self.crs_geodetic, self.crs_ecef)
        self.transformer_to_enu = Transformer.from_pipeline(f"""
            +proj=pipeline
            +step +proj=cart +ellps=WGS84
            +step +proj=topocentric +lat_0={self.ref_lat} +lon_0={self.ref_lon} +h_0={self.ref_alt} +datum=WGS84
        """)

    def geo_to_ecef(self, lat, lon, alt=0.0):
        x, y, z = self.transformer_to_ecef.transform(lat, lon, alt)
        return x, y, z

    def geo_to_enu(self, lat, lon, alt=0.0):
        e, n, u = self.transformer_to_enu.transform(lon, lat, alt)
        return e, n, u


# === KML Parsing ===
def parse_kml(file_path):
    """Parses KML and extracts vertiports & paths."""
    namespace = {"kml": "http://www.opengis.net/kml/2.2"}
    tree = ET.parse(file_path)
    root = tree.getroot()

    placemarks = root.findall(".//{http://www.opengis.net/kml/2.2}Placemark")

    vertiports, paths = [], []
    for placemark in placemarks:
        name_elem = placemark.find("{http://www.opengis.net/kml/2.2}name")
        coordinates_elem = placemark.find(".//{http://www.opengis.net/kml/2.2}coordinates")

        if name_elem is not None and coordinates_elem is not None:
            name = name_elem.text
            coordinates = coordinates_elem.text.strip()
            coord_list = [tuple(map(float, c.split(','))) for c in coordinates.split()]

            if "vertiport" in name.lower():
                vertiports.append({"name": name, "coordinates": coord_list})
            elif "path" in name.lower():
                paths.append({"name": name, "coordinates": coord_list})

    return vertiports, paths


# === Finding the Closest Vertiports Using Haversine Distance ===
def find_two_nearest_vertiports(point, vertiports):
    """Finds the two nearest distinct vertiports based on geodesic distance."""
    distances = []
    
    for vertiport in vertiports:
        for v_point in vertiport["coordinates"]:
            distance = geodesic((v_point[1], v_point[0]), (point[1], point[0])).meters
            distances.append((distance, vertiport["name"]))

    # Sort by distance
    distances.sort()

    if len(distances) < 2:
        return distances[0][1], distances[0][1]  # If only one vertiport exists, return it twice
    
    start_vp = distances[0][1]  # Closest vertiport
    end_vp = distances[1][1] if distances[1][1] != start_vp else distances[2][1]  # Ensure it's different

    return start_vp, end_vp


# === Processing Data ===
def process_data(vertiports, paths, transformer):
    vertiport_results = []
    vertiport_dict = {}

    # Process vertiports
    for i, vertiport in enumerate(vertiports):
        transformed_points = []

        for coord in vertiport["coordinates"]:
            lon, lat, alt = coord
            ecef_coords = transformer.geo_to_ecef(lat, lon, alt)
            enu_coords = transformer.geo_to_enu(lat, lon, alt)

            transformed_points.append({
                "longitude": lon,
                "latitude": lat,
                "height": alt,
                "cartesian": {"x": ecef_coords[0], "y": ecef_coords[1], "z": ecef_coords[2]},
                "neuDistances": {"north": enu_coords[1], "east": enu_coords[0], "up": enu_coords[2]}
            })

        vertiport_results.append({"index": i, "name": vertiport["name"], "points": transformed_points})
        vertiport_dict[vertiport["name"]] = transformed_points[0]

    # Process paths
    corrected_paths = []
    for path in paths:
        start_point = path["coordinates"][0]
        end_point = path["coordinates"][-1]

        # Find the two nearest distinct vertiports
        start_vp_name, end_vp_name = find_two_nearest_vertiports(start_point, vertiports)

        expected_start_point = vertiport_dict.get(start_vp_name, {})
        expected_end_point = vertiport_dict.get(end_vp_name, {})

        corrected_path_points = []
        for coord in path["coordinates"]:
            lon, lat, alt = coord
            ecef_coords = transformer.geo_to_ecef(lat, lon, alt)
            enu_coords = transformer.geo_to_enu(lat, lon, alt)

            corrected_path_points.append({
                "longitude": lon,
                "latitude": lat,
                "height": alt,
                "cartesian": {"x": ecef_coords[0], "y": ecef_coords[1], "z": ecef_coords[2]},
                "neuDistances": {"north": enu_coords[1], "east": enu_coords[0], "up": enu_coords[2]}
            })

        if corrected_path_points:
            corrected_path_points[0] = expected_start_point
            corrected_path_points[-1] = expected_end_point

        corrected_paths.append({
            "index": path["name"],
            "name": path["name"],
            "start_vertiport": start_vp_name,
            "end_vertiport": end_vp_name,
            "start_point": expected_start_point,
            "end_point": expected_end_point,
            "path_points": corrected_path_points
        })

    return vertiport_results, corrected_paths


# === Execution ===
if __name__ == "__main__":
    transformer = CoordinateTransformer(REF_LON, REF_LAT, REF_ALT)

    # Parse KML file
    vertiports, paths = parse_kml(KML_FILE_PATH)

    # Process data
    vertiport_results, corrected_paths = process_data(vertiports, paths, transformer)

    # Save results
    with open("corrected_vertiport_transformation.json", 'w') as f:
        json.dump(vertiport_results, f, indent=4)

    with open("corrected_path_transformation.json", 'w') as f:
        json.dump(corrected_paths, f, indent=4)

    print("Transformation complete. JSON files saved.")
