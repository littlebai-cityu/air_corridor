import xml.etree.ElementTree as ET
import pandas as pd

class KMLParser:
    def __init__(self, file_path):
        """
        Initialize the KML parser with a given file path.
        :param file_path: Path to the KML file.
        """
        self.file_path = file_path
        self.namespace = {"kml": "http://www.opengis.net/kml/2.2"}
        self.obstacles = []
        self.vertiports = []
        self.paths = []

    def parse_kml(self):
        """
        Parses the KML file and extracts obstacles, vertiports, and paths.
        """
        try:
            tree = ET.parse(self.file_path)
            root = tree.getroot()
            placemarks = root.findall(".//{http://www.opengis.net/kml/2.2}Placemark")

            for placemark in placemarks:
                name_elem = placemark.find("{http://www.opengis.net/kml/2.2}name")
                coordinates_elem = placemark.find(".//{http://www.opengis.net/kml/2.2}coordinates")

                if name_elem is not None and coordinates_elem is not None:
                    name = name_elem.text
                    coordinates = coordinates_elem.text.strip()

                    # Convert to (longitude, latitude, altitude) tuples
                    coord_list = [tuple(map(float, c.split(','))) for c in coordinates.split()]

                    if "obstacle" in name.lower():
                        self.obstacles.append({"name": name, "coordinates": coord_list})
                    elif "vertiport" in name.lower():
                        self.vertiports.append({"name": name, "coordinates": coord_list})
                    elif "path" in name.lower():
                        self.paths.append({"name": name, "coordinates": coord_list})

        except Exception as e:
            print(f"Error parsing KML file: {e}")

    def get_obstacles(self):
        """
        Returns a DataFrame of obstacles.
        """
        return pd.DataFrame(self.obstacles)

    def get_vertiports(self):
        """
        Returns a DataFrame of vertiports.
        """
        return pd.DataFrame(self.vertiports)

    def get_paths(self):
        """
        Returns a DataFrame of paths.
        """
        return pd.DataFrame(self.paths)

    def save_to_csv(self, obstacles_file="obstacles.csv", vertiports_file="vertiports.csv", paths_file="paths.csv"):
        """
        Saves extracted data to CSV files.
        :param obstacles_file: Filename for obstacles CSV.
        :param vertiports_file: Filename for vertiports CSV.
        :param paths_file: Filename for paths CSV.
        """
        self.get_obstacles().to_csv(obstacles_file, index=False)
        self.get_vertiports().to_csv(vertiports_file, index=False)
        self.get_paths().to_csv(paths_file, index=False)
        print("Data saved to CSV files.")

