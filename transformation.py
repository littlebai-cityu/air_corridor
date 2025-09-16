import json
import pickle
import simplekml
from coordinate_transformer import CoordinateTransformer

def convert_routes_to_geo(routes, grid_res=10.0, ref_lon=114.162559945072, 
                          ref_lat=22.3158202017388, ref_alt=0.0):
    transformer = CoordinateTransformer(ref_lon=ref_lon, ref_lat=ref_lat, ref_alt=ref_alt)
    geo_routes = []

    for route in routes:
        geo_route = []
        for point in route:
            east = point[0] * grid_res
            north = point[1] * grid_res
            up = point[2] * grid_res
            lat, lon, alt = transformer.enu_to_geo(east, north, up + 7)
            geo_route.append((lon, lat, alt))
        geo_routes.append(geo_route)
    return geo_routes, transformer

def save_routes_as_kml(geo_routes, output_file="routes.kml", 
                      route_colors=None, route_widths=None):
    kml = simplekml.Kml()
    if route_colors is None:
        route_colors = []
        for i in range(len(geo_routes)):
            hue = i / len(geo_routes) if len(geo_routes) > 1 else 0.5
            if hue < 1/6:
                r, g, b = 1, 6*hue, 0
            elif hue < 2/6:
                r, g, b = 2-6*hue, 1, 0
            elif hue < 3/6:
                r, g, b = 0, 1, 6*hue-2
            elif hue < 4/6:
                r, g, b = 0, 4-6*hue, 1
            elif hue < 5/6:
                r, g, b = 6*hue-4, 0, 1
            else:
                r, g, b = 1, 0, 6-6*hue
            color = simplekml.Color.rgb(int(255*r), int(255*g), int(255*b), 200)
            route_colors.append(color)

    if route_widths is None:
        route_widths = [4.0] * len(geo_routes)

    fld = kml.newfolder(name="UAV Routes")
    for i, route in enumerate(geo_routes):
        ls = fld.newlinestring(name=f"Route {i+1}")
        ls.coords = route
        ls.style.linestyle.color = route_colors[i]
        ls.style.linestyle.width = route_widths[i]
        ls.altitudemode = simplekml.AltitudeMode.relativetoground

        if route:
            start = fld.newpoint(name=f"Start {i+1}")
            start.coords = [route[0]]
            start.style.iconstyle.icon.href = 'http://maps.google.com/mapfiles/kml/paddle/grn-circle.png'
            start.style.iconstyle.scale = 1.0
            start.altitudemode = simplekml.AltitudeMode.relativetoground

            end = fld.newpoint(name=f"End {i+1}")
            end.coords = [route[-1]]
            end.style.iconstyle.icon.href = 'http://maps.google.com/mapfiles/kml/paddle/red-circle.png'
            end.style.iconstyle.scale = 1.0
            end.altitudemode = simplekml.AltitudeMode.relativetoground

        ls.description = f"Route {i+1}，with {len(route)} waypoints"

    kml.save(output_file)
    print(f"✅ 已保存 KML: {output_file}")

def generate_vertiports_from_routes(geo_routes, transformer, output_file="vertiport.json"):
    vertiports = []
    vertiport_names = []
    index_counter = 0

    for i, route in enumerate(geo_routes):
        entry = []
        for point in [route[0], route[-1]]:
            lon, lat, alt = point
            x, y, z = transformer.geo_to_ecef(lat, lon, alt)
            e, n, u = transformer.geo_to_enu(lat, lon, alt)

            name = f"vertiport {index_counter + 1}"
            vertiports.append({
                "index": index_counter,
                "name": name,
                "points": [{
                    "longitude": lon,
                    "latitude": lat,
                    "height": alt,
                    "cartesian": {"x": x, "y": y, "z": z},
                    "neuDistances": {"north": n, "east": e, "up": u}
                }]
            })
            entry.append(name)
            index_counter += 1
        vertiport_names.append(tuple(entry))

    with open(output_file, "w", encoding="utf-8") as f:
        json.dump(vertiports, f, ensure_ascii=False, indent=4)
    print(f"✅ 已保存 vertiport.json，共 {index_counter} 个")

    return vertiport_names

def generate_paths_from_routes(geo_routes, transformer, vertiport_name_map, output_file="path.json"):
    paths = []

    for i, route in enumerate(geo_routes):
        start_name, end_name = vertiport_name_map[i]
        start_lon, start_lat, start_alt = route[0]
        end_lon, end_lat, end_alt = route[-1]

        x1, y1, z1 = transformer.geo_to_ecef(start_lat, start_lon, start_alt)
        e1, n1, u1 = transformer.geo_to_enu(start_lat, start_lon, start_alt)

        x2, y2, z2 = transformer.geo_to_ecef(end_lat, end_lon, end_alt)
        e2, n2, u2 = transformer.geo_to_enu(end_lat, end_lon, end_alt)

        path_points = []
        for lon, lat, alt in route:
            x, y, z = transformer.geo_to_ecef(lat, lon, alt)
            e, n, u = transformer.geo_to_enu(lat, lon, alt)
            path_points.append({
                "longitude": lon,
                "latitude": lat,
                "height": alt,
                "cartesian": {"x": x, "y": y, "z": z},
                "neuDistances": {"north": n, "east": e, "up": u}
            })

        path = {
            "index": i,
            "name": f"path {i+1}",
            "start_vertiport": start_name,
            "end_vertiport": end_name,
            "start_point": {
                "longitude": start_lon,
                "latitude": start_lat,
                "height": start_alt,
                "cartesian": {"x": x1, "y": y1, "z": z1},
                "neuDistances": {"north": n1, "east": e1, "up": u1}
            },
            "end_point": {
                "longitude": end_lon,
                "latitude": end_lat,
                "height": end_alt,
                "cartesian": {"x": x2, "y": y2, "z": z2},
                "neuDistances": {"north": n2, "east": e2, "up": u2}
            },
            "path_points": path_points
        }

        paths.append(path)

    with open(output_file, "w", encoding="utf-8") as f:
        json.dump(paths, f, ensure_ascii=False, indent=4)
    print(f"✅ 已保存 path.json 共 {len(paths)} 条")

def main():
    try:
        input_file = './results_new/conops2/trajectories_with_vertical_segments.pkl'
        output_json = 'routes_geo_custom.json'
        output_kml = 'routes_with_intersection.kml'
        output_vertiport = 'vertiport.json'
        output_path = 'path.json'

        with open(input_file, 'rb') as f:
            routes = pickle.load(f)

        print(f"📦 载入航线数：{len(routes)}")

        geo_routes, transformer = convert_routes_to_geo(
            routes,
            grid_res=10.0
        )

        with open(output_json, 'w', encoding='utf-8') as f:
            geo_json = [{
                "route_id": i,
                "waypoints": [
                    {"longitude": lon, "latitude": lat, "height": alt}
                    for lon, lat, alt in route
                ],
                "num_waypoints": len(route)
            } for i, route in enumerate(geo_routes)]
            json.dump(geo_json, f, ensure_ascii=False, indent=4)

        save_routes_as_kml(geo_routes, output_kml)

        vertiport_name_map = generate_vertiports_from_routes(geo_routes, transformer, output_vertiport)
        generate_paths_from_routes(geo_routes, transformer, vertiport_name_map, output_path)

        print("✅ 全部文件生成完成！")

    except Exception as e:
        import traceback
        print("❌ 出错：", str(e))
        traceback.print_exc()

if __name__ == "__main__":
    main()
