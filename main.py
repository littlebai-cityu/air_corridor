from ObstacleGrid import ObstacleGridBuilder

builder = ObstacleGridBuilder(
    kml_path='data/Mk1.kml',  # Added the data/ prefix to match directory structure
    ref_lon=114.162559945072,
    ref_lat=22.3158202017388,
    ref_alt=0.0,
    resolution=5.0,
    buffer_cells=0
)

builder.build_grid()
builder.visualize_grid(limit=5000)
builder.visualize_topdown()
grid = builder.get_grid()
builder.save_to_npy('output_grid.npy')