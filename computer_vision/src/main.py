from geometry_utils import *

if __name__ == '__main__':

    path = './test_data/1647624852.848528616.pcd'
    a = read_pointcloud(path=path, view=False)
    geometry, center, top_surface_corners = get_width_length_height(
        a, max_plane_idx=2, view=False)
    print(geometry)
    print(center)
    print(top_surface_corners)