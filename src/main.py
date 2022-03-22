from geometry_utils import *

if __name__ == '__main__':
    #points = np.random.rand(10,3)
    #colors = np.zeros_like(points)
    #randompcd = init_pointcloud(points = points, colors = colors, view = False)

    path = './test_data/1647624852.848528616.pcd'
    a = read_pointcloud(path=path, view=False)
    geometry, center = get_width_length_height(a, max_plane_idx=2, view=False)
    print(geometry)
    print(center)