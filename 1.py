import numpy as np
from shapely.geometry import Polygon, Point, MultiPolygon
from scipy.spatial import ConvexHull

def parse_obj(file_path):
    vertices = []
    faces = []
    mesh_indices = []
    current_mesh_indices = []
    with open(file_path, 'r') as f:
        for line in f:
            if line.startswith('v '):
                parts = line.strip().split()
                x, y, z = map(float, parts[1:4])
                vertices.append((x, y, z))
            elif line.startswith('f '):
                parts = line.strip().split()
                # 顶点索引，从1开始，去除法线索引
                idx = [int(p.split('//')[0]) - 1 for p in parts[1:]]
                faces.append(idx)
                current_mesh_indices.extend(idx)
            elif line.startswith('g '):
                # 新mesh开始，存储上一个mesh顶点索引
                if current_mesh_indices:
                    mesh_indices.append(list(set(current_mesh_indices)))
                    current_mesh_indices = []
        # 最后一组mesh
        if current_mesh_indices:
            mesh_indices.append(list(set(current_mesh_indices)))

    vertices = np.array(vertices)
    return vertices, faces, mesh_indices

def build_obstacle_polygons(vertices, mesh_indices):
    polygons = []
    for indices in mesh_indices:
        points_2d = vertices[indices][:, [1, 2]]  # 取x,z平面
        if len(points_2d) < 3:
            continue
        hull = ConvexHull(points_2d)
        hull_points = points_2d[hull.vertices]
        poly = Polygon(hull_points)
        polygons.append(poly)
    return polygons

def generate_grid_map(polygons, grid_size=50):
    # 获取地图范围
    all_points = np.vstack([np.array(poly.exterior.coords) for poly in polygons])
    x_min, z_min = all_points.min(axis=0)
    x_max, z_max = all_points.max(axis=0)

    width = int(np.ceil((x_max - x_min) / grid_size))
    height = int(np.ceil((z_max - z_min) / grid_size))

    grid_map = np.zeros((height, width), dtype=np.uint8)
    multi_poly = MultiPolygon(polygons)

    for i in range(height):
        for j in range(width):
            x = x_min + j * grid_size + grid_size / 2
            z = z_min + i * grid_size + grid_size / 2
            point = Point(x, z)
            if multi_poly.contains(point):
                grid_map[i, j] = 1

    return grid_map, (x_min, z_min), grid_size

def print_grid_map(grid_map):
    for row in grid_map[::-1]:  # 翻转一下Y轴显示更直观
        print(''.join('#' if cell else '.' for cell in row))

if __name__ == '__main__':
    obj_file = "C:\\Users\\24816\\Desktop\\comprehensive project\\Environment map.obj"# 你的obj文件路径
    vertices, faces, mesh_indices = parse_obj(obj_file)
    polygons = build_obstacle_polygons(vertices, mesh_indices)
    grid_map, origin, grid_size = generate_grid_map(polygons, grid_size=50)
    print_grid_map(grid_map)
