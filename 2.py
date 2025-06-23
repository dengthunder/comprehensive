import heapq
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from collections import defaultdict


def load_grid_from_txt(filename):
    grid = []
    with open(filename, 'r') as f:
        for line in f:
            grid.append(list(map(int, line.strip().split())))
    return grid


def downsample_map(original_map, factor=2):
    h, w = len(original_map), len(original_map[0])
    original_map_np = np.array(original_map)
    new_h, new_w = h // factor, w // factor
    downsampled_map = np.zeros((new_h, new_w), dtype=int)
    for i in range(new_h):
        for j in range(new_w):
            block = original_map_np[i * factor:(i + 1) * factor, j * factor:(j + 1) * factor]
            downsampled_map[i, j] = 1 if np.any(block == 1) else 0
    return downsampled_map.tolist()


def heuristic(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1])


def astar(grid, start, goal):
    directions = [(0, 1), (1, 0), (0, -1), (-1, 0)]
    open_set = []
    heapq.heappush(open_set, (heuristic(start, goal), 0, start))
    came_from = {}
    gscore = {start: 0}
    visited = set()
    while open_set:
        _, cost, current = heapq.heappop(open_set)
        if current == goal:
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            path.append(start)
            return path[::-1]
        visited.add(current)
        for dx, dy in directions:
            nx, ny = current[0] + dx, current[1] + dy
            neighbor = (nx, ny)
            if 0 <= nx < len(grid) and 0 <= ny < len(grid[0]) and grid[nx][ny] == 0:
                if neighbor in visited:
                    continue
                tentative = cost + 1
                if tentative < gscore.get(neighbor, float('inf')):
                    came_from[neighbor] = current
                    gscore[neighbor] = tentative
                    fscore = tentative + heuristic(neighbor, goal)
                    heapq.heappush(open_set, (fscore, tentative, neighbor))
    return []


def coverage_path_full_astar(grid, start):
    h, w = len(grid), len(grid[0])
    visited = set()
    visited.add(start)
    path = [start]
    free_points = set()
    for i in range(0, h, 2):
        for j in range(0, w, 2):
            if grid[i][j] == 0:
                free_points.add((i, j))
    if start not in free_points and grid[start[0]][start[1]] == 0:
        free_points.add(start)
    free_points.discard(start)
    current = start
    while free_points:
        sorted_targets = sorted(free_points, key=lambda p: (heuristic(current, p), p[1]))
        found_path = None
        for target in sorted_targets:
            sub_path = astar(grid, current, target)
            if sub_path:
                found_path = sub_path
                break
        if not found_path:
            break
        for p in found_path[1:]:
            path.append(p)
            visited.add(p)
            if p in free_points:
                free_points.remove(p)
        current = path[-1]
    return path


def visualize_path_dynamic(grid, path, save_path=None, car_width_cells=10.630942091616):
    grid_show = [[1 if cell == 1 else 0 for cell in row] for row in grid]
    fig, ax = plt.subplots(figsize=(12, 12))

    # 预先分析整个路径，找出所有重复的线段
    point_visit_count = {}
    repeated_segments = set()

    # 统计每个点的访问次数和位置
    for idx, point in enumerate(path):
        if point not in point_visit_count:
            point_visit_count[point] = []
        point_visit_count[point].append(idx)

    # 标记所有重复访问的线段（从第二次访问开始的所有线段都是红色）
    for point, visit_indices in point_visit_count.items():
        if len(visit_indices) >= 2:  # 访问次数≥2次
            # 从第二次访问开始，所有到达该点的线段都标为红色
            for visit_idx in visit_indices[1:]:  # 跳过第一次访问
                if visit_idx > 0:  # 确保不是起点
                    repeated_segments.add(visit_idx)

    for i, current_point in enumerate(path):
        ax.cla()
        ax.imshow(grid_show, cmap='Greys', origin='upper')

        # 绘制从开始到当前点的路径
        for j in range(1, i + 1):  # 从第1个点开始（跳过起点）
            # 绘制线段 j-1 -> j
            x_vals = [path[j - 1][1], path[j][1]]
            y_vals = [path[j - 1][0], path[j][0]]

            # 判断该线段是否为重复访问
            if j in repeated_segments:
                ax.plot(x_vals, y_vals, color='red', linewidth=car_width_cells, alpha=0.8)
            else:
                ax.plot(x_vals, y_vals, color='blue', linewidth=car_width_cells, alpha=0.9)

        # 绘制起点正方形（绿色）- 调整大小
        start_square_size = 1.5  # 起点正方形边长，调整为更合适的大小
        start_square = patches.Rectangle(
            (path[0][1] - start_square_size / 2, path[0][0] - start_square_size / 2),
            start_square_size, start_square_size,
            linewidth=0.1, edgecolor='black', facecolor='green', zorder=5
        )
        ax.add_patch(start_square)

        # 绘制当前位置正方形（红色）- 调整大小
        current_square_size = 1.5  # 当前位置正方形边长，调整为更合适的大小
        current_square = patches.Rectangle(
            (current_point[1] - current_square_size / 2, current_point[0] - current_square_size / 2),
            current_square_size, current_square_size,
            linewidth=2, edgecolor='black', facecolor='red', zorder=5
        )
        ax.add_patch(current_square)

        ax.set_title(f"A* Path Progress: Step {i + 1}/{len(path)}")

        # 添加图例说明
        from matplotlib.lines import Line2D
        legend_elements = [
            Line2D([0], [0], marker='s', color='w', markerfacecolor='green', markersize=10.630942091616, label='start'),
            Line2D([0], [0], marker='s', color='w', markerfacecolor='red', markersize=10.630942091616, label='location'),
            Line2D([0], [0], color='blue', linewidth=3, label='first'),
            Line2D([0], [0], color='red', linewidth=3, label='repeat')
        ]
        ax.legend(handles=legend_elements, loc='upper right')

        plt.pause(0.005)

    if save_path:
        plt.savefig(save_path, dpi=300)
    plt.show()


def get_coverage_set(grid, path, car_width_cells):
    h, w = len(grid), len(grid[0])
    coverage_set = set()
    offset = int(car_width_cells // 2)
    for (x, y) in path:
        for i in range(x - offset, x + offset + 1):
            for j in range(y - offset, y + offset + 1):
                if 0 <= i < h and 0 <= j < w and grid[i][j] == 0:
                    coverage_set.add((i, j))
    return coverage_set


if __name__ == "__main__":
    filename = "occupancy_grid.txt"
    grid = load_grid_from_txt(filename)
    downsample_factor = 6
    grid = downsample_map(grid, factor=downsample_factor)
    start = (1, 1)
    if grid[start[0]][start[1]] == 1:
        found = False
        for i in range(len(grid)):
            for j in range(len(grid[0])):
                if grid[i][j] == 0:
                    start = (i, j)
                    found = True
                    break
            if found:
                break
    path = coverage_path_full_astar(grid, start)
    car_width_cells = 10.630942091616
    scale_factor = 1.0
    image_file = "path_visualization.png"
    # 按访问次数统计每个通行格子
    point_visit_count = defaultdict(int)
    for p in path:
        if grid[p[0]][p[1]] == 0:
            point_visit_count[p] += 1

    # 重复次数（每个点重复访问的次数）
    repeated = sum(v - 1 for v in point_visit_count.values() if v > 1)

    # 所有访问次数的总和
    total_path_visits = sum(v for v in point_visit_count.values())

    # 重复率：重复访问次数 ÷ 总访问次数
    repetition_rate = repeated / total_path_visits if total_path_visits > 0 else 0

    coverage_set = get_coverage_set(grid, path, car_width_cells)
    total_free_cells = sum(row.count(0) for row in grid)
    covered = len(coverage_set)
    coverage_rate = (covered - repeated) / total_free_cells if total_free_cells > 0 else 0

    print(f"地图中可通行区域总数: {total_free_cells}")
    print(f"有效覆盖面积（格子数）: {(covered - repeated)}")
    print(f"覆盖率: {coverage_rate:.2%}")
    print(f"重复率: {repetition_rate:.2%}")
    visualize_path_dynamic(grid, path, save_path=image_file,
                           car_width_cells=car_width_cells, )