import heapq
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D

# ---------- 地图加载 ----------
def load_grid_from_txt(filename):
    grid = []
    with open(filename, 'r') as f:
        for line in f:
            grid.append(list(map(int, line.strip().split())))
    return grid

# ---------- 地图降采样 ----------
def downsample_map(original_map, factor=2):
    h, w = len(original_map), len(original_map[0])
    original_map_np = np.array(original_map)
    new_h, new_w = h // factor, w // factor
    downsampled_map = np.zeros((new_h, new_w), dtype=int)
    for i in range(new_h):
        for j in range(new_w):
            block = original_map_np[i*factor:(i+1)*factor, j*factor:(j+1)*factor]
            downsampled_map[i, j] = 1 if np.any(block == 1) else 0
    return downsampled_map.tolist()

# ---------- 启发式函数 ----------
def heuristic(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1])

# ---------- A* ----------
def astar(grid, start, goal):
    directions = [(0,1),(1,0),(0,-1),(-1,0)]
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

# ---------- 全覆盖路径 ----------
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

# ---------- 连续路径可视化 ----------
def visualize_path_line_segments(grid, path, coverage_mask, robot_width_pixel, save_path=None):
    plt.figure(figsize=(10, 10))
    grid_show = [[1 if cell == 1 else 0 for cell in row] for row in grid]
    plt.imshow(grid_show, cmap='Greys', origin='upper')

    for i in range(len(path) - 1):
        (x1, y1), (x2, y2) = path[i], path[i+1]
        count = coverage_mask[x2][y2]
        color = 'red' if count > 1 else 'blue'
        line = Line2D([y1, y2], [x1, x2], linewidth=robot_width_pixel, color=color, alpha=0.8)
        plt.gca().add_line(line)

    plt.scatter(path[0][1], path[0][0], color='green', s=80, label='Start')
    plt.scatter(path[-1][1], path[-1][0], color='orange', s=80, label='End')

    plt.title("Path Visualization with Red Repeats (Line-Based)")
    plt.legend()
    if save_path:
        plt.savefig(save_path, dpi=300)
    plt.show()

# ---------- 主程序 ----------
if __name__ == "__main__":
    filename = r"C:\Users\24816\Desktop\comprehensive project\occupancy_grid.txt"
    downsample_factor = 6
    grid = load_grid_from_txt(filename)
    grid = downsample_map(grid, factor=downsample_factor)

    total_free = sum(cell == 0 for row in grid for cell in row)

    start = (1, 1)
    if grid[start[0]][start[1]] == 1:
        for i in range(len(grid)):
            for j in range(len(grid[0])):
                if grid[i][j] == 0:
                    start = (i, j)
                    break

    path = coverage_path_full_astar(grid, start)

    robot_width_pixel = int(round(492 * 0.35 / 16.198117188))
    robot_width_in_cells = robot_width_pixel / downsample_factor
    total_path_length = len(path)
    total_path_area = total_path_length * robot_width_in_cells

    coverage_mask = np.zeros((len(grid), len(grid[0])), dtype=int)
    for p in path:
        coverage_mask[p[0], p[1]] += 1

    repeated_points = [p for p in path if coverage_mask[p[0], p[1]] > 1]
    repeated_count = len(set(repeated_points))
    repeated_area = repeated_count * robot_width_in_cells

    coverage_rate = (total_path_area - repeated_area) / total_free if total_free > 0 else 0
    repeat_rate = repeated_area / total_path_area if total_path_area > 0 else 0

    print(f"总空白区域点数: {total_free}")
    print(f"路径长度: {total_path_length}")
    print(f"车宽（格子单位）: {robot_width_in_cells:.3f}")
    print(f"路径覆盖面积: {total_path_area:.2f}")
    print(f"重复覆盖面积: {repeated_area:.2f}")
    print(f"覆盖率: {coverage_rate * 100:.2f}%")
    print(f"重复率: {repeat_rate * 100:.2f}%")

    output_file = r"C:\Users\24816\Desktop\comprehensive project\path_output.txt"
    with open(output_file, 'w') as f:
        f.write(f"路径长度: {total_path_length}\n")
        f.write(f"覆盖率: {coverage_rate * 100:.2f}%\n")
        f.write(f"重复率: {repeat_rate * 100:.2f}%\n")
        for p in path:
            f.write(f"{p}\n")

    image_file = r"C:\Users\24816\Desktop\comprehensive project\path_visualization.png"
    visualize_path_line_segments(grid, path, coverage_mask, robot_width_pixel, save_path=image_file)
