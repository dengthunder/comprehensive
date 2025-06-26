import heapq
import numpy as np
import matplotlib.pyplot as plt
import matplotlib
from scipy.ndimage import binary_dilation

matplotlib.rcParams['font.sans-serif'] = ['SimHei']
matplotlib.rcParams['axes.unicode_minus'] = False

def load_grid_from_txt(filename):
    grid = []
    with open(filename, 'r') as f:
        for line in f:
            grid.append(list(map(int, line.strip().split())))
    return grid

def inflate_obstacles(grid, inflate_radius=7):
    grid_np = np.array(grid)
    struct = np.ones((2*inflate_radius + 1, 2*inflate_radius + 1))
    obstacle_bool = (grid_np == 1)
    inflated_obstacle = binary_dilation(obstacle_bool, structure=struct)
    inflated_grid = np.where(inflated_obstacle, 1, 0)
    return inflated_grid.tolist()

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

def upscale_grid(grid, sub_factor):
    h, w = len(grid), len(grid[0])
    fine_grid = np.zeros((h * sub_factor, w * sub_factor), dtype=int)
    for i in range(h):
        for j in range(w):
            if grid[i][j] == 1:
                fine_grid[i * sub_factor:(i + 1) * sub_factor,
                           j * sub_factor:(j + 1) * sub_factor] = 1
    return fine_grid

def get_coverage_mask(grid, path, car_length, car_width, sub_factor=3):
    h, w = len(grid), len(grid[0])
    fine_h, fine_w = h * sub_factor, w * sub_factor
    coverage_count = np.zeros((fine_h, fine_w), dtype=int)
    half_len = car_length // 2
    half_wid = car_width // 2
    for px, py in path:
        cx, cy = px * sub_factor + sub_factor // 2, py * sub_factor + sub_factor // 2
        for dx in range(-half_len, half_len + 1):
            for dy in range(-half_wid, half_wid + 1):
                nx, ny = cx + dx, cy + dy
                if 0 <= nx < fine_h and 0 <= ny < fine_w:
                    coverage_count[nx][ny] += 1
    return coverage_count

def adjust_turning_overlap(path, coverage_mask, fine_grid_mask, car_length, car_width, sub_factor):
    half_len = car_length // 2
    half_wid = car_width // 2
    for i in range(1, len(path) - 1):
        prev, curr, nex = path[i - 1], path[i], path[i + 1]
        dir1 = (curr[0] - prev[0], curr[1] - prev[1])
        dir2 = (nex[0] - curr[0], nex[1] - curr[1])
        if dir1 != dir2:
            cx, cy = curr[0] * sub_factor + sub_factor // 2, curr[1] * sub_factor + sub_factor // 2
            for dx in range(-half_len, half_len + 1):
                for dy in range(-half_wid, half_wid + 1):
                    nx, ny = cx + dx, cy + dy
                    if (0 <= nx < coverage_mask.shape[0] and 0 <= ny < coverage_mask.shape[1] and
                        fine_grid_mask[nx][ny] == 0 and coverage_mask[nx][ny] > 1):
                        coverage_mask[nx][ny] -= 1

def compute_coverage_stats(coverage_mask, fine_grid_mask):
    total_repeat_cells = np.sum((coverage_mask > 1) & (fine_grid_mask == 0))
    total_covered_cells = np.sum((coverage_mask >= 1) & (fine_grid_mask == 0))
    total_free_cells = np.sum(fine_grid_mask == 0)
    coverage_rate = total_covered_cells / total_free_cells if total_free_cells > 0 else 0
    repetition_rate = total_repeat_cells / total_free_cells if total_free_cells > 0 else 0
    print(f"细分后可通行细胞数: {total_free_cells}")
    print(f"被覆盖细胞数: {total_covered_cells}")
    print(f"重复覆盖细胞数: {total_repeat_cells}")
    print(f"✅ 覆盖率: {coverage_rate:.2%}")
    print(f"🔁 重复率: {repetition_rate:.2%}")

def visualize_coverage_map(fine_grid_mask, coverage_mask):
    fig, ax = plt.subplots(figsize=(10, 10))
    ax.imshow(np.array(fine_grid_mask), cmap='Greys', origin='upper')
    overlay = np.ma.masked_where(coverage_mask == 0, coverage_mask)
    heatmap = ax.imshow(overlay, cmap='Reds', origin='upper', alpha=0.7)
    plt.colorbar(heatmap, ax=ax, label="覆盖次数")
    ax.set_title("高精度覆盖热图")
    plt.show()

def save_path_to_txt(path, filename="path_points.txt"):
    with open(filename, 'w') as f:
        for p in path:
            f.write(f"({p[0]},{p[1]})\n")

if __name__ == "__main__":
    filename = "occupancy_grid.txt"
    grid = load_grid_from_txt(filename)

    print("🔄 膨胀障碍物（膨胀半径7）...")
    grid = inflate_obstacles(grid, inflate_radius=7)

    downsample_factor = 6
    grid = downsample_map(grid, factor=downsample_factor)

    start = (2, 2)
    if grid[start[0]][start[1]] == 1:
        for i in range(len(grid)):
            for j in range(len(grid[0])):
                if grid[i][j] == 0:
                    start = (i, j)
                    break

    path = coverage_path_full_astar(grid, start)
    car_length_cells = 2
    car_width_cells = 6
    sub_factor = 3
    fine_grid_mask = upscale_grid(grid, sub_factor)
    coverage_mask = get_coverage_mask(grid, path, car_length_cells, car_width_cells, sub_factor)
    adjust_turning_overlap(path, coverage_mask, fine_grid_mask, car_length_cells, car_width_cells, sub_factor)
    compute_coverage_stats(coverage_mask, fine_grid_mask)
    visualize_coverage_map(fine_grid_mask, coverage_mask)
    save_path_to_txt(path)
    print(f"路径点已保存到 path_points.txt，共 {len(path)} 点")
