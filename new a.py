import heapq
import numpy as np
import matplotlib.pyplot as plt
from scipy.ndimage import binary_dilation

# 配置matplotlib，支持中文显示，解决负号显示问题
plt.rcParams['font.sans-serif'] = ['SimHei']  # 黑体
plt.rcParams['axes.unicode_minus'] = False

# ---------- 地图加载 ----------
def load_grid_from_txt(filename):
    grid = []
    with open(filename, 'r') as f:
        for line in f:
            grid.append(list(map(int, line.strip().split())))
    return grid

# ---------- 地图膨胀 ----------
def inflate_obstacles(grid, inflate_radius=2):
    grid_np = np.array(grid)
    struct = np.ones((2*inflate_radius + 1, 2*inflate_radius + 1))
    obstacle_bool = (grid_np == 1)
    inflated_obstacle = binary_dilation(obstacle_bool, structure=struct)
    inflated_grid = np.where(inflated_obstacle, 1, 0)
    return inflated_grid.tolist()

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

# ---------- 启发式函数（曼哈顿距离） ----------
def heuristic(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1])

# ---------- A* 算法 ----------
def astar(grid, start, goal):
    directions = [(0,1),(1,0),(0,-1),(-1,0)]  # 右，下，左，上
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

# ---------- 全覆盖路径规划，间隔为2 ----------
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

# ---------- 动态路径可视化 ----------
def visualize_path_dynamic(grid, path, save_path=None):
    grid_show = [[1 if cell == 1 else 0 for cell in row] for row in grid]
    plt.figure(figsize=(10, 10))

    path_x = []
    path_y = []

    for i, p in enumerate(path):
        path_x.append(p[1])
        path_y.append(p[0])

        plt.cla()
        plt.imshow(grid_show, cmap='Greys', origin='upper')
        plt.plot(path_x, path_y, color='blue', linewidth=2)
        plt.scatter(path_x[0], path_y[0], color='green', label='起点')
        plt.scatter(path_x[-1], path_y[-1], color='red', label='当前点' if i != len(path) - 1 else '终点')
        plt.title(f"A* 路径进度: 步骤 {i + 1}/{len(path)}")
        plt.legend()
        plt.pause(0.005)

    if save_path:
        plt.savefig(save_path, dpi=300)
    plt.show()

# ---------- 主程序 ----------
if __name__ == "__main__":
    filename = r"C:\Users\24816\Desktop\comprehensive project\occupancy_grid.txt"
    downsample_factor = 6

    print("🔄 读取地图...")
    grid = load_grid_from_txt(filename)

    print("🔄 膨胀障碍物（安全距离2像素）...")
    grid = inflate_obstacles(grid, inflate_radius=7)

    print(f"🔄 降采样，因子={downsample_factor}...")
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
    print(f"起点: {start}")

    print("🛣️ 规划全覆盖路径...")
    path = coverage_path_full_astar(grid, start)

    output_file = r"C:\Users\24816\Desktop\comprehensive project\path_output.txt"
    with open(output_file, 'w') as f:
        f.write(f"路径长度: {len(path)}\n")
        for p in path:
            f.write(f"{p}\n")

    print(f"路径长度: {len(path)}")
    for p in path:
        print(p)

    print("📊 动态绘制路径...")
    image_file = r"C:\Users\24816\Desktop\comprehensive project\path_visualization.png"
    visualize_path_dynamic(grid, path, save_path=image_file)

    print("✅ 完成")
