import heapq
import numpy as np
import matplotlib.pyplot as plt

#实际花费代价（g）：从起点到当前节点的真实路径长度；
#预估代价（h）：从当前节点到终点的预估距离（用启发式函数估计）；

# ---------- 地图加载 ----------
def load_grid_from_txt(filename):
    grid = []
    with open(filename, 'r') as f:
        for line in f:
            grid.append(list(map(int, line.strip().split())))
    return grid

# ---------- 地图降采样 ----------
def downsample_map(original_map, factor=2):
    h, w = len(original_map), len(original_map[0])  # 原始地图高度、宽度
    original_map_np = np.array(original_map)
    new_h, new_w = h // factor, w // factor # 降采样后地图尺寸
    downsampled_map = np.zeros((new_h, new_w), dtype=int)

    for i in range(new_h):
        for j in range(new_w):
            block = original_map_np[i*factor:(i+1)*factor, j*factor:(j+1)*factor]
            downsampled_map[i, j] = 1 if np.any(block == 1) else 0

    return downsampled_map.tolist()

# ---------- 启发式函数（曼哈顿距离） ----------
def heuristic(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1])

# ---------- A* 算法，方向优先右、下、左、上，步长为1 ----------
def astar(grid, start, goal):
    directions = [(0,1),(1,0),(0,-1),(-1,0)]  # 右 > 下 > 左 > 上
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

# ---------- 全覆盖路径规划，覆盖点间隔为2 ----------
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
        plt.scatter(path_x[0], path_y[0], color='green', label='Start')
        plt.scatter(path_x[-1], path_y[-1], color='red', label='Current' if i != len(path) - 1 else 'End')
        plt.title(f"A* Path Progress: Step {i + 1}/{len(path)}")
        plt.legend()
        plt.pause(0.005)

    if save_path:
        plt.savefig(save_path, dpi=300)
    plt.show()


# ---------- 主程序 ----------
if __name__ == "__main__":
    filename = r"C:\Users\24816\Desktop\comprehensive project\occupancy_grid.txt"
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

    output_file = r"C:\Users\24816\Desktop\comprehensive project\path_output.txt"
    with open(output_file, 'w') as f:
        f.write(f"路径长度: {len(path)}\n")
        for p in path:
            f.write(f"{p}\n")

    print(f"路径长度: {len(path)}")
    for p in path:
        print(p)

    image_file = r"C:\Users\24816\Desktop\comprehensive project\path_visualization.png"
    visualize_path_dynamic(grid, path, save_path=image_file)

    # 计算覆盖率和重复率
    total_free_cells = sum(row.count(0) for row in grid)
    visited_once = set()
    repeated = 0

    for p in path:
        if grid[p[0]][p[1]] == 0:
            if p in visited_once:
                repeated += 1
            else:
                visited_once.add(p)

    covered = len(visited_once)
    total_path = len(path)

    coverage_rate = covered / total_free_cells if total_free_cells > 0 else 0
    repetition_rate = repeated / total_path if total_path > 0 else 0
import heapq
import numpy as np
import matplotlib.pyplot as plt

#实际花费代价（g）：从起点到当前节点的真实路径长度；
#预估代价（h）：从当前节点到终点的预估距离（用启发式函数估计）；

# ---------- 地图加载 ----------
def load_grid_from_txt(filename):
    grid = []
    with open(filename, 'r') as f:
        for line in f:
            grid.append(list(map(int, line.strip().split())))
    return grid

# ---------- 地图降采样 ----------
def downsample_map(original_map, factor=2):
    h, w = len(original_map), len(original_map[0])  # 原始地图高度、宽度
    original_map_np = np.array(original_map)
    new_h, new_w = h // factor, w // factor # 降采样后地图尺寸
    downsampled_map = np.zeros((new_h, new_w), dtype=int)

    for i in range(new_h):
        for j in range(new_w):
            block = original_map_np[i*factor:(i+1)*factor, j*factor:(j+1)*factor]
            downsampled_map[i, j] = 1 if np.any(block == 1) else 0

    return downsampled_map.tolist()

# ---------- 启发式函数（曼哈顿距离） ----------
def heuristic(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1])

# ---------- A* 算法，方向优先右、下、左、上，步长为1 ----------
def astar(grid, start, goal):
    directions = [(0,1),(1,0),(0,-1),(-1,0)]  # 右 > 下 > 左 > 上
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

# ---------- 全覆盖路径规划，覆盖点间隔为2 ----------
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
        plt.scatter(path_x[0], path_y[0], color='green', label='Start')
        plt.scatter(path_x[-1], path_y[-1], color='red', label='Current' if i != len(path) - 1 else 'End')
        plt.title(f"A* Path Progress: Step {i + 1}/{len(path)}")
        plt.legend()
        plt.pause(0.005)

    if save_path:
        plt.savefig(save_path, dpi=300)
    plt.show()


# ---------- 主程序 ----------
if __name__ == "__main__":
    filename = r"C:\Users\24816\Desktop\comprehensive project\occupancy_grid.txt"
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

    output_file = r"C:\Users\24816\Desktop\comprehensive project\path_output.txt"
    with open(output_file, 'w') as f:
        f.write(f"路径长度: {len(path)}\n")
        for p in path:
            f.write(f"{p}\n")

    print(f"路径长度: {len(path)}")
    for p in path:
        print(p)

    image_file = r"C:\Users\24816\Desktop\comprehensive project\path_visualization.png"
    visualize_path_dynamic(grid, path, save_path=image_file)

    # 计算覆盖率和重复率
    total_free_cells = sum(row.count(0) for row in grid)
    visited_once = set()
    repeated = 0

    for p in path:
        if grid[p[0]][p[1]] == 0:
            if p in visited_once:
                repeated += 1
            else:
                visited_once.add(p)

    covered = len(visited_once)
    total_path = len(path)

    coverage_rate = covered / total_free_cells if total_free_cells > 0 else 0
    repetition_rate = repeated / total_path if total_path > 0 else 0
