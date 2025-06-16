import heapq
import matplotlib.pyplot as plt
import numpy as np

def load_grid_from_txt(filename):
    grid = []
    with open(filename, 'r') as f:
        for line in f:
            row = list(map(int, line.strip().split()))
            grid.append(row)
    return grid

def heuristic(a, b):
    return abs(a[0]-b[0]) + abs(a[1]-b[1])

def astar(grid, start, goal):
    neighbors = [(0,1),(1,0),(-1,0),(0,-1)]
    close_set = set()
    came_from = {}
    gscore = {start:0}
    fscore = {start:heuristic(start, goal)}
    oheap = []
    heapq.heappush(oheap, (fscore[start], start))

    while oheap:
        _, current = heapq.heappop(oheap)

        if current == goal:
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            path.append(start)
            return path[::-1]

        close_set.add(current)
        for i, j in neighbors:
            neighbor = current[0] + i, current[1] + j
            if 0 <= neighbor[0] < len(grid) and 0 <= neighbor[1] < len(grid[0]):
                if grid[neighbor[0]][neighbor[1]] == 1:
                    continue
            else:
                continue
            tentative_g_score = gscore[current] + 1
            if neighbor in close_set and tentative_g_score >= gscore.get(neighbor, 0):
                continue
            if tentative_g_score < gscore.get(neighbor, float('inf')):
                came_from[neighbor] = current
                gscore[neighbor] = tentative_g_score
                fscore[neighbor] = tentative_g_score + heuristic(neighbor, goal)
                heapq.heappush(oheap, (fscore[neighbor], neighbor))
    return []

def get_all_cleanable_cells(grid):
    cells = []
    for y in range(len(grid)):
        for x in range(len(grid[0])):
            if grid[y][x] == 0:
                cells.append((y, x))
    return cells

def plan_coverage_path(grid, start):
    cleanable = set(get_all_cleanable_cells(grid))
    path = []
    current = start
    while cleanable:
        nearest = min(cleanable, key=lambda cell: heuristic(current, cell))
        sub_path = astar(grid, current, nearest)
        if not sub_path:
            cleanable.discard(nearest)
            continue
        path.extend(sub_path[1:])
        current = nearest
        cleanable.discard(current)
    return path

# 读取地图
filename = "C:\\Users\\24816\\Desktop\\comprehensive project\\occupancy_grid.txt"
grid_map = load_grid_from_txt(filename)

# 起点，左上角
start_point = (0, 0)

# 规划全覆盖路径
coverage_path = plan_coverage_path(grid_map, start_point)

# 输出路径
print("规划路径点数:", len(coverage_path))
print(coverage_path)

# 可视化地图和路径
grid_array = np.array(grid_map)
plt.figure(figsize=(8,8))
plt.imshow(grid_array, cmap='Greys_r')
if coverage_path:
    ys, xs = zip(*coverage_path)
    plt.plot(xs, ys, color='red', linewidth=1)
plt.title("Coverage Path on Grid Map")
plt.xlabel("X")
plt.ylabel("Y")
plt.show()
