import heapq

def load_grid_from_txt(filename):
    grid = []
    with open(filename, 'r') as f:
        for line in f:
            row = list(map(int, line.strip().split()))
            grid.append(row)
    return grid

def heuristic(a, b):
    # 曼哈顿距离作为启发函数
    return abs(a[0] - b[0]) + abs(a[1] - b[1])

def astar(grid, start, goal):
    rows, cols = len(grid), len(grid[0])
    open_set = []
    heapq.heappush(open_set, (0 + heuristic(start, goal), 0, start))  # (f_score, g_score, position)
    came_from = {}
    g_score = {start: 0}

    while open_set:
        _, current_g, current = heapq.heappop(open_set)

        if current == goal:
            # 回溯路径
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            path.append(start)
            path.reverse()
            return path

        for dx, dy in [(-1,0),(1,0),(0,-1),(0,1)]:  # 上下左右四邻域
            neighbor = (current[0] + dx, current[1] + dy)
            x, y = neighbor
            if 0 <= x < rows and 0 <= y < cols:
                if grid[x][y] == 1:
                    # 障碍，不可通行
                    continue
                tentative_g_score = current_g + 1
                if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score = tentative_g_score + heuristic(neighbor, goal)
                    heapq.heappush(open_set, (f_score, tentative_g_score, neighbor))

    return None  # 无路径

if __name__ == "__main__":
    filename = r"C:\\Users\\24816\\Desktop\\comprehensive project\\occupancy_grid.txt"
    grid_map = load_grid_from_txt(filename)

    start = (3, 3)
    goal = (len(grid_map) - 5, len(grid_map[0]) - 5)  # 默认终点是右下角
    path = astar(grid_map, start, goal)

    if path is None:
        print("没有找到路径")
    else:
        print("找到路径，路径点坐标为：")
        for p in path:
            print(p)
