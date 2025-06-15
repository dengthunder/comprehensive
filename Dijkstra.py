import heapq

def load_grid_from_txt(filename):
    grid = []
    with open(filename, 'r') as f:
        for line in f:
            row = list(map(int, line.strip().split()))
            grid.append(row)
    return grid

def dijkstra(grid, start, goal):
    rows, cols = len(grid), len(grid[0])
    distances = {start: 0}
    came_from = {}
    queue = [(0, start)]
    directions = [(0,1),(1,0),(0,-1),(-1,0)]  # 四方向

    while queue:
        dist, current = heapq.heappop(queue)
        if current == goal:
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            path.append(start)
            return path[::-1]

        for dx, dy in directions:
            nx, ny = current[0] + dx, current[1] + dy
            if 0 <= nx < rows and 0 <= ny < cols and grid[nx][ny] == 0:
                new_dist = dist + 1
                if new_dist < distances.get((nx, ny), float('inf')):
                    distances[(nx, ny)] = new_dist
                    came_from[(nx, ny)] = current
                    heapq.heappush(queue, (new_dist, (nx, ny)))

    return []  # 无路径时返回空列表

if __name__ == "__main__":
    filename = r"C:/Users/24816/Desktop/comprehensive project/occupancy_grid.txt"
    grid_map = load_grid_from_txt(filename)

    start = (30, 30)  # 起点，左上角
    goal = (len(grid_map) - 1, len(grid_map[0]) - 1)  # 终点，右下角

    path = dijkstra(grid_map, start, goal)

    if path:
        print("找到路径：")
        for p in path:
            print(p)
    else:
        print("路径不存在")
