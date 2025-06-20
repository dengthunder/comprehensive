import heapq
import numpy as np
import matplotlib.pyplot as plt


# 实际花费代价（g）：从起点到当前节点的真实路径长度；
# 预估代价（h）：从当前节点到终点的预估距离（用启发式函数估计）；
# ---------- 地图加载 ----------
def load_grid_from_txt(filename):
    grid = []
    with open(filename, 'r') as f:
        for line in f:
            grid.append(list(map(int, line.strip().split())))
    return grid


# ---------- 地图降采样 ----------
def downsample_map(original_map, factor=1):
    h, w = len(original_map), len(original_map[0])  # 原始地图高度、宽度
    original_map_np = np.array(original_map)
    new_h, new_w = h // factor, w // factor  # 降采样后地图尺寸
    downsampled_map = np.zeros((new_h, new_w), dtype=int)
    for i in range(new_h):
        for j in range(new_w):
            block = original_map_np[i * factor:(i + 1) * factor,
                    j * factor:(j + 1) * factor]  # 对应原始map的block区域，从地图中提取一个 factor x factor 的小区域
            downsampled_map[i, j] = 1 if np.any(block == 1) else 0  # 将数组转换成嵌套的 Python 原生列表形式
    return downsampled_map.tolist()


# ---------- 启发式函数（曼哈顿距离） ----------
def heuristic(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1])  # 横纵坐标距离之和


# ---------- A* 算法，方向优先右、下、左、上，步长为1 ----------
def astar(grid, start, goal):
    directions = [(0, 1), (1, 0), (0, -1), (-1, 0)]  # 右 > 下 > 左 > 上
    open_set = []
    heapq.heappush(open_set, (heuristic(start, goal), 0, start))  # 初始节点入队：f值、g值、位置
    came_from = {}
    gscore = {start: 0}
    visited = set()
    while open_set:  # 只要 open_set 还有待处理的节点，就继续搜索
        _, cost, current = heapq.heappop(open_set)  # 弹出f值最小的点，解包出g值和当前节点的坐标
        if current == goal:  # 找到目标节点，开始回溯路径
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            path.append(start)
            return path[::-1]  # 路径反转为从起点到终点
        visited.add(current)  # 当前点标记为访问过
        for dx, dy in directions:
            nx, ny = current[0] + dx, current[1] + dy
            neighbor = (nx, ny)  # 相邻节点的位置
            if 0 <= nx < len(grid) and 0 <= ny < len(grid[0]) and grid[nx][ny] == 0:  # 判断是否在地图范围内并且是否可通行
                if neighbor in visited:
                    continue
                tentative = cost + 1  # 从起点走到邻居节点的"尝试路径代价g"，即当前路径代价 cost 加上一步的代价
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
    visited.add(start)  # 创建访问集合 visited，并将起点加入，避免重复访问。
    path = [start]
    # 采样点间隔为2，构造覆盖点集合
    free_points = set()
    for i in range(0, h, 1):  # 以步长为2遍历地图，找到所有可通行的点 (i, j) 并加入 free_points，这些是希望覆盖的目标点。
        for j in range(0, w, 1):
            if grid[i][j] == 0:
                free_points.add((i, j))
    # 确保起点包含在free_points中
    if start not in free_points and grid[start[0]][start[1]] == 0:
        free_points.add(start)
    free_points.discard(start)  # 从目标集合中去除起点，避免重复走。
    current = start
    while free_points:  # 只要还有未覆盖的点，就继续规划路径。
        # 按距离和靠右优先排序
        sorted_targets = sorted(free_points,
                                key=lambda p: (heuristic(current, p), p[1]))  # 将所有目标点按启发式函数（距离）排序，距离相同则靠右（x坐标大）优先。
        found_path = None  # 用于存放从 current 到目标的可行路径
        for target in sorted_targets:  # 遍历排序后的目标点，使用 A* 算法规划路径。找到第一个成功的路径就退出。
            sub_path = astar(grid, current, target)
            if sub_path:
                found_path = sub_path
                break
        if not found_path:
            # 找不到路径就跳出
            break
        for p in found_path[1:]:  # 将 found_path 中除第一个点（当前点）外的点依次加入主路径，并标记访问。如果这些点是目标点就从 free_points 中删除。
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
        plt.cla()  # 清除当前画布
        plt.imshow(grid_show, cmap='Greys', origin='upper')
        plt.plot(path_x, path_y, color='blue', linewidth=(0.35 / 16.198117188) * 492)
        plt.scatter(path_x[0], path_y[0], color='green', label='Start')
        plt.scatter(path_x[-1], path_y[-1], color='red', label='Current' if i != len(path) - 1 else 'End')
        plt.title(f"A* Path Progress: Step {i + 1}/{len(path)}")
        plt.legend()
        plt.pause(0.005)
    if save_path:
        plt.savefig(save_path, dpi=300)
    plt.show()


# ---------- 计算重复率和覆盖率 ----------
def calculate_repetition_rate(path):
    """
    计算路径的重复率
    重复率 = (总访问次数 - 不同网格数) / 不同网格数
    """
    total_points = len(path)  # 总访问次数
    unique_points = len(set(path))  # 访问的不同网格数量
    repeated_times = total_points - unique_points  # 重复访问次数

    if unique_points == 0:
        return 0.0, total_points, unique_points, repeated_times

    repetition_ratio = repeated_times / unique_points  # 重复率
    return repetition_ratio, total_points, unique_points, repeated_times


def calculate_coverage_rate(grid, path):
    """
    计算覆盖率
    覆盖率 = 实际清扫面积 / 总可通行面积
    """
    # 计算地图中总的可通行区域面积
    total_passable_cells = 0
    for i in range(len(grid)):
        for j in range(len(grid[0])):
            if grid[i][j] == 0:
                total_passable_cells += 1

    # 计算实际清扫面积（路径覆盖的不同网格数）
    actual_cleaned_area = len(set(path))

    # 计算覆盖率
    if total_passable_cells == 0:
        return 0.0, actual_cleaned_area, total_passable_cells

    coverage_ratio = actual_cleaned_area / total_passable_cells
    return coverage_ratio, actual_cleaned_area, total_passable_cells


# ---------- 主程序 ----------
if __name__ == "__main__":
    filename = r"C:\Users\24816\Desktop\comprehensive project\occupancy_grid.txt"
    grid = load_grid_from_txt(filename)
    downsample_factor = 6
    # 地图降采样
    grid = downsample_map(grid, factor=downsample_factor)
    # 起点选择
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

    # 执行全覆盖路径规划
    path = coverage_path_full_astar(grid, start)

    # 计算重复率
    repetition_ratio, total_points, unique_points, repeated_times = calculate_repetition_rate(path)

    # 计算覆盖率
    coverage_ratio, actual_cleaned_area, total_passable_cells = calculate_coverage_rate(grid, path)

    # 保存路径到txt文件
    output_file = r"C:\Users\24816\Desktop\comprehensive project\path_output.txt"
    with open(output_file, 'w') as f:
        f.write(f"路径长度: {len(path)}\n")
        f.write("=" * 60 + "\n")
        f.write("重复率分析:\n")
        f.write(f"总访问次数 (路径长度): {total_points}\n")
        f.write(f"覆盖的不同网格数量: {unique_points}\n")
        f.write(f"重复访问次数: {repeated_times}\n")
        f.write(f"重复率 p (重复清扫面积占总面积比): {repetition_ratio:.4f}\n")
        f.write(f"重复率百分比: {repetition_ratio * 100:.2f}%\n")
        f.write("=" * 60 + "\n")
        f.write("覆盖率分析:\n")
        f.write(f"实际清扫面积 (不同网格数): {actual_cleaned_area}\n")
        f.write(f"总可通行面积 (可通行网格数): {total_passable_cells}\n")
        f.write(f"覆盖率 c (实际清扫面积占总可通行面积比): {coverage_ratio:.4f}\n")
        f.write(f"覆盖率百分比: {coverage_ratio * 100:.2f}%\n")
        f.write("=" * 60 + "\n")
        f.write("综合指标:\n")
        f.write(f"路径效率 (1-重复率): {(1 - repetition_ratio):.4f}\n")
        f.write(f"清扫完整性 (覆盖率): {coverage_ratio:.4f}\n")
        f.write("=" * 60 + "\n")
        f.write("路径坐标:\n")
        for p in path:
            f.write(f"{p}\n")

    # 控制台输出
    print(f"路径长度: {len(path)}")
    print("=" * 60)
    print("重复率分析:")
    print(f"总访问次数 (路径长度): {total_points}")
    print(f"覆盖的不同网格数量: {unique_points}")
    print(f"重复访问次数: {repeated_times}")
    print(f"重复率 p (重复清扫面积占总面积比): {repetition_ratio:.4f}")
    print(f"重复率百分比: {repetition_ratio * 100:.2f}%")
    print("=" * 60)
    print("覆盖率分析:")
    print(f"实际清扫面积 (不同网格数): {actual_cleaned_area}")
    print(f"总可通行面积 (可通行网格数): {total_passable_cells}")
    print(f"覆盖率 c (实际清扫面积占总可通行面积比): {coverage_ratio:.4f}")
    print(f"覆盖率百分比: {coverage_ratio * 100:.2f}%")
    print("=" * 60)
    print("综合指标:")
    print(f"路径效率 (1-重复率): {(1 - repetition_ratio):.4f}")
    print(f"清扫完整性 (覆盖率): {coverage_ratio:.4f}")
    print("=" * 60)

    # 可视化路径
    image_file = r"C:\Users\24816\Desktop\comprehensive project\path_visualization.png"
    visualize_path_dynamic(grid, path, save_path=image_file)