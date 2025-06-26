import heapq
import numpy as np
import matplotlib.pyplot as plt
import matplotlib
from scipy.ndimage import binary_dilation

matplotlib.rcParams['font.sans-serif'] = ['SimHei'] # 支持中文显示
matplotlib.rcParams['axes.unicode_minus'] = False   # 正常显示负号

def load_grid_from_txt(filename): #加载栅格地图，去掉括号以读取坐标
    grid = []
    with open(filename, 'r') as f:
        for line in f:
            grid.append(list(map(int, line.strip().split())))
    return grid

def inflate_obstacles(grid, inflate_radius=7): #障碍物膨胀
    grid_np = np.array(grid)
    struct = np.ones((2*inflate_radius + 1, 2*inflate_radius + 1))
    obstacle_bool = (grid_np == 1)
    inflated_obstacle = binary_dilation(obstacle_bool, structure=struct)  #进行膨胀
    inflated_grid = np.where(inflated_obstacle, 1, 0)
    return inflated_grid.tolist()

def downsample_map(original_map, factor=2): #地图降采样，以加快规划速度，默认每 factor×factor 个格合并为一个格
    h, w = len(original_map), len(original_map[0]) # 获取原始地图的高度和宽度，h为行，w为列
    original_map_np = np.array(original_map)   #将列表转换为NumPy数组
    new_h, new_w = h // factor, w // factor    # 计算降采样后的新高度和宽度
    downsampled_map = np.zeros((new_h, new_w), dtype=int)  #初始化新的降采样地图，全为0
    for i in range(new_h): #双重循环，访问降采样后每个新格子，对应原图中的一个 factor × factor 小块区域
        for j in range(new_w):
            block = original_map_np[i * factor:(i + 1) * factor, j * factor:(j + 1) * factor]
            downsampled_map[i, j] = 1 if np.any(block == 1) else 0 #判断这块区域中是否有障碍，如果区域中有障碍，就把新格设为 1
    return downsampled_map.tolist()

def heuristic(a, b):#曼哈顿距离启发函数
    return abs(a[0] - b[0]) + abs(a[1] - b[1])

def astar(grid, start, goal):#A规划函数
    directions = [(0, 1), (1, 0), (0, -1), (-1, 0)] #定义机器人能移动的方向，四连通方式
    open_set = [] #优先当前点
    heapq.heappush(open_set, (heuristic(start, goal), 0, start)) #g = 0：起点到当前点的实际代价，h = heuristic(start, goal)：起点到目标的估计代价
    came_from = {}       # 记录每个点是从哪个点走来的，用于回溯路径
    gscore = {start: 0}  # g值字典：起点到某个点的实际总代价
    visited = set()      # 记录已经访问过的点，避免重复扩展
    while open_set:      # 主循环：只要还有点可扩展，就继续搜索
        _, cost, current = heapq.heappop(open_set)  # 取出 f 最小的节点
        if current == goal:
            path = []     # 回溯路径
            while current in came_from:
                path.append(current)
                current = came_from[current]
            path.append(start)
            return path[::-1]  # 反转返回路径
        visited.add(current)   # 标记当前点为已访问
        for dx, dy in directions:  # 遍历四个方向
            nx, ny = current[0] + dx, current[1] + dy
            neighbor = (nx, ny)
            if 0 <= nx < len(grid) and 0 <= ny < len(grid[0]) and grid[nx][ny] == 0: #新点 nx, ny 是否在地图边界内，并且该点是否为通行区域
                if neighbor in visited: #已访问过则跳过
                    continue
                tentative = cost + 1  # 临时代价：从起点到 neighbor 的代价（一步代价为1），cost为当前点的g值，此处为下一个点的
                if tentative < gscore.get(neighbor, float('inf')): #如果这个路径比之前找到的更优
                    came_from[neighbor] = current
                    gscore[neighbor] = tentative  # 更新 g 值
                    fscore = tentative + heuristic(neighbor, goal) # 计算 f = g + h
                    heapq.heappush(open_set, (fscore, tentative, neighbor)) # 加入候选点
    return [] #为空还没找到终点，说明无路径可达，返回空路径

def coverage_path_full_astar(grid, start): #实现全覆盖路径规划
    h, w = len(grid), len(grid[0])  #读取地图大小
    visited = set()   # 创建一个集合，用于记录访问过的点
    visited.add(start) # 起点加入访问集合
    path = [start]     # 初始化路径，初始路径中只有起点
    free_points = set()  # 以步长2挑选自由点作为目标点集合
    for i in range(0, h, 2):
        for j in range(0, w, 2):
            if grid[i][j] == 0:
                free_points.add((i, j))
    if start not in free_points and grid[start[0]][start[1]] == 0: #如果起点是自由区域（不是障碍），但因为步长为2没有被选中，也应加入目标点集合
        free_points.add(start)
    free_points.discard(start) # 起点不再是目标点
    current = start   # 当前的位置初始化为起点
    while free_points:  #只要还有目标点未访问，就继续运行
        sorted_targets = sorted(free_points, key=lambda p: (heuristic(current, p), p[1]))  #将所有未访问的目标点按照距离当前点的曼哈顿距离排序，如果距离相等，按横坐标 p[1] 再次排序
        found_path = None
        for target in sorted_targets:
            sub_path = astar(grid, current, target)  # 使用 A* 算法规划到该目标点的路径
            if sub_path:
                found_path = sub_path   # 一旦找到一条可行路径就停止
                break
        if not found_path:  # 无法到达剩余目标，则提前终止整个路径覆盖
            break
        for p in found_path[1:]: # 将找到的路径加入全局路径
            path.append(p)  # 添加到主路径中
            visited.add(p)  # 标记为访问过
            if p in free_points:
                free_points.remove(p)   # 若该点是目标点，移除它
        current = path[-1]  # 更新当前位置为路径的最后一个点，作为下一次路径起点
    return path

def upscale_grid(grid, sub_factor): #将低分辨率的栅格地图grid放大为更高分辨率地图
    h, w = len(grid), len(grid[0])
    fine_grid = np.zeros((h * sub_factor, w * sub_factor), dtype=int)
    for i in range(h):
        for j in range(w):
            if grid[i][j] == 1:
                fine_grid[i * sub_factor:(i + 1) * sub_factor,
                           j * sub_factor:(j + 1) * sub_factor] = 1
    return fine_grid

def get_coverage_mask(grid, path, car_length, car_width, sub_factor=3): #计算机器人行驶路径上被覆盖的区域（膨胀考虑机器人尺寸）
    h, w = len(grid), len(grid[0])
    fine_h, fine_w = h * sub_factor, w * sub_factor
    coverage_count = np.zeros((fine_h, fine_w), dtype=int)
    half_len = car_length // 2
    half_wid = car_width // 2
    for px, py in path:
        cx, cy = px * sub_factor + sub_factor // 2, py * sub_factor + sub_factor // 2
        for dx in range(-half_len, half_len + 1): #遍历机器人覆盖范围的每个格子
            for dy in range(-half_wid, half_wid + 1):
                nx, ny = cx + dx, cy + dy
                if 0 <= nx < fine_h and 0 <= ny < fine_w:
                    coverage_count[nx][ny] += 1  #覆盖计数器加1，表示该细分格子被覆盖了一次
    return coverage_count  #返回完整的覆盖计数矩阵

def adjust_turning_overlap(path, coverage_mask, fine_grid_mask, car_length, car_width, sub_factor): #对路径中转弯点的覆盖计数进行调整，减少转弯处重叠区域的覆盖次数，避免重复计算过多
    half_len = car_length // 2
    half_wid = car_width // 2
    for i in range(1, len(path) - 1):  #遍历路径中每个除首尾外的点，考虑当前点和其前后相邻点
        prev, curr, nex = path[i - 1], path[i], path[i + 1]  #取出当前点的前一个点prev，当前点curr和下一个点nex
        dir1 = (curr[0] - prev[0], curr[1] - prev[1])
        dir2 = (nex[0] - curr[0], nex[1] - curr[1])
        if dir1 != dir2:  #判断是否转弯（前后方向不同即转弯点）
            cx, cy = curr[0] * sub_factor + sub_factor // 2, curr[1] * sub_factor + sub_factor // 2
            for dx in range(-half_len, half_len + 1):
                for dy in range(-half_wid, half_wid + 1):
                    nx, ny = cx + dx, cy + dy
                    if (0 <= nx < coverage_mask.shape[0] and 0 <= ny < coverage_mask.shape[1] and
                        fine_grid_mask[nx][ny] == 0 and coverage_mask[nx][ny] > 1):
                        coverage_mask[nx][ny] -= 1  #将该点的覆盖计数减1，减少重复覆盖量，避免转弯处统计过多覆盖

def compute_coverage_stats(coverage_mask, fine_grid_mask):
    total_repeat_cells = np.sum((coverage_mask > 1) & (fine_grid_mask == 0))  #计算重复覆盖的自由格子数量（覆盖次数大于1且非障碍）
    total_covered_cells = np.sum((coverage_mask >= 1) & (fine_grid_mask == 0)) #计算被覆盖的自由格子总数（覆盖次数至少为1且非障碍）
    total_free_cells = np.sum(fine_grid_mask == 0)  #计算自由格子总数
    coverage_rate = total_covered_cells / total_free_cells if total_free_cells > 0 else 0  #计算覆盖率
    repetition_rate = total_repeat_cells / total_free_cells if total_free_cells > 0 else 0  #计算重复率
    print(f"细分后可通行细胞数: {total_free_cells}")
    print(f"被覆盖细胞数: {total_covered_cells}")
    print(f"重复覆盖细胞数: {total_repeat_cells}")
    print(f" 覆盖率: {coverage_rate:.2%}")
    print(f" 重复率: {repetition_rate:.2%}")

def visualize_coverage_map(fine_grid_mask, coverage_mask):
    fig, ax = plt.subplots(figsize=(10, 10))  #创建一个 matplotlib 图像窗口和坐标轴对象，大小为 10 x 10 英寸
    ax.imshow(np.array(fine_grid_mask), cmap='Greys', origin='upper')  #origin='upper' 表示图像的原点在左上角
    overlay = np.ma.masked_where(coverage_mask == 0, coverage_mask)  #只显示覆盖过的区域（覆盖次数 ≥ 1）
    heatmap = ax.imshow(overlay, cmap='Reds', origin='upper', alpha=0.7) #半透明叠加颜色
    plt.colorbar(heatmap, ax=ax, label="覆盖次数")
    ax.set_title("高精度覆盖热图")
    plt.show()

def save_path_to_txt(path, filename="path_points.txt"): #写入文件
    with open(filename, 'w') as f:
        for p in path:
            f.write(f"({p[0]},{p[1]})\n")

if __name__ == "__main__":
    filename = r"C:\Users\24816\Desktop\project\shange\occupancy_grid.txt"
    grid = load_grid_from_txt(filename)

    print("膨胀障碍物（膨胀半径7）...")
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
