import heapq
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
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
    struct = np.ones((2 * inflate_radius + 1, 2 * inflate_radius + 1))
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
            block = original_map_np[i * factor:(i + 1) * factor, j * factor:(j + 1) * factor]
            downsampled_map[i, j] = 1 if np.any(block == 1) else 0
    return downsampled_map.tolist()


# ---------- 启发式函数（曼哈顿距离） ----------
def heuristic(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1])


# ---------- A* 算法 ----------
def astar(grid, start, goal):
    directions = [(0, 1), (1, 0), (0, -1), (-1, 0)]  # 右，下，左，上
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


# ---------- 改进的动态路径可视化 ----------
def visualize_path_dynamic(grid, path, save_path=None, car_width_cells=10.630942091616):
    grid_show = [[1 if cell == 1 else 0 for cell in row] for row in grid]
    fig, ax = plt.subplots(figsize=(12, 12))

    # 预先分析整个路径，统计每个线段的出现次数
    segment_count = {}
    repeated_segments = set()

    # 统计每个线段(from_point, to_point)的出现次数
    for i in range(1, len(path)):
        from_point = path[i - 1]
        to_point = path[i]
        # 创建线段标识符（确保方向一致性，小的点在前）
        segment = tuple(sorted([from_point, to_point]))

        if segment not in segment_count:
            segment_count[segment] = []
        segment_count[segment].append(i)  # 记录该线段在路径中的位置索引

    # 找出所有重复的线段（出现次数≥2的线段，从第二次开始标记为红色）
    for segment, indices in segment_count.items():
        if len(indices) >= 2:  # 线段被走过≥2次
            # 从第二次开始，所有该线段都标记为红色
            for idx in indices[1:]:  # 跳过第一次出现
                repeated_segments.add(idx)

    for i, current_point in enumerate(path):
        ax.cla()
        ax.imshow(grid_show, cmap='Greys', origin='upper')

        # 绘制从开始到当前点的路径
        for j in range(1, i + 1):  # 从第1个点开始（跳过起点）
            # 绘制线段 j-1 -> j
            x_vals = [path[j - 1][1], path[j][1]]
            y_vals = [path[j - 1][0], path[j][0]]

            # 判断该线段是否为重复线段
            if j in repeated_segments:
                ax.plot(x_vals, y_vals, color='red', linewidth=car_width_cells, alpha=0.8)
            else:
                ax.plot(x_vals, y_vals, color='blue', linewidth=car_width_cells, alpha=0.9)

        # 绘制起点正方形（绿色）
        start_square_size = 1.5  # 起点正方形边长
        start_square = patches.Rectangle(
            (path[0][1] - start_square_size / 2, path[0][0] - start_square_size / 2),
            start_square_size, start_square_size,
            linewidth=3, edgecolor='darkgreen', facecolor='green', zorder=5
        )
        ax.add_patch(start_square)

        # 绘制当前位置正方形（红色，边长为car_width_cells）
        current_square_size = car_width_cells / 8  # 调整大小使其合适显示
        current_square = patches.Rectangle(
            (current_point[1] - current_square_size / 2, current_point[0] - current_square_size / 2),
            current_square_size, current_square_size,
            linewidth=4, edgecolor='darkred', facecolor='red', zorder=6
        )
        ax.add_patch(current_square)

        ax.set_title(f"A* 路径进度: 步骤 {i + 1}/{len(path)}")

        # 添加图例说明
        from matplotlib.lines import Line2D
        legend_elements = [
            Line2D([0], [0], marker='s', color='w', markerfacecolor='green',
                   markeredgecolor='darkgreen', markeredgewidth=2, markersize=10, label='起点'),
            Line2D([0], [0], marker='s', color='w', markerfacecolor='red',
                   markeredgecolor='darkred', markeredgewidth=2, markersize=10, label='当前位置'),
            Line2D([0], [0], color='blue', linewidth=4, label='首次经过线段'),
            Line2D([0], [0], color='red', linewidth=4, label='重复经过线段')
        ]
        ax.legend(handles=legend_elements, loc='upper right')

        plt.pause(0.005)

    if save_path:
        plt.savefig(save_path, dpi=300)
    plt.show()


# ---------- 主程序 ----------
if __name__ == "__main__":
    filename = r"occupancy_grid.txt"
    downsample_factor = 6

    print("读取地图...")
    grid = load_grid_from_txt(filename)

    print("膨胀障碍物（安全距离2像素）...")
    grid = inflate_obstacles(grid, inflate_radius=7)

    print(f"降采样，因子={downsample_factor}...")
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

    print("规划全覆盖路径...")
    path = coverage_path_full_astar(grid, start)

    car_width_cells = 10.630942091616

    # 计算统计数据
    visited_once = set()
    repeated = 0
    for p in path:
        if grid[p[0]][p[1]] == 0:
            if p in visited_once:
                repeated += 1
            else:
                visited_once.add(p)

    total_path = len(path)
    total_free_cells = sum(row.count(0) for row in grid)
    repetition_rate = repeated / total_free_cells if total_path > 0 else 0
    coverage_set = get_coverage_set(grid, path, car_width_cells)
    covered = len(coverage_set)
    coverage_rate = (covered - repeated) / total_free_cells if total_free_cells > 0 else 0

    print(f"地图中可通行区域总数: {total_free_cells}")
    print(f"有效覆盖面积（格子数）: {(covered - repeated)}")
    print(f"覆盖率: {coverage_rate:.2%}")
    print(f"重复率: {repetition_rate:.2%}")
    print(f"起点: {start}")

    print("动态绘制路径...")
    image_file = r"path_visualization.png"
    visualize_path_dynamic(grid, path, save_path=image_file, car_width_cells=car_width_cells)
    print("完成")

    import sim
    import time
    import math
    import matplotlib.pyplot as plt


    def set_wheel_speed(clientID, left_joint, right_joint, left_speed, right_speed):  # 设置左右轮速度
        sim.simxSetJointTargetVelocity(clientID, left_joint, -left_speed, sim.simx_opmode_streaming)
        sim.simxSetJointTargetVelocity(clientID, right_joint, -right_speed, sim.simx_opmode_streaming)


    def get_robot_position(clientID, robot_handle):  # 获取机器人当前位置 -1世界坐标系
        _, position = sim.simxGetObjectPosition(clientID, robot_handle, -1, sim.simx_opmode_blocking)
        return position[0], position[1]


    def get_robot_orientation(clientID, robot_handle):  # 返回机器人 z 轴方向角
        _, orientation = sim.simxGetObjectOrientation(clientID, robot_handle, -1, sim.simx_opmode_blocking)
        return orientation[2]


    def read_proximity_sensor(clientID, sensor_handle):  # 读取避障传感器状态，返回是否检测到前方障碍物
        _, detection_state, _, _, _ = sim.simxReadProximitySensor(clientID, sensor_handle, sim.simx_opmode_blocking)
        return detection_state


    def rotate_to_target_angle(clientID, robot_handle, left_joint, right_joint, target_angle,
                               max_turn_speed=0.8):  # 原地转向目标角度
        print("进入原地旋转模式...")
        while True:
            current_angle = get_robot_orientation(clientID, robot_handle)  # 获取 z 轴方向角
            angle_error = (target_angle - current_angle + math.pi) % (2 * math.pi) - math.pi  # 计算角度差

            if abs(angle_error) < 0.02:  # 使用反向轮转实现原地转向，直到误差小于 0.02 弧度
                set_wheel_speed(clientID, left_joint, right_joint, 0, 0)  # 如果误差足够小，立即停止轮子（左右速度都设为0），退出循环。
                print("原地旋转完成，对准目标角度")
                break

            turn_speed = max(-max_turn_speed, min(max_turn_speed,
                                                  angle_error))  # 使用比例控制，把 angle_error直接当作速度，限制其不超过±max_turn_speed，避免旋转过快不稳定。
            set_wheel_speed(clientID, left_joint, right_joint, -turn_speed, turn_speed)
            time.sleep(0.03)  # 用于控制执行频率、避免循环过快


    def drive_to_target_with_avoidance(clientID, robot_handle, left_joint, right_joint,
                                       sensor_handle, target, base_speed=8, tolerance=0.05):
        # 角度PID控制  raw_turn_speed = Kp*error + Ki*∑error + Kd*d(error)/dt，传函中，角速度为输入，角度为输出
        prev_angle_error = 0.0  # 前馈项
        integral_error = 0.0  # 积分项
        prev_turn_speed = 0.0  # 上一次转向速度

        max_wheel_speed = 12  # 最大轮子速度限制
        angle_large_thresh = 0.05  # 如果角度偏差大于这个，减慢前进速度
        angle_small_thresh = 0.01  # 角度很小时停止转动
        max_turn_change = 0.08  # 单次转速变化限制，防止震荡
        max_turn_speed = 0.9  # 最大原地转向速度
        # PID控制器的参数
        Kp_angle = 1.0
        Ki_angle = 0.08
        Kd_angle = 0.35
        Kp_distance = 1.6

        avoidance_counter = 0
        MAX_AVOIDANCE_COUNT = 10  # 避障传感器触发超过10次，就强制原地对准方向
        rotation_threshold = math.radians(30)  # 角度偏差超过30度，先原地转向

        while True:
            # 当前位置与目标距离
            x, y = get_robot_position(clientID, robot_handle)
            dx = target[0] - x
            dy = target[1] - y
            distance = math.hypot(dx, dy)

            # 判断是否到达目标点
            if distance < tolerance:
                set_wheel_speed(clientID, left_joint, right_joint, 0, 0)
                print(f'到达目标点 {target}，当前位置 ({x:.2f}, {y:.2f})，距离误差 {distance:.4f}')
                break

            target_angle = math.atan2(dy, dx)  # 当前应朝向的角度（

            # 避障计数过多，强制原地转向
            if avoidance_counter > MAX_AVOIDANCE_COUNT:
                rotate_to_target_angle(clientID, robot_handle, left_joint, right_joint, target_angle)
                avoidance_counter = 0
                continue

            # 若当前角度差超过 30 度，则先原地转向
            current_angle = get_robot_orientation(clientID, robot_handle)
            angle_error_direct = (target_angle - current_angle + math.pi) % (2 * math.pi) - math.pi
            if abs(angle_error_direct) > rotation_threshold:
                rotate_to_target_angle(clientID, robot_handle, left_joint, right_joint, target_angle)
                continue

            # 检测障碍，短暂后退，并略微转弯
            detection_state = read_proximity_sensor(clientID, sensor_handle)
            if detection_state == 1:
                avoidance_counter += 1
                print(f"检测到障碍，避障计数: {avoidance_counter}")
                set_wheel_speed(clientID, left_joint, right_joint, -1.0, -0.3)
                time.sleep(0.1)
                continue

            # PID 控制器进行连续路径跟踪
            angle_error = angle_error_direct  # 当前误差
            integral_error += angle_error  # 积分误差
            integral_error = max(-0.5, min(0.5, integral_error))  # 限制积分误差
            delta_error = angle_error - prev_angle_error  # 微分误差

            # 转弯速度
            raw_turn_speed = (
                    Kp_angle * angle_error +
                    Ki_angle * integral_error +
                    Kd_angle * delta_error
            )
            raw_turn_speed = max(-max_turn_speed, min(max_turn_speed, raw_turn_speed))

            # 限制一次转向速度变化量，避免过猛震荡
            turn_diff = raw_turn_speed - prev_turn_speed
            if abs(turn_diff) > max_turn_change:
                raw_turn_speed = prev_turn_speed + max_turn_change * (1 if turn_diff > 0 else -1)

            prev_turn_speed = raw_turn_speed
            prev_angle_error = angle_error

            # 根据角度误差决定线速度
            if abs(angle_error) > angle_large_thresh:  # 角度差大，只进行转向
                linear_speed = 0.4
            elif abs(angle_error) < angle_small_thresh:  # 角度小于阈值，加速
                raw_turn_speed = 0.0
                linear_speed = base_speed * (1 - math.exp(-2.5 * distance))
            else:  # 中等情况，边转向边前进
                linear_speed = 0.6

            left_speed = linear_speed - raw_turn_speed
            right_speed = linear_speed + raw_turn_speed

            left_speed = max(-max_wheel_speed, min(max_wheel_speed, left_speed))
            right_speed = max(-max_wheel_speed, min(max_wheel_speed, right_speed))

            set_wheel_speed(clientID, left_joint, right_joint, left_speed, right_speed)
            time.sleep(0.03)


    def read_scaled_path_from_file(filename):  # 读取路径坐标
        path_points = []  # 创建一个空列表，用于存储处理后的路径点坐标
        with open(filename, 'r') as file:
            for line in file:
                line = line.strip().replace('(', '').replace(')', '')
                if line == '':
                    continue
                parts = line.split(',')
                if len(parts) >= 2:
                    try:
                        x_raw, y_raw = float(parts[0]), float(parts[1])
                        x = x_raw / 30.46 * 6 - 10
                        y = y_raw / 30.46 * 6
                        path_points.append((x, y))
                    except ValueError:
                        print(f'跳过无效行: {line}')
        return path_points


    def main():
        sim.simxFinish(-1)
        clientID = sim.simxStart('127.0.0.1', 19997, True, True, 5000, 5)

        if clientID != -1:
            print('已连接到 CoppeliaSim')

            _, robot_handle = sim.simxGetObjectHandle(clientID, 'saodiche', sim.simx_opmode_blocking)
            _, left_joint = sim.simxGetObjectHandle(clientID, 'joint_left', sim.simx_opmode_blocking)
            _, right_joint = sim.simxGetObjectHandle(clientID, 'joint_right', sim.simx_opmode_blocking)
            _, sensor_handle = sim.simxGetObjectHandle(clientID, 'Proximity_sensor', sim.simx_opmode_blocking)
            # 启动数据流，始持续传输位置信息、角度信息和避障传感器状态
            sim.simxGetObjectPosition(clientID, robot_handle, -1, sim.simx_opmode_streaming)
            sim.simxGetObjectOrientation(clientID, robot_handle, -1, sim.simx_opmode_streaming)
            sim.simxReadProximitySensor(clientID, sensor_handle, sim.simx_opmode_streaming)
            time.sleep(0.1)

            path_points = read_scaled_path_from_file(
                'C:\\Users\\24816\\Desktop\\comprehensive project\\path_output.txt')
            print(f'路径点数量: {len(path_points)}')

            trace = []
            for idx, point in enumerate(path_points):
                print(f'[{idx + 1}] 移动到: {point}')
                drive_to_target_with_avoidance(
                    clientID, robot_handle, left_joint, right_joint,
                    sensor_handle, point, base_speed=5, tolerance=0.05)
                pos = get_robot_position(clientID, robot_handle)
                trace.append(pos)

            # 绘制轨迹
            if trace:
                x_vals = [p[0] for p in trace]
                y_vals = [p[1] for p in trace]
                tx = [p[0] for p in path_points]
                ty = [p[1] for p in path_points]

                plt.figure(figsize=(8, 6))
                plt.plot(x_vals, y_vals, 'bo-', label='机器人轨迹')
                plt.plot(tx, ty, 'rx--', label='目标路径点')
                plt.xlabel('X')
                plt.ylabel('Y')
                plt.legend()
                plt.grid(True)
                plt.axis('equal')
                plt.show()

            sim.simxFinish(clientID)
            print('🔚 连接关闭')
        else:
            print('无法连接 CoppeliaSim，请检查远程API设置')


    if __name__ == '__main__':
        main()
