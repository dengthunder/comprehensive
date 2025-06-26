import sim
import time
import math
import heapq
import numpy as np
import matplotlib
import matplotlib.pyplot as plt



#################### A*路径规划相关 ####################
def load_grid_from_txt(filename): #加载地图
    grid = []
    with open(filename, 'r') as f:
        for line in f:
            grid.append(list(map(int, line.strip().split()))) #拆分空格来返回二维数组
    return np.array(grid)

def inflate_obstacles(grid, inflate_radius=7): #膨胀障碍物，使路径更安全，避免贴障碍行驶
    from scipy.ndimage import binary_dilation
    obstacle_map = (grid == 1)
    structure = np.ones((inflate_radius*2+1, inflate_radius*2+1))
    inflated = binary_dilation(obstacle_map, structure=structure).astype(int) #利用 binary_dilation 膨胀障碍，扩大障碍范围。生成一个全1的结构体structure作为膨胀核。
    inflated_grid = grid.copy()
    inflated_grid[inflated == 1] = 1
    return inflated_grid

def heuristic(a, b): #曼哈顿距离作为A*估价函数，计算两个点行列差的绝对值之和。
    return abs(a[0] - b[0]) + abs(a[1] - b[1])

def astar(grid, start, goal): #A*寻路算法，输入网格地图、起点和终点，返回路径列表（网格坐标）

    rows, cols = grid.shape
    open_set = []
    heapq.heappush(open_set, (0 + heuristic(start, goal), 0, start))
    came_from = {}
    g_score = {start: 0}
    closed_set = set()

    neighbors = [(1,0), (-1,0), (0,1), (0,-1)]

    while open_set:
        _, cost, current = heapq.heappop(open_set)

        if current == goal:
            # 重建路径
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            path.append(start)
            path.reverse()
            return path

        closed_set.add(current)

        for dx, dy in neighbors:
            neighbor = (current[0] + dx, current[1] + dy)
            if 0 <= neighbor[0] < rows and 0 <= neighbor[1] < cols:
                if grid[neighbor[0]][neighbor[1]] == 1:
                    continue  # 障碍
                if neighbor in closed_set:
                    continue

                tentative_g = g_score[current] + 1
                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f_score = tentative_g + heuristic(neighbor, goal)
                    heapq.heappush(open_set, (f_score, tentative_g, neighbor))
    return None  # 无路径

def grid_to_world(point): #坐标转换，适配栅格坐标与仿真世界坐标之间的映射
    # 栅格坐标转世界坐标
    x_raw, y_raw = point[1], point[0]  # 注意行列转XY
    x = x_raw / 30.46 * 6 - 10
    y = y_raw / 30.46 * 6
    return (x, y)

def world_to_grid(point):
    # 世界坐标转栅格坐标
    x, y = point
    x_raw = int(round((x + 10) / 6 * 30.46))
    y_raw = int(round(y / 6 * 30.46))
    return (y_raw, x_raw)  # 返回行列坐标


#################### CoppeliaSim控制相关 ####################
def set_wheel_speed(clientID, left_joint, right_joint, left_speed, right_speed):
    sim.simxSetJointTargetVelocity(clientID, left_joint, -left_speed, sim.simx_opmode_streaming)
    sim.simxSetJointTargetVelocity(clientID, right_joint, -right_speed, sim.simx_opmode_streaming)

def get_robot_position(clientID, robot_handle): #获取机器人当前位置
    _, pos = sim.simxGetObjectPosition(clientID, robot_handle, -1, sim.simx_opmode_blocking)
    return pos[0], pos[1]

def get_robot_orientation(clientID, robot_handle): #获取机器人朝向角
    _, ori = sim.simxGetObjectOrientation(clientID, robot_handle, -1, sim.simx_opmode_blocking)
    return ori[2]

def read_proximity_sensor(clientID, sensor_handle):#读取距离传感器状态。
    _, detection_state, _, _, _ = sim.simxReadProximitySensor(clientID, sensor_handle, sim.simx_opmode_blocking)
    return detection_state

def rotate_to_target_angle(clientID, robot_handle, left_joint, right_joint, target_angle, max_turn_speed=0.8):#实现机器人原地旋转，调整方向到目标角度。
    print("进入原地旋转模式...")
    while True:
        current_angle = get_robot_orientation(clientID, robot_handle)
        angle_error = (target_angle - current_angle + math.pi) % (2 * math.pi) - math.pi
        if abs(angle_error) < 0.02:
            set_wheel_speed(clientID, left_joint, right_joint, 0, 0)
            print("原地旋转完成，对准目标角度")
            break
        turn_speed = max(-max_turn_speed, min(max_turn_speed, angle_error))
        set_wheel_speed(clientID, left_joint, right_joint, -turn_speed, turn_speed)
        time.sleep(0.03)


#################### 路径跟踪与动态重规划 ####################
def drive_to_target_with_avoidance(clientID, robot_handle, left_joint, right_joint,
                                   sensor_handle, target, path_points, current_index,
                                   grid_map, base_speed=8, tolerance=0.05):
    #PID控制相关变量初始化
    prev_angle_error = 0.0
    integral_error = 0.0
    prev_turn_speed = 0.0

    max_wheel_speed = 15
    angle_large_thresh = 0.05
    angle_small_thresh = 0.01
    max_turn_change = 0.15 #单次允许转速变化最大值，避免转向突变
    max_turn_speed = 0.9 #最大转向速度
    lookahead_k = 5  # 前瞻步数
    #PID控制器参数
    Kp_angle = 1.0
    Ki_angle = 0.08
    Kd_angle = 0.35
    #避障计数器与阈值，连续避障次数超过MAX_AVOIDANCE_COUNT将触发动态路径重规划。
    avoidance_counter = 0
    MAX_AVOIDANCE_COUNT = 10
    rotation_threshold = math.radians(15)

    while True:
        x, y = get_robot_position(clientID, robot_handle)
        dx = target[0] - x
        dy = target[1] - y
        distance = math.hypot(dx, dy) #用math.hypot(dx, dy)计算两点距离，作为当前到目标点的误差

        if distance < tolerance: #判断是否已到达目标点 若达到目标，停止轮子运动，打印当前状态，跳出循环，结束该目标点跟踪。
            set_wheel_speed(clientID, left_joint, right_joint, 0, 0)
            print(f'到达目标点 {target}，当前位置 ({x:.2f}, {y:.2f})，距离误差 {distance:.4f}')
            break

        # 前瞻角度计算
        lookahead_index = min(current_index + lookahead_k, len(path_points) - 1)
        future_target = path_points[lookahead_index]
        dx_f = future_target[0] - x
        dy_f = future_target[1] - y
        future_angle = math.atan2(dy_f, dx_f)  #利用atan2计算机器人到前瞻点的方向角future_angle

        current_angle = get_robot_orientation(clientID, robot_handle) #读取机器人当前航向角
        future_angle_error = (future_angle - current_angle + math.pi) % (2 * math.pi) - math.pi

        target_angle = math.atan2(dy, dx)
        angle_error_direct = (target_angle - current_angle + math.pi) % (2 * math.pi) - math.pi #计算当前目标方向的角度误差，规范化到[-π, π]

        # 连续避障失败处理，触发动态重规划
        if avoidance_counter > MAX_AVOIDANCE_COUNT:
            print("连续避障失败，触发动态路径重规划...")
            new_path_points = dynamic_replan_path(clientID, robot_handle, grid_map, target, path_points, current_index)#调用dynamic_replan_path函数重新规划路径
            if new_path_points is None or len(new_path_points) == 0:
                print("重规划失败，保持当前目标。")
                avoidance_counter = 0
            else:
                print(f"重规划成功，新增路径长度：{len(new_path_points)}")
                # 将新路径替换当前路径点列表中当前索引后的点
                path_points[current_index+1:] = new_path_points[1:]  # 从新路径第二个点开始追加，保持当前位置为第一个点
                avoidance_counter = 0
            # 重新计算当前目标点，跳出循环重新开始追踪新路径点
            break

        if abs(angle_error_direct) > rotation_threshold: #若当前目标点方向误差大于30度，调用rotate_to_target_angle进行原地旋转调整方向，之后重新循环。
            rotate_to_target_angle(clientID, robot_handle, left_joint, right_joint, target_angle)
            continue

        if read_proximity_sensor(clientID, sensor_handle) == 1:
            avoidance_counter += 1
            print(f"检测到障碍，避障计数: {avoidance_counter}")
            set_wheel_speed(clientID, left_joint, right_joint, -1.0, -0.3) #设置轮子反向旋转避障（左轮速度-1，右轮-0.3，轻微偏转）
            time.sleep(0.1)
            continue

        # PID角度控制
        angle_error = angle_error_direct #赋值角度误差。
        integral_error += angle_error #更新积分项，并限制积分项范围防止积分饱和
        integral_error = max(-0.5, min(0.5, integral_error))
        delta_error = angle_error - prev_angle_error #计算误差变化率
        #PID计算得到转向速度，比例、积分、微分部分相加
        raw_turn_speed = (
            Kp_angle * angle_error +
            Ki_angle * integral_error +
            Kd_angle * delta_error
        )
        raw_turn_speed = max(-max_turn_speed, min(max_turn_speed, raw_turn_speed)) #限制转速变化速率,如果变化过大，则按最大变化量调整

        turn_diff = raw_turn_speed - prev_turn_speed
        if abs(turn_diff) > max_turn_change:
            raw_turn_speed = prev_turn_speed + max_turn_change * (1 if turn_diff > 0 else -1)

        prev_turn_speed = raw_turn_speed #更新上一转向速度和角度误差，供下次循环计算
        prev_angle_error = angle_error

        # 前瞻加速控制
        if abs(future_angle_error) > angle_large_thresh: #误差大，速度0.4保证转向稳定。
            linear_speed = 0.6
        elif abs(future_angle_error) < angle_small_thresh:
            raw_turn_speed = 0.0
            linear_speed = base_speed * (1 - math.exp(-2.5 * distance))
        else:  #误差中等时，线速度固定2
            linear_speed = 2

        left_speed = linear_speed - raw_turn_speed #左轮速度 = 线速度 - 转向速度
        right_speed = linear_speed + raw_turn_speed #右轮速度 = 线速度 + 转向速度

        left_speed = max(-max_wheel_speed, min(max_wheel_speed, left_speed))
        right_speed = max(-max_wheel_speed, min(max_wheel_speed, right_speed))

        set_wheel_speed(clientID, left_joint, right_joint, left_speed, right_speed)
        time.sleep(0.03)

def dynamic_replan_path(clientID, robot_handle, grid_map, target, path_points, current_index):
    """
    触发动态路径重规划：从当前位置到路径中下一个目标点规划新的路径
    """
    x, y = get_robot_position(clientID, robot_handle)
    start_grid = world_to_grid((x, y))
    goal_grid = world_to_grid(target) #通过world_to_grid函数，把机器人的世界坐标位置和目标点target转换成对应的栅格地图坐标

    print(f"动态重规划：起点{start_grid} -> 终点{goal_grid}")
    new_path_grid = astar(grid_map, start_grid, goal_grid) #调用astar函数，在grid_map中，从start_grid规划到goal_grid的新路径，返回的是栅格坐标点列表

    if new_path_grid is None:
        print("A*路径规划失败")
        return None

    # 栅格路径转换为世界坐标路径
    new_path_world = [grid_to_world(p) for p in new_path_grid]

    return new_path_world


#################### 路径读取 ####################
def read_scaled_path_from_file(filename):
    path_points = []
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


#################### 主函数 ####################
def main():
    matplotlib.rcParams['font.sans-serif'] = ['SimHei']  # ✅ 中文显示
    matplotlib.rcParams['axes.unicode_minus'] = False
    sim.simxFinish(-1)
    clientID = sim.simxStart('127.0.0.1', 19997, True, True, 5000, 5)
    if clientID == -1:
        print('无法连接 CoppeliaSim，请检查远程API设置')
        return

    print('已连接到 CoppeliaSim')

    # 获取句柄
    _, robot_handle = sim.simxGetObjectHandle(clientID, 'saodiche', sim.simx_opmode_blocking)
    _, left_joint = sim.simxGetObjectHandle(clientID, 'joint_left', sim.simx_opmode_blocking)
    _, right_joint = sim.simxGetObjectHandle(clientID, 'joint_right', sim.simx_opmode_blocking)
    _, sensor_handle = sim.simxGetObjectHandle(clientID, 'Proximity_sensor', sim.simx_opmode_blocking)

    # 启动数据流
    sim.simxGetObjectPosition(clientID, robot_handle, -1, sim.simx_opmode_streaming)
    sim.simxGetObjectOrientation(clientID, robot_handle, -1, sim.simx_opmode_streaming)
    sim.simxReadProximitySensor(clientID, sensor_handle, sim.simx_opmode_streaming)
    time.sleep(0.1)

    # 读取栅格地图，并膨胀障碍物
    grid_map_raw = load_grid_from_txt('C:\\Users\\24816\\Desktop\\project\\shange\\occupancy_grid.txt')
    grid_map = inflate_obstacles(grid_map_raw, inflate_radius=7)
    print(f'地图大小: {grid_map.shape}, 障碍物膨胀完成')

    # 读取原始路径点（世界坐标）
    path_points = read_scaled_path_from_file('C:\\Users\\24816\\Desktop\\project\\A\\path_points.txt')
    print(f'初始路径点数量: {len(path_points)}')

    trace = []
    idx = 0
    while idx < len(path_points):
        target = path_points[idx]
        print(f'[{idx + 1}/{len(path_points)}] 移动到目标点: {target}')

        drive_to_target_with_avoidance(clientID, robot_handle, left_joint, right_joint,
                                       sensor_handle, target, path_points, idx,
                                       grid_map, base_speed=5, tolerance=0.05)

        pos = get_robot_position(clientID, robot_handle)
        trace.append(pos)

        idx += 1  # 目标点索引递增

    # 绘制轨迹和路径
    if trace:
        x_vals = [p[0] for p in trace]
        y_vals = [p[1] for p in trace]
        tx = [p[0] for p in path_points]
        ty = [p[1] for p in path_points]

        plt.figure(figsize=(8, 6))
        plt.plot(x_vals, y_vals, 'bo-', label='robot road')
        plt.plot(tx, ty, 'rx--', label='target')
        plt.xlabel('X')
        plt.ylabel('Y')
        plt.legend()
        plt.title('map')
        plt.grid(True)
        plt.axis('equal')
        plt.show()

    sim.simxFinish(clientID)
    print('🔚 连接关闭')


if __name__ == '__main__':
    main()
