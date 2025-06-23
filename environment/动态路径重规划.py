import sim
import time
import math
import matplotlib.pyplot as plt
import heapq

# ------------------ 工具函数 ------------------

def set_wheel_speed(clientID, left_joint, right_joint, left_speed, right_speed):
    sim.simxSetJointTargetVelocity(clientID, left_joint, -left_speed, sim.simx_opmode_streaming)
    sim.simxSetJointTargetVelocity(clientID, right_joint, -right_speed, sim.simx_opmode_streaming)

def get_robot_position(clientID, robot_handle):
    _, position = sim.simxGetObjectPosition(clientID, robot_handle, -1, sim.simx_opmode_blocking)
    return position[0], position[1]

def get_robot_orientation(clientID, robot_handle):
    _, orientation = sim.simxGetObjectOrientation(clientID, robot_handle, -1, sim.simx_opmode_blocking)
    return orientation[2]

def read_proximity_sensor(clientID, sensor_handle):
    _, detection_state, _, _, _ = sim.simxReadProximitySensor(clientID, sensor_handle, sim.simx_opmode_blocking)
    return detection_state

def rotate_to_target_angle(clientID, robot_handle, left_joint, right_joint, target_angle, max_turn_speed=0.8):
    print("🔄 进入原地旋转模式...")
    while True:
        current_angle = get_robot_orientation(clientID, robot_handle)
        angle_error = (target_angle - current_angle + math.pi) % (2 * math.pi) - math.pi

        if abs(angle_error) < 0.02:
            set_wheel_speed(clientID, left_joint, right_joint, 0, 0)
            print("✅ 原地旋转完成，对准目标角度")
            break

        turn_speed = max(-max_turn_speed, min(max_turn_speed, angle_error))
        set_wheel_speed(clientID, left_joint, right_joint, -turn_speed, turn_speed)
        time.sleep(0.03)

def load_grid_from_txt(filename):
    grid = []
    with open(filename, 'r') as file:
        for line in file:
            line = line.strip()
            if not line:
                continue
            # 只保留0和1，过滤空格
            row = []
            for ch in line:
                if ch in ('0', '1'):
                    row.append(int(ch))
            if row:
                grid.append(row)
    return grid

def world_to_grid(x, y):
    x_raw = (x + 10) / 6 * 30.46
    y_raw = y / 6 * 30.46
    return int(round(y_raw)), int(round(x_raw))  # 行i, 列j

def grid_to_world(i, j):
    x = j / 30.46 * 6 - 10
    y = i / 30.46 * 6
    return x, y

def a_star(grid, start, goal):
    rows, cols = len(grid), len(grid[0])
    heap = [(0 + abs(goal[0]-start[0]) + abs(goal[1]-start[1]), 0, start, [])]
    visited = set()

    while heap:
        f, cost, current, path = heapq.heappop(heap)
        if current in visited:
            continue
        visited.add(current)
        path = path + [current]
        if current == goal:
            return path

        for dx, dy in [(-1,0), (1,0), (0,-1), (0,1)]:
            ni, nj = current[0] + dx, current[1] + dy
            if 0 <= ni < rows and 0 <= nj < cols and grid[ni][nj] == 0:
                heapq.heappush(heap, (cost+1 + abs(goal[0]-ni) + abs(goal[1]-nj), cost+1, (ni, nj), path))
    return []

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
                    print(f'❌ 跳过无效行: {line}')
    return path_points

# ------------------ 核心函数 ------------------

def drive_to_target_with_avoidance(clientID, robot_handle, left_joint, right_joint,
                                   sensor_handle, target, base_speed=2.8, tolerance=0.05):
    prev_angle_error = 0.0
    integral_error = 0.0
    prev_turn_speed = 0.0

    max_wheel_speed = 4.0
    angle_large_thresh = 0.05
    angle_small_thresh = 0.01
    max_turn_change = 0.08
    max_turn_speed = 0.9

    Kp_angle = 1.0
    Ki_angle = 0.08
    Kd_angle = 0.35
    Kp_distance = 1.6

    avoidance_counter = 0
    MAX_AVOIDANCE_COUNT = 10

    while True:
        x, y = get_robot_position(clientID, robot_handle)
        dx = target[0] - x
        dy = target[1] - y
        distance = math.hypot(dx, dy)

        if distance < tolerance:
            set_wheel_speed(clientID, left_joint, right_joint, 0, 0)
            print(f'✅ 到达目标点 {target}，当前位置 ({x:.2f}, {y:.2f})，距离误差 {distance:.4f}')
            break

        target_angle = math.atan2(dy, dx)

        if avoidance_counter > MAX_AVOIDANCE_COUNT:
            print("🚨 避障失败，开始路径重规划...")

            current_grid = world_to_grid(x, y)
            goal_grid = world_to_grid(target[0], target[1])
            new_path = a_star(global_grid_map, current_grid, goal_grid)

            if new_path:
                new_world_path = [grid_to_world(i, j) for i, j in new_path]
                print(f"✅ 新路径长度: {len(new_world_path)}，返回重规划路径")
                return new_world_path
            else:
                print("❌ 重规划失败，维持原地旋转")
                rotate_to_target_angle(clientID, robot_handle, left_joint, right_joint, target_angle)
                avoidance_counter = 0
                continue

        detection_state = read_proximity_sensor(clientID, sensor_handle)

        if detection_state == 1:
            avoidance_counter += 1
            print(f"⚠️ 检测到障碍，避障计数: {avoidance_counter}")
            set_wheel_speed(clientID, left_joint, right_joint, -1.0, -0.3)
            time.sleep(0.3)
            continue

        current_angle = get_robot_orientation(clientID, robot_handle)
        angle_error = (target_angle - current_angle + math.pi) % (2 * math.pi) - math.pi

        integral_error += angle_error
        integral_error = max(-0.5, min(0.5, integral_error))
        delta_error = angle_error - prev_angle_error

        raw_turn_speed = (
            Kp_angle * angle_error +
            Ki_angle * integral_error +
            Kd_angle * delta_error
        )
        raw_turn_speed = max(-max_turn_speed, min(max_turn_speed, raw_turn_speed))

        turn_diff = raw_turn_speed - prev_turn_speed
        if abs(turn_diff) > max_turn_change:
            raw_turn_speed = prev_turn_speed + max_turn_change * (1 if turn_diff > 0 else -1)

        prev_turn_speed = raw_turn_speed
        prev_angle_error = angle_error

        if abs(angle_error) > angle_large_thresh:
            linear_speed = 0.2
        elif abs(angle_error) < angle_small_thresh:
            raw_turn_speed = 0.0
            linear_speed = min(base_speed, Kp_distance * distance)
        else:
            linear_speed = 0.4

        left_speed = linear_speed - raw_turn_speed
        right_speed = linear_speed + raw_turn_speed

        left_speed = max(-max_wheel_speed, min(max_wheel_speed, left_speed))
        right_speed = max(-max_wheel_speed, min(max_wheel_speed, right_speed))

        set_wheel_speed(clientID, left_joint, right_joint, left_speed, right_speed)
        time.sleep(0.03)

    return None  # 正常结束无新路径返回

# ------------------ 主程序 ------------------

def main():
    sim.simxFinish(-1)
    clientID = sim.simxStart('127.0.0.1', 19997, True, True, 5000, 5)

    if clientID != -1:
        print('✅ 已连接到 CoppeliaSim')

        _, robot_handle = sim.simxGetObjectHandle(clientID, 'saodiche', sim.simx_opmode_blocking)
        _, left_joint = sim.simxGetObjectHandle(clientID, 'joint_left', sim.simx_opmode_blocking)
        _, right_joint = sim.simxGetObjectHandle(clientID, 'joint_right', sim.simx_opmode_blocking)
        _, sensor_handle = sim.simxGetObjectHandle(clientID, 'Proximity_sensor', sim.simx_opmode_blocking)

        sim.simxGetObjectPosition(clientID, robot_handle, -1, sim.simx_opmode_streaming)
        sim.simxGetObjectOrientation(clientID, robot_handle, -1, sim.simx_opmode_streaming)
        sim.simxReadProximitySensor(clientID, sensor_handle, sim.simx_opmode_streaming)
        time.sleep(0.1)

        global global_grid_map
        global_grid_map = load_grid_from_txt('C:\\Users\\24816\\Desktop\\comprehensive project\\occupancy_grid.txt')

        path_points = read_scaled_path_from_file('C:\\Users\\24816\\Desktop\\comprehensive project\\path_output.txt')
        print(f'📄 路径点数量: {len(path_points)}')

        trace = []
        i = 0
        while i < len(path_points):
            print(f'➡️ [{i + 1}] 移动到: {path_points[i]}')
            result = drive_to_target_with_avoidance(
                clientID, robot_handle, left_joint, right_joint,
                sensor_handle, path_points[i], base_speed=2.8, tolerance=0.05)

            pos = get_robot_position(clientID, robot_handle)
            trace.append(pos)

            if isinstance(result, list):
                # 插入新路径，剔除第一个点（当前位置）
                path_points = path_points[:i+1] + result[1:] + path_points[i+1:]
                print(f"🔄 路径已更新，新的路径点数: {len(path_points)}")

            i += 1

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
            plt.title('机器人路径追踪（实时避障 + 动态重规划）')
            plt.grid(True)
            plt.axis('equal')
            plt.show()

        sim.simxFinish(clientID)
        print('🔚 连接关闭')
    else:
        print('❌ 无法连接 CoppeliaSim，请检查远程API设置')

if __name__ == '__main__':
    main()
