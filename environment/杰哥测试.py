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
                                   sensor_handle, target, path_points, current_index,
                                   base_speed=8, tolerance=0.05):
    prev_angle_error = 0.0
    integral_error = 0.0
    prev_turn_speed = 0.0

    max_wheel_speed = 12
    angle_large_thresh = 0.05
    angle_small_thresh = 0.01
    max_turn_change = 0.08
    max_turn_speed = 0.9
    lookahead_k = 3  # 前瞻步数

    Kp_angle = 1.0
    Ki_angle = 0.08
    Kd_angle = 0.35
    Kp_distance = 1.6

    avoidance_counter = 0
    MAX_AVOIDANCE_COUNT = 10
    rotation_threshold = math.radians(30)

    while True:
        x, y = get_robot_position(clientID, robot_handle)
        dx = target[0] - x
        dy = target[1] - y
        distance = math.hypot(dx, dy)

        if distance < tolerance:
            set_wheel_speed(clientID, left_joint, right_joint, 0, 0)
            print(f'到达目标点 {target}，当前位置 ({x:.2f}, {y:.2f})，距离误差 {distance:.4f}')
            break

        # --- 前瞻角度计算 ---
        lookahead_index = min(current_index + lookahead_k, len(path_points) - 1)
        future_target = path_points[lookahead_index]
        dx_f = future_target[0] - x
        dy_f = future_target[1] - y
        future_angle = math.atan2(dy_f, dx_f)

        current_angle = get_robot_orientation(clientID, robot_handle)
        future_angle_error = (future_angle - current_angle + math.pi) % (2 * math.pi) - math.pi

        target_angle = math.atan2(dy, dx)
        angle_error_direct = (target_angle - current_angle + math.pi) % (2 * math.pi) - math.pi

        # 强制原地转向机制
        if avoidance_counter > MAX_AVOIDANCE_COUNT:
            rotate_to_target_angle(clientID, robot_handle, left_joint, right_joint, target_angle)
            avoidance_counter = 0
            continue

        if abs(angle_error_direct) > rotation_threshold:
            rotate_to_target_angle(clientID, robot_handle, left_joint, right_joint, target_angle)
            continue

        if read_proximity_sensor(clientID, sensor_handle) == 1:
            avoidance_counter += 1
            print(f"检测到障碍，避障计数: {avoidance_counter}")
            set_wheel_speed(clientID, left_joint, right_joint, -1.0, -0.3)
            time.sleep(0.1)
            continue

        # PID 控制角度
        angle_error = angle_error_direct
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

        # ---------- 前瞻判断加速 ----------
        if abs(future_angle_error) > angle_large_thresh:
            linear_speed = 0.4  # 前瞻角度偏差大，减速
        elif abs(future_angle_error) < angle_small_thresh:
            raw_turn_speed = 0.0
            linear_speed = base_speed * (1 - math.exp(-2.5 * distance))
        else:
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
                sensor_handle, point, path_points, idx, base_speed=5, tolerance=0.05)
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
            plt.title('机器人路径追踪（实时避障）')
            plt.grid(True)
            plt.axis('equal')
            plt.show()

        sim.simxFinish(clientID)
        print('🔚 连接关闭')
    else:
        print('无法连接 CoppeliaSim，请检查远程API设置')


if __name__ == '__main__':
    main()
