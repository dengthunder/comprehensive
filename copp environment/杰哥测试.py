import sim
import time
import math
import matplotlib.pyplot as plt


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
    rotation_threshold = math.radians(30)  # 30度阈值（≈0.523弧度）

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

        detection_state = read_proximity_sensor(clientID, sensor_handle)
        if detection_state == 1:
            avoidance_counter += 1
            print(f"⚠️ 检测到障碍，避障计数: {avoidance_counter}")
            set_wheel_speed(clientID, left_joint, right_joint, -1.0, -0.3)
            time.sleep(0.3)
            continue

        # PID 控制器进行连续路径跟踪
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

        # 根据角度误差决定线速度
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

        path_points = read_scaled_path_from_file('C:\\Users\\24816\\Desktop\\comprehensive project\\path_output.txt')
        print(f'📄 路径点数量: {len(path_points)}')

        trace = []
        for idx, point in enumerate(path_points):
            print(f'➡️ [{idx + 1}] 移动到: {point}')
            drive_to_target_with_avoidance(
                clientID, robot_handle, left_joint, right_joint,
                sensor_handle, point, base_speed=2.8, tolerance=0.05)
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
        print('❌ 无法连接 CoppeliaSim，请检查远程API设置')


if __name__ == '__main__':
    main()
