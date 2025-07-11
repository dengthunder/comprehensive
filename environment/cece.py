import sim
import time
import math
import matplotlib.pyplot as plt

# 设置轮子速度
def set_wheel_speed(clientID, left_joint, right_joint, left_speed, right_speed):
    sim.simxSetJointTargetVelocity(clientID, left_joint, -left_speed, sim.simx_opmode_streaming)
    sim.simxSetJointTargetVelocity(clientID, right_joint, -right_speed, sim.simx_opmode_streaming)

# 获取机器人位置
def get_robot_position(clientID, robot_handle):
    _, position = sim.simxGetObjectPosition(clientID, robot_handle, -1, sim.simx_opmode_blocking)
    return position[0], position[1]

# 获取机器人朝向（z轴偏航角）
def get_robot_orientation(clientID, robot_handle):
    _, orientation = sim.simxGetObjectOrientation(clientID, robot_handle, -1, sim.simx_opmode_blocking)
    return orientation[2]

# 主控制函数：前往目标点
def drive_to_target(clientID, robot_handle, left_joint, right_joint, target, base_speed=2.8, tolerance=0.05):
    prev_angle_error = 0.0
    integral_error = 0.0
    prev_turn_speed = 0.0

    # 限制参数
    max_wheel_speed = 4.0
    angle_large_thresh = 0.05
    angle_small_thresh = 0.01
    max_turn_change = 0.08
    max_turn_speed = 0.9

    # PID控制参数
    Kp_angle = 1.0
    Ki_angle = 0.08
    Kd_angle = 0.35
    Kp_distance = 1.6

    while True:
        x, y = get_robot_position(clientID, robot_handle)
        dx = target[0] - x
        dy = target[1] - y
        distance = math.hypot(dx, dy)

        if distance < tolerance:
            set_wheel_speed(clientID, left_joint, right_joint, 0, 0)
            print(f'✅ 到达目标点 {target}，当前位置 ({x:.2f}, {y:.2f})，距离误差 {distance:.4f}')
            break

        current_angle = get_robot_orientation(clientID, robot_handle)
        target_angle = math.atan2(dy, dx)
        angle_error = (target_angle - current_angle + math.pi) % (2 * math.pi) - math.pi

        # PID控制（带积分项限制）
        integral_error += angle_error
        integral_error = max(-0.5, min(0.5, integral_error))

        delta_error = angle_error - prev_angle_error
        raw_turn_speed = (
            Kp_angle * angle_error +
            Ki_angle * integral_error +
            Kd_angle * delta_error
        )
        raw_turn_speed = max(-max_turn_speed, min(max_turn_speed, raw_turn_speed))

        # 滤波：限制转向速度变化率
        turn_diff = raw_turn_speed - prev_turn_speed
        if abs(turn_diff) > max_turn_change:
            raw_turn_speed = prev_turn_speed + max_turn_change * (1 if turn_diff > 0 else -1)

        prev_turn_speed = raw_turn_speed
        prev_angle_error = angle_error

        # 决定是否减速或直行
        if abs(angle_error) > angle_large_thresh:
            linear_speed = 0.2
        elif abs(angle_error) < angle_small_thresh:
            raw_turn_speed = 0.0
            linear_speed = min(base_speed, Kp_distance * distance)
        else:
            linear_speed = 0.4

        # 速度分配到左右轮
        left_speed = linear_speed - raw_turn_speed
        right_speed = linear_speed + raw_turn_speed

        left_speed = max(-max_wheel_speed, min(max_wheel_speed, left_speed))
        right_speed = max(-max_wheel_speed, min(max_wheel_speed, right_speed))

        set_wheel_speed(clientID, left_joint, right_joint, left_speed, right_speed)
        time.sleep(0.03)

# 从路径文件读取并缩放路径点
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

# 主函数
def main():
    sim.simxFinish(-1)  # 关闭所有连接
    clientID = sim.simxStart('127.0.0.1', 19997, True, True, 5000, 5)

    if clientID != -1:
        print('✅ 已连接到 CoppeliaSim')

        # 获取句柄
        _, robot_handle = sim.simxGetObjectHandle(clientID, 'saodiche', sim.simx_opmode_blocking)
        _, left_joint = sim.simxGetObjectHandle(clientID, 'joint_left', sim.simx_opmode_blocking)
        _, right_joint = sim.simxGetObjectHandle(clientID, 'joint_right', sim.simx_opmode_blocking)

        # 初始化读取位置和角度
        sim.simxGetObjectPosition(clientID, robot_handle, -1, sim.simx_opmode_streaming)
        sim.simxGetObjectOrientation(clientID, robot_handle, -1, sim.simx_opmode_streaming)
        time.sleep(0.1)

        path_points = read_scaled_path_from_file('C:\\Users\\24816\\Desktop\\comprehensive project\\path_output.txt')
        print(f'📄 路径点数量: {len(path_points)}')

        trace = []

        for idx, point in enumerate(path_points):
            print(f'➡️ [{idx+1}] 移动到: {point}')
            drive_to_target(clientID, robot_handle, left_joint, right_joint, point, base_speed=2.8, tolerance=0.05)
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
            plt.title('机器人路径追踪')
            plt.grid(True)
            plt.axis('equal')
            plt.show()

        sim.simxFinish(clientID)
        print('🔚 连接关闭')
    else:
        print('❌ 无法连接 CoppeliaSim，请检查远程API设置')

if __name__ == '__main__':
    main()
