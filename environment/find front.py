import sim
import time
import math
import matplotlib.pyplot as plt

# 设置轮子速度
def set_wheel_speed(clientID, left_joint, right_joint, left_speed, right_speed):
    sim.simxSetJointTargetVelocity(clientID, left_joint, left_speed, sim.simx_opmode_streaming)
    sim.simxSetJointTargetVelocity(clientID, right_joint, right_speed, sim.simx_opmode_streaming)

# 获取机器人位置
def get_robot_position(clientID, robot_handle):
    _, position = sim.simxGetObjectPosition(clientID, robot_handle, -1, sim.simx_opmode_blocking)
    return position[0], position[1]

# 获取机器人朝向
def get_robot_orientation(clientID, robot_handle):
    _, orientation = sim.simxGetObjectOrientation(clientID, robot_handle, -1, sim.simx_opmode_blocking)
    return orientation[2]

# 从路径中选取“前瞻”目标点
def find_lookahead_point(path, current_pos, lookahead_dist):
    for point in path:
        if math.hypot(point[0] - current_pos[0], point[1] - current_pos[1]) > lookahead_dist:
            return point
    return path[-1]  # 如果都太近，返回最后一个点

# 路径跟踪控制器（整合路径前瞻 + PID + 滤波）
def drive_with_lookahead(clientID, robot_handle, left_joint, right_joint, path, base_speed=1.5):
    prev_angle_error = 0.0
    integral_error = 0.0
    prev_turn_speed = 0.0

    lookahead_distance = 0.5
    tolerance = 0.2
    max_wheel_speed = 4.0

    # PID参数
    Kp_angle = 1.2
    Ki_angle = 0.15
    Kd_angle = 0.5
    Kp_distance = 1.0

    angle_large_thresh = 0.06
    angle_small_thresh = 0.015
    max_turn_change = 0.08
    max_turn_speed = 0.5

    trace = []
    path_index = 0
    total_points = len(path)

    while path_index < total_points:
        x, y = get_robot_position(clientID, robot_handle)
        current_pos = (x, y)
        trace.append(current_pos)

        target = find_lookahead_point(path[path_index:], current_pos, lookahead_distance)
        distance = math.hypot(target[0] - x, target[1] - y)

        if distance < tolerance and path_index < total_points - 1:
            path_index += 1
            continue

        current_angle = get_robot_orientation(clientID, robot_handle)
        target_angle = math.atan2(target[1] - y, target[0] - x)
        angle_error = (target_angle - current_angle + math.pi) % (2 * math.pi) - math.pi

        # PID控制
        integral_error += angle_error
        integral_error = max(-1.0, min(1.0, integral_error))  # 限制积分项
        delta_error = angle_error - prev_angle_error

        raw_turn_speed = (
            Kp_angle * angle_error +
            Ki_angle * integral_error +
            Kd_angle * delta_error
        )
        raw_turn_speed = max(-max_turn_speed, min(max_turn_speed, raw_turn_speed))

        # 滤波
        turn_diff = raw_turn_speed - prev_turn_speed
        if abs(turn_diff) > max_turn_change:
            raw_turn_speed = prev_turn_speed + max_turn_change * (1 if turn_diff > 0 else -1)
        prev_turn_speed = raw_turn_speed
        prev_angle_error = angle_error

        # 根据角度误差调整线速度
        if abs(angle_error) > angle_large_thresh:
            linear_speed = 0.2
        elif abs(angle_error) < angle_small_thresh:
            raw_turn_speed = 0.0
            linear_speed = min(base_speed, Kp_distance * distance)
        else:
            linear_speed = 0.6

        # 合成速度指令
        left_speed = linear_speed - raw_turn_speed
        right_speed = linear_speed + raw_turn_speed

        left_speed = max(-max_wheel_speed, min(max_wheel_speed, left_speed))
        right_speed = max(-max_wheel_speed, min(max_wheel_speed, right_speed))

        set_wheel_speed(clientID, left_joint, right_joint, left_speed, right_speed)
        time.sleep(0.05)

    # 到终点后停止
    set_wheel_speed(clientID, left_joint, right_joint, 0, 0)
    print('✅ 路径执行完毕')
    return trace

# 读取路径
def read_scaled_path_from_file(filename):
    path_points = []
    with open(filename, 'r') as file:
        for line in file:
            line = line.strip().replace('(', '').replace(')', '')
            if line:
                parts = line.split(',')
                try:
                    x_raw, y_raw = float(parts[0]), float(parts[1])
                    x = x_raw / 30.46 * 6 - 10
                    y = y_raw / 30.46 * 6
                    path_points.append((x, y))
                except:
                    print(f"⚠️ 无效路径点：{line}")
    return path_points

# 主函数
def main():
    sim.simxFinish(-1)
    clientID = sim.simxStart('127.0.0.1', 19997, True, True, 5000, 5)

    if clientID != -1:
        print('✅ 成功连接到 CoppeliaSim')
        _, robot_handle = sim.simxGetObjectHandle(clientID, 'saodiche', sim.simx_opmode_blocking)
        _, left_joint = sim.simxGetObjectHandle(clientID, 'joint_left', sim.simx_opmode_blocking)
        _, right_joint = sim.simxGetObjectHandle(clientID, 'joint_right', sim.simx_opmode_blocking)

        sim.simxGetObjectPosition(clientID, robot_handle, -1, sim.simx_opmode_streaming)
        sim.simxGetObjectOrientation(clientID, robot_handle, -1, sim.simx_opmode_streaming)
        time.sleep(0.2)

        path = read_scaled_path_from_file('C:\\Users\\24816\\Desktop\\comprehensive project\\path_output.txt')
        print(f'📍 路径点数量: {len(path)}')

        trace = drive_with_lookahead(clientID, robot_handle, left_joint, right_joint, path)

        # 绘图
        if trace:
            plt.figure(figsize=(8, 6))
            plt.plot([p[0] for p in trace], [p[1] for p in trace], 'b.-', label='机器人轨迹')
            plt.plot([p[0] for p in path], [p[1] for p in path], 'rx--', label='目标路径')
            plt.xlabel('X')
            plt.ylabel('Y')
            plt.legend()
            plt.grid(True)
            plt.title('机器人路径跟踪（带路径前瞻）')
            plt.axis('equal')
            plt.show()

        sim.simxFinish(clientID)
        print('🔚 已断开连接')
    else:
        print('❌ 无法连接到 CoppeliaSim')

if __name__ == '__main__':
    main()