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
    return orientation[2]  # 只考虑 z 轴偏航角

# PID控制参数
Kp_angle = 1.2
Kd_angle = 0.6
Kp_distance = 1.0

def drive_to_target(clientID, robot_handle, left_joint, right_joint, target, base_speed=1.0, tolerance=0.2):
    prev_angle_error = 0.0
    prev_turn_speed = 0.0

    max_wheel_speed = 3.0
    angle_large_thresh = 0.15  # 启用转向控制（误差大）
    angle_small_thresh = 0.05  # 启用直线控制（误差小）
    max_turn_change = 0.2      # PID输出变化速率限制

    while True:
        x, y = get_robot_position(clientID, robot_handle)
        dx = target[0] - x
        dy = target[1] - y
        distance = math.hypot(dx, dy)

        if distance < tolerance:
            set_wheel_speed(clientID, left_joint, right_joint, 0, 0)
            print(f'✅ 到达目标点 {target}，当前位置 ({x:.2f}, {y:.2f})，距离误差 {distance:.4f}')
            break

        _, orientation = sim.simxGetObjectOrientation(clientID, robot_handle, -1, sim.simx_opmode_blocking)
        current_angle = orientation[2]
        target_angle = math.atan2(dy, dx)
        angle_error = (target_angle - current_angle + math.pi) % (2 * math.pi) - math.pi

        delta_error = angle_error - prev_angle_error
        raw_turn_speed = Kp_angle * angle_error + Kd_angle * delta_error
        raw_turn_speed = max(-1.0, min(1.0, raw_turn_speed))

        # 平滑变化，防止抖动
        turn_diff = raw_turn_speed - prev_turn_speed
        if abs(turn_diff) > max_turn_change:
            raw_turn_speed = prev_turn_speed + max_turn_change * (1 if turn_diff > 0 else -1)

        prev_turn_speed = raw_turn_speed
        prev_angle_error = angle_error

        if abs(angle_error) > angle_large_thresh:
            # 角度误差大：主要转向，低速前进
            linear_speed = 0.2
        elif abs(angle_error) < angle_small_thresh:
            # 角度误差小：主要前进，不转向
            raw_turn_speed = 0.0
            linear_speed = min(base_speed, Kp_distance * distance)
        else:
            # 角度中间状态：边转边走
            linear_speed = 0.5

        left_speed = linear_speed - raw_turn_speed
        right_speed = linear_speed + raw_turn_speed

        # 限速
        left_speed = max(-max_wheel_speed, min(max_wheel_speed, left_speed))
        right_speed = max(-max_wheel_speed, min(max_wheel_speed, right_speed))

        set_wheel_speed(clientID, left_joint, right_joint, left_speed, right_speed)
        time.sleep(0.05)



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

        sim.simxGetObjectPosition(clientID, robot_handle, -1, sim.simx_opmode_streaming)
        sim.simxGetObjectOrientation(clientID, robot_handle, -1, sim.simx_opmode_streaming)
        time.sleep(0.1)

        path_points = read_scaled_path_from_file('C:\\Users\\24816\\Desktop\\comprehensive project\\path_output.txt')
        print(f'📄 路径点数量: {len(path_points)}')

        trace = []

        for idx, point in enumerate(path_points):
            print(f'➡️ [{idx+1}] 移动到: {point}')
            drive_to_target(clientID, robot_handle, left_joint, right_joint, point, base_speed=1.5)
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
