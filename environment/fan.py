import sim
import time
import math

def set_wheel_speed(clientID, left_joint, right_joint, left_speed, right_speed):
    sim.simxSetJointTargetVelocity(clientID, left_joint, left_speed, sim.simx_opmode_streaming)
    sim.simxSetJointTargetVelocity(clientID, right_joint, right_speed, sim.simx_opmode_streaming)

def get_robot_position(clientID, robot_handle):
    _, position = sim.simxGetObjectPosition(clientID, robot_handle, -1, sim.simx_opmode_blocking)
    return position[0], position[1]

def drive_to_target(clientID, robot_handle, left_joint, right_joint, target, speed=2.0, tolerance=0.1):
    while True:
        x, y = get_robot_position(clientID, robot_handle)
        dx = target[0] - x
        dy = target[1] - y
        distance = math.hypot(dx, dy)

        if distance < tolerance:
            set_wheel_speed(clientID, left_joint, right_joint, 0, 0)
            print(f'✅ 到达目标点 {target}，当前位置 ({x:.2f}, {y:.2f})，距离误差 {distance:.4f}')
            break

        set_wheel_speed(clientID, left_joint, right_joint, speed, speed)
        time.sleep(0.05)

def read_path_from_file(filename):
    path = []
    with open(filename, 'r') as file:
        for line in file:
            if line.strip():
                x_str, y_str = line.strip().split(',')
                path.append((float(x_str), float(y_str)))
    return path

def main():
    sim.simxFinish(-1)
    clientID = sim.simxStart('127.0.0.1', 19997, True, True, 5000, 5)

    if clientID != -1:
        print('✅ 成功连接 CoppeliaSim')

        _, robot_handle = sim.simxGetObjectHandle(clientID, 'saodiche', sim.simx_opmode_blocking)
        _, left_joint = sim.simxGetObjectHandle(clientID, 'joint_left', sim.simx_opmode_blocking)
        _, right_joint = sim.simxGetObjectHandle(clientID, 'joint_right', sim.simx_opmode_blocking)

        path_points = read_path_from_file('path_output.txt')
        print(f'📄 共读取路径点 {len(path_points)} 个')

        for idx, point in enumerate(path_points):
            print(f'➡️ [{idx+1}] 前往路径点: {point}')
            drive_to_target(clientID, robot_handle, left_joint, right_joint, point, speed=1.5, tolerance=0.1)

        sim.simxFinish(clientID)
        print('🔚 已断开连接')
    else:
        print('❌ 连接失败，请确认CoppeliaSim远程API服务已启动')

if __name__ == '__main__':
    main()
