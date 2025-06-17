import sim
import time
import math


# 从文件中读取路径点并缩小比例
def load_path_from_file(filename):
    path_points = []
    with open(filename, 'r') as file:
        for line in file:
            line = line.strip().replace('(', '').replace(')', '').replace('\n', '')
            if line == '':
                continue
            parts = line.split(',')
            if len(parts) >= 2:
                try:
                    x, y = float(parts[0]), float(parts[1])
                    path_points.append((x  / 30.75 * 6 - 10, y / 30.75 * 6))  # 缩小比例
                except ValueError:
                    print(f"跳过无效行: {line}")
    return path_points


# 模拟速度控制的移动函数
def move_to_position(clientID, robot_handle, start_pos, end_pos, speed=0.5):
    dx = end_pos[0] - start_pos[0]
    dy = end_pos[1] - start_pos[1]
    dz = end_pos[2] - start_pos[2]

    distance = math.sqrt(dx**2 + dy**2 + dz**2)
    if distance == 0:
        return

    steps = int(distance / (speed * 0.3))  # 控制步数（越多越平滑）
    steps = max(1, steps)

    for i in range(1, steps + 1):
        x = start_pos[0] + dx * i / steps
        y = start_pos[1] + dy * i / steps
        z = start_pos[2] + dz * i / steps
        sim.simxSetObjectPosition(clientID, robot_handle, -1, [x, y, z], sim.simx_opmode_blocking)
        time.sleep(0.05)  # 控制步进速度


def main():
    sim.simxFinish(-1)  # 清除旧连接
    clientID = sim.simxStart('127.0.0.1', 19997, True, True, 5000, 5)

    if clientID != -1:
        print('✅ 成功连接 CoppeliaSim')

        # 获取机器人句柄（修改为你场景中实际名称）
        err_code, robot_handle = sim.simxGetObjectHandle(clientID, 'Sphere', sim.simx_opmode_blocking)
        if err_code != 0:
            print('❌ 获取机器人句柄失败！')
            sim.simxFinish(clientID)
            return

        # 读取路径点
        filename = 'C:\\Users\\24816\\Desktop\\comprehensive project\\path_output.txt'
        path_points = load_path_from_file(filename)
        print(f'📌 共读取路径点: {len(path_points)}')

        # 获取初始位置
        err, current_pos = sim.simxGetObjectPosition(clientID, robot_handle, -1, sim.simx_opmode_blocking)
        if err != 0:
            print("❌ 获取当前坐标失败")
            sim.simxFinish(clientID)
            return

        # 循环移动
        for idx, point in enumerate(path_points):
            x_target, y_target = point
            target_pos = [x_target, y_target, 0.12]  # z轴固定高度
            print(f'➡️ [{idx+1}] 正在移动到: {target_pos}')
            move_to_position(clientID, robot_handle, current_pos, target_pos, speed=2)
            current_pos = target_pos
            print(f'✅ 已移动到点: {point}\n')

        sim.simxFinish(clientID)
        print('🔚 仿真结束，已断开连接。')

    else:
        print('❌ 连接失败，请确认CoppeliaSim已开启远程API接口')


if __name__ == '__main__':
    main()
