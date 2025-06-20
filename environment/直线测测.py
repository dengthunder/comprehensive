import sim
import time

# 设置左右轮速度
def set_wheel_speed(clientID, left_joint, right_joint, left_speed, right_speed):
    sim.simxSetJointTargetVelocity(clientID, left_joint, left_speed, sim.simx_opmode_oneshot)
    sim.simxSetJointTargetVelocity(clientID, right_joint, right_speed, sim.simx_opmode_oneshot)

# 控制机器人走直线
def move_straight(clientID, left_joint, right_joint, speed=1.5, duration=3.0):
    """
    控制机器人以指定轮速走直线，持续时间 duration（单位秒）
    """
    # 初始化停稳
    set_wheel_speed(clientID, left_joint, right_joint, 0, 0)
    time.sleep(0.2)  # 防止启动抖动

    # 设置左右轮相同速度，走直线
    set_wheel_speed(clientID, left_joint, right_joint, speed, speed)

    # 持续运行一段时间
    start_time = time.time()
    while time.time() - start_time < duration:
        time.sleep(0.01)

    # 停车
    set_wheel_speed(clientID, left_joint, right_joint, 0, 0)
    print(f"✅ 直线行驶 {duration:.1f} 秒完成")

# 主程序
def main():
    sim.simxFinish(-1)  # 关闭旧连接
    clientID = sim.simxStart('127.0.0.1', 19997, True, True, 5000, 5)

    if clientID != -1:
        print('✅ 成功连接到 CoppeliaSim')

        # 获取机器人底盘与轮子句柄
        _, robot_handle = sim.simxGetObjectHandle(clientID, 'saodiche', sim.simx_opmode_blocking)
        _, left_joint = sim.simxGetObjectHandle(clientID, 'joint_left', sim.simx_opmode_blocking)
        _, right_joint = sim.simxGetObjectHandle(clientID, 'joint_right', sim.simx_opmode_blocking)

        # 启动位置与方向数据流
        sim.simxGetObjectPosition(clientID, robot_handle, -1, sim.simx_opmode_streaming)
        sim.simxGetObjectOrientation(clientID, robot_handle, -1, sim.simx_opmode_streaming)
        time.sleep(0.2)

        # 直线运行函数调用
        move_straight(clientID, left_joint, right_joint, speed=2.0, duration=9.0)

        sim.simxFinish(clientID)
        print('🔚 已断开与 CoppeliaSim 的连接')
    else:
        print('❌ 无法连接 CoppeliaSim，请检查远程API设置')

if __name__ == '__main__':
    main()
