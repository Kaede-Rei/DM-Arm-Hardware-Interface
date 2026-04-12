#!/usr/bin/env python3
import pinocchio as pin
import tsid
import numpy as np
import meshcat
from pinocchio.visualize import MeshcatVisualizer
import time

urdf_path = "/home/kaede-rei/ros2-workspace/moveit-tutorials-ws/src/dm_arm_description/urdf/dm_arm_description.urdf.xacro"

# 创建 TSID 机器人包装器
robot = tsid.RobotWrapper(urdf_path, [], pin.JointModelFreeFlyer(), False)
model = robot.model()
data = robot.data()

print(f"模型加载成功！")
print(f"  关节数 (nq): {model.nq}")
print(f"  速度维度 (nv): {model.nv}")

# meshcat 可视化器
vis = meshcat.Visualizer()
vis.open()
print("\n打开浏览器访问: http://localhost:7000/static/")

# 创建 Pinocchio 的 MeshcatVisualizer，需要碰撞模型和视觉模型
# 从 URDF 构建几何模型（假设 URDF 中包含 mesh 路径）
try:
    # 构建碰撞模型和视觉模型
    collision_model = pin.buildGeomFromUrdf(
        model, urdf_path, pin.GeometryType.COLLISION
    )
    visual_model = pin.buildGeomFromUrdf(model, urdf_path, pin.GeometryType.VISUAL)
except Exception as e:
    print(f"构建几何模型失败: {e}")
    print("尝试只使用视觉模型...")
    visual_model = pin.buildGeomFromUrdf(model, urdf_path, pin.GeometryType.VISUAL)
    collision_model = None

# 创建可视化器
viz = MeshcatVisualizer(model, collision_model, visual_model, data=data)
viz.initViewer(vis, loadModel=True)

# 显示中立位置
q_neutral = pin.neutral(model)
print(f"\n中立位置关节向量 (大小 {len(q_neutral)}):")
print(q_neutral)

# 将机器人配置发布到 meshcat
viz.display(q_neutral)

# 动态演示
print("\n开始动态演示（按 Ctrl+C 停止）...")
dt = 0.05
t = 0.0
try:
    while True:
        # 简单的正弦运动，只改变第一个关节
        q = q_neutral.copy()
        if model.existJointName("joint1"):
            joint_id = model.getJointId("joint1")
            q[model.idx_qs[joint_id]] = 0.5 * np.sin(t)

        viz.display(q)
        time.sleep(dt)
        t += dt
except KeyboardInterrupt:
    print("\n演示结束。")

print("\n脚本运行完毕，meshcat 窗口保持打开。")
input("按 Enter 键退出...")
